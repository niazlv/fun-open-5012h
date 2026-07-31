/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Amplitude-modulation detection: whether the carrier in the record is
 * carrying anything, how deep and how fast - without needing to know what
 * the carrier's frequency actually is.
 *
 * That last part is the whole reason this module exists. Sampling is a
 * LINEAR operation, so a record of A(t)*cos(2*pi*fc*t) taken at Fs holds
 * A(nT)*cos(2*pi*fa*nT + phi), where fa is fc folded into the band. The
 * envelope A(t) survives the fold EXACTLY; only the carrier's identity is
 * lost. So the depth and the rate below are true readings even when the
 * carrier is past nyquist and alias.c can only offer a list of candidates
 * for what it was.
 *
 * Pure C, no hardware dependencies: host-tested against synthetic waveforms.
 *
 *
 * NOT BUILT INTO THE FIRMWARE. There is no RAM for it.
 *
 * The module is finished and its behaviour is pinned by the host tests
 * (tests/host_test.c, `make test`), but it is deliberately absent from
 * make/Makefile's source list, so nothing below costs the instrument a byte.
 *
 * What stops it is TCM, measured rather than estimated:
 *
 *   baseline image      tcm 48 928 of 64 K -> 16 608 free
 *   linker guard        16 384 for stack + heap
 *   slack, before this  224 bytes
 *
 * The buffers below come to 3584 bytes (envelope 2 K, its requantised copy
 * 512, its spectrum 1 K), so the module is 2984 bytes short. Flash is not the
 * constraint and never was: the code costs 3760 bytes against ~31 K spare.
 *
 * Two things were tried and measured. Moving these buffers into the
 * .tcm_borrowed region beside fft.o's frees 376 bytes - the whole gap between
 * that region's 29.5 K floor and the 29 745 it actually holds - and no more,
 * because the region is inside TCM and moving bytes within TCM creates none.
 * Stripping the module to a single 8-bit envelope and borrowing the spectrum
 * buffer from the caller does link, with 80 bytes to spare; the comment over
 * .tcm_borrowed in linker/gd32f407ve.ld records what an 88-byte margin cost
 * this project the last time, so that is not a version worth shipping.
 *
 * The 16 K guard is 8 K of stack (enforced by STACK_RESERVE in syscalls.c,
 * where _sbrk refuses to bring the heap closer) plus 8 K of heap headroom. No
 * file in this project calls malloc; _malloc_r is in the image only because
 * newlib's printf family drags it in. So most of that 8 K is probably dead,
 * and "probably" is the word that has to go before this is reconsidered -
 * debug_coredump.c already carries stack_free, so the device can answer it.
 *
 * When there IS room, note that a metric of its own is the wrong shape for
 * this: AM is a rare enough case that a permanent cell in the measurements
 * panel is not worth the screen it occupies. The natural home is the signal
 * classifier - classify.c already names what a record IS, and "AM 45%"
 * belongs beside "Sine" and "Square" rather than in a field of its own.
 */

#ifndef _AM_H_
#define _AM_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fft.h"

/*- Definitions -------------------------------------------------------------*/
// Envelope points the detector works with. The block is the record divided
// by this, so the envelope always spans the WHOLE record however fast the
// sampling was: its resolution is 1/record_time, the same figure a
// whole-record spectrum gets. Equal to FFT_SIZE because the envelope then
// goes through the very transform the spectrum does - see am_analyze().
#define AM_ENV_POINTS     FFT_SIZE

// Carrier cycles one block must contain before its RMS means anything.
// Below this the block RMS tracks the carrier's own phase rather than the
// envelope, and the detector reads its own estimator instead of the signal.
// See the artifact note over env_residual_hz() in am.c for what that leaves
// behind even above the gate.
#define AM_MIN_CYCLES     4.0f

// Times the envelope spectrum's floor the modulation line must stand to
// count as a line at all. Same figure the peak finder and alias.c use.
//
// It is the WEAKEST of the three gates here and deliberately not the one
// doing the work: a synthetic record has no floor worth the name, so a 0.27%
// ripple stands 57 dB above it and passes this comfortably. What rejects it
// is the depth floor below.
#define AM_MIN_SNR        3.0f

// The block-RMS estimator's own ripple, as a fraction of the envelope.
//
// The residual left by a block that does not hold a whole number of carrier
// cycles is suppressed by about 1/(4*pi*cycles) - so a "modulation" shallower
// than that is the estimator rather than the signal, and the bound tightens
// as the carrier gets faster instead of being one number for every record.
//
// Measured against the bound on synthetic carriers: a 17.4-cycle block leaves
// 0.27% where this predicts 0.46%, a 4.1-cycle block leaves 1.08% against
// 1.95%. Conservative by about 1.8x at both ends, which is the right
// direction for a bound to be wrong in.
#define AM_RESIDUAL_K     0.08f

// A cross-check the predicted frequency of that ripple cannot do.
//
// Predicting WHERE the residual lands needs 2*carrier folded into the
// envelope band, and that doubles any error in the carrier reading before
// folding turns it into a difference of large numbers. Measured: a carrier
// read 11 kHz off put the prediction 24 kHz out, twelve envelope bins, and
// the same arithmetic on a two-tone beat landed the prediction ON the real
// modulation and rejected it. The frequency is not knowable well enough.
//
// What IS knowable: a real modulation does not care how the record was cut
// into blocks, and the estimator's beat is made of the cut. Rebuilding the
// envelope with a block one sample shorter moves the beat by 4*fa*Tb/block
// of the band - at least 16/block by the AM_MIN_CYCLES gate, so a fifth of
// the band on a 96 KB record - while a real line stays put. A line that
// keeps less than this fraction of its depth was the cut talking.
#define AM_CROSS_KEEP     0.5f

// Blocks under this cannot be shortened by one and still mean anything, so
// the cross-check is skipped and the depth floor carries the record alone
#define AM_CROSS_MIN_BLOCK  8

// Envelope bins under which a "rate" is the record drifting rather than
// modulating: at one bin the whole record holds a single cycle, and nothing
// separates that from a slow tilt or the trigger walking.
#define AM_MIN_RATE_BINS  3.0f

// Counts of RMS the carrier must reach before its envelope is worth
// measuring at all. Under this the envelope is the ADC's own noise, and its
// ripple is the noise's ripple.
#define AM_MIN_CARRIER    4.0f

// Peak-to-peak depth past which the modulation is deep enough to be keying
// rather than AM - but only when the envelope also SITS at its two levels
// for at least AM_BURST_LEVEL of the record each. A 100% sinusoidal AM
// reaches the same depth and passes through its extremes without stopping,
// which is exactly what tells the two apart.
#define AM_BURST_DEPTH    0.8f
#define AM_BURST_LEVEL    0.2f

// Depth under which nothing is claimed whatever the arithmetic says, and
// under which the envelope counts as steady. Eight bits through a real
// frontend do not support a reading below this, and every record the tests
// feed in is cleaner than any record a probe will bring.
#define AM_FLAT_DEPTH     0.02f

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  // The record cannot answer: no carrier was handed in, the record is too
  // short to cut into blocks, the carrier is too slow for a block to hold,
  // or the only rate found is the estimator's own beat.
  AM_NO_EVIDENCE,

  // A carrier whose envelope is steady. A POSITIVE result and not a
  // failure: it is what separates FM and PSK from AM, since those carry
  // their information at a constant amplitude.
  AM_FLAT,

  // The envelope carries a rate and a depth, and both are reported.
  AM_MODULATED,

  // The envelope sits at two levels rather than travelling between them:
  // OOK/ASK keying, or a packet against a quiet background.
  AM_BURST,

  // The envelope moves, and no single rate explains it: drift, fading,
  // several modulations at once, or a record that is simply noise.
  AM_ERRATIC,
} AmVerdict;

typedef struct
{
  AmVerdict verdict;

  // Modulation index m of A(t) = A0*(1 + m*cos(2*pi*fm*t)), taken from the
  // envelope's line at rate_hz rather than from its extremes, so envelope
  // noise spreads across the spectrum instead of inflating the reading.
  float depth;

  // (Emax-Emin)/(Emax+Emin) over the whole record. Agrees with depth for a
  // sinusoidal modulation and deliberately does not for anything else -
  // keying and single events show up in the gap between the two.
  float depth_pp;

  // Rate of the line found, and 0 when none stood at all. Populated even
  // when the line was then REJECTED, so a bench can see what was thrown
  // away and why - any verdict other than AM_MODULATED or AM_BURST means
  // depth and rate_hz describe a line the record would not stand behind.
  float rate_hz;
  float snr_db;          // that line over the envelope's floor

  // The carrier AS OBSERVED, i.e. what was handed in. When it is an alias
  // this is NOT the real carrier - the readings above still are. Pass it
  // through alias_candidates() to name what it could have been.
  float carrier_hz;

  float env_bin_hz;      // envelope resolution: 1/record_time
  float env_nyquist_hz;  // fastest modulation this record could show
  float cycles_per_block;
  int   block;           // record samples behind one envelope point

  // The line found did not survive being re-cut (see AM_CROSS_KEEP): it is
  // made of the block boundaries rather than of the signal. Never set
  // together with a verdict of AM_MODULATED or AM_BURST.
  bool  artifact;

  const char *reason;    // one short line, for the panel and for the tests
} AmAnalysis;

/*- Prototypes --------------------------------------------------------------*/
// Envelope analysis of a record: cut it into AM_ENV_POINTS blocks, take the
// RMS of each about its own mean, and read the series that comes out.
//
// data/size/offset/period_ns are the usual record; carrier_hz is the
// frequency the spectrum READ, i.e. fft_analyze()'s fundamental, folded or
// not. It is needed for two things that have nothing to do with naming the
// carrier: sizing the estimator's validity (AM_MIN_CYCLES) and predicting
// where the estimator's own artifact lands.
//
// What this is not:
//
//   - a demodulator. It reports that a modulation is present, how deep and
//     how fast. It does not recover what was modulated onto the carrier.
//
//   - able to tell AM from a sum of tones. A carrier with two sidebands and
//     three independent generators at the same three frequencies are the
//     same signal, and no envelope test can separate what addition already
//     made identical. Two oscillators beating read as AM here, at their
//     difference frequency, and that reading is correct.
//
//   - able to tell a modulation from a carrier beating with its OWN folded
//     harmonic. A hard-edged carrier near Fs/4 has its 3rd harmonic fold
//     back onto its own fundamental, and near Fs/6 onto nyquist; the two
//     then beat, and the record genuinely contains an envelope moving at
//     that rate. Measured over hard-edged carriers from 19 to 34 MHz at 125
//     MS/s, 2.3% of them read as AM this way, at depths up to 31%, while no
//     sinusoidal carrier did at any frequency. It is the same folding
//     alias.c reasons about, arriving in the envelope instead of the
//     spectrum: when alias_check() reports harmonics AND the rate here
//     matches the spacing between the folded lines, the beat is the
//     carrier's own and not somebody's modulation.
//
//   - usable on a carrier the record cannot resolve. The record must supply
//     AM_ENV_POINTS blocks of at least AM_MIN_CYCLES carrier cycles each,
//     which on a full 96 KB record at 125 MS/s means an observed carrier at
//     or above ~2.6 MHz. Slowing the timebase lowers that bound in
//     proportion, and slowing it does NOT lose the carrier: the sample clock
//     is a prescaler on the ADC, so a slower record still undersamples the
//     same RF, it just folds it somewhere else.
//
// The modulation rates a record can show run from AM_MIN_RATE_BINS *
// env_bin_hz to env_nyquist_hz, both reported back so the caller can say
// what the record was capable of rather than only what it found.
void am_analyze(const uint8_t *data, int size, int offset, int period_ns,
    float carrier_hz, AmAnalysis *out);

const char *am_verdict_name(AmVerdict v);

#endif // _AM_H_
