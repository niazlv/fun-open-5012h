/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Above-nyquist reasoning for the spectrum: which true frequencies could
 * have produced the peak that was measured, and whether the record itself
 * rules the in-band reading out.
 *
 * Pure C, no hardware dependencies: host-tested against synthetic waveforms.
 */

#ifndef _ALIAS_H_
#define _ALIAS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fft.h"

/*- Definitions -------------------------------------------------------------*/
#define ALIAS_MAX_CANDIDATES  8

// How far past the frontend's usable band the candidate ladder is walked.
// Beyond this the analog chain delivers nothing a peak could be made of, so
// naming those frequencies would be padding the list rather than answering.
#define ALIAS_LADDER_MULT     3

// Where this instrument's analog chain stops passing anything useful. It is
// the ONE number the whole module is calibrated by, and it is deliberately a
// single number rather than a response curve - see alias_check().
//
// The value is a placeholder from the part's rating, not a measurement. To
// replace it with one: feed a clean square at f, read the h3 line (at
// 3f, or at its folded position when 3f is past nyquist), and subtract the
// -9.54 dB a square's 3rd harmonic starts at. That is the chain's loss at
// 3f. Walk f up until the h3 line disappears into the floor; that f*3 is
// this number. One bench point exists so far: a 25 MHz square read h3 at
// -22.9 dB, so the chain is already 13.4 dB down at 75 MHz.
#define ALIAS_FRONTEND_HZ     100.0e6f

// Odd harmonics are the ones a hard-edged source cannot be without: a square
// of ANY duty cycle has them, while its even harmonics vanish at 50%.
#define ALIAS_HARM_LAST       9

// Relative level below which an h3 line cannot be explained by an in-band
// hard-edged source, and above which it cannot be explained by an aliased
// one. A square starts h3 at -9.54 dB; the chain then eats some of it. The
// lowest above-nyquist candidate is Fs - a, so ITS h3 sits at 3*(Fs - a) -
// past 187 MHz for this instrument at full rate, where the chain is dead.
// The gap between "one octave into the rolloff" and "three times past the
// band edge" is wide, and this sits in it.
#define ALIAS_HARM_MIN_DB     (-30.0f)

// Times the spectrum floor a line must stand to count as present at all.
// Same figure the peak finder uses, and the same figure again in dB.
#define ALIAS_MIN_SNR         3.0f
#define ALIAS_MIN_SNR_DB      9.54f

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  float freq;   // Hz - the true frequency this candidate proposes
  int   order;  // n in f = n*Fs +- a; 0 is the in-band reading
} AliasCandidate;

typedef enum
{
  // The record cannot answer: every odd harmonic folds onto DC or onto the
  // fundamental itself, or h3 would be past the frontend even if the reading
  // is honest, or the noise floor sits above where h3 would be.
  ALIAS_NO_EVIDENCE,

  // Harmonics stand where only an in-band source can put them. Reading the
  // peak as the real frequency is safe.
  ALIAS_IN_BAND,

  // No harmonic content worth the name. Two signals produce that and the
  // record cannot tell them apart: a genuine sine below nyquist, and a
  // hard-edged signal from above it whose harmonics are past the frontend.
  // cand[] is then the honest answer instead of one number.
  ALIAS_POSSIBLE,
} AliasVerdict;

typedef struct
{
  AliasVerdict verdict;
  int    slots;        // usable odd-harmonic positions the test could check
  float  harm_db;      // strongest of them, dB relative to the fundamental
  float  floor_db;     // weakest line the record could have shown, same ref
  int    count;        // entries in cand[], ascending; [0] is the in-band one
  AliasCandidate cand[ALIAS_MAX_CANDIDATES];
  const char *reason;  // one short line, for the panel and for the tests
} AliasAnalysis;

/*- Prototypes --------------------------------------------------------------*/
// Every true frequency that folds to `observed_hz` at this sample rate, up
// to `limit_hz`, ascending: a, Fs-a, Fs+a, 2Fs-a, ...
//
// The list is complete and it is irreducible. k*(n*Fs +- a) = k*n*Fs +- k*a,
// and k*n*Fs is zero modulo Fs, so EVERY harmonic of EVERY candidate folds
// to the same place as every other candidate's - positions carry no
// information about n, for any harmonic, at any sample rate. Decimating does
// not help either: Fs' = Fs/M divides Fs, so anything indistinguishable at
// Fs stays indistinguishable at Fs'. Separating these needs a sample rate
// that is not a divisor of this one, i.e. hardware.
int alias_candidates(float observed_hz, float nyquist_hz, float limit_hz,
    AliasCandidate *out, int max);

// Whether the record's own harmonic content agrees with reading
// `an->fundamental` as a real in-band frequency.
//
// What makes this work without a calibrated response curve is one asymmetry.
// A hard-edged source at frequency f puts h3 at 3f. If f is in band, 3f is
// at most 3*nyquist and the frontend still passes some of it. If f is an
// alias, the LOWEST it can be is Fs - a, so its h3 starts at 3*(Fs - a) -
// three times past the band edge, where nothing survives. So a visible h3
// is evidence only an in-band source can produce, and its ABSENCE is what
// leaves the two readings tied.
//
// The test is refused rather than guessed in three cases, all of them real:
//   - 3f is past the frontend anyway, so h3's absence proves nothing (which
//     is what stops a genuine 50 MHz square from being called an alias)
//   - the fold lands h3 on DC or on the fundamental, where it cannot be
//     read. At f = Fs/4 every odd harmonic folds onto the fundamental, at
//     f = Fs/3 onto DC or the fundamental - both are honest in-band squares
//     that a naive "no h3 -> alias" check calls aliased.
//   - the noise floor sits above where h3 would be
//
// Note what is NOT a gate: whether the record LOOKS like a square. An
// aliased square arrives as a bare sine - the harmonics that made it a
// square are exactly what the frontend removed - so gating on the time
// domain would reject precisely the case worth catching.
//
// mag is the spectrum `an` was analysed from; bandwidth_hz is
// ALIAS_FRONTEND_HZ unless a bench measurement has replaced it.
void alias_check(const float *mag, const FftAnalysis *an, float bandwidth_hz,
    AliasAnalysis *out);

#endif // _ALIAS_H_
