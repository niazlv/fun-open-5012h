/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Radix-2 FFT spectrum for the oscilloscope, sized for the Cortex-M4 FPU.
 *
 * Pure C (float + libm), no hardware dependencies: this module is also
 * compiled and tested on the host against synthetic waveforms.
 */

#ifndef _FFT_H_
#define _FFT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>

/*- Definitions -------------------------------------------------------------*/
#define FFT_SIZE       512
#define FFT_BINS       (FFT_SIZE / 2)
#define FFT_MAX_PEAKS  6
#define FFT_MAX_HARM   12   // harmonics the comb search will look for

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int    bin;       // bin the peak was found in
  float  freq;      // Hz, parabolically interpolated between bins
  float  mag;       // linear magnitude, interpolated
  float  rel_db;    // dB relative to the strongest peak (<= 0)
  int    harmonic;  // n, when this peak is the n-th harmonic of `fundamental`
} FftPeak;

typedef struct
{
  int      count;                  // peaks found, strongest first
  FftPeak  peak[FFT_MAX_PEAKS];
  float    fundamental;            // Hz; the comb the peaks belong to, 0 if none
  int      harmonics;              // how many peaks the comb explained
  int      thd_x10;                // THD in 0.1% units; -1 when not computable
  float    bin_hz;                 // frequency resolution (span between bins)
  float    nyquist_hz;             // top of the spectrum
  float    noise_db;               // mean spectrum floor, dB below the peak
} FftAnalysis;

/*- Prototypes --------------------------------------------------------------*/
// Windows (Hann) and transforms FFT_SIZE samples taken from the ring buffer
// starting at `offset`, writes FFT_BINS linear magnitudes. DC is removed
// before windowing so bin 0 does not swamp the display.
void fft_spectrum(const uint8_t *data, int size, int offset, float *mag);

// Same, but the FFT_SIZE input points are decimated by `decim` from the
// record, so the transform spans FFT_SIZE*decim samples instead of
// FFT_SIZE. That is what buys frequency resolution: df = 1/(span * period),
// i.e. a whole-record transform resolves 1/record_time regardless of the
// sample rate. Every output point is a 3*decim-tap weighted average (three
// stacked box filters), which keeps out-of-band content from folding back
// into the spectrum. The effective sample period is period_ns * decim.
void fft_spectrum_decim(const uint8_t *data, int size, int offset, int decim,
    float *mag);

// Largest decimation a record of `size` samples supports (whole-record
// transform): the value that gives the best frequency resolution.
int fft_max_decimation(int size);

// Dominant-peak frequency in Hz from a magnitude spectrum, refined with
// parabolic interpolation between bins. sample_period_ns is the period of
// the samples fed to fft_spectrum. Returns 0 when there is no clear peak.
int fft_peak_frequency(const float *mag, int sample_period_ns);

// Full spectrum breakdown: the strongest peaks, and the fundamental whose
// harmonic comb explains them. A mains record with 50/100/150/250 Hz peaks
// reports fundamental = 50 Hz even when the 150 Hz harmonic is the tallest
// one and no crossing counter could ever agree on a frequency.
// sample_period_ns is the period of the samples fed to fft_spectrum_decim,
// i.e. the raw record period multiplied by the decimation.
void fft_analyze(const float *mag, int sample_period_ns, FftAnalysis *out);

#endif // _FFT_H_
