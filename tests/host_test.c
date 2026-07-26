/*
 * Host-side tests for measure.c and fft.c: generate synthetic waveforms with
 * known parameters and check the measured values against ground truth.
 *
 * Build & run (no hardware needed), from the repository root:
 *   cc -O2 -I. tests/host_test.c measure.c fft.c classify.c uart_decode.c \
 *      logic_decode.c nec_decode.c onewire_decode.c ws2812_decode.c -lm \
 *      -o /tmp/scope_test && /tmp/scope_test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "measure.h"
#include "fft.h"
#include "classify.h"
#include "logic_decode.h"

#define ZERO_POINT 128

static int g_failures = 0;

static void check_near(const char *name, double got, double want, double tol_pct)
{
  double tol = fabs(want) * tol_pct / 100.0;

  if (tol < 1e-9)
    tol = 1e-9;

  if (fabs(got - want) <= tol)
  {
    printf("  PASS %-28s got %12.3f  want %12.3f\n", name, got, want);
  }
  else
  {
    printf("  FAIL %-28s got %12.3f  want %12.3f (tol %.1f%%)\n", name, got, want, tol_pct);
    g_failures++;
  }
}

// Synthesize into a ring buffer with an arbitrary start offset to exercise
// the wrap handling
static void synth(uint8_t *buf, int size, int offset,
    double (*fn)(double phase, void *arg), void *arg,
    double freq_hz, double period_ns, double amp, double dc)
{
  for (int i = 0; i < size; i++)
  {
    double t = i * period_ns * 1e-9;
    double phase = fmod(freq_hz * t, 1.0);
    double v = dc + amp * fn(phase, arg);
    int c = (int)lround(v);

    if (c < 0) c = 0;
    if (c > 255) c = 255;

    buf[(offset + i) % size] = (uint8_t)c;
  }
}

static double fn_sine(double ph, void *arg)     { (void)arg; return sin(2 * M_PI * ph); }
static double fn_square(double ph, void *arg)   { double d = *(double *)arg; return ph < d ? 1.0 : -1.0; }
static double fn_triangle(double ph, void *arg) { (void)arg; return ph < 0.5 ? (4 * ph - 1) : (3 - 4 * ph); }

int main(void)
{
  enum { SIZE = 24576 };
  static uint8_t buf[SIZE];
  Measure m;

  // --- 1 kHz sine, 1 us sample period, amplitude 100 counts, offset +10 ---
  // 24000 samples = exactly 24 periods, so the mean has no partial-period bias
  printf("sine 1 kHz amp=100 dc=+10 (offset 12345):\n");
  synth(buf, 24000, 12345 % 24000, fn_sine, NULL, 1000.0, 1000.0, 100.0, ZERO_POINT + 10);
  measure_run(buf, 24000, 12345 % 24000, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 1000, 0.5);
  check_near("vpp counts", m.vmax - m.vmin, 200, 2);
  check_near("mean c100", m.mean_c100, 1000, 10);
  // RMS of sine with DC: sqrt(dc^2 + a^2/2) = sqrt(100 + 5000) = 71.41
  check_near("rms c100", m.rms_c100, 7141, 2);
  check_near("duty x10", m.duty_x10, 500, 4);

  // --- 30% duty square, 12.34 kHz ---
  printf("square 12.34 kHz duty=30%% amp=80:\n");
  double duty = 0.30;
  synth(buf, SIZE, 777, fn_square, &duty, 12340.0, 1000.0, 80.0, ZERO_POINT);
  measure_run(buf, SIZE, 777, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 12340, 0.5);
  check_near("duty x10", m.duty_x10, 300, 4);
  check_near("rms c100", m.rms_c100, 8000, 2);

  // --- triangle 250 Hz: mean 0, rms = a/sqrt(3) ---
  printf("triangle 250 Hz amp=90:\n");
  synth(buf, SIZE, 0, fn_triangle, NULL, 250.0, 1000.0, 90.0, ZERO_POINT);
  measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 250, 0.5);
  check_near("rms c100", m.rms_c100, 90.0 / sqrt(3.0) * 100, 2.5);

  // --- noise robustness: sine + noise, hysteresis must prevent double counts ---
  printf("sine 5 kHz amp=40 + uniform noise +-6:\n");
  srand(1);
  synth(buf, SIZE, 100, fn_sine, NULL, 5000.0, 1000.0, 40.0, ZERO_POINT);
  for (int i = 0; i < SIZE; i++)
  {
    int v = buf[i] + (rand() % 13) - 6;
    buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
  }
  measure_run(buf, SIZE, 100, 1000, ZERO_POINT, &m);
  check_near("frequency", m.frequency, 5000, 1.0);

  // --- flat line: no frequency ---
  printf("flat line:\n");
  memset(buf, ZERO_POINT + 3, SIZE);
  measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
  check_near("frequency (none)", m.frequency, 0, 0);
  check_near("rms c100", m.rms_c100, 300, 1);

  // --- FFT: 1.9 MHz sine at 16 ns period (62.5 MS/s) ---
  printf("fft sine 1.9 MHz @ 16ns:\n");
  static float mag[FFT_BINS];
  synth(buf, SIZE, 4242, fn_sine, NULL, 1.9e6, 16.0, 90.0, ZERO_POINT);
  fft_spectrum(buf, SIZE, 4242, mag);
  check_near("peak freq", fft_peak_frequency(mag, 16), 1.9e6, 1.0);

  // --- FFT: two tones, stronger one wins ---
  printf("fft 3 MHz strong + 800 kHz weak @ 16ns:\n");
  for (int i = 0; i < SIZE; i++)
  {
    double t = i * 16e-9;
    double v = ZERO_POINT + 80 * sin(2 * M_PI * 3e6 * t) + 25 * sin(2 * M_PI * 8e5 * t);
    buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
  }
  fft_spectrum(buf, SIZE, 0, mag);
  check_near("peak freq", fft_peak_frequency(mag, 16), 3e6, 1.0);

  // --- FFT: flat input has no peak ---
  printf("fft flat:\n");
  memset(buf, ZERO_POINT, SIZE);
  fft_spectrum(buf, SIZE, 0, mag);
  check_near("peak freq (none)", fft_peak_frequency(mag, 16), 0, 0);

  // ====================== whole-record spectrum =========================
  // The scope's own case: a 24576-sample record at 4 us covers 98 ms, so a
  // whole-record transform (decimation 48) resolves 10 Hz and can tell
  // mains apart from its harmonics. Transforming 512 raw samples of the
  // same record covers 2 ms and resolves 500 Hz: everything below is one
  // smear and the reported frequency is whatever the smear peaks at.
  {
    enum { REC = 24576, PER = 4000 };
    FftAnalysis an;
    int decim = fft_max_decimation(REC);

    check_near("max decimation", decim, 48, 0);

    // --- mains composite: 50 + 100 + 150 + 250 Hz, third harmonic the
    // strongest, plus HF interference above the decimated Nyquist ---
    printf("spectrum: mains composite (150 Hz is the tallest peak):\n");
    srand(7);

    for (int i = 0; i < REC; i++)
    {
      double t = i * PER * 1e-9;
      double v = ZERO_POINT
          + 60 * sin(2 * M_PI * 50.0 * t)
          + 25 * sin(2 * M_PI * 100.0 * t + 0.7)
          + 70 * sin(2 * M_PI * 150.0 * t + 1.3)
          + 15 * sin(2 * M_PI * 250.0 * t)
          + 20 * sin(2 * M_PI * 20000.0 * t)   // interference, aliases to 832 Hz
          + (rand() % 9) - 4;                  // noise

      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
    }

    fft_spectrum_decim(buf, REC, 0, decim, mag);
    fft_analyze(mag, PER * decim, &an);

    check_near("bin resolution", an.bin_hz, 1e9 / ((double)REC * PER), 1);
    check_near("nyquist", an.nyquist_hz, 1e9 / (2.0 * PER * decim), 1);
    check_near("fundamental", an.fundamental, 50.0, 4);
    check_near("strongest peak is 150", an.peak[0].freq, 150.0, 4);
    check_near("... labelled h3", an.peak[0].harmonic, 3, 0);
    check_near("peaks found", an.count >= 4, 1, 0);

    // Every harmonic must be in the list, at its own level relative to 150
    for (int h = 1; h <= 5; h++)
    {
      static const double amp[6] = { 0, 60, 25, 70, 0, 15 };
      char name[32];
      int found = -1;

      if (amp[h] == 0)
        continue;

      for (int i = 0; i < an.count; i++)
      {
        if (an.peak[i].harmonic == h)
          found = i;
      }

      snprintf(name, sizeof(name), "h%d present", h);
      check_near(name, found >= 0, 1, 0);

      if (found >= 0)
      {
        // Compared as an amplitude ratio: a relative tolerance on a dB
        // value near zero is meaninglessly tight
        snprintf(name, sizeof(name), "h%d level", h);
        check_near(name, pow(10.0, an.peak[found].rel_db / 20.0), amp[h] / 70.0, 10);
      }
    }

    // The 20 kHz interference folds to 832 Hz; the decimation filter has to
    // push it far below the signal instead of letting it become a peak
    {
      double worst = -99.0;

      for (int i = 0; i < an.count; i++)
      {
        if (an.peak[i].harmonic == 0 && an.peak[i].rel_db > worst)
          worst = an.peak[i].rel_db;
      }

      printf("  (strongest non-harmonic peak: %.1f dB)\n", worst);
      check_near("alias suppressed", worst < -20.0, 1, 0);
    }

    // --- missing fundamental: only 300/450/600 Hz present -> 150 Hz ---
    printf("spectrum: missing fundamental (300+450+600):\n");

    for (int i = 0; i < REC; i++)
    {
      double t = i * PER * 1e-9;
      double v = ZERO_POINT
          + 50 * sin(2 * M_PI * 300.0 * t)
          + 40 * sin(2 * M_PI * 450.0 * t)
          + 30 * sin(2 * M_PI * 600.0 * t);

      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : (int)lround(v));
    }

    fft_spectrum_decim(buf, REC, 0, decim, mag);
    fft_analyze(mag, PER * decim, &an);
    check_near("fundamental", an.fundamental, 150.0, 4);
    check_near("300 Hz is h2", an.peak[0].harmonic, 2, 0);

    // --- a clean sine has no distortion and one peak ---
    printf("spectrum: clean 400 Hz sine:\n");
    synth(buf, REC, 3000, fn_sine, NULL, 400.0, PER, 90.0, ZERO_POINT);
    fft_spectrum_decim(buf, REC, 3000, decim, mag);
    fft_analyze(mag, PER * decim, &an);
    check_near("fundamental", an.fundamental, 400.0, 2);
    check_near("thd < 5%", an.thd_x10 >= 0 && an.thd_x10 < 50, 1, 0);

    // --- decim 1 must still agree with the plain transform ---
    printf("spectrum: decim 1 == fft_spectrum:\n");
    {
      static float mag2[FFT_BINS];
      double diff = 0;

      synth(buf, REC, 111, fn_sine, NULL, 1e5, 16.0, 90.0, ZERO_POINT);
      fft_spectrum(buf, REC, 111, mag);
      fft_spectrum_decim(buf, REC, 111, 1, mag2);

      for (int i = 0; i < FFT_BINS; i++)
        diff += fabs(mag[i] - mag2[i]);

      check_near("identical", diff, 0, 0);
    }
  }

  // ============================= classifier =============================
  {
    SignalClass sc;
    double d;

    printf("classify:\n");

    synth(buf, SIZE, 55, fn_sine, NULL, 2000.0, 1000.0, 90.0, ZERO_POINT);
    measure_run(buf, SIZE, 55, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("sine -> SINE", sc.type, SIG_SINE, 0);

    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square -> SQUARE", sc.type, SIG_SQUARE, 0);

    d = 0.15;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("pulse 15%% -> PULSE", sc.type, SIG_PULSE, 0);

    synth(buf, SIZE, 0, fn_triangle, NULL, 500.0, 1000.0, 90.0, ZERO_POINT);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("triangle -> TRIANGLE", sc.type, SIG_TRIANGLE, 0);

    // Sawtooth: slow ramp up, fast drop
    for (int i = 0; i < SIZE; i++)
    {
      double ph = fmod(500.0 * i * 1e-6, 1.0);
      buf[i] = (uint8_t)(ZERO_POINT - 90 + (int)(180.0 * ph));
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("sawtooth -> SAW", sc.type, SIG_SAW, 0);

    memset(buf, ZERO_POINT, SIZE);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("flat -> DC", sc.type, SIG_DC, 0);

    srand(7);
    for (int i = 0; i < SIZE; i++)
      buf[i] = (uint8_t)(ZERO_POINT - 40 + rand() % 80);
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("noise -> NOISE", sc.type, SIG_NOISE, 0);

    // Square with glitch spikes (full-swing needles): must stay SQUARE with
    // the right frequency — a single needle used to trip the noise gate
    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    srand(11);
    for (int g = 0; g < 6; g++)
    {
      int at = 500 + (rand() % (SIZE - 1000));
      for (int w = 0; w < 3; w++)
        buf[at + w] = (uint8_t)((buf[at + w] > ZERO_POINT) ? 5 : 250);
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square+spikes -> SQUARE", sc.type, SIG_SQUARE, 0);
    check_near("square+spikes freq", m.frequency, 3000, 1.0);

    // Square with noise: noise peaks inflate vmax/vmin but must not push
    // the RMS/amplitude ratio out of the square bucket
    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 3000.0, 1000.0, 80.0, ZERO_POINT);
    srand(12);
    for (int i = 0; i < SIZE; i++)
    {
      int v = buf[i] + (rand() % 25) - 12;
      buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
    classify_signal(&m, NULL, 0, 0, &sc);
    check_near("square+noise -> SQUARE", sc.type, SIG_SQUARE, 0);
    check_near("square+noise freq", m.frequency, 3000, 1.0);
    check_near("square+noise duty", m.duty_x10, 500, 5);

    // Short 1 MHz burst in a long quiet record ("a spike on a slow
    // timebase"): percentiles erase it, the spike-trimmed peaks must not —
    // this is what the auto-setup steers by
    for (int i = 0; i < SIZE; i++)
    {
      if (i < 400) // 16 periods, 1.6% of the record: under the 2% percentile
        buf[i] = (uint8_t)(((i % 25) < 12) ? 208 : 48);
      else
        buf[i] = 48;
    }
    measure_run(buf, SIZE, 0, 40, ZERO_POINT, &m);
    check_near("burst pk_hi sees it", m.pk_hi >= 190, 1, 0);
    check_near("burst pk_lo", m.pk_lo <= 60, 1, 0);
    check_near("burst pctl top blind", m.top < 100, 1, 0); // documents why pk exists
    check_near("burst freq", m.frequency, 1000000, 2.0);

    // Mains hum picked up by hand on a floating probe: a 50 Hz fundamental
    // buried in harmonics, HF junk and noise. This is the record the
    // auto-setup surveys (HS_5_ms => 24576 samples at 4.096 us = 100 ms),
    // and both the frequency AND its regularity flag have to survive.
    printf("dirty mains:\n");
    {
      unsigned seed = 12345;

      for (int i = 0; i < SIZE; i++)
      {
        double t = i * 4.096e-6;
        double v = ZERO_POINT
            + 60.0 * sin(2 * M_PI * 50.0 * t)          // the fundamental
            + 14.0 * sin(2 * M_PI * 150.0 * t + 1.1)   // 3rd harmonic
            +  7.0 * sin(2 * M_PI * 250.0 * t + 2.7)   // 5th harmonic
            +  5.0 * sin(2 * M_PI * 13000.0 * t);      // switching junk
        seed = seed * 1103515245u + 12345u;
        v += (double)((int)((seed >> 16) & 0xff) - 128) * 5.0 / 128.0;

        int c = (int)lround(v);
        buf[i] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c));
      }

      measure_run(buf, SIZE, 0, 4096, ZERO_POINT, &m);
      check_near("mains freq", m.frequency, 50, 4.0);
      check_near("mains periods >= 3", m.periods >= 3, 1, 0);
      check_near("mains regular >= 60%", m.period_good_pct >= 60, 1, 0);
      check_near("mains swing", m.pk_hi - m.pk_lo, 160, 25.0);

      // The SAME signal seen on a fast timebase (HS_10_us => 32 ns per
      // sample = 786 us of record, 4% of one period). It must NOT look like
      // a valid fast signal, or the auto-setup would chase the junk instead
      // of the hum. This is the case that used to send it into millivolts.
      for (int i = 0; i < SIZE; i++)
      {
        double t = i * 32e-9;
        double v = ZERO_POINT
            + 60.0 * sin(2 * M_PI * 50.0 * t + 0.3)
            + 14.0 * sin(2 * M_PI * 150.0 * t + 1.1)
            +  7.0 * sin(2 * M_PI * 250.0 * t + 2.7)
            +  5.0 * sin(2 * M_PI * 13000.0 * t);
        seed = seed * 1103515245u + 12345u;
        v += (double)((int)((seed >> 16) & 0xff) - 128) * 5.0 / 128.0;

        int c = (int)lround(v);
        buf[i] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c));
      }

      measure_run(buf, SIZE, 0, 32, ZERO_POINT, &m);
      printf("    (fragment reads f=%d Hz, %d periods, %d%% regular, swing %d)\n",
          m.frequency, m.periods, m.period_good_pct, m.pk_hi - m.pk_lo);
      check_near("fragment not a fast signal",
          !(m.frequency >= 50000 && m.periods >= 3 && m.period_good_pct >= 60), 1, 0);
    }

    // Real signals that do NOT repeat evenly: UART frames, bus traffic, an
    // SWD line. Their intervals are as irregular as noise, so the auto-setup
    // cannot use regularity to tell them apart — it uses level_pct, the
    // share of samples resting at a rail. This is the pair of records that
    // has to come out different, or auto-setup walks past logic signals.
    printf("digital vs noise (level_pct):\n");
    {
      unsigned seed = 999;
      int bit = 40;   // samples per bit
      int i = 0;

      // 8N1-ish traffic: idle high, frames of random data, random gaps
      memset(buf, 210, SIZE);

      while (i < SIZE - 12 * bit)
      {
        int byte = (int)((seed = seed * 1103515245u + 12345u) >> 16) & 0xff;
        int bits[10];

        bits[0] = 0;                      // start
        for (int b = 0; b < 8; b++)
          bits[1 + b] = (byte >> b) & 1;
        bits[9] = 1;                      // stop

        for (int b = 0; b < 10; b++)
        {
          for (int s = 0; s < bit; s++)
            buf[i++] = bits[b] ? 210 : 46;
        }

        // idle gap of 0..3 byte times
        int gap = (int)(((seed = seed * 1103515245u + 12345u) >> 16) % 4) * 10 * bit;

        for (int s = 0; s < gap && i < SIZE; s++)
          buf[i++] = 210;
      }

      measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
      printf("    (uart traffic: %d periods, %d%% regular, %d%% at level)\n",
          m.periods, m.period_good_pct, m.level_pct);
      check_near("traffic is digital", m.level_pct >= 65, 1, 0);
      check_near("traffic has a frequency", m.frequency > 0, 1, 0);

      // Same amplitude, no structure: this must stay rejected
      for (int k = 0; k < SIZE; k++)
      {
        int acc = 0;

        for (int j = 0; j < 4; j++) // sum of uniforms ~ gaussian
        {
          seed = seed * 1103515245u + 12345u;
          acc += (int)((seed >> 16) & 0xff);
        }

        int v = 128 + (acc / 4 - 128) * 82 / 100;
        buf[k] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      }

      measure_run(buf, SIZE, 0, 1000, ZERO_POINT, &m);
      printf("    (noise: %d periods, %d%% regular, %d%% at level)\n",
          m.periods, m.period_good_pct, m.level_pct);
      check_near("noise is not digital", m.level_pct < 65, 1, 0);
      check_near("noise is not regular", m.period_good_pct < 60, 1, 0);
    }

    // THD: clean sine through the real FFT should be low, square high
    static float mag2[FFT_BINS];
    // 2 MHz sine at 16 ns -> bin = 2e6 * 512 * 16e-9 = 16.4
    synth(buf, SIZE, 0, fn_sine, NULL, 2.0e6, 16.0, 90.0, ZERO_POINT);
    fft_spectrum(buf, SIZE, 0, mag2);
    measure_run(buf, SIZE, 0, 16, ZERO_POINT, &m);
    classify_signal(&m, mag2, FFT_BINS, 16, &sc);
    check_near("sine type w/fft", sc.type, SIG_SINE, 0);
    if (sc.thd_x10 < 0 || sc.thd_x10 > 50)
    {
      printf("  FAIL sine THD low               got %d want <50 (=5%%)\n", sc.thd_x10);
      g_failures++;
    }
    else
      printf("  PASS sine THD low               got %.1f%%\n", sc.thd_x10 / 10.0);

    d = 0.5;
    synth(buf, SIZE, 0, fn_square, &d, 2.0e6, 16.0, 80.0, ZERO_POINT);
    fft_spectrum(buf, SIZE, 0, mag2);
    measure_run(buf, SIZE, 0, 16, ZERO_POINT, &m);
    classify_signal(&m, mag2, FFT_BINS, 16, &sc);
    // ideal square THD (h2..h5) = sqrt(1/9 + 1/25) ~= 38%
    if (sc.thd_x10 < 250)
    {
      printf("  FAIL square THD high            got %d want >250 (=25%%)\n", sc.thd_x10);
      g_failures++;
    }
    else
      printf("  PASS square THD high            got %.1f%%\n", sc.thd_x10 / 10.0);
  }

  // ============================ UART decoder ============================
  {
    static LogicScratch scratch;
    LogicResult lr;
    const char *msg = "Hello!";

    printf("uart:\n");

    // Build 8N1 stream: idle-high, start low, LSB first. 115200 baud
    // sampled at 1 MS/s (8.68 samples/bit)
    for (int pass = 0; pass < 2; pass++)
    {
      double baud = (pass == 0) ? 115200.0 : 9600.0;
      double sps_per_bit = 1e6 / baud;
      int pos = (int)(20 * sps_per_bit); // leading idle before the first frame

      memset(buf, 200, SIZE); // idle high at 200 counts

      for (const char *c = msg; *c; c++)
      {
        int frame_bits[10];

        frame_bits[0] = 0; // start
        for (int b = 0; b < 8; b++)
          frame_bits[1 + b] = (*c >> b) & 1;
        frame_bits[9] = 1; // stop

        for (int b = 0; b < 10; b++)
        {
          int end = (int)((pos / sps_per_bit + 1) * sps_per_bit + 0.5);
          int level = frame_bits[b] ? 200 : 56;

          for (; pos < end && pos < SIZE; pos++)
            buf[pos] = (uint8_t)level;
        }

        // inter-frame idle gap of 2 bit times
        int gap_end = pos + (int)(2 * sps_per_bit);
        for (; pos < gap_end && pos < SIZE; pos++)
          buf[pos] = 200;
      }

      // add noise +-4 counts
      srand(3);
      for (int i = 0; i < SIZE; i++)
      {
        int v = buf[i] + (rand() % 9) - 4;
        buf[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
      }

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);

      printf(" baud %d:\n", (int)baud);
      check_near("proto uart", lr.proto, PROTO_UART, 0);
      check_near("bytes decoded", n, (int)strlen(msg), 0);
      check_near("baud detected", lr.rate, baud, 0);
      check_near("positions ascend", lr.pos[n-1] > lr.pos[0], 1, 0);

      bool match = (n == (int)strlen(msg));
      for (int i = 0; i < n && match; i++)
        match = (lr.bytes[i] == (uint8_t)msg[i]);

      if (match)
        printf("  PASS payload                    \"%.*s\"\n", n, (char *)lr.bytes);
      else
      {
        printf("  FAIL payload mismatch: got \"%.*s\"\n", n, (char *)lr.bytes);
        g_failures++;
      }
    }

    // ----- 1-Wire: reset + presence + 0xCC 0x44, 1 us sample period -----
    printf("1-wire:\n");
    {
      int pos = 0;
      memset(buf, 200, SIZE);

      #define EMIT(level, us) do { \
        int e = pos + (us); \
        for (; pos < e && pos < SIZE; pos++) buf[pos] = (level) ? 200 : 56; \
      } while (0)

      EMIT(1, 300);
      EMIT(0, 480);  /* reset */
      EMIT(1, 70);
      EMIT(0, 100);  /* presence */
      EMIT(1, 300);

      uint8_t ow_bytes[2] = { 0xCC, 0x44 };
      for (int by = 0; by < 2; by++)
      {
        for (int b = 0; b < 8; b++)
        {
          int bit = (ow_bytes[by] >> b) & 1;
          EMIT(0, bit ? 6 : 60);
          EMIT(1, bit ? 64 : 10);
        }
      }
      #undef EMIT

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      check_near("proto 1-wire", lr.proto, PROTO_ONEWIRE, 0);
      check_near("bytes", n, 2, 0);
      check_near("byte0 = CC", lr.bytes[0], 0xCC, 0);
      check_near("byte1 = 44", lr.bytes[1], 0x44, 0);
    }

    // ----- WS2812: 2 LEDs GRB, 40 ns sample period -----
    printf("ws2812:\n");
    {
      double t = 60000; /* reset gap first */
      memset(buf, 56, SIZE); /* idle low */

      #define EMITNS(level, ns) do { \
        double e = t + (ns); \
        for (; t < e; t += 40.0) { int p = (int)(t / 40.0); \
          if (p < SIZE) buf[p] = (level) ? 200 : 56; } \
      } while (0)

      uint8_t ws_bytes[6] = { 0x11, 0x22, 0x33, 0xAA, 0x55, 0xFF };
      for (int by = 0; by < 6; by++)
      {
        for (int b = 7; b >= 0; b--)
        {
          int bit = (ws_bytes[by] >> b) & 1;
          EMITNS(1, bit ? 800 : 400);
          EMITNS(0, bit ? 450 : 850);
        }
      }
      #undef EMITNS

      int n = logic_decode(buf, SIZE, 0, 40, PROTO_AUTO, &scratch, &lr);
      check_near("proto ws2812", lr.proto, PROTO_WS2812, 0);
      check_near("bytes", n, 6, 0);
      bool ok = (n == 6);
      for (int i = 0; i < n && ok; i++)
        ok = (lr.bytes[i] == ws_bytes[i]);
      check_near("payload match", ok, 1, 0);
    }

    // ----- NEC: addr 0x04 cmd 0x08, 10 us sample period -----
    printf("nec:\n");
    {
      int pos = 0;
      memset(buf, 200, SIZE);

      #define EMITUS(level, us) do { \
        int e = pos + (us) / 10; \
        for (; pos < e && pos < SIZE; pos++) buf[pos] = (level) ? 200 : 56; \
      } while (0)

      EMITUS(1, 5000);
      EMITUS(0, 9000);   /* lead mark */
      EMITUS(1, 4500);   /* lead space */

      uint32_t word = 0x04 | (0xFBu << 8) | (0x08u << 16) | (0xF7u << 24);
      for (int b = 0; b < 32; b++)
      {
        EMITUS(0, 560);
        EMITUS(1, (word >> b) & 1 ? 1690 : 560);
      }
      EMITUS(0, 560);    /* stop mark */
      #undef EMITUS

      int n = logic_decode(buf, SIZE, 0, 10000, PROTO_AUTO, &scratch, &lr);
      check_near("proto nec", lr.proto, PROTO_NEC, 0);
      check_near("bytes", n, 4, 0);
      check_near("addr", lr.bytes[0], 0x04, 0);
      check_near("cmd", lr.bytes[2], 0x08, 0);
      check_near("no errors", lr.errors, 0, 0);
    }

    // ----- RAW fallback: 100 kHz square looks like a clock -----
    printf("raw fallback:\n");
    {
      double dd = 0.5;
      synth(buf, SIZE, 0, fn_square, &dd, 100000.0, 1000.0, 80.0, ZERO_POINT);

      int n = logic_decode(buf, SIZE, 0, 1000, PROTO_AUTO, &scratch, &lr);
      check_near("proto raw", lr.proto, PROTO_RAW, 0);
      check_near("bytes > 0", n > 0, 1, 0);
      check_near("pattern AA/55", lr.bytes[0] == 0xAA || lr.bytes[0] == 0x55, 1, 0);
    }

    // ----- structured decoders must all reject a sine -----
    synth(buf, SIZE, 0, fn_sine, NULL, 5000.0, 1000.0, 90.0, ZERO_POINT);
    check_near("sine not uart", uart_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    check_near("sine not 1-wire", onewire_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    check_near("sine not ws2812", ws2812_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
    check_near("sine not nec", nec_decode(buf, SIZE, 0, 1000, &scratch, &lr), 0, 0);
  }

  printf("\n%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
  return g_failures ? 1 : 0;
}
