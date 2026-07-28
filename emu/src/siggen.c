/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * What is on the probe, and what the instrument does to it before the firmware
 * ever sees a number.
 *
 * Three stages, in the order the signal meets them:
 *
 *   1. sig_probe_uv()   the source itself. Ideal, apart from the finite edge
 *                       rate every real generator has.
 *
 *   2. the front end    probe, attenuator relays, amplifier and offset DAC.
 *                       This is where a square wave stops having corners:
 *                       finite bandwidth rounds every edge, and the amplifier
 *                       clips long before the converter does.
 *
 *   3. the AD9288       two 8-bit converters sharing one package. Quantisation,
 *                       code-to-code nonlinearity, aperture jitter, a noise
 *                       floor, and - because the two halves interleave - the
 *                       gain and offset mismatch between them.
 *
 * Every impairment is switchable from the command line, so the difference
 * between an ideal instrument and this one is something you can look at
 * rather than take on faith. Defaults are the datasheet's typical figures.
 *
 * The front end's gain per range is not a free choice: it is fixed by the
 * divider network the relays switch. No schematic here carries exact resistor
 * values, so the numbers come from the record of what those ranges measure -
 * the factory calibration constants in config.c. An emulated instrument that
 * is correctly calibrated is the right thing to start from; it does not hand
 * the firmware any answers, because everything past the byte this file
 * produces is the firmware's own arithmetic.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "emu.h"
#include "../../src/core/common.h"

/*- Definitions -------------------------------------------------------------*/
#define ZERO_COUNT   128       /* the converter's mid-scale                   */

/* Microvolts of input per ADC count, per attenuator range: the inverse of the
 * calibrated counts-to-millivolts factor, mV = (raw - 128) * mult / 1024. */
static const double g_uv_per_count[VS_COUNT] =
{
  4941 * 1000.0 / 1024,    /* the 50 mV menu range shares the 100 mV relay */
  4808 * 1000.0 / 1024,    /* 100 mV/div                                   */
  8343 * 1000.0 / 1024,    /* 200 mV/div                                   */
  20449 * 1000.0 / 1024,   /* 500 mV/div                                   */
  42447 * 1000.0 / 1024,   /* 1 V/div                                      */
  86232 * 1000.0 / 1024,   /* 2 V/div                                      */
  196726 * 1000.0 / 1024,  /* 5 V/div                                      */
  386029 * 1000.0 / 1024,  /* 10 V/div                                     */
};

/* Microvolts of input offset per DAC count: one screen pixel is worth
 * dac_mult/1024 DAC counts, and a pixel is that range's mV-per-pixel. */
static const double g_uv_per_dac[VS_COUNT] =
{
  1024.0 * 2 * 1000 / 2308,     1024.0 * 4 * 1000 / 4548,
  1024.0 * 8 * 1000 / 5277,     1024.0 * 20 * 1000 / 5384,
  1024.0 * 40 * 1000 / 5160,    1024.0 * 80 * 1000 / 5082,
  1024.0 * 200 * 1000 / 5542,   1024.0 * 400 * 1000 / 5630,
};

#define DAC_ZERO_CODE  2010    /* the code that puts 0 V at mid-screen      */

/* How far past the top of the ADC window the amplifier still behaves. Drive it
 * harder than this and the output flattens on the rail rather than following
 * the input - which is the difference between a trace that clips at the edge
 * of the screen and one that clips somewhere off it. */
#define AMP_HEADROOM_COUNTS  40.0

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  WAVE_DC, WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE, WAVE_SAW, WAVE_PULSE,
  WAVE_NOISE, WAVE_UART, WAVE_GLITCH, WAVE_AM, WAVE_BURST, WAVE_RC,
} WaveKind;

typedef struct
{
  WaveKind kind;
  double   freq_hz;
  double   amp_uv;         /* peak: half the peak-to-peak swing            */
  double   offset_uv;
  double   duty;           /* 0..1                                         */
  double   rise_ns;        /* the SOURCE's own 10-90% edge time            */
  double   noise_uv;       /* rms, at the probe tip                        */
  double   phase;
  double   baud;
  unsigned data;
  int      uart_bytes;     /* frames sent back to back per burst            */
  int      uart_idle;      /* idle bit times between bursts                 */
  uint8_t  uart_text[64];  /* text= payload; overrides data/bytes when set  */
  int      uart_text_len;
  double   glitch_every;
  double   glitch_ns;
  double   mod_hz;
  double   mod_depth;
  double   burst_cycles;
  double   burst_every;
} Waveform;

/* The instrument's own imperfections. Defaults are the AD9288 datasheet's
 * typical figures and the 5012H's rated 100 MHz analog bandwidth. */
typedef struct
{
  double bw_hz;            /* front-end -3 dB bandwidth                    */
  double overshoot;        /* fraction, from the amplifier's peaking       */
  double jitter_ps;        /* aperture jitter, rms                         */
  double dnl_lsb;          /* converter differential nonlinearity, rms     */
  double adc_noise_lsb;    /* thermal + quantisation noise floor, rms      */
  double il_gain;          /* converter A gain relative to B               */
  double il_offset;        /* and its offset, in counts                    */
  double probe_div;        /* 1 for a 1x probe, 10 for a 10x               */
  bool   ideal;            /* switch every impairment above off            */
} Frontend;

/*- Variables ---------------------------------------------------------------*/
static Waveform g_wave =
{
  .kind = WAVE_SINE, .freq_hz = 1000, .amp_uv = 500000, .offset_uv = 0,
  .duty = 0.5, .rise_ns = 0, .noise_uv = 0, .phase = 0,
  .baud = 115200, .data = 0x41, .uart_bytes = 4, .uart_idle = 14,
  .glitch_every = 16, .glitch_ns = 40,
  .mod_hz = 100, .mod_depth = 0.5, .burst_cycles = 8, .burst_every = 64,
};

static Frontend g_afe_model =
{
  .bw_hz = 100e6,          /* the instrument is a 100 MHz scope            */
  .overshoot = 0.04,       /* a few percent, typical of a fast amplifier   */
  .jitter_ps = 2.0,        /* AD9288: 2 ps rms aperture jitter             */
  .dnl_lsb = 0.35,         /* AD9288: +-0.35 LSB typical DNL               */
  .adc_noise_lsb = 0.35,   /* what an ENOB of ~7.2 bits amounts to         */
  .il_gain = 1.008,        /* the two converters are not identical         */
  .il_offset = 5.0,        /* nor are their offsets - hence calib_channel_delta */
  .probe_div = 1.0,
  .ideal = false,
};

static char g_desc[256];
static char g_afe_desc[256];

/* Per-code converter error. Real DNL is fixed silicon, not noise: the same
 * code is always wrong by the same amount, which is why it shows up in a
 * spectrum as harmonics rather than as a raised floor. */
static double g_code_error[256];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Deterministic hash-based noise: the same instant always yields the same
// value, so a capture is reproducible and screenshots do not shimmer.
static double hash_gauss(uint64_t a, uint64_t b)
{
  uint64_t x = a * 0x9E3779B97F4A7C15ull + b * 0x632BE59BD9B4E019ull;
  double u1, u2;

  x ^= x >> 30; x *= 0xBF58476D1CE4E5B9ull;
  x ^= x >> 27; x *= 0x94D049BB133111EBull;
  x ^= x >> 31;

  u1 = (double)(uint32_t)x / 4294967296.0;
  u2 = (double)(uint32_t)(x >> 32) / 4294967296.0;

  return (u1 + u2 - 1.0) * 1.732;   // unit variance, near enough
}

//-----------------------------------------------------------------------------
static void build_code_errors(void)
{
  double acc = 0;

  // Integral nonlinearity is the running sum of the differential kind, so
  // build it that way and take out the average slope, which is just gain
  for (int i = 0; i < 256; i++)
  {
    acc += g_afe_model.dnl_lsb * hash_gauss((uint64_t)i, 0xC0DEC0DEull);
    g_code_error[i] = acc;
  }

  {
    double first = g_code_error[0], last = g_code_error[255];

    for (int i = 0; i < 256; i++)
      g_code_error[i] -= first + (last - first) * i / 255.0;
  }
}

/*- The source --------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// One transition, with a finite edge rate and the slight overshoot a real
// amplifier leaves behind it. Returns 0 before the edge and 1 well after.
static double edge(double t_ns, double rise_ns, double overshoot)
{
  double y;

  if (rise_ns <= 0)
    return t_ns >= 0 ? 1.0 : 0.0;

  if (t_ns <= 0)
    return 0.0;

  if (t_ns < rise_ns)
    y = 0.5 - 0.5 * cos(M_PI * t_ns / rise_ns);
  else
    y = 1.0;

  // Ringing after the edge: a damped oscillation at roughly the bandwidth
  // limit, which is what gives a fast pulse its overshoot and settling tail
  if (overshoot > 0 && t_ns > 0)
  {
    double tau = rise_ns * 1.5;
    y += overshoot * exp(-t_ns / tau) * sin(M_PI * t_ns / rise_ns);
  }

  return y;
}

//-----------------------------------------------------------------------------
// The effective edge time at the converter: the source's own rise time and the
// front end's, combined the way a scope's manual says to combine them.
// t_total = sqrt(t_source^2 + t_frontend^2), with t_frontend = 0.35 / BW.
//
// This is what "the edges are not square any more" means quantitatively, and
// doing it here - analytically, on the ideal waveform - keeps the result
// independent of how the emulator happens to schedule its samples.
static double effective_rise_ns(void)
{
  double src = g_wave.rise_ns;
  double fe;

  if (g_afe_model.ideal || g_afe_model.bw_hz <= 0)
    return src;

  fe = 0.35 / g_afe_model.bw_hz * 1e9;

  return sqrt(src * src + fe * fe);
}

//-----------------------------------------------------------------------------
// A burst of back-to-back 8N1 frames, then a real gap before it repeats.
//
// The gap is not decoration. A UART line idles between messages, and the
// firmware's decoder uses exactly that to tell UART from any other continuous
// bitstream: it requires a run of at least a whole frame's worth of idle
// somewhere in the record (uart_decode.c) and refuses to guess otherwise. A
// generator that ran frames end to end with a couple of idle bits would be
// decoded as raw bits - correctly, because that is not what a UART looks like.
#define UART_FRAME_BITS  10       /* start + 8 data + stop */

static int uart_frames(const Waveform *w)
{
  return w->uart_text_len > 0 ? w->uart_text_len : w->uart_bytes;
}

//-----------------------------------------------------------------------------
static int uart_slots(const Waveform *w)
{
  return uart_frames(w) * UART_FRAME_BITS + w->uart_idle;
}

//-----------------------------------------------------------------------------
static unsigned uart_frame_byte(const Waveform *w, int frame)
{
  // text= payload verbatim (any bytes, UTF-8 included); otherwise a run of
  // consecutive values starting at data=, so a burst is not one byte repeated
  if (w->uart_text_len > 0)
    return w->uart_text[frame];

  return (w->data + (unsigned)frame) & 0xff;
}

//-----------------------------------------------------------------------------
static int uart_slot_level(const Waveform *w, int slot)
{
  int frame, pos;

  if (slot < 0)
    return 1;

  frame = slot / UART_FRAME_BITS;
  pos = slot % UART_FRAME_BITS;

  if (frame >= uart_frames(w))
    return 1;                    // the idle gap between bursts

  if (pos == 0)
    return 0;                    // start bit

  if (pos == 9)
    return 1;                    // stop bit

  return (int)((uart_frame_byte(w, frame) >> (pos - 1)) & 1);
}

//-----------------------------------------------------------------------------
static double shaped_square(double p, double period_ns, double duty,
    double rise_ns, double overshoot)
{
  double t = p * period_ns;
  double hi_end = duty * period_ns;

  // Idles low, then each edge swings it by the full 2. The previous period's
  // two edges are carried in as well: they have settled to a net zero by now,
  // but their ringing tails have not, and dropping them would make every
  // period start artificially clean.
  return -1.0 + 2.0 * (
        edge(t, rise_ns, overshoot)
      - edge(t - hi_end, rise_ns, overshoot)
      + edge(t + period_ns, rise_ns, overshoot)
      - edge(t + period_ns - hi_end, rise_ns, overshoot));
}

//-----------------------------------------------------------------------------
// The signal at the probe tip, in microvolts, at time t. The nanosecond
// argument is a double so that the converter's picosecond-scale aperture
// jitter is representable.
static double probe_uv_at(double t_ns)
{
  const Waveform *w = &g_wave;
  double period_ns = (w->freq_hz > 0) ? (1e9 / w->freq_hz) : 0;
  double rise = effective_rise_ns();
  double over = g_afe_model.ideal ? 0 : g_afe_model.overshoot;
  double p = 0, v = 0;
  double amp = w->amp_uv;

  if (period_ns > 0)
  {
    p = fmod(t_ns / period_ns + w->phase, 1.0);

    if (p < 0)
      p += 1.0;
  }

  // A sine does not have edges to round, so the bandwidth limit shows up as
  // plain amplitude roll-off: the single-pole response at the signal's own
  // frequency.
  if (!g_afe_model.ideal && g_afe_model.bw_hz > 0 &&
      (w->kind == WAVE_SINE || w->kind == WAVE_AM || w->kind == WAVE_BURST))
  {
    double r = w->freq_hz / g_afe_model.bw_hz;
    amp /= sqrt(1 + r * r);
  }

  switch (w->kind)
  {
    case WAVE_DC:
      v = 0;
      break;

    case WAVE_SINE:
      v = sin(2 * M_PI * p);
      break;

    case WAVE_SQUARE:
      v = shaped_square(p, period_ns, w->duty, rise, over);
      break;

    case WAVE_PULSE:
      v = (shaped_square(p, period_ns, w->duty, rise, over) + 1) / 2;
      break;

    case WAVE_TRIANGLE:
      v = (p < 0.5) ? (4 * p - 1) : (3 - 4 * p);
      break;

    case WAVE_SAW:
      // The flyback is an edge like any other, and it gets rounded like one
      v = 2 * p - 1 - 2 * edge(p * period_ns - period_ns * 0.999, rise, over);
      break;

    case WAVE_NOISE:
      v = hash_gauss((uint64_t)llround(t_ns * 8), 0x5EEDull);
      break;

    case WAVE_RC:
      // Charging and discharging through a resistor: the shape a probe
      // compensation trimmer is adjusted against
      v = (p < w->duty)
          ? (1 - exp(-5 * p / w->duty))
          : exp(-5 * (p - w->duty) / (1 - w->duty));
      v = 2 * v - 1;
      break;

    case WAVE_AM:
      v = sin(2 * M_PI * p) *
          (1 + w->mod_depth * sin(2 * M_PI * t_ns * w->mod_hz / 1e9));
      break;

    case WAVE_BURST:
    {
      double cycle = fmod(t_ns / period_ns, w->burst_every);

      v = (cycle < w->burst_cycles) ? sin(2 * M_PI * p) : 0;
      break;
    }

    case WAVE_GLITCH:
    {
      // A clean square wave that every so often grows one narrow runt pulse -
      // the signal the firmware's glitch finder exists to catch
      double cycles = t_ns / period_ns;
      double into = fmod(cycles, w->glitch_every);
      double t_in = p * period_ns;
      double g_at = 0.72 * period_ns;

      v = shaped_square(p, period_ns, w->duty, rise, over);

      if (into >= w->glitch_every - 1)
        v += 2 * (edge(t_in - g_at, rise, over) -
                  edge(t_in - g_at - w->glitch_ns, rise, over));

      if (v > 1) v = 1;
      break;
    }

    case WAVE_UART:
    {
      // 8N1 at the configured baud: one byte, then idle, repeating. Built out
      // of the same shaped edges as everything else, so the decoder sees real
      // transitions with real rise times rather than an idealised bit pattern.
      double bit_ns = 1e9 / w->baud;
      int slots = uart_slots(w);
      double pos = fmod(t_ns / bit_ns, (double)slots);
      double lvl = 1.0;   // the line idles high

      for (int i = 0; i < slots; i++)
      {
        int cur = uart_slot_level(w, i);
        int prev = uart_slot_level(w, i - 1);

        if (cur != prev)
          lvl += (cur - prev) * edge((pos - i) * bit_ns, rise, over);
      }

      v = 2 * lvl - 1;
      break;
    }
  }

  return v * amp + w->offset_uv;
}

//-----------------------------------------------------------------------------
int64_t sig_probe_uv(uint64_t t_ns)
{
  return (int64_t)llround(probe_uv_at((double)t_ns));
}

//-----------------------------------------------------------------------------
// The signal's own DC component, which is what the coupling capacitor removes.
// Taken analytically rather than by filtering: above the input capacitor's
// ~10 Hz corner the two agree, and this one cannot drift with scheduling.
static double wave_dc_uv(void)
{
  const Waveform *w = &g_wave;

  switch (w->kind)
  {
    case WAVE_SQUARE:
    case WAVE_GLITCH: return w->offset_uv + w->amp_uv * (2 * w->duty - 1);
    case WAVE_PULSE:  return w->offset_uv + w->amp_uv * w->duty;
    case WAVE_UART:
    {
      int slots = uart_slots(w);
      int high = 0;

      for (int i = 0; i < slots; i++)
        high += uart_slot_level(w, i);

      return w->offset_uv + w->amp_uv * (2.0 * high / slots - 1);
    }
    default:          return w->offset_uv;
  }
}

/*- The front end and the converter -----------------------------------------*/

//-----------------------------------------------------------------------------
uint8_t sig_adc_sample(uint64_t t_ns, const AfeState *afe, int converter)
{
  const Frontend *fe = &g_afe_model;
  int scale = afe->relay_scale;
  double t = (double)t_ns;
  double v_uv, offset_uv, counts;

  if (scale < 0 || scale >= VS_COUNT)
    scale = VS_200_mV;   // the relays' reset state

  // Aperture jitter: the sample is taken a little early or late, every time.
  // On a slow signal this is invisible; on a fast edge it is the dominant
  // noise term, which is exactly what the firmware's jitter measurement sees.
  if (!fe->ideal && fe->jitter_ps > 0)
    t += fe->jitter_ps * 1e-3 * hash_gauss(t_ns, 0x71773Full);

  v_uv = probe_uv_at(t);

  // A 10x probe divides before anything else, and the firmware multiplies the
  // reading back by ten - which is why the two have to be set to match
  if (fe->probe_div > 1)
    v_uv /= fe->probe_div;

  if (g_wave.noise_uv > 0)
    v_uv += g_wave.noise_uv * hash_gauss(t_ns, 0x0150Eull + (uint64_t)converter);

  if (afe->ac_coupling)
    v_uv -= wave_dc_uv() / (fe->probe_div > 1 ? fe->probe_div : 1);

  // The offset DAC moves the signal against the converter's window
  offset_uv = (double)(afe->dac_code - DAC_ZERO_CODE) * g_uv_per_dac[scale];

  counts = ZERO_COUNT + (v_uv - offset_uv) / g_uv_per_count[scale];

  if (!fe->ideal)
  {
    // The amplifier saturates before the converter does. Past its headroom the
    // output stops following the input and flattens onto the rail, so an
    // overdriven trace does not merely leave the screen - it goes flat.
    double lo = -AMP_HEADROOM_COUNTS, hi = 255 + AMP_HEADROOM_COUNTS;

    if (counts < lo) counts = lo;
    if (counts > hi) counts = hi;

    // Interleaving: the two halves of the AD9288 do not have identical gain or
    // offset, and their mismatch is what buffer.c's channel delta calibration
    // exists to null out
    if (converter == 1)
      counts = ZERO_COUNT + (counts - ZERO_COUNT) * fe->il_gain + fe->il_offset;

    if (fe->adc_noise_lsb > 0)
      counts += fe->adc_noise_lsb * hash_gauss(t_ns ^ 0xA5A5A5A5ull,
          (uint64_t)converter);
  }

  // Quantisation, and then the converter's own code errors on top: the same
  // code is always wrong by the same amount, so this shows up in a spectrum
  // as spurs rather than as a raised noise floor
  {
    int code = (int)lround(counts);

    if (code < 0)   return 0;
    if (code > 255) return 255;

    if (!fe->ideal)
    {
      code = (int)lround((double)code + g_code_error[code]);

      if (code < 0)   code = 0;
      if (code > 255) code = 255;
    }

    return (uint8_t)code;
  }
}

/*- Spec parsing ------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Engineering notation: 1k, 2.5M, 500m, 20n, 3.3
static double parse_value(const char *s)
{
  char *end;
  double v = strtod(s, &end);

  if (end == s)
    return 0;

  switch (*end)
  {
    case 'p': return v * 1e-12;
    case 'n': return v * 1e-9;
    case 'u': return v * 1e-6;
    case 'm': return v * 1e-3;
    case 'k': case 'K': return v * 1e3;
    case 'M': return v * 1e6;
    case 'G': return v * 1e9;
    default:  return v;
  }
}

//-----------------------------------------------------------------------------
static void describe(void)
{
  static const char *names[] =
  {
    "dc", "sine", "square", "triangle", "saw", "pulse",
    "noise", "uart", "glitch", "am", "burst", "rc",
  };

  snprintf(g_desc, sizeof(g_desc),
      "%s  %.4g Hz  %.4g Vpp  offset %.4g V  source noise %.3g Vrms",
      names[g_wave.kind], g_wave.freq_hz, g_wave.amp_uv * 2e-6,
      g_wave.offset_uv * 1e-6, g_wave.noise_uv * 1e-6);

  if (g_afe_model.ideal)
  {
    snprintf(g_afe_desc, sizeof(g_afe_desc),
        "front end: IDEAL (impairments off)");
  }
  else
  {
    snprintf(g_afe_desc, sizeof(g_afe_desc),
        "front end: %.4g MHz BW (edge %.2f ns), %.0f%% overshoot, "
        "%.1f ps jitter, %.2f LSB DNL, %.2f LSB noise, "
        "interleave %+.1f%%/%+.0f LSB, %gx probe",
        g_afe_model.bw_hz / 1e6, effective_rise_ns(),
        g_afe_model.overshoot * 100, g_afe_model.jitter_ps,
        g_afe_model.dnl_lsb, g_afe_model.adc_noise_lsb,
        (g_afe_model.il_gain - 1) * 100, g_afe_model.il_offset,
        g_afe_model.probe_div);
  }
}

//-----------------------------------------------------------------------------
bool sig_configure(const char *spec, char *err, size_t errsz)
{
  static const struct { const char *name; WaveKind kind; } kinds[] =
  {
    { "dc", WAVE_DC }, { "sine", WAVE_SINE }, { "square", WAVE_SQUARE },
    { "triangle", WAVE_TRIANGLE }, { "saw", WAVE_SAW }, { "pulse", WAVE_PULSE },
    { "noise", WAVE_NOISE }, { "uart", WAVE_UART }, { "glitch", WAVE_GLITCH },
    { "am", WAVE_AM }, { "burst", WAVE_BURST }, { "rc", WAVE_RC },
  };

  char buf[512];
  char *save = NULL, *tok;
  bool first = true;

  snprintf(buf, sizeof(buf), "%s", spec);

  // text= swallows the rest of the spec verbatim - a payload is arbitrary
  // bytes (spaces and UTF-8 included), so it cannot go through the tokenizer.
  // Put it last on the command line.
  {
    char *t = strstr(buf, "text=");

    if (t)
    {
      const char *payload = t + 5;
      size_t n = strlen(payload);

      if (n > sizeof(g_wave.uart_text))
        n = sizeof(g_wave.uart_text);

      memcpy(g_wave.uart_text, payload, n);
      g_wave.uart_text_len = (int)n;

      *t = 0; // the tokenizer sees only what came before
    }
  }

  for (tok = strtok_r(buf, " \t,", &save); tok; tok = strtok_r(NULL, " \t,", &save))
  {
    char *eq = strchr(tok, '=');
    const char *k;
    double v;

    if (first && !eq)
    {
      bool found = false;

      for (unsigned i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++)
      {
        if (0 == strcmp(tok, kinds[i].name))
        {
          g_wave.kind = kinds[i].kind;
          found = true;
        }
      }

      if (!found)
      {
        snprintf(err, errsz, "unknown waveform '%s'", tok);
        return false;
      }

      first = false;
      continue;
    }

    first = false;

    if (!eq)
    {
      snprintf(err, errsz, "expected key=value, got '%s'", tok);
      return false;
    }

    *eq = 0;
    k = tok;
    v = parse_value(eq + 1);

    if      (0 == strcmp(k, "f"))      g_wave.freq_hz = v;
    else if (0 == strcmp(k, "amp"))    g_wave.amp_uv = v * 1e6;
    else if (0 == strcmp(k, "vpp"))    g_wave.amp_uv = v * 1e6 / 2;
    else if (0 == strcmp(k, "ofs"))    g_wave.offset_uv = v * 1e6;
    else if (0 == strcmp(k, "duty"))   g_wave.duty = (v > 1) ? v / 100 : v;
    else if (0 == strcmp(k, "rise"))   g_wave.rise_ns = v * 1e9;
    else if (0 == strcmp(k, "noise"))  g_wave.noise_uv = v * 1e6;
    else if (0 == strcmp(k, "phase"))  g_wave.phase = v;
    else if (0 == strcmp(k, "baud"))   g_wave.baud = v;
    else if (0 == strcmp(k, "data"))
    {
      g_wave.data = (unsigned)strtoul(eq + 1, NULL, 0);
      g_wave.uart_text_len = 0;    // data= and text= are alternatives
    }
    else if (0 == strcmp(k, "bytes"))  g_wave.uart_bytes = (int)v;
    else if (0 == strcmp(k, "idle"))   g_wave.uart_idle = (int)v;
    else if (0 == strcmp(k, "every"))  g_wave.glitch_every = v;
    else if (0 == strcmp(k, "width"))  g_wave.glitch_ns = v * 1e9;
    else if (0 == strcmp(k, "modf"))   g_wave.mod_hz = v;
    else if (0 == strcmp(k, "depth"))  g_wave.mod_depth = v;
    else if (0 == strcmp(k, "cycles")) g_wave.burst_cycles = v;
    else if (0 == strcmp(k, "period")) g_wave.burst_every = v;
    else
    {
      snprintf(err, errsz, "unknown signal parameter '%s'", k);
      return false;
    }
  }

  if (g_wave.duty <= 0 || g_wave.duty >= 1)
    g_wave.duty = 0.5;

  if (g_wave.uart_bytes < 1)
    g_wave.uart_bytes = 1;

  // Below a full frame of idle the decoder cannot tell this from any other
  // continuous bitstream, and says so; keep the generator honest instead
  if (g_wave.uart_idle < UART_FRAME_BITS + 2)
    g_wave.uart_idle = UART_FRAME_BITS + 2;

  describe();

  return true;
}

//-----------------------------------------------------------------------------
bool sig_configure_afe(const char *spec, char *err, size_t errsz)
{
  char buf[512];
  char *save = NULL, *tok;

  snprintf(buf, sizeof(buf), "%s", spec);

  for (tok = strtok_r(buf, " \t,", &save); tok; tok = strtok_r(NULL, " \t,", &save))
  {
    char *eq = strchr(tok, '=');
    const char *k;
    double v;

    if (!eq)
    {
      if (0 == strcmp(tok, "ideal"))
      {
        g_afe_model.ideal = true;
        continue;
      }

      if (0 == strcmp(tok, "real"))
      {
        g_afe_model.ideal = false;
        continue;
      }

      snprintf(err, errsz, "expected key=value, got '%s'", tok);
      return false;
    }

    *eq = 0;
    k = tok;
    v = parse_value(eq + 1);

    if      (0 == strcmp(k, "bw"))        g_afe_model.bw_hz = v;
    else if (0 == strcmp(k, "overshoot")) g_afe_model.overshoot = (v > 1) ? v / 100 : v;
    else if (0 == strcmp(k, "jitter"))    g_afe_model.jitter_ps = v * 1e12;
    else if (0 == strcmp(k, "dnl"))       g_afe_model.dnl_lsb = v;
    else if (0 == strcmp(k, "adcnoise"))  g_afe_model.adc_noise_lsb = v;
    else if (0 == strcmp(k, "ilgain"))    g_afe_model.il_gain = 1 + ((v > 0.5) ? v / 100 : v);
    else if (0 == strcmp(k, "iloffset"))  g_afe_model.il_offset = v;
    else if (0 == strcmp(k, "probe"))     g_afe_model.probe_div = (v >= 10) ? 10 : 1;
    else
    {
      snprintf(err, errsz, "unknown front-end parameter '%s'", k);
      return false;
    }
  }

  build_code_errors();
  describe();

  return true;
}

//-----------------------------------------------------------------------------
const char *sig_describe(void)
{
  return g_desc;
}

//-----------------------------------------------------------------------------
const char *sig_describe_afe(void)
{
  return g_afe_desc;
}

//-----------------------------------------------------------------------------
void sig_init(void)
{
  build_code_errors();
  describe();
}
