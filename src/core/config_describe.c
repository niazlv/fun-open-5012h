/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
 * Copyright (c) 2026, Niaz Leushkin <niazlv03@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "utils.h"
#include "common.h"
#include "config.h"

/*
 * Field descriptions, for rendering any Config - the live one or an entry
 * still on the flash - as readable lines.
 *
 * offsetof again rather than a switch over field names, for the same reason
 * the lazy table uses it: this list sits next to a struct whose layout is
 * pinned and edited often, and a description that has quietly drifted off its
 * field is worse than no description at all. Renaming a field breaks the
 * build here; moving one is followed automatically.
 */
typedef enum
{
  FT_INT,      // signed decimal
  FT_BOOL,
  FT_HEX,      // uint32_t, as hex - magic and the checksums
  FT_I64,
  FT_IARR,     // int[count], one line per element
  FT_S16A,     // int16_t[count], same, for a field that had to fit a word
  FT_MAP,      // int[count] of which only the non-zero entries are interesting
  FT_WARR,     // PanelWidget[count]: metric and where it was put
} FieldType;

#define FIELD(f, t)     { #f, offsetof(Config, f), t, 1 }
#define FIELD_N(f, t, n) { #f, offsetof(Config, f), t, n }

static const struct
{
  const char *name;
  uint16_t offset;
  uint8_t type;
  uint8_t count;
} g_fields[] =
{
  FIELD(magic, FT_HEX),
  FIELD(size, FT_INT),
  FIELD(version, FT_INT),
  FIELD(count, FT_INT),
  FIELD(power_cycles, FT_INT),
  FIELD(charge_cycles, FT_INT),

  FIELD(lcd_bl_level, FT_INT),
  FIELD(lcd_dim_timeout, FT_INT),

  FIELD(ac_coupling, FT_BOOL),
  FIELD(x10, FT_BOOL),

  FIELD(trigger_mode, FT_INT),
  FIELD(trigger_edge, FT_INT),
  FIELD(trigger_level, FT_INT),
  FIELD(trigger_level_mv, FT_INT),

  FIELD(horizontal_scale, FT_INT),
  FIELD(horizontal_position, FT_I64),
  FIELD(horizontal_position_px, FT_INT),
  FIELD(horizontal_period, FT_INT),

  FIELD(vertical_scale, FT_INT),
  FIELD(vertical_position, FT_INT),
  FIELD(vertical_position_mv, FT_INT),
  FIELD(vertical_mult, FT_INT),

  FIELD(sample_rate_limit, FT_INT),

  FIELD(measure_display, FT_BOOL),
  FIELD(measure_panel_mode, FT_INT),
  FIELD(measure_panel_font, FT_INT),
  FIELD(measure_panel_bg, FT_INT),
  FIELD(measure_layout_mode, FT_INT),
  FIELD(probe_ratio, FT_INT),
  FIELD_N(measure_widget, FT_WARR, PANEL_WIDGETS_MAX),
  FIELD(show_vpp, FT_BOOL),
  FIELD(show_freq, FT_BOOL),
  FIELD(show_duty, FT_BOOL),
  FIELD(show_vrms, FT_BOOL),
  FIELD(show_vavg, FT_BOOL),
  FIELD(show_type, FT_BOOL),
  FIELD(show_thd, FT_BOOL),
  FIELD(show_jitter, FT_BOOL),
  FIELD(show_fft_freq, FT_BOOL),
  FIELD(show_alias, FT_BOOL),
  FIELD(show_vp, FT_BOOL),
  FIELD(show_vmax, FT_BOOL),
  FIELD(show_vmin, FT_BOOL),
  FIELD(show_vamp, FT_BOOL),
  FIELD(show_period, FT_BOOL),
  FIELD(show_width_pos, FT_BOOL),
  FIELD(show_width_neg, FT_BOOL),
  FIELD(measure_line_set, FT_BOOL),
  FIELD_N(measure_line, FT_IARR, MEASURE_LINE_SLOTS),

  FIELD(shift_mode_enabled, FT_BOOL),
  FIELD(shift_hold_lock, FT_BOOL),
  FIELD(key_remapping_enabled, FT_BOOL),
  FIELD_N(key_mapping, FT_MAP, 32),

  FIELD(decoder_proto, FT_INT),
  FIELD(decoder_stop, FT_BOOL),
  FIELD(decoder_stop_start, FT_BOOL),
  FIELD(decoder_baud, FT_INT),
  FIELD(decoder_fit_mode, FT_INT),
  FIELD(decoder_bits_mode, FT_INT),
  FIELD(spi_clock_hz, FT_INT),
  FIELD(spi_order, FT_INT),
  FIELD(man_rate, FT_INT),
  FIELD(man_polarity, FT_INT),
  FIELD(swd_clock_hz, FT_INT),

  FIELD(persist_mode, FT_INT),
  FIELD(average_mode, FT_INT),
  FIELD(draw_mode, FT_INT),
  FIELD(grid_mode, FT_INT),
  FIELD(vpp_dc_off, FT_BOOL),
  FIELD(roll_from, FT_INT),
  FIELD(startup_app_mode, FT_INT),
  FIELD(ui_scale, FT_INT),

  FIELD(snake_high_score, FT_INT),
  FIELD(flappy_high_score, FT_INT),
  FIELD(g2048_high_score, FT_INT),
  FIELD(tetris_high_score, FT_INT),

  FIELD(calib_ref_mv, FT_INT),
  FIELD_N(calib_nl2, FT_S16A, 2),
  FIELD(calib_crc, FT_HEX),
  FIELD(calib_channel_delta, FT_INT),
  FIELD(calib_dac_zero, FT_INT),
  FIELD_N(calib_dac_mult, FT_IARR, VS_COUNT),
  FIELD_N(calib_vs_mult, FT_IARR, VS_COUNT),

  FIELD(crc, FT_HEX),
};

//-----------------------------------------------------------------------------
// How many lines a descriptor renders: arrays get one per element, everything
// else gets one
static int field_lines(int i)
{
  if (FT_IARR == g_fields[i].type || FT_S16A == g_fields[i].type)
    return g_fields[i].count;

  // The widget array gets its count first and then one line per slot
  if (FT_WARR == g_fields[i].type)
    return g_fields[i].count + 1;

  return 1;
}

//-----------------------------------------------------------------------------
bool config_describe(const Config *cfg, int index, char *buf, int size)
{
  const uint8_t *base = (const uint8_t *)cfg;

  if (!cfg || index < 0)
    return false;

  for (int i = 0; i < ARRAY_SIZE(g_fields); i++)
  {
    int lines = field_lines(i);

    if (index >= lines)
    {
      index -= lines;
      continue;
    }

    const void *p = base + g_fields[i].offset;
    const char *name = g_fields[i].name;

    switch (g_fields[i].type)
    {
      case FT_BOOL:
        snprintf(buf, size, "%-24s%s", name, *(const bool *)p ? "true" : "false");
        break;

      case FT_HEX:
        snprintf(buf, size, "%-24s%08lX", name,
            (unsigned long)*(const uint32_t *)p);
        break;


      case FT_I64:
      {
        // No %lld in this newlib build's nano printf, and the value is a time
        // in ns that fits a long in every reachable case
        int64_t v = *(const int64_t *)p;
        snprintf(buf, size, "%-24s%ld", name, (long)v);
        break;
      }

      case FT_IARR:
      {
        char label[28];
        snprintf(label, sizeof(label), "%s[%d]", name, index);
        snprintf(buf, size, "%-24s%d", label, ((const int *)p)[index]);
        break;
      }

      case FT_S16A:
      {
        char label[28];
        snprintf(label, sizeof(label), "%s[%d]", name, index);
        snprintf(buf, size, "%-24s%d", label, ((const int16_t *)p)[index]);
        break;
      }

      case FT_WARR:
      {
        // One line per placed reading, and one for the count, because a layout
        // is a thing you debug by seeing where its pieces are
        const PanelWidget *w = (const PanelWidget *)p;
        int used = 0;

        for (int k = 0; k < g_fields[i].count; k++)
        {
          if (MEASURE_NONE != w[k].metric)
            used++;
        }

        if (0 == index)
        {
          snprintf(buf, size, "%-24s%d of %d placed", name, used,
              g_fields[i].count);
        }
        else
        {
          char label[28];

          snprintf(label, sizeof(label), "%s[%d]", name, index - 1);

          if (MEASURE_NONE == w[index - 1].metric)
            snprintf(buf, size, "%-24s-", label);
          else
            snprintf(buf, size, "%-24sm%d at %d,%d %s", label,
                w[index - 1].metric,
                w[index - 1].x * PANEL_WIDGET_STEP,
                w[index - 1].y * PANEL_WIDGET_STEP,
                (w[index - 1].flags & PW_LARGE) ? "large" : "small");
        }
        break;
      }

      case FT_MAP:
      {
        // 32 entries of which all but a few are zero: the count is the
        // information, and the whole array would bury the rest of the dump
        const uint32_t *map = (const uint32_t *)p;
        int used = 0;

        for (int k = 0; k < g_fields[i].count; k++)
        {
          if (0 != map[k])
            used++;
        }

        snprintf(buf, size, "%-24s%d of %d set", name, used,
            g_fields[i].count);
        break;
      }

      default:
        snprintf(buf, size, "%-24s%d", name, *(const int *)p);
        break;
    }

    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
// The calibration as text, a line at a time, for the page whose whole purpose
// is that these numbers can be copied onto paper. Flash is a cache for them:
// the sector they live in is the one the store wears out, it is erased by a
// firmware that changes sizeof(Config), and nothing in here survives the
// silicon actually failing. Eighteen numbers written down do.
bool config_get_calib_line(int index, char *buf, int size)
{
  static const char *const scale_names[VS_COUNT] =
  {
    "50mV", "100mV", "200mV", "500mV", "1V", "2V", "5V", "10V",
  };

  switch (index)
  {
    case 0:
      snprintf(buf, size, "zero %d   delta %d   ref %d mV",
          config.calib_dac_zero, config.calib_channel_delta,
          config.calib_ref_mv);
      return true;

    case 1:
      snprintf(buf, size, "%-16s%-10s%s", "range", "offset(O)", "gain(S)");
      return true;

    default:
      break;
  }

  index -= 2;

  if (index < 0 || index >= VS_COUNT)
    return false;

  // A range added to VS_COUNT without a name here leaves a NULL in the array,
  // and the number still matters more than the label it is missing
  snprintf(buf, size, "%-16s%-10d%d",
      scale_names[index] ? scale_names[index] : "?",
      config.calib_dac_mult[index], config.calib_vs_mult[index]);

  return true;
}
