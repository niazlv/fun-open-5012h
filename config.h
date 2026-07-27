/*
 * Copyright (c) 2019-2020, Alex Taradov <alex@taradov.com>
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

/*- Definitions -------------------------------------------------------------*/
#define CALIB_MULTIPLIER   1024

/*- Types -------------------------------------------------------------------*/
// The measurable quantities, by name. The panel takes a set of them (the
// show_* flags), the status line takes one per slot by number - the line has
// room for exactly two values in the large font, so which two is a choice the
// user makes, not something to rotate through.
enum
{
  MEASURE_NONE = 0,   // status-line slot only: leave it to the trigger readout
  MEASURE_VPP,
  MEASURE_FREQ,
  MEASURE_DUTY,
  MEASURE_VRMS,
  MEASURE_VAVG,
  MEASURE_TYPE,
  MEASURE_THD,
  MEASURE_COUNT,
};

#define MEASURE_LINE_SLOTS 2

// Health of the settings store, reported by config_get_state(). FRESH on any
// boot other than the very first one means the previous session's settings
// did not survive - the symptom this used to produce with no way to see it.
typedef enum
{
  CONFIG_STORE_FRESH,        // nothing usable in flash, defaults in use
  CONFIG_STORE_LOADED,       // restored at boot, nothing saved since
  CONFIG_STORE_OK,           // last save was written and read back
  CONFIG_STORE_ERASE_ERROR,
  CONFIG_STORE_WRITE_ERROR,
  CONFIG_STORE_VERIFY_ERROR, // written, but it did not read back
} ConfigStoreStatus;

typedef struct
{
  uint32_t magic;
  int      size;
  int      version;
  int      count;

  int      power_cycles;
  int      charge_cycles;

  int      lcd_bl_level;

  bool     ac_coupling;
  bool     x10;

  int      trigger_mode;
  int      trigger_edge;
  int      trigger_level;
  int      trigger_level_mv;

  int      horizontal_scale;
  int64_t  horizontal_position;
  int      horizontal_position_px;
  int      horizontal_period;

  int      vertical_scale;
  int      vertical_position;
  int      vertical_position_mv;
  int      vertical_mult;

  int      sample_rate_limit;

  bool     measure_display;

  // General settings
  int      screen_brightness;
  bool     shift_mode_enabled;
  bool     shift_mode_active;
  
  // Key remapping (store original button codes)
  uint32_t key_mapping[32];  // Map for button remapping
  bool     key_remapping_enabled;

  // Measurements. Two independent places, because they are good at different
  // things: the translucent panel over the trace lists a whole set of metrics
  // in a small font, the status line shows two of them in the large one and
  // is readable from across a bench.
  //
  // measure_panel_mode: 0 = panel on (default), 1 = off. Zero-is-default is
  // deliberate - configs saved before this field existed read as 0 and get the
  // panel. The show_* flags pick which metrics the panel lists; all-false
  // means the default set. Carved out of padding so sizeof(Config) and stored
  // configs (including calibration) stay valid.
  int      measure_panel_mode;
  bool     show_vpp;
  bool     show_freq;
  bool     show_duty;
  bool     show_vrms;
  bool     show_vavg;
  bool     show_type;
  bool     show_thd;

  // False in every config saved before the slots existed, which is how the
  // one-time migration in config_init tells "the user asked for an empty
  // status line" from "this config predates the question". Bumping VERSION
  // instead would have invalidated every stored entry, calibration included.
  bool     measure_line_set;

  // What the status line's two slots show, by metric number. MEASURE_NONE
  // leaves a slot to the trigger edge / level / position readouts, and two
  // of them give the stock status line back.
  int      measure_line[MEASURE_LINE_SLOTS];

  // Logic decoder: forced protocol (proto_t; 0 = auto) and stop-on-decode
  int      decoder_proto;
  bool     decoder_stop;
  bool     decoder_reserved[3];

  // Reference level, in mV, that the gain (S) calibration aims at: whatever
  // you can measure accurately and apply to the input. Carved out of padding
  // so sizeof(Config) and every stored calibration stay valid; a config saved
  // before this existed reads 0, which the gain step treats as "not set yet".
  int      calib_ref_mv;

  uint32_t padding[17];  // Reduced padding to accommodate new fields

  int      calib_channel_delta;
  int      calib_dac_zero;
  int      calib_dac_mult[VS_COUNT];
  int      calib_vs_mult[VS_COUNT];

  uint32_t crc;
} Config;

/*- Variables ---------------------------------------------------------------*/
extern Config config;

/*- Prototypes --------------------------------------------------------------*/
void config_init(void);
void config_reset(void);
void config_task(void);

// Saves now rather than at the next timer tick, and waits for the flash
void config_flush(void);

// One line of store health for the System Information page
void config_get_state(char *buf, int size);

#endif // _CONFIG_H_
