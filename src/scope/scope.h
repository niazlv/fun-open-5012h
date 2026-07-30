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

#ifndef _SCOPE_H_
#define _SCOPE_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// The scope's section of the system menu and its pages in the menu's Help
// section, see scope_menu.c
extern const menu_def_t scope_menu;
extern const menu_def_t scope_help_menu;

// Calibration mode: the scope screen turns into the calibration UI (raw ADC
// readout plus one adjustable parameter). The menu toggles this directly and
// then calls scope_calibration_changed().
extern bool scope_calibration_mode;

// UART decoder rate table. config.decoder_baud indexes both arrays; index 0
// is "Auto" and a zero rate, which is what the decoder reads as "work it out
// from the record". Labels and rates live together so a menu entry cannot
// drift away from the rate it names.
#define DECODER_BAUD_COUNT  9
extern const char *const decoder_baud_labels[DECODER_BAUD_COUNT];
extern const int decoder_baud_values[DECODER_BAUD_COUNT];

// Manchester carries no rate of its own that a record can always recover, so
// the ones worth naming are listed. Index 0 is auto.
#define DECODER_MAN_RATE_COUNT  10
extern const char *const decoder_man_rate_labels[DECODER_MAN_RATE_COUNT];
extern const int decoder_man_rate_values[DECODER_MAN_RATE_COUNT];

/*- Prototypes --------------------------------------------------------------*/
void scope_init(bool calibration_mode);
void scope_buttons_handler(int buttons);
void scope_task(void);
void scope_redraw_all(void);

// Draw-free trigger helpers for the system menu (the scope screen is
// repainted in full when the menu closes)
void scope_apply_trigger_level(void);
void scope_trigger_50_percent(void);
void scope_calibration_changed(void);
// Find Z, D and O by measurement, with nothing connected to the input. Gain
// (S) needs a reference amplitude and is not part of it.
void scope_autocal_start(void);

// Numeric calibration entry from the menu: change config.calib_* directly,
// then call scope_calib_apply(). touch_dac only for the two parameters that
// feed the offset DAC (zero and DAC step) - it restarts acquisition.
void scope_calib_apply(bool touch_dac);
// Reference level the gain step aims at, normalized (a config saved before
// the field existed reads 0)
int scope_calib_ref_mv(void);
void scope_set_vertical_scale(int scale);
int scope_get_fps(void);

// Point the scope at the head of a message: decoder view on, trigger on a
// start bit and at the left of the screen, window from the rate, and every
// record decoded until one arrives that caught the line idle before its
// first frame. Freezes on that one. Takes effect on the scope's next tick,
// so it is safe to call from a menu.
void scope_decode_catch_start(void);
void scope_decode_redraw(void);
void scope_spi_clock_capture(void);
void scope_swd_clock_capture(void);

// Forget the accumulated persistence envelope and averaging state: the menu
// calls it when it toggles either feature; the scope itself calls it on
// every pan/zoom/scale change
void scope_display_settings_changed(void);

// The measurements panel's font or background changed. The cells hold text
// composed to the width of the font that was current when they were built, so
// they have to be dropped rather than re-measured a tick later at the wrong
// width - and the band's height moves with the font, which the trace area
// underneath has to be told about.
void scope_measure_panel_changed(void);

// Live state of the measurements panel, for the System Info page: this is
// the path that has gone blank after an auto-setup, and the flags below say
// which stage of it stopped (see scope_get_panel_state)
void scope_get_panel_state(char *buf, int size);

#endif // _SCOPE_H_

