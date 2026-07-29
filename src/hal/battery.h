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

#ifndef _BATTERY_H_
#define _BATTERY_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>

/*- Types -------------------------------------------------------------------*/
// What the charger pin and the cell voltage together say. FULL is "the TP4056
// has released CHRG and the cell is still above the float threshold", which is
// a charger that has finished as well as a pack just taken off one - the pin
// cannot tell those apart and neither can this.
typedef enum
{
  BATTERY_DISCHARGING,
  BATTERY_CHARGING,
  BATTERY_FULL,
} battery_state_t;

/*- Prototypes --------------------------------------------------------------*/
void battery_init(void);

// Cell voltage in mV, median-filtered and smoothed - the number the icon and
// every readout are derived from, not a raw conversion
int battery_voltage(void);

// State of charge, 0..100, as displayed: off the same curve as the icon and
// through the same hysteresis, so no two places on screen disagree
int battery_percent(void);

battery_state_t battery_state(void);

// may_draw says whether the top right corner is the caller's to paint in right
// now: false while a modal owns the display, and the icon then keeps its
// reading up to date without putting it on screen. The gauge does not know
// what is on the display and is not the right place to work it out - see
// lcd_backlight_task(), which is told the same kind of thing for the same
// reason.
void battery_task(bool may_draw);

void battery_redraw(void);

// One line for the System Information page
void battery_get_state(char *buf, int size);

// The Battery page, line by line; returns false once past the last one
bool battery_get_line(int index, char *buf, int size);

void battery_low_handler(void);

#endif // _BATTERY_H_
