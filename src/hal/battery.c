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

/*
 * Battery gauge.
 *
 * PB1 carries VBAT/2 into ADC0 channel 9 and PB15 carries the TP4056's CHRG
 * pin, which is open drain: pulled low while current is going into the cell
 * and released once it is not. Those two wires are the whole of the hardware,
 * so everything below is about turning them into a number that means
 * something.
 *
 * Three things had to change before the icon could be believed:
 *
 *   - a single conversion is not the cell voltage. The rail moves with the
 *     backlight, with a capture burst, with a game's SPI traffic; the icon was
 *     drawn straight off one reading and so it stepped up and down while
 *     nothing about the battery had changed. Readings now go through a median
 *     of the last 15 (7.5 s, which rejects a load step outright rather than
 *     averaging it in) and then a smoothing filter.
 *
 *   - eight thresholds between 3.5 V and 3.9 V are not a discharge curve. A
 *     Li-Ion cell leaves the charger at 4.2 V and spends most of its capacity
 *     between 4.1 V and 3.7 V, so a gauge that reads "full" at 3.9 V shows
 *     full for the first half of the run and then falls off a cliff. There is
 *     one curve now, in per cent, and the icon is a fill of that per cent.
 *
 *   - per cent still moves when the reading crosses a knee of the curve, so
 *     the displayed value is rate-limited to 1 % per update and only moves at
 *     all once it is 2 % away from the filtered one. That is what stops the
 *     last segment flickering while the number underneath dithers.
 *
 * None of this makes it a coulomb counter: it is a voltage lookup, it reads
 * low under load and high just off the charger, and the Battery page says so.
 * What it does do is stay put when the battery has not changed, which is the
 * property that makes a gauge readable.
 *
 * ...and then there is saying that current is going in at all. That was two
 * pixels of dark green blinking on the end of the fill, and the arithmetic
 * behind it is the whole problem: the blink was the gap between the level and
 * the end of the bar, capped at two pixels, and a charger spends almost all of
 * its time at the top of the curve where that gap is one pixel or none. So the
 * one state the user most wants to see - it is plugged in, it is working - was
 * the state the icon had the least room to show. Charging now says so three
 * ways at once, none of which depend on how full the pack is: the outline
 * changes colour, a bolt is drawn across the middle of the icon, and the rest
 * of the bar sweeps up to full and starts again behind a level that stays put.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "hal_gpio.h"
#include "config.h"
#include "utils.h"
#include "timer.h"
#include "lcd.h"
#include "menu_widget.h"  // INFO_HEAD, for the page rendered at the bottom
#include "battery.h"

/*- Definitions -------------------------------------------------------------*/
// The icon, top right. The outline is 20x10 with a 2x4 nub on its right, and
// the fill inside it is 16x6 - so one per cent is not one pixel, and the fill
// is computed in per cent and rounded rather than counted in segments.
#define BATTERY_X                289
#define BATTERY_Y                4
#define BATTERY_BODY_W           20
#define BATTERY_BODY_H           10
#define BATTERY_FILL_X           (BATTERY_X + 2)
#define BATTERY_FILL_Y           (BATTERY_Y + 2)
#define BATTERY_FILL_W           16
#define BATTERY_FILL_H           6

// Charging, in its own colour: the fill keeps the level's red/amber/green,
// because a pack at 10 % is still a pack at 10 % while it charges, and the
// outline and the sweep carry the plugged-in part of the message. A colour
// difference on the outline is legible across a bench without looking at it,
// which is what a status icon is for.
#define BATTERY_CHARGE_COLOR     LCD_COLOR(0, 210, 255)
#define BATTERY_SWEEP_COLOR      LCD_COLOR(0, 80, 105)

// The bolt, centred in the fill. Six rows is the height of the fill, so it is
// as large as the icon can hold - anything smaller stops reading as a bolt.
#define BATTERY_BOLT_W           5
#define BATTERY_BOLT_H           6
#define BATTERY_BOLT_X           (BATTERY_FILL_X + (BATTERY_FILL_W - BATTERY_BOLT_W) / 2)
#define BATTERY_BOLT_Y           BATTERY_FILL_Y

// Full scale at the cell: 3.3 V reference through the divider on PB1
#define BATTERY_REF_VOLTAGE      6600

// Where main.c's battery_low_handler() takes the device down, and 0 % on the
// curve below - the two are deliberately the same number, so the gauge reaches
// zero at the moment the firmware stops rather than some way before it
#define BATTERY_LOW_VOLTAGE      3500

// Above this with the charger pin released, the pack is either full or has
// just come off a charger. Under load it falls back through here in minutes.
// The hysteresis is what stops a filtered reading sitting exactly on the
// threshold from flipping the state - and with it the run-time estimate's
// anchor - twice a second.
#define BATTERY_FULL_VOLTAGE     4150
#define BATTERY_FULL_HYST        50

// What the charge current adds to the terminal voltage across the cell's own
// resistance. Subtracted before the curve is read, so plugging the charger in
// does not make the gauge jump twenty points.
#define BATTERY_CHARGE_LIFT      120

#define BATTERY_SAMPLE_INTERVAL  500

// The charging sweep: one step every interval, and back to the start after
// the last one. Four steps of 500 ms is a two-second cycle - slow enough to
// read as filling up rather than as something flashing for attention.
#define BATTERY_ANIM_INTERVAL    500
#define BATTERY_ANIM_STEPS       4

// Consecutive filtered readings under the floor before the device is halted -
// 10 s on top of a median that is already 7.5 s wide, so nothing short of a
// pack that really is flat gets there.
//
// Patience is the whole point. Halting is the most destructive thing in this
// file, the user cannot undo it, and a gauge that halts the device wrongly is
// worse in every case than one that shows the wrong number for a few seconds.
#define BATTERY_LOW_SAMPLES      20

// ...and a reading below this is not a battery at all. A single cell this flat
// cannot run the board - the regulators give up first - so a number down here
// says the measurement is broken, not the pack, and a broken measurement must
// not be allowed to take the device down. This is exactly the class of failure
// the divider's sample time produced.
#define BATTERY_SANE_VOLTAGE     2800

// Median window. Odd, so the median is a sample rather than an average of two.
#define BATTERY_SAMPLES          15

// Consecutive agreeing reads before the charger pin is believed. It is a clean
// digital signal; this is only here so that a glitch cannot bump the charge
// counter or restart the run-time estimate.
#define BATTERY_STATE_SAMPLES    2

// How far the filtered per cent has to be from the displayed one before the
// display follows it, and how fast it may follow. The step is a quarter of
// whatever the gap is, so dither across a knee of the curve moves the display
// by one at a time while a real change is caught up with in a few updates -
// a flat one-per-update rate makes a large correction take half a minute,
// which looks exactly like a gauge that is filling up rather than one that is
// converging.
#define BATTERY_PCT_DEADBAND     2
#define BATTERY_PCT_STEP_DIV     4

/*
 * Settling, after a boot: the span in which the displayed value simply IS the
 * filtered one and nothing may be halted on it.
 *
 * Not a fixed number of samples, because the thing being waited for is not a
 * clock. The reading climbs after power is applied - the divider has to come
 * up and the converter's sampling capacitor has to charge - and how long that
 * takes is a property of the board, not something this file gets to assume. So
 * the test is whether the window has stopped moving: a reading still on its way
 * up spreads the fifteen samples across tens of millivolts, a settled one does
 * not. BATTERY_SETTLE_MAX is only a backstop, in case the spread never closes
 * because something really is oscillating.
 */
#define BATTERY_SETTLE_SPREAD    40   // mV across the window
#define BATTERY_SETTLE_MAX       120  // ...or 60 s, whichever comes first

// Before the run-time estimate is worth printing: enough elapsed time and
// enough movement that the division is not mostly quantisation
#define BATTERY_EST_MIN_MS       (5 * 60 * 1000)
#define BATTERY_EST_MIN_DELTA    3
#define BATTERY_EST_MAX_MINUTES  (99 * 60)

HAL_GPIO_PIN(VBAT,     B, 1)
HAL_GPIO_PIN(CHARGING, B, 15)

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  int16_t mv;
  int8_t  pct;
} battery_point_t;

/*- Constants ---------------------------------------------------------------*/
/*
 * Discharge curve of a single Li-Ion cell under a light load, as breakpoints
 * of a piecewise-linear map from resting voltage to state of charge. The shape
 * is the point: the top 20 % is 220 mV wide and the middle 40 % is 100 mV
 * wide, which is why a gauge built out of evenly spaced thresholds cannot be
 * right at both ends at once.
 *
 * Anchored at BATTERY_LOW_VOLTAGE for 0 % and 4200 mV for 100 %, in ascending
 * order - battery_percent_of() walks it and interpolates between neighbours.
 */
static const battery_point_t battery_curve[] =
{
  { BATTERY_LOW_VOLTAGE,   0 },
  { 3600,                  5 },
  { 3680,                 10 },
  { 3730,                 20 },
  { 3770,                 30 },
  { 3790,                 40 },
  { 3820,                 50 },
  { 3870,                 60 },
  { 3920,                 70 },
  { 3980,                 80 },
  { 4060,                 90 },
  { 4200,                100 },
};

static const char *const battery_state_names[] =
{
  "discharging", "charging", "full",
};

/*
 * The bolt, one bit per pixel with bit 4 leftmost:
 *
 *     . . # # .
 *     . # # . .
 *     # # # # #
 *     . . # # .
 *     . # # . .
 *     # # . . .
 *
 * A stroke down and to the left, a bar across the middle, and the stroke
 * picked up again to the right of where it stopped - which is the kick that
 * makes five pixels of zigzag read as a lightning bolt rather than as a slash.
 */
static const uint8_t battery_bolt[BATTERY_BOLT_H] =
{
  0x06, 0x0c, 0x1f, 0x06, 0x0c, 0x18,
};

/*- Variables ---------------------------------------------------------------*/
static int g_battery_sample_timer = TIMER_DISABLE;

// The charging sweep: running only while charging, 0..BATTERY_ANIM_STEPS-1
static int g_battery_anim_timer = TIMER_DISABLE;
static int g_battery_anim_phase = 0;

// The median window, primed at init so every slot is always a real reading
static uint16_t g_battery_samples[BATTERY_SAMPLES];
static int g_battery_sample_index = 0;

// mV * 16, so the smoothing filter converges instead of stalling a few
// millivolts out on integer truncation
static int32_t g_battery_acc = 0;

static int g_battery_voltage = 0;     // filtered, mV
static int g_battery_raw_mv = 0;      // the last conversion, unfiltered
static int g_battery_median_mv = 0;   // the window median, before smoothing
static int g_battery_window_min = 0;  // over the median window: how far the
static int g_battery_window_max = 0;  // rail moves under this workload

static int g_battery_percent = -1;    // displayed, -1 until the first sample
static int g_battery_percent_raw = 0; // straight off the curve

static battery_state_t g_battery_state = BATTERY_DISCHARGING;
static bool g_charger_raw = false;    // last pin read, before the debounce
static bool g_charger_on = false;     // ...and after it
static int g_charger_count = 0;

static int g_battery_low_count = 0;

// Updates left before settling is given up on, and how many of the samples in
// the window were taken by battery_task() rather than put there by the priming
// in battery_init(). See BATTERY_SETTLE_SPREAD.
static int g_battery_settle = BATTERY_SETTLE_MAX;
static int g_battery_filled = 0;

// The icon is off screen because a modal is up, and owes the corner a repaint
// as soon as one closes
static bool g_battery_hidden = false;

// Anchor for the run-time estimate, reset whenever the state changes: a rate
// measured across a plug event describes neither side of it
static uint32_t g_battery_est_stamp = 0;
static int g_battery_est_percent = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int read_battery_voltage(void)
{
  return (ADC0->RDATA * BATTERY_REF_VOLTAGE) / 4096;
}

//-----------------------------------------------------------------------------
// Median of the window, and its extremes on the way past. Insertion sort of 15
// entries every 500 ms - a few hundred comparisons, against a filter that has
// to reject an outlier outright rather than let it pull an average.
static int battery_median(void)
{
  uint16_t sorted[BATTERY_SAMPLES];

  memcpy(sorted, g_battery_samples, sizeof(sorted));

  for (int i = 1; i < BATTERY_SAMPLES; i++)
  {
    uint16_t v = sorted[i];
    int j = i - 1;

    while (j >= 0 && sorted[j] > v)
    {
      sorted[j+1] = sorted[j];
      j--;
    }

    sorted[j+1] = v;
  }

  g_battery_window_min = sorted[0];
  g_battery_window_max = sorted[BATTERY_SAMPLES-1];

  return sorted[BATTERY_SAMPLES/2];
}

//-----------------------------------------------------------------------------
static int battery_percent_of(int mv)
{
  const battery_point_t *lo, *hi;

  if (mv <= battery_curve[0].mv)
    return 0;

  for (int i = 1; i < ARRAY_SIZE(battery_curve); i++)
  {
    if (mv >= battery_curve[i].mv)
      continue;

    lo = &battery_curve[i-1];
    hi = &battery_curve[i];

    return lo->pct + (mv - lo->mv) * (hi->pct - lo->pct) / (hi->mv - lo->mv);
  }

  return 100;
}

//-----------------------------------------------------------------------------
// The charger pin, debounced. Open drain with a pull-up: low means current is
// going into the cell.
static bool battery_charger_on(void)
{
  bool pin = !HAL_GPIO_CHARGING_read();

  if (pin == g_charger_raw)
  {
    if (g_charger_count < BATTERY_STATE_SAMPLES)
      g_charger_count++;
  }
  else
  {
    g_charger_raw = pin;
    g_charger_count = 1;
  }

  if (g_charger_count >= BATTERY_STATE_SAMPLES)
    g_charger_on = g_charger_raw;

  return g_charger_on;
}

//-----------------------------------------------------------------------------
static void battery_draw_frame(void)
{
  int frame;

  // Charging outranks the low-battery red: a pack under 15 % that is on a
  // charger is not the thing red is warning about, and the state the user is
  // looking for an answer about is the charger.
  if (BATTERY_CHARGING == g_battery_state)
    frame = BATTERY_CHARGE_COLOR;
  else if (g_battery_percent >= 0 && g_battery_percent < 15 &&
      BATTERY_DISCHARGING == g_battery_state)
    frame = LCD_RED_COLOR;
  else
    frame = LCD_WHITE_COLOR;

  // Outline and nub, then the one-pixel gap between the outline and the fill.
  // Lines rather than a filled 20x10 rectangle with a black one over it: every
  // pixel inside the icon is written exactly once per redraw, so the icon does
  // not flash black each time the level is repainted.
  lcd_hline(BATTERY_X, BATTERY_X + BATTERY_BODY_W - 1, BATTERY_Y, frame);
  lcd_hline(BATTERY_X, BATTERY_X + BATTERY_BODY_W - 1,
      BATTERY_Y + BATTERY_BODY_H - 1, frame);
  lcd_vline(BATTERY_X, BATTERY_Y, BATTERY_Y + BATTERY_BODY_H - 1, frame);
  lcd_vline(BATTERY_X + BATTERY_BODY_W - 1, BATTERY_Y,
      BATTERY_Y + BATTERY_BODY_H - 1, frame);
  lcd_fill_rect(BATTERY_X + BATTERY_BODY_W, BATTERY_Y + 3, 2, 4, frame);

  lcd_hline(BATTERY_X + 1, BATTERY_X + BATTERY_BODY_W - 2, BATTERY_Y + 1,
      LCD_BLACK_COLOR);
  lcd_hline(BATTERY_X + 1, BATTERY_X + BATTERY_BODY_W - 2,
      BATTERY_Y + BATTERY_BODY_H - 2, LCD_BLACK_COLOR);
  lcd_vline(BATTERY_X + 1, BATTERY_Y + 2, BATTERY_Y + BATTERY_BODY_H - 3,
      LCD_BLACK_COLOR);
  lcd_vline(BATTERY_X + BATTERY_BODY_W - 2, BATTERY_Y + 2,
      BATTERY_Y + BATTERY_BODY_H - 3, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
// The bolt, over whatever the level left behind in the middle of the icon:
// black where it lands on a painted column and the charge colour where it
// lands on an empty one. That is what makes it the one part of the icon whose
// legibility does not depend on how full the pack is - and a pack on a charger
// is at the top of the curve nearly all of the time, which is exactly where
// the bar has no room left to say anything.
//
// `painted` is how far along the fill area something has been drawn, fill plus
// sweep, in columns.
static void battery_draw_bolt(int painted)
{
  for (int row = 0; row < BATTERY_BOLT_H; row++)
  {
    uint8_t bits = battery_bolt[row];

    for (int col = 0; col < BATTERY_BOLT_W; col++)
    {
      int x = BATTERY_BOLT_X + col;

      if (0 == (bits & (1 << (BATTERY_BOLT_W - 1 - col))))
        continue;

      lcd_draw_pixel(x, BATTERY_BOLT_Y + row,
          ((x - BATTERY_FILL_X) < painted) ? LCD_BLACK_COLOR :
          BATTERY_CHARGE_COLOR);
    }
  }
}

//-----------------------------------------------------------------------------
static void battery_draw_level(void)
{
  bool charging = (BATTERY_CHARGING == g_battery_state);
  int pct = (g_battery_percent < 0) ? 0 : g_battery_percent;
  int fill = (pct * BATTERY_FILL_W + 50) / 100;
  int ghost = 0;
  int rest, color;

  if (pct < 15)
    color = LCD_RED_COLOR;
  else if (pct < 40)
    color = LCD_COLOR(255, 200, 0);
  else
    color = LCD_GREEN_COLOR;

  // Charging reads as the rest of the bar sweeping up to full and starting
  // again, rather than as the level itself moving: the bar keeps saying how
  // full the pack is while the sweep says current is going in. Rounded up, so
  // a gap of one or two columns is a tail that is drawn rather than one that
  // divides away to nothing.
  if (charging)
  {
    rest = BATTERY_FILL_W - fill;
    ghost = (rest * (g_battery_anim_phase + 1) + BATTERY_ANIM_STEPS - 1) /
        BATTERY_ANIM_STEPS;
  }

  rest = BATTERY_FILL_W - fill - ghost;

  if (fill > 0)
    lcd_fill_rect(BATTERY_FILL_X, BATTERY_FILL_Y, fill, BATTERY_FILL_H, color);

  if (ghost > 0)
  {
    lcd_fill_rect(BATTERY_FILL_X + fill, BATTERY_FILL_Y, ghost,
        BATTERY_FILL_H, BATTERY_SWEEP_COLOR);
  }

  if (rest > 0)
  {
    lcd_fill_rect(BATTERY_FILL_X + fill + ghost, BATTERY_FILL_Y, rest,
        BATTERY_FILL_H, LCD_BLACK_COLOR);
  }

  if (charging)
    battery_draw_bolt(fill + ghost);
}

//-----------------------------------------------------------------------------
// Always both halves. The icon used to repaint its level without its outline,
// and an application that had painted over the top right corner - which is all
// of the full-screen ones - was left with a coloured strip floating in its
// picture until something else redrew the screen.
static void battery_draw(void)
{
  battery_draw_frame();
  battery_draw_level();
}

//-----------------------------------------------------------------------------
// Displayed per cent follows the filtered one, but not instantly and not at a
// fixed rate: nothing happens until the two are BATTERY_PCT_DEADBAND apart,
// which is what keeps the last pixel of the fill from flickering while the
// reading dithers across a knee of the curve, and then it closes a quarter of
// the gap per update, which puts a real change on screen in a few seconds.
//
// `snap` is for the moments where following at all would be a lie rather than
// a smoothing: the settling period after a boot, and the step the reading
// takes when the charger is plugged in or pulled out.
static void battery_set_percent(int raw, bool snap)
{
  int diff = raw - g_battery_percent;
  int step;

  if (snap || g_battery_percent < 0 || g_battery_settle > 0)
  {
    g_battery_percent = raw;
    return;
  }

  if (diff > -BATTERY_PCT_DEADBAND && diff < BATTERY_PCT_DEADBAND)
    return;

  step = ((diff < 0) ? -diff : diff) / BATTERY_PCT_STEP_DIV;

  if (step < 1)
    step = 1;

  g_battery_percent += (diff < 0) ? -step : step;
}

//-----------------------------------------------------------------------------
static void battery_sample(void)
{
  battery_state_t state;
  int soc_mv;

  g_battery_raw_mv = read_battery_voltage();

  g_battery_samples[g_battery_sample_index] = g_battery_raw_mv;
  g_battery_sample_index = (g_battery_sample_index + 1) % BATTERY_SAMPLES;

  if (g_battery_filled < BATTERY_SAMPLES)
    g_battery_filled++;

  g_battery_median_mv = battery_median();

  g_battery_acc += ((g_battery_median_mv * 16) - g_battery_acc) / 4;
  g_battery_voltage = (g_battery_acc + 8) / 16;

  if (battery_charger_on())
  {
    state = BATTERY_CHARGING;
  }
  else if (g_battery_voltage >= BATTERY_FULL_VOLTAGE ||
      (BATTERY_FULL == g_battery_state &&
       g_battery_voltage >= BATTERY_FULL_VOLTAGE - BATTERY_FULL_HYST))
  {
    state = BATTERY_FULL;
  }
  else
  {
    state = BATTERY_DISCHARGING;
  }

  // The cell sits BATTERY_CHARGE_LIFT above its resting voltage for as long as
  // the charger is pushing current, so the curve is read below the terminals
  soc_mv = (BATTERY_CHARGING == state) ?
      g_battery_voltage - BATTERY_CHARGE_LIFT : g_battery_voltage;

  g_battery_percent_raw = (BATTERY_FULL == state) ? 100 :
      battery_percent_of(soc_mv);

  if (state != g_battery_state)
  {
    // A charger connected while the device is running. Not a cycle in the
    // datasheet sense, and it never was - it is the count of times this pack
    // has been put on charge, which is what the number is read as.
    if (BATTERY_CHARGING == state)
      config.charge_cycles++;

    g_battery_state = state;

    // From the start of the sweep either way round, so the icon does not come
    // out of a plug event with the bar half painted
    g_battery_anim_phase = 0;

    if (BATTERY_CHARGING == state)
      g_battery_anim_timer = BATTERY_ANIM_INTERVAL;
    else
      g_battery_anim_timer = TIMER_DISABLE;

    // The lift comes off or goes on with the charger, so the displayed value
    // steps here rather than walking to the new one a per cent at a time
    battery_set_percent(g_battery_percent_raw, true);

    // Whatever rate was being measured belonged to the other side of this.
    // Anchored after the snap above, or the step would be counted as drain.
    g_battery_est_stamp = timer_ms();
    g_battery_est_percent = g_battery_percent;
  }
  else
  {
    battery_set_percent(g_battery_percent_raw, false);
  }

  // Settled once the window holds nothing but samples this function took and
  // they agree with each other. A reading still climbing after power-up fails
  // the second test, which is the point: that climb is what used to be read as
  // a flat battery and used to halt the device on the way up.
  if (g_battery_settle > 0)
  {
    g_battery_settle--;

    if (g_battery_filled >= BATTERY_SAMPLES &&
        (g_battery_window_max - g_battery_window_min) <= BATTERY_SETTLE_SPREAD)
      g_battery_settle = 0;
  }

  // Off the filtered voltage, so a load step no longer counts towards it: the
  // median has to have moved, which takes half the window. And not while
  // settling, and not on a reading too low to have come from a pack that is
  // running the board - see BATTERY_SANE_VOLTAGE.
  if (g_battery_settle > 0)
  {
    g_battery_low_count = 0;
  }
  else if (g_battery_voltage < BATTERY_LOW_VOLTAGE &&
      g_battery_voltage >= BATTERY_SANE_VOLTAGE && BATTERY_CHARGING != state)
  {
    g_battery_low_count++;

    if (g_battery_low_count >= BATTERY_LOW_SAMPLES)
      battery_low_handler();
  }
  else
  {
    g_battery_low_count = 0;
  }
}

//-----------------------------------------------------------------------------
void battery_init(void)
{
  HAL_GPIO_CHARGING_in();
  HAL_GPIO_CHARGING_pullup();

  HAL_GPIO_VBAT_analog();

  RCU->APB2EN_b.ADC0EN = 1;

  ADC_Common->SYNCCTL_b.ADCCK = 7; // HCLK / 20

  ADC0->OVSAMPCTL = ADC0_OVSAMPCTL_OVSEN_Msk | (7/*256x*/ << ADC0_OVSAMPCTL_OVSR_Pos) |
      (8/*Shift 8-bits*/ << ADC0_OVSAMPCTL_OVSS_Pos);

  ADC0->CTL1_b.ADCON = 1;
  delay_ms(1);

  ADC0->CTL1_b.RSTCLB = 1;
  while (ADC0->CTL1_b.RSTCLB);

  ADC0->CTL1_b.CLB = 1;
  while (ADC0->CTL1_b.CLB);

  ADC0->RSQ2_b.RSQ0 = 9;

  /*
   * Channel 9, which is the one the sequence above converts. SAMPT1 holds the
   * sample times of channels 0 to 9 and this line used to write SPT0, so the
   * divider on PB1 was sampled for the reset default of 3 cycles - 240 ns at
   * this ADC clock, against a resistive divider that needs microseconds to
   * charge the converter's sampling capacitor through it.
   *
   * That is the whole of "the gauge fills up from zero after every boot". The
   * capacitor starts each session empty, a 3-cycle window tops it up by a
   * fraction of the way to the pin each time, and the reading therefore climbs
   * towards the real voltage over the first minute of every power cycle -
   * every boot, every time, exactly as reported. It never quite got there
   * either: the steady-state reading was low too, which is why a pack with
   * charge left in it could still trip the 3.5 V cutoff.
   *
   * 480 cycles is the longest the converter offers, ~38 us, and there is
   * nothing to spend it on - this channel is read twice a second.
   */
  ADC0->SAMPT1_b.SPT9 = 7; // 480 cycles

  ADC0->CTL1_b.CTN = 1;

  ADC0->CTL1_b.SWRCST = 1;
  while (0 == ADC0->STAT_b.EOC);

  // ...and then throw that first result away. 256x oversampling at 480 cycles
  // is ~10 ms per result, so this waits out two more of them with the
  // converter running continuously: what the window is primed from is a
  // settled reading rather than the one that came straight out of calibration.
  // A fixed delay rather than a second EOC handshake on purpose - this runs
  // before anything is on screen, and a converter that never raises the flag
  // would hang the boot instead of showing a wrong gauge.
  delay_ms(25);

  // Prime the whole window with that conversion rather than letting it fill
  // from zero: every slot is then a real reading from the first sample on, and
  // the median needs no "is it full yet" case
  g_battery_raw_mv = read_battery_voltage();

  for (int i = 0; i < BATTERY_SAMPLES; i++)
    g_battery_samples[i] = g_battery_raw_mv;

  g_battery_median_mv = g_battery_raw_mv;
  g_battery_window_min = g_battery_raw_mv;
  g_battery_window_max = g_battery_raw_mv;
  g_battery_voltage = g_battery_raw_mv;
  g_battery_acc = g_battery_raw_mv * 16;

  g_battery_percent_raw = battery_percent_of(g_battery_voltage);
  g_battery_percent = g_battery_percent_raw;

  g_battery_est_stamp = timer_ms();
  g_battery_est_percent = g_battery_percent;

  timer_add(&g_battery_sample_timer);
  timer_add(&g_battery_anim_timer);

  g_battery_sample_timer = BATTERY_SAMPLE_INTERVAL;

  battery_draw();
}

//-----------------------------------------------------------------------------
int battery_voltage(void)
{
  return g_battery_voltage;
}

//-----------------------------------------------------------------------------
int battery_percent(void)
{
  return (g_battery_percent < 0) ? 0 : g_battery_percent;
}

//-----------------------------------------------------------------------------
battery_state_t battery_state(void)
{
  return g_battery_state;
}

//-----------------------------------------------------------------------------
void battery_redraw(void)
{
  battery_draw();
}

//-----------------------------------------------------------------------------
void battery_task(bool may_draw)
{
  bool redraw = false;

  if (0 == g_battery_sample_timer)
  {
    int percent = g_battery_percent;
    battery_state_t state = g_battery_state;

    g_battery_sample_timer = BATTERY_SAMPLE_INTERVAL;

    battery_sample();

    // The per cent is the whole of what the icon shows, so a millivolt that
    // moved without moving it is not worth a repaint. The old gauge redrew on
    // every changed reading, which on a filtered value is every single time.
    redraw = (percent != g_battery_percent || state != g_battery_state);
  }

  if (0 == g_battery_anim_timer)
  {
    g_battery_anim_timer = BATTERY_ANIM_INTERVAL;
    g_battery_anim_phase = (g_battery_anim_phase + 1) % BATTERY_ANIM_STEPS;

    redraw = true;
  }

  // Measuring carries on regardless; only the drawing waits
  if (!may_draw)
  {
    g_battery_hidden = true;
    return;
  }

  // Whatever was covering the corner has closed. Nothing repaints the icon on
  // the gauge's behalf - the oscilloscope is the only application that even
  // knows it exists - so it puts itself back, whether or not the reading moved
  // while it was off screen.
  if (g_battery_hidden)
  {
    g_battery_hidden = false;
    redraw = true;
  }

  if (redraw)
    battery_draw();
}

//-----------------------------------------------------------------------------
// Minutes to empty, or to full while charging; -1 when there is not yet enough
// movement to divide by. Measured from the last state change, so it is the
// average over this run rather than an instantaneous rate - which is the
// honest thing to report when the load is a game one minute and an idle menu
// the next.
static int battery_estimate_minutes(void)
{
  uint32_t minutes = (timer_ms() - g_battery_est_stamp) / 60000u;
  int delta, target, est;

  if ((timer_ms() - g_battery_est_stamp) < BATTERY_EST_MIN_MS || 0 == minutes)
    return -1;

  if (BATTERY_CHARGING == g_battery_state)
  {
    delta = g_battery_percent - g_battery_est_percent;
    target = 100 - g_battery_percent;
  }
  else
  {
    delta = g_battery_est_percent - g_battery_percent;
    target = g_battery_percent;
  }

  if (delta < BATTERY_EST_MIN_DELTA)
    return -1;

  est = (int)minutes * target / delta;

  return (est > BATTERY_EST_MAX_MINUTES) ? BATTERY_EST_MAX_MINUTES : est;
}

//-----------------------------------------------------------------------------
static int battery_format_mv(char *buf, int size, int mv)
{
  return snprintf(buf, size, "%d.%02d V", mv / 1000, (mv % 1000) / 10);
}

//-----------------------------------------------------------------------------
void battery_get_state(char *buf, int size)
{
  char volts[16];

  battery_format_mv(volts, sizeof(volts), g_battery_voltage);

  snprintf(buf, size, "Battery: %d%%, %s, %s", battery_percent(), volts,
      battery_state_names[g_battery_state]);
}

//-----------------------------------------------------------------------------
// The Battery page
//-----------------------------------------------------------------------------
enum
{
  BP_CHARGE, BP_VOLTAGE, BP_FILTER, BP_WINDOW, BP_STATE, BP_SETTLE,
  BP_ESTIMATE, BP_CYCLES, BP_ADC,
  BP_GAP,
  BP_HEAD, BP_T1, BP_T2, BP_T3, BP_T4,
  BP_GAP2,
  BP_HEAD2, BP_T5, BP_T6, BP_T7,
  BP_COUNT,
};

bool battery_get_line(int index, char *buf, int size)
{
  char volts[16];

  switch (index)
  {
    case BP_CHARGE:
      snprintf(buf, size, "Charge:    %d %%   (what the icon fills to)",
          battery_percent());
      break;

    case BP_VOLTAGE:
      battery_format_mv(volts, sizeof(volts), g_battery_voltage);
      snprintf(buf, size, "Cell:      %s   filtered, %d mV", volts,
          g_battery_voltage);
      break;

    // The two numbers behind the displayed one: what the curve says right now,
    // and what the last conversion read. A gap between this and Charge above
    // is the hysteresis working, not a fault.
    case BP_FILTER:
      snprintf(buf, size, "Instant:   %d %% off the curve, last read %d mV",
          g_battery_percent_raw, g_battery_raw_mv);
      break;

    // How far the rail moves under whatever is running: a wide window is a
    // heavy load or a tired cell, and it is the reason a single conversion
    // could never be trusted
    case BP_WINDOW:
      snprintf(buf, size, "Window:    %d - %d mV across %d readings",
          g_battery_window_min, g_battery_window_max, BATTERY_SAMPLES);
      break;

    case BP_STATE:
      snprintf(buf, size, "State:     %s   (CHRG pin %s)",
          battery_state_names[g_battery_state],
          g_charger_on ? "low" : "released");
      break;

    // Settling, and what it is holding off: the low-battery halt. A gauge that
    // says "settling" a minute after boot is a reading that will not sit still,
    // and that is worth seeing rather than guessing at.
    case BP_SETTLE:
      if (g_battery_settle > 0)
      {
        snprintf(buf, size, "Filter:    settling, %d of %d readings, cutoff off",
            g_battery_filled, BATTERY_SAMPLES);
      }
      else if (g_battery_voltage < BATTERY_SANE_VOLTAGE)
      {
        snprintf(buf, size, "Filter:    below %d mV - not believed, cutoff off",
            BATTERY_SANE_VOLTAGE);
      }
      else
      {
        snprintf(buf, size, "Filter:    settled, cutoff armed at %d mV",
            BATTERY_LOW_VOLTAGE);
      }

      break;

    case BP_ESTIMATE:
    {
      int minutes = battery_estimate_minutes();

      if (minutes < 0)
      {
        snprintf(buf, size, "Estimate:  not enough of this run measured yet");
      }
      else
      {
        snprintf(buf, size, "Estimate:  %dh %02dm to %s, at this run's rate",
            minutes / 60, minutes % 60,
            (BATTERY_CHARGING == g_battery_state) ? "full" : "empty");
      }

      break;
    }

    case BP_CYCLES:
      snprintf(buf, size, "Put on charge: %d times", config.charge_cycles);
      break;

    case BP_ADC:
      snprintf(buf, size, "ADC0 ch9:  %lu / 4095, full scale %d.%03d V",
          (unsigned long)ADC0->RDATA, BATTERY_REF_VOLTAGE / 1000,
          BATTERY_REF_VOLTAGE % 1000);
      break;

    case BP_GAP:
    case BP_GAP2:
      buf[0] = 0;
      break;

    case BP_HEAD:
      snprintf(buf, size, INFO_HEAD "What the per cent is");
      break;

    case BP_T1:
      snprintf(buf, size, "A lookup of the cell voltage against the");
      break;

    case BP_T2:
      snprintf(buf, size, "discharge curve of one Li-Ion cell, not a");
      break;

    case BP_T3:
      snprintf(buf, size, "charge counter. It reads low under load and");
      break;

    case BP_T4:
      snprintf(buf, size, "high for a while after the charger comes off.");
      break;

    case BP_HEAD2:
      snprintf(buf, size, INFO_HEAD "Where it stops");
      break;

    case BP_T5:
      snprintf(buf, size, "0 %% is %d mV, which is where the firmware",
          BATTERY_LOW_VOLTAGE);
      break;

    case BP_T6:
      snprintf(buf, size, "halts rather than run the cell down. The gauge");
      break;

    case BP_T7:
      snprintf(buf, size, "therefore reaches zero as the device stops.");
      break;

    default:
      return false;
  }

  return true;
}
