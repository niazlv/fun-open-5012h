/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Sixteen kilobytes of screen, onto a panel that is the wrong shape for it.
 */

#ifndef _BK_VIDEO_H_
#define _BK_VIDEO_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "bk_config.h"

/*- Types -------------------------------------------------------------------*/
/*
 * How to read the video memory.
 *
 * There is no mode bit in the hardware to ask. The controller always sends 512
 * dots a line and always drives the colour outputs from pairs of bits; what
 * the manuals call "32 symbols" and "64 symbols" is the monitor choosing how
 * wide to draw a character, and users switched their television to black and
 * white to stop the fringes on the narrow one. So this is a decision about the
 * viewer, and it belongs to whoever is looking at the screen.
 */
typedef enum
{
    BK_SCREEN_COLOR = 0,        // 256 x 256, four colours from a palette
    BK_SCREEN_MONO,             // 512 x 256, one bit a dot
} bk_screen_t;

/*
 * 256 lines into 240 rows of panel.
 *
 * Crop keeps every line at its own size and loses sixteen of them, split
 * between the top and the bottom by an adjustable amount - which is right for
 * a game, because a game's playfield is in the middle and the sixteen lines it
 * loses are border.
 *
 * Squash drops one line in sixteen and shows the whole screen. Right for
 * anything with text at the bottom edge, wrong for anything where a horizontal
 * line of the picture is load-bearing.
 */
typedef enum
{
    BK_FIT_CROP = 0,
    BK_FIT_SQUASH,
} bk_fit_t;

/*
 * 512 dots into a 320 pixel panel, which is the other way this machine does
 * not fit.
 *
 * Whole shows all of it by taking the dots in pairs, exactly as the colour
 * mode does - so both modes put the same part of the screen in the same place
 * at the same size, and switching between them stops being a jump. A pair
 * with one dot lit is drawn grey rather than rounded either way, which keeps
 * 512-dot text readable at 256 pixels instead of turning it into a smear.
 *
 * Window is one dot to one pixel over 320 of the 512, and Pan chooses which
 * 320. Right when the exact dots matter and nothing else will do.
 */
typedef enum
{
    BK_MONO_WHOLE = 0,
    BK_MONO_WINDOW,
} bk_mono_t;

/*- Definitions -------------------------------------------------------------*/
/*
 * Scratch for the conversion, supplied by the caller.
 *
 * It is 5 KB and it is not a static array in bk_video.c on purpose: every
 * byte of this firmware's .bss lands in the 64 KB of tightly coupled memory,
 * which the oscilloscope has 83% of before the emulator is compiled. The main
 * SRAM the capture ring leaves free is where this belongs, next to the
 * emulated machine itself.
 */
#define BK_VIDEO_BUFFER_SIZE    (320 * 16)

/*- Prototypes --------------------------------------------------------------*/
void bk_video_init(void);
void bk_video_set_buffer(uint8_t *buf);

/*
 * Paint. `full` ignores what has changed and repaints everything, which is
 * what an overlay closing over the top of it needs.
 *
 *
 * Returns the number of panel rows actually pushed - the useful measure of how
 * much a frame cost, because the panel is bit-banged and rows are the unit it
 * is slow in.
 */
int bk_video_draw(bool full);

void bk_video_set_screen(bk_screen_t screen);
void bk_video_set_fit(bk_fit_t fit);
void bk_video_set_mono(bk_mono_t mono);
void bk_video_set_top(int line);            // crop: first line shown, 0..16
void bk_video_set_pan(int dot);             // mono: first dot shown, 0..192
/*
 * Whether anything wants somewhere to write.
 *
 * It does not cost the picture anything in colour: 256 dots in a 320 pixel
 * panel leave 64 pixels over, and those were black. The picture moves to the
 * left edge instead of the middle and the column on the right becomes the
 * place for text - so the machine still gets all 256 lines it would otherwise
 * have had to give sixteen of away.
 *
 * Mono has no margin to use - 512 dots windowed to 320 fills the panel - so
 * there it does cost sixteen rows off the bottom, and there is nowhere else
 * for them to come from.
 */
void bk_video_set_status(bool on);

// Where the machine's picture ends and anything else may begin
int bk_video_bottom(void);

// The part of the panel the picture does not cover. Width zero means none.
void bk_video_spare(int *x, int *y, int *w, int *h);

// The palette the screen is being drawn with, as the panel wants it. For an
// application that has to draw something of its own in matching colours.
const uint16_t *bk_video_palette(void);

#endif // _BK_VIDEO_H_
