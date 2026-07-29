/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The instrument's front panel, drawn around the screen.
 *
 * The emulator's window used to be the 320x240 panel and nothing else, which
 * says nothing about where a key is on the real thing. This draws the body and
 * the eighteen keys in the arrangement they actually have, labelled with the
 * keyboard key that works them, and lets them be clicked.
 *
 * Everything here is measured in panel units, which are LCD pixels: the window
 * is these numbers times the magnification, so the layout does not have to
 * know what magnification it is being drawn at.
 */

#ifndef _PANEL_H_
#define _PANEL_H_

#ifndef EMU_NO_SDL

#include <stdint.h>
#include <SDL.h>

/* The whole body, in panel units */
#define PANEL_W   360
#define PANEL_H   482

/* Where the LCD sits in it */
#define PANEL_SCREEN_X  20
#define PANEL_SCREEN_Y  20

/* Draws the body and the keys. `pressed` is the button mask as the board sees
 * it, so a key lights up whether it was reached by mouse or by keyboard. */
void panel_draw(SDL_Renderer *renderer, int scale, uint32_t pressed);

/* The button under a window pixel, or 0 for none */
uint32_t panel_hit(int scale, int x, int y);

#endif /* EMU_NO_SDL */
#endif /* _PANEL_H_ */
