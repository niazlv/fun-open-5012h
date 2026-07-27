/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Simple Flappy Bird Game
 * Basic side-scrolling bird game
 */

#ifndef _FLAPPY_BIRD_H_
#define _FLAPPY_BIRD_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t flappy_bird_menu;
extern const menu_def_t flappy_bird_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void flappy_bird_init(void);
void flappy_bird_task(void);
void flappy_bird_buttons_handler(int buttons);
void flappy_bird_cleanup(void);
void flappy_bird_redraw(void);

#endif // _FLAPPY_BIRD_H_
