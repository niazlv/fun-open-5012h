/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Simple Snake Game
 * Classic snake game implementation
 */

#ifndef _SNAKE_GAME_H_
#define _SNAKE_GAME_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t snake_game_menu;
extern const menu_def_t snake_game_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void snake_game_init(void);
void snake_game_task(void);
void snake_game_buttons_handler(int buttons);
void snake_game_cleanup(void);
void snake_game_redraw(void);

#endif // _SNAKE_GAME_H_
