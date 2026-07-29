/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 2048, on a board of 3x3 to 6x6
 */

#ifndef _GAME2048_H_
#define _GAME2048_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t game2048_menu;
extern const menu_def_t game2048_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void game2048_init(void);
void game2048_task(void);
void game2048_buttons_handler(int buttons);
void game2048_cleanup(void);
void game2048_redraw(void);

#endif // _GAME2048_H_
