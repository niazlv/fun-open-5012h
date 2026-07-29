/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Tetris
 */

#ifndef _TETRIS_H_
#define _TETRIS_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t tetris_menu;
extern const menu_def_t tetris_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void tetris_init(void);
void tetris_task(void);
void tetris_buttons_handler(int buttons);
void tetris_cleanup(void);
void tetris_redraw(void);

#endif // _TETRIS_H_
