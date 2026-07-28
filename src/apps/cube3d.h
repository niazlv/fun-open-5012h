/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 3D Cube Renderer
 * Rotating, back-face culled 3D cube demo
 */

#ifndef _CUBE3D_H_
#define _CUBE3D_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define CUBE3D_CENTER_X    (LCD_WIDTH / 2)
#define CUBE3D_CENTER_Y    (LCD_HEIGHT / 2)
// Sized so the projected cube stays inside the render area at any rotation:
// half the space diagonal (61) times the worst-case perspective factor
// (1.43) is 87 px around the center
#define CUBE3D_SIZE        70
#define CUBE3D_DISTANCE    200

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t cube3d_menu;
extern const menu_def_t cube3d_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void cube3d_init(void);
void cube3d_task(void);
void cube3d_buttons_handler(int buttons);
void cube3d_cleanup(void);
void cube3d_redraw(void);

#endif // _CUBE3D_H_
