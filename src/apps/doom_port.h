/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DOOM - id Software's renderer running a real WAD level.
 *
 * The engine itself is in doom/; this is the application shell that owns the
 * player, the input mapping and the frame loop. Screen geometry comes from
 * doom/doom.h, because the projection tables in the asset pack are generated
 * for one specific view size.
 */

#ifndef _DOOM_PORT_H_
#define _DOOM_PORT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "buttons.h"
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t doom_port_menu;
extern const menu_def_t doom_port_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void doom_port_init(void);
void doom_port_task(void);
void doom_port_buttons_handler(int buttons);
void doom_port_cleanup(void);
void doom_port_redraw(void);

#endif // _DOOM_PORT_H_
