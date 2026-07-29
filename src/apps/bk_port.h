/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Электроника БК-0010-01, on an oscilloscope.
 */

#ifndef _BK_PORT_H_
#define _BK_PORT_H_

/*- Includes ----------------------------------------------------------------*/
#include "menu_widget.h"

/*- Prototypes --------------------------------------------------------------*/
void bk_port_init(void);
void bk_port_task(void);
void bk_port_buttons_handler(int buttons);
void bk_port_cleanup(void);
void bk_port_redraw(void);

extern const menu_def_t bk_port_menu;
extern const menu_def_t bk_port_help_menu;

#endif // _BK_PORT_H_
