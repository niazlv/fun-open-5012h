/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Application launcher: the root screen of the UI stack.
 * Applications are described by a const table and run as screens on top.
 */

#ifndef _LAUNCHER_H_
#define _LAUNCHER_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include "menu_widget.h"

/*- Prototypes --------------------------------------------------------------*/
void launcher_start(void);

// Open an application by name, bypassing the list. Returns false if there is
// no such application.
bool launcher_start_app(const char *name);

// Exit the running application back to the launcher (callable from apps)
void launcher_exit_app(void);

// The running application, for the system menu: its own settings menu (NULL
// if it has none) and its name. Both are NULL while the launcher is on top.
const menu_def_t *launcher_app_menu(void);
const char *launcher_app_name(void);
bool launcher_app_running(void);

#endif // _LAUNCHER_H_
