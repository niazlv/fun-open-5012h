/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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
// if it has none), its help pages (NULL if it has none) and its name. All are
// NULL while the launcher is on top.
//
// Settings and help are two separate tables on purpose: the system menu shows
// the settings among the application's own rows and folds the help pages into
// the one Help section at the root, so every application documents itself in
// the same place.
const menu_def_t *launcher_app_menu(void);
const menu_def_t *launcher_app_help(void);
const char *launcher_app_name(void);
bool launcher_app_running(void);

// Whether the main loop may sleep the core between passes while this
// application runs. See the definition for why the oscilloscope may not.
bool launcher_app_may_idle(void);

#endif // _LAUNCHER_H_
