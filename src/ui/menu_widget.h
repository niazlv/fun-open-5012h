/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Generic menu widget
 *
 * Menus are described by const tables of menu_item_t and rendered by one
 * shared widget: navigation, scrolling, separator skipping, toggles and
 * value editing are implemented once. A submenu is just another menu screen
 * pushed onto the UI stack.
 *
 * The same file provides the info page: a modal, fullscreen text screen used
 * for every help and info dialog, so an application only has to supply the
 * text. A page that does not fit on the screen scrolls, so there is no length
 * an application has to keep its text under.
 */

#ifndef _MENU_WIDGET_H_
#define _MENU_WIDGET_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// Info page body: first line origin. This is the grid of a page that fits on
// one screen, which is the grid a custom body() draws on. A page longer than
// that scrolls on a tighter grid of its own; a body() page never does.
#define INFO_X         20
#define INFO_Y         45
#define INFO_LINE_H    13

// Section heading. A line that starts with this marker is drawn as a heading
// and becomes a jump target, so a page the size of a manual is read in
// sections instead of by holding an arrow down:
//
//   INFO_HEAD "CAN",
//   "CAN reads either wire of the pair:",
#define INFO_HEAD      "\1"

/*- Types -------------------------------------------------------------------*/
typedef enum
{
  MI_ACTION = 0, // run action.fn(action.arg)
  MI_SUBMENU,    // open a nested menu
  MI_TOGGLE,     // flip *toggle.value in place, rendered as ON/OFF
  MI_NUMBER,     // edit *number.value in place with LEFT/RIGHT
  MI_CHOICE,     // cycle *choice.value through choice.labels
  MI_SEPARATOR,
} mi_kind_t;

typedef struct menu_item_s
{
  mi_kind_t kind;
  const char *label;
  const char *desc; // optional second line, fullscreen style only
  union
  {
    struct
    {
      void (*fn)(const void *arg);
      const void *arg;
    } action;
    struct
    {
      const struct menu_item_s *items;
      int count;
    } submenu;
    struct
    {
      bool *value;
      void (*on_change)(void);
    } toggle;
    struct
    {
      int *value;
      int min, max;
      int step, step_repeat;
      const char *suffix;      // e.g. "%", may be NULL
      void (*apply)(int value); // live preview, may be NULL
    } number;
    struct
    {
      int *value;                 // index into labels
      const char *const *labels;
      int count;
      void (*on_change)(void);    // may be NULL
    } choice;
  } u;
} menu_item_t;

typedef struct
{
  const char *title; // fullscreen header; for an application menu, the row
                     // label under which the system menu nests it
  const menu_item_t *items;
  int count;
} menu_def_t;

/*
 * Modal text page: a title and `count` lines, supplied either as a ready-made
 * array or by a callback that renders them on demand.
 *
 * A page never paints itself. There used to be a third option - a body()
 * callback that drew at its own coordinates - and it was a trap: the page had
 * to know where the bottom of the panel was, and anything it put below that
 * was silently lost. Nothing warned, on screen or at build time. The System
 * Information page had reached the last usable row, so the next field added to
 * it would just not have appeared.
 *
 * With lines, the widget owns the layout. It picks the roomy grid when the
 * content fits, the tight one when it does not, and scrolls with a scrollbar
 * past that - so a page can grow by any amount and stays readable, without
 * anyone checking arithmetic against INFO_BODY_BOTTOM.
 */
typedef struct
{
  const char *title;
  const char *const *lines; // may be NULL if line_fn is set
  int count;

  // Renders line `index` instead of reading lines[index]. For text that is
  // generated rather than stored: the config dump is eighty lines, and
  // holding all of it would cost 3 KB of a 64 KB TCM that keeps 16 KB back
  // for stack and heap - the linker rejects that. The callback owns the
  // storage it returns and it need only stay valid until the next call, so
  // one shared buffer does. Takes precedence over lines[].
  const char *(*line_fn)(int index);
} info_page_t;

/*- Prototypes --------------------------------------------------------------*/
void menu_open_fullscreen(const menu_def_t *def);
void menu_open_dialog(const menu_def_t *def); // fullscreen, MENU/LEFT close it
void menu_open_popup(const menu_def_t *def, int x, int y);

// A single row of an existing menu as a shortcut to that one setting, opened
// over whatever is on screen. `editing` starts it with the arrows already on
// the value, which is the point of a shortcut; MENU closes it. Sharing the row
// with the menu is deliberate - a shortcut with its own copy of the labels is a
// shortcut that will one day name a different thing than the menu does.
void menu_open_item_popup(const menu_item_t *item, int x, int y, bool editing);
void menu_open_info(const info_page_t *page);
void menu_close_popups(void);

// MI_ACTION helper: .u.action = { menu_action_info, &some_info_page }
void menu_action_info(const void *arg);

#endif // _MENU_WIDGET_H_
