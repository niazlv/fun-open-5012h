/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
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
 * text.
 */

#ifndef _MENU_WIDGET_H_
#define _MENU_WIDGET_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// Info page body: first line origin. Line spacing shrinks automatically when
// a page has more lines than fit, so pages with a custom body() can place
// their own text on the same grid.
#define INFO_X         20
#define INFO_Y         45
#define INFO_LINE_H    13

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

// Modal text page: a title, a list of lines, and optionally a body() callback
// for values that have to be computed when the page is opened
typedef struct
{
  const char *title;
  const char *const *lines; // may be NULL
  int count;
  void (*body)(void);       // drawn after the lines, may be NULL
} info_page_t;

/*- Prototypes --------------------------------------------------------------*/
void menu_open_fullscreen(const menu_def_t *def);
void menu_open_popup(const menu_def_t *def, int x, int y);
void menu_open_info(const info_page_t *page);
void menu_close_popups(void);

// MI_ACTION helper: .u.action = { menu_action_info, &some_info_page }
void menu_action_info(const void *arg);

#endif // _MENU_WIDGET_H_
