/*
 * Generic menu widget
 *
 * Menus are described by const tables of menu_item_t and rendered by one
 * shared widget: navigation, scrolling, separator skipping, toggles and
 * value editing are implemented once. A submenu is just another menu screen
 * pushed onto the UI stack.
 */

#ifndef _MENU_WIDGET_H_
#define _MENU_WIDGET_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

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
  const char *title; // fullscreen style header, unused for popups
  const menu_item_t *items;
  int count;
} menu_def_t;

/*- Prototypes --------------------------------------------------------------*/
void menu_open_fullscreen(const menu_def_t *def);
void menu_open_popup(const menu_def_t *def, int x, int y);

#endif // _MENU_WIDGET_H_
