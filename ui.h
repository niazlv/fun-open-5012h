/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * UI screen stack
 *
 * Every full-screen or overlay UI element (launcher, running app, popup menu,
 * modal dialog) is a screen on a single stack. The top screen receives input
 * and tick; when a screen is popped, everything it covered is repainted via
 * draw(full=true), so no screen needs to know what is underneath it.
 */

#ifndef _UI_H_
#define _UI_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define UI_MAX_DEPTH   8

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  void (*enter)(void *ctx);              // pushed onto the stack
  void (*leave)(void *ctx);              // popped off the stack
  void (*draw)(void *ctx, bool full);    // full: repaint everything (just pushed or revealed)
  void (*tick)(void *ctx);               // periodic work, top screen only
  bool (*input)(void *ctx, int buttons); // return true if consumed
  bool opaque;                           // covers the whole display
} ui_screen_t;

/*- Prototypes --------------------------------------------------------------*/
void ui_init(void);
void ui_task(void);
bool ui_handle_input(int buttons);
void ui_push(const ui_screen_t *screen, void *ctx);
void ui_pop(void);
void ui_pop_to_root(void);
void ui_request_redraw(void);
const ui_screen_t *ui_top_screen(void);
int ui_depth(void);

#endif // _UI_H_
