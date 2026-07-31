/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
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
  int x, y, w, h;
} ui_rect_t;

typedef struct
{
  void (*enter)(void *ctx);              // pushed onto the stack
  void (*leave)(void *ctx);              // popped off the stack
  void (*draw)(void *ctx, bool full);    // full: repaint everything (just pushed or revealed)
  void (*tick)(void *ctx);               // periodic work, top screen only
  bool (*input)(void *ctx, int buttons); // return true if consumed

  /*
   * The rectangle this screen covers, if it is smaller than the display. What
   * it is for is the repaint that follows the screen off the stack: only what
   * it uncovered has to be painted again, and the screens below paint it
   * through a clip window rather than over the whole panel (see ui_pop).
   *
   * Leave it NULL for a screen that covers everything - a fullscreen menu, a
   * modal page - and for one whose area is not worth the bookkeeping. The
   * whole panel is then repainted, which is what always happened.
   */
  bool (*bounds)(void *ctx, ui_rect_t *rect);
  bool opaque;                           // covers the whole display

  // ...and owns it: a modal the user opened, not an application. Device status
  // painted at fixed coordinates - the battery icon in the top right corner -
  // stays off the screen while one of these is up, instead of landing in the
  // middle of a dialog's border. Everything it missed is drawn when the modal
  // closes, so nothing has to be repainted on its behalf.
  bool modal;
} ui_screen_t;

/*- Prototypes --------------------------------------------------------------*/
void ui_init(void);
void ui_task(void);
bool ui_handle_input(int buttons);
void ui_push(const ui_screen_t *screen, void *ctx);
void ui_pop(void);
void ui_pop_to_root(void);
void ui_request_redraw(void);

// ...and a repaint of every visible screen, for something that changed the
// LAYOUT rather than a value on it (see config.ui_scale)
void ui_request_full_redraw(void);

/*
 * Something changed under the screens that are open: a setting a menu row
 * edited, an action that ran. Marks - it does not schedule anything, because
 * the screen that changed is not on top and must not be painted over the menu
 * that is.
 *
 * What it turns off is the shortcut in ui_pop(): a screen that closes normally
 * only uncovers a rectangle, and the rest of the display is still what it was.
 * That stops being true the moment something below has a new value to show, so
 * the next repaint goes back to being the whole visible stack.
 */
void ui_invalidate_below(void);
const ui_screen_t *ui_top_screen(void);
int ui_depth(void);

// Whether a modal owns the display right now. See ui_screen_t::modal.
bool ui_modal_active(void);

#endif // _UI_H_
