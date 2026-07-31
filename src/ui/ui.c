/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * UI screen stack
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "ui.h"

/*- Types -------------------------------------------------------------------*/
typedef struct
{
  const ui_screen_t *scr;
  void *ctx;
} ui_entry_t;

/*- Variables ---------------------------------------------------------------*/
static ui_entry_t g_stack[UI_MAX_DEPTH];
static int g_depth = 0;
static int g_redraw_from = -1; // stack index a full repaint starts at, -1 = none
static bool g_top_dirty = false;

/*
 * DAMAGE.
 *
 * A screen that closes normally uncovers its own rectangle and nothing else:
 * the display under it was frozen the whole time it was up, because only the
 * top screen is ticked. So the repaint that follows it off the stack does not
 * have to be the whole panel - the screens below can paint through a clip
 * window the size of what was uncovered, and everything outside it is left
 * standing, which is both correct and the only way the eye does not see a
 * flash every time a submenu closes.
 *
 * g_damage is that rectangle, the union of it if several screens went at once
 * (a popup chain closing). g_damage_clip says the pending repaint may be
 * limited to it - false means the whole panel, which is what a push, a
 * fullscreen screen, or a layout change asks for.
 *
 * g_below_stale is the one thing the rectangle cannot cover: a menu row that
 * edited a setting leaves the application below showing the old value, and
 * repainting the hole the menu left would not put that right. It is sticky
 * until a repaint of the whole panel clears it.
 */
static ui_rect_t g_damage;
static bool g_damage_clip = false;
static bool g_below_stale = false;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void schedule_redraw(int from)
{
  if (g_redraw_from < 0 || from < g_redraw_from)
    g_redraw_from = from;
}

//-----------------------------------------------------------------------------
static void damage_add(const ui_rect_t *r)
{
  int x1 = g_damage.x + g_damage.w;
  int y1 = g_damage.y + g_damage.h;

  if (r->x < g_damage.x)
    g_damage.x = r->x;

  if (r->y < g_damage.y)
    g_damage.y = r->y;

  if (r->x + r->w > x1)
    x1 = r->x + r->w;

  if (r->y + r->h > y1)
    y1 = r->y + r->h;

  g_damage.w = x1 - g_damage.x;
  g_damage.h = y1 - g_damage.y;
}

//-----------------------------------------------------------------------------
// Everything below the topmost opaque screen is covered and can be skipped
static int base_visible_index(void)
{
  for (int i = g_depth - 1; i > 0; i--)
  {
    if (g_stack[i].scr->opaque)
      return i;
  }

  return 0;
}

//-----------------------------------------------------------------------------
void ui_init(void)
{
  g_depth = 0;
  g_redraw_from = -1;
  g_top_dirty = false;
  g_damage_clip = false;
  g_below_stale = false;
}

//-----------------------------------------------------------------------------
void ui_push(const ui_screen_t *screen, void *ctx)
{
  if (g_depth >= UI_MAX_DEPTH)
    return;

  g_stack[g_depth].scr = screen;
  g_stack[g_depth].ctx = ctx;
  g_depth++;

  if (screen->enter)
    screen->enter(ctx);

  // A screen that arrives is not a screen that left: whatever was uncovered
  // this pass has been joined by something that has to be painted in full,
  // and the repaint below is not the one to do it through a keyhole
  g_damage_clip = false;

  schedule_redraw(g_depth - 1);
}

//-----------------------------------------------------------------------------
void ui_pop(void)
{
  const ui_entry_t *e;
  ui_rect_t r;

  if (g_depth <= 1)
    return;

  e = &g_stack[--g_depth];

  // Before leave(): a screen is entitled to let go of its instance there, and
  // the rectangle is read out of it
  if (g_below_stale || NULL == e->scr->bounds || !e->scr->bounds(e->ctx, &r))
    g_damage_clip = false;      // the whole panel, as it always was
  else if (g_redraw_from < 0)
  {
    g_damage = r;               // the first area uncovered this pass
    g_damage_clip = true;
  }
  else if (g_damage_clip)
    damage_add(&r);             // a chain of popups closing at once

  if (e->scr->leave)
    e->scr->leave(e->ctx);

  schedule_redraw(base_visible_index());
}

//-----------------------------------------------------------------------------
void ui_pop_to_root(void)
{
  while (g_depth > 1)
  {
    g_depth--;

    if (g_stack[g_depth].scr->leave)
      g_stack[g_depth].scr->leave(g_stack[g_depth].ctx);
  }

  g_damage_clip = false;

  schedule_redraw(0);
}

//-----------------------------------------------------------------------------
void ui_request_redraw(void)
{
  g_top_dirty = true;
}

//-----------------------------------------------------------------------------
// Everything visible, from the ground up, exactly as a pop repaints what it
// uncovers. For a screen that has changed SIZE rather than contents: a redraw of
// the top alone leaves whatever it used to cover standing where it was.
void ui_request_full_redraw(void)
{
  g_damage_clip = false;
  schedule_redraw(base_visible_index());
}

//-----------------------------------------------------------------------------
void ui_invalidate_below(void)
{
  g_below_stale = true;
  g_damage_clip = false;
}

//-----------------------------------------------------------------------------
const ui_screen_t *ui_top_screen(void)
{
  return g_depth ? g_stack[g_depth - 1].scr : NULL;
}

//-----------------------------------------------------------------------------
int ui_depth(void)
{
  return g_depth;
}

//-----------------------------------------------------------------------------
bool ui_modal_active(void)
{
  return g_depth && g_stack[g_depth - 1].scr->modal;
}

//-----------------------------------------------------------------------------
bool ui_handle_input(int buttons)
{
  if (0 == g_depth)
    return false;

  ui_entry_t top = g_stack[g_depth - 1];

  if (top.scr->input)
    return top.scr->input(top.ctx, buttons);

  return false;
}

//-----------------------------------------------------------------------------
void ui_task(void)
{
  if (0 == g_depth)
    return;

  if (g_redraw_from >= 0)
  {
    int from = g_redraw_from;

    g_redraw_from = -1;
    g_top_dirty = false;

    if (from >= g_depth)
      from = g_depth - 1;

    // Through the hole a closed screen left, when that is all this repaint is
    // for. Every screen still draws itself in full and does not know the
    // difference - what falls outside the hole is dropped a layer down, in
    // the display driver, where the cost of it is the bus and not the code.
    if (g_damage_clip)
    {
      lcd_set_clip(g_damage.x, g_damage.y, g_damage.w, g_damage.h);
    }
    else
    {
      // The whole visible stack is being painted, so whatever changed under
      // it is on the screen again by the end of this loop
      g_below_stale = false;
    }

    for (int i = from; i < g_depth; i++)
    {
      if (g_stack[i].scr->draw)
        g_stack[i].scr->draw(g_stack[i].ctx, true);
    }

    lcd_clip_none();
    g_damage_clip = false;
  }
  else if (g_top_dirty)
  {
    g_top_dirty = false;

    if (g_stack[g_depth - 1].scr->draw)
      g_stack[g_depth - 1].scr->draw(g_stack[g_depth - 1].ctx, false);
  }

  if (g_stack[g_depth - 1].scr->tick)
    g_stack[g_depth - 1].scr->tick(g_stack[g_depth - 1].ctx);
}
