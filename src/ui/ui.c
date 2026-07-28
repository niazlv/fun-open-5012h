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

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void schedule_redraw(int from)
{
  if (g_redraw_from < 0 || from < g_redraw_from)
    g_redraw_from = from;
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

  schedule_redraw(g_depth - 1);
}

//-----------------------------------------------------------------------------
void ui_pop(void)
{
  if (g_depth <= 1)
    return;

  g_depth--;

  if (g_stack[g_depth].scr->leave)
    g_stack[g_depth].scr->leave(g_stack[g_depth].ctx);

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

  schedule_redraw(0);
}

//-----------------------------------------------------------------------------
void ui_request_redraw(void)
{
  g_top_dirty = true;
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

    for (int i = from; i < g_depth; i++)
    {
      if (g_stack[i].scr->draw)
        g_stack[i].scr->draw(g_stack[i].ctx, true);
    }
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
