/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Generic menu widget
 */

/*- Includes ----------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "buttons.h"
#include "config.h"
#include "ui.h"
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define MENU_MAX_OPEN         6

// Popup style (context/system menu). The geometry is per text size and lives in
// g_ui_metrics below; these are the colours and the one margin both sizes share.
#define POPUP_MARGIN          2
#define POPUP_EDIT_BG         LCD_COLOR(255, 200, 0)
#define POPUP_EDIT_TEXT       LCD_COLOR(0, 0, 0)
#define POPUP_BG              LCD_COLOR(240, 240, 240)
#define POPUP_BORDER          LCD_COLOR(128, 128, 128)
#define POPUP_TEXT            LCD_COLOR(0, 0, 0)
#define POPUP_SEL_BG          LCD_COLOR(0, 120, 215)
#define POPUP_SEL_TEXT        LCD_COLOR(255, 255, 255)
#define POPUP_SEPARATOR       LCD_COLOR(160, 160, 160)
#define POPUP_SHADOW          LCD_COLOR(64, 64, 64)

// Info page style (help and info dialogs)
#define INFO_BG               LCD_COLOR(240, 240, 240)
#define INFO_FG               LCD_COLOR(0, 0, 0)
#define INFO_TITLE            LCD_COLOR(0, 70, 150)
#define INFO_DIM              LCD_COLOR(110, 110, 110)
#define INFO_HEAD_BG          LCD_COLOR(200, 216, 236)
#define INFO_BAR_BG           LCD_COLOR(206, 206, 212)
#define INFO_BAR_FG           LCD_COLOR(0, 120, 215)
#define INFO_TITLE_Y          20
#define INFO_FOOTER_Y         (LCD_HEIGHT - 25)
#define INFO_BODY_BOTTOM      (INFO_FOOTER_Y - 11) // last line origin
#define INFO_TEXT_R           (LCD_WIDTH - 24) // right edge of the text column
#define INFO_BAR_X            (LCD_WIDTH - 20)
#define INFO_BAR_W            4

// A page that scrolls gets a tighter grid: four more lines per screen, and
// the pages that need them are the long ones
#define INFO_SCROLL_Y         40
#define INFO_SCROLL_LINE_H    10

// Fullscreen style (launcher)
#define FS_TITLE_H            40
#define FS_FOOTER_H           30
#define FS_ITEM_H             30
#define FS_MARGIN             10
#define FS_CONTENT_Y          FS_TITLE_H
#define FS_CONTENT_H          (LCD_HEIGHT - FS_TITLE_H - FS_FOOTER_H)
#define FS_VISIBLE_ITEMS      (FS_CONTENT_H / FS_ITEM_H)
#define FS_BG                 LCD_COLOR(20, 20, 40)
#define FS_TEXT               LCD_COLOR(255, 255, 255)
#define FS_SEL_BG             LCD_COLOR(0, 120, 215)
#define FS_SEL_TEXT           LCD_COLOR(255, 255, 255)
#define FS_BORDER             LCD_COLOR(100, 100, 100)
#define FS_TITLE_COLOR        LCD_COLOR(255, 200, 0)

#define FW_SMALL              6 // font_6x8 glyph width
#define FW_LARGE              8 // terminus_8x16 glyph width

/*- Types -------------------------------------------------------------------*/
/*
 * Everything about a popup that the text size decides. A table rather than an
 * arithmetic scale, because none of these numbers scale by the same factor: the
 * font is 33% wider and 100% taller, the row grows by the font's height and not
 * by its width, and the width is what the longest LABEL needs rather than a
 * multiple of anything.
 *
 * Widths: 180 is what the menus were drawn at, and 244 is the same rows at 8 px
 * a character - the longest label in the firmware is "Arrange layout..." at 17,
 * and a row has to hold a value ("1000x", "Oscilloscope") hard against its right
 * edge on the same line. Submenus stay narrower than their parent, as before.
 *
 * text_dy centres the glyph in the row: (item_h - font height) / 2, written out
 * so that a row can be nudged without recomputing it.
 */
typedef struct
{
  const Font *font;
  uint8_t char_w;
  uint8_t item_h;
  uint8_t text_dy;
  uint16_t width;
  uint16_t sub_width;
} UiMetrics;
typedef struct
{
  bool used;
  bool fullscreen;
  bool is_submenu;
  const char *title;
  const menu_item_t *items;
  int count;
  int sel;
  int scroll;
  int vis;      // rows that fit on screen; count > vis means the menu scrolls
  bool editing; // value edit focus: arrows adjust the value, not the cursor
  // A one-row shortcut (menu_open_item_popup): MENU closes it outright rather
  // than stepping out of the value first. There is nothing else on it to step
  // out to, and a shortcut that takes two presses to dismiss is not one.
  bool shortcut;
  int x, y, w, h;
} menu_inst_t;

// Info page state. The page itself is const in flash; where we are in it is
// not, and one instance is enough because an info page is the end of the
// road: it is modal, nothing opens on top of it, and any key that is not a
// scroll key closes it.
typedef struct
{
  const info_page_t *page;
  int count;   // text lines, 0 for a page that only has a body()
  int top;     // first visible line
  int vis;     // DISPLAY rows on one screen - a line may take more than one
  int line_h;
  int y0;      // origin of the first visible line
  int heads;   // section headings in the page, 0 = no sections
  bool large;  // body in the 8x16 font
  int cols;    // characters that fit one row, and one continuation row
  int cols_cont;
  int rows;    // display rows the whole page takes, wrapping counted
  int max_top; // the last line that may be at the top: the end of the page
} info_inst_t;

/*- Variables ---------------------------------------------------------------*/
static menu_inst_t g_inst[MENU_MAX_OPEN];
static info_inst_t g_info;

static const UiMetrics g_ui_metrics[UI_SCALE_COUNT] =
{
  [UI_SCALE_NORMAL] = { FONT_SMALL, FW_SMALL, 20, 6, 180, 160 },
  [UI_SCALE_LARGE]  = { FONT_LARGE, FW_LARGE, 26, 5, 244, 224 },
};

/*- Forward Declarations ----------------------------------------------------*/
static void menu_leave(void *ctx);
static void menu_draw(void *ctx, bool full);
static bool menu_input(void *ctx, int buttons);
static bool menu_bounds(void *ctx, ui_rect_t *rect);

//-----------------------------------------------------------------------------
static const ui_screen_t menu_screen_popup =
{
  .leave  = menu_leave,
  .draw   = menu_draw,
  .input  = menu_input,
  .bounds = menu_bounds,
  .opaque = false,
};

// The popup above leaves the top right corner of the screen alone, so it is
// not marked modal and the battery icon keeps updating under it. This one
// covers the display end to end.
static const ui_screen_t menu_screen_fs =
{
  .leave  = menu_leave,
  .draw   = menu_draw,
  .input  = menu_input,
  .opaque = true,
  .modal  = true,
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The size the menus are being drawn at. Read per draw and not cached: the
// setting is changed FROM a menu, and the menu it is changed from has to come
// back at the new size on the redraw that follows the keypress.
static const UiMetrics *um(void)
{
  int s = config.ui_scale;

  return &g_ui_metrics[((unsigned)s < UI_SCALE_COUNT) ? s : UI_SCALE_NORMAL];
}

//-----------------------------------------------------------------------------
static menu_inst_t *alloc_inst(void)
{
  for (int i = 0; i < MENU_MAX_OPEN; i++)
  {
    if (!g_inst[i].used)
    {
      memset(&g_inst[i], 0, sizeof(menu_inst_t));
      g_inst[i].used = true;
      return &g_inst[i];
    }
  }

  return NULL;
}

//-----------------------------------------------------------------------------
static int first_selectable(const menu_inst_t *m)
{
  for (int i = 0; i < m->count; i++)
  {
    if (m->items[i].kind != MI_SEPARATOR)
      return i;
  }

  return 0;
}

//-----------------------------------------------------------------------------
// The text shown on the right edge of an item row, "" if none
static const char *item_value_str(const menu_item_t *it, char *buf, int size)
{
  if (MI_TOGGLE == it->kind)
    return *it->u.toggle.value ? "ON" : "OFF";

  if (MI_TOGGLE_INV == it->kind)
    return *it->u.toggle.value ? "OFF" : "ON";

  if (MI_NUMBER == it->kind)
  {
    snprintf(buf, size, "%d%s", *it->u.number.value,
        it->u.number.suffix ? it->u.number.suffix : "");
    return buf;
  }

  if (MI_CHOICE == it->kind)
  {
    int v = *it->u.choice.value;

    if (v < 0 || v >= it->u.choice.count)
      return "?";

    return it->u.choice.labels[v];
  }

  if (MI_SUBMENU == it->kind)
    return ">";

  return "";
}

//-----------------------------------------------------------------------------
static void popup_draw_row(const menu_inst_t *m, int index)
{
  const UiMetrics *u = um();
  const menu_item_t *it = &m->items[index];
  int x = m->x + POPUP_MARGIN;
  int y = m->y + POPUP_MARGIN + (index - m->scroll) * u->item_h;
  int w = m->w - 2 * POPUP_MARGIN;
  bool selected = (index == m->sel);
  char buf[16];

  if (index < m->scroll || index >= m->scroll + m->vis)
    return;

  if (MI_SEPARATOR == it->kind)
  {
    lcd_fill_rect(x, y, w, u->item_h, POPUP_BG);
    lcd_fill_rect(x + 5, y + u->item_h / 2 - 1, w - 10, 1, POPUP_SEPARATOR);
    return;
  }

  uint16_t bg = selected ? POPUP_SEL_BG : POPUP_BG;
  uint16_t fg = selected ? POPUP_SEL_TEXT : POPUP_TEXT;

  lcd_fill_rect(x, y, w, u->item_h, bg);
  lcd_set_font(u->font);
  lcd_set_color(bg, fg);
  lcd_puts(x + 8, y + u->text_dy, it->label);

  const char *value = item_value_str(it, buf, sizeof(buf));

  if (value[0])
  {
    int vx = x + w - 8 - (int)strlen(value) * u->char_w;

    // Edit focus: the VALUE inverts, saying the arrows now belong to it
    if (selected && m->editing)
    {
      lcd_fill_rect(vx - 3, y + 3, (int)strlen(value) * u->char_w + 6,
          u->item_h - 6, POPUP_EDIT_BG);
      lcd_set_color(POPUP_EDIT_BG, POPUP_EDIT_TEXT);
    }

    lcd_puts(vx, y + u->text_dy, value);
  }
}

//-----------------------------------------------------------------------------
static void popup_draw(const menu_inst_t *m, bool full)
{
  if (full)
    lcd_fill_rect(m->x + 2, m->y + 2, m->w, m->h, POPUP_SHADOW);

  lcd_fill_rect(m->x, m->y, m->w, m->h, POPUP_BG);
  lcd_draw_rect(m->x, m->y, m->w - 1, m->h - 1, POPUP_BORDER);

  for (int i = m->scroll; i < m->scroll + m->vis && i < m->count; i++)
    popup_draw_row(m, i);

  if (m->count > m->vis)
  {
    // The arrows keep the small font at both sizes: they mark the edge of the
    // list rather than being read, and a large one would sit on the values
    lcd_set_font(FONT_SMALL);
    lcd_set_color(POPUP_BG, POPUP_TEXT);

    if (m->scroll > 0)
      lcd_puts(m->x + m->w - 14, m->y + 3, "^");

    if (m->scroll + m->vis < m->count)
      lcd_puts(m->x + m->w - 14, m->y + m->h - 11, "v");
  }
}

//-----------------------------------------------------------------------------
static void fs_draw_item(const menu_inst_t *m, int index)
{
  const menu_item_t *it = &m->items[index];
  int item_y = FS_CONTENT_Y + (index - m->scroll) * FS_ITEM_H;
  bool selected = (index == m->sel);
  char buf[16];

  if (item_y < FS_CONTENT_Y || item_y >= FS_CONTENT_Y + FS_CONTENT_H)
    return;

  if (MI_SEPARATOR == it->kind)
  {
    lcd_fill_rect(FS_MARGIN + 5, item_y + FS_ITEM_H / 2 - 1,
        LCD_WIDTH - 2 * FS_MARGIN - 10, 2, FS_BORDER);
    return;
  }

  uint16_t bg = selected ? FS_SEL_BG : FS_BG;
  uint16_t fg = selected ? FS_SEL_TEXT : FS_TEXT;

  lcd_fill_rect(FS_MARGIN, item_y, LCD_WIDTH - 2 * FS_MARGIN, FS_ITEM_H, bg);

  if (selected)
    lcd_draw_rect(FS_MARGIN, item_y, LCD_WIDTH - 2 * FS_MARGIN - 1, FS_ITEM_H - 1, fg);

  lcd_set_font(FONT_LARGE);
  lcd_set_color(bg, fg);
  lcd_puts(FS_MARGIN + 5, item_y + 2, it->label);

  const char *value = item_value_str(it, buf, sizeof(buf));

  if (value[0])
  {
    int vx = LCD_WIDTH - FS_MARGIN - 5 - strlen(value) * FW_LARGE;

    // Edit focus: the VALUE inverts, saying the arrows now belong to it
    if (selected && m->editing)
    {
      lcd_fill_rect(vx - 4, item_y + 4, strlen(value) * FW_LARGE + 8, 22,
          POPUP_EDIT_BG);
      lcd_set_color(POPUP_EDIT_BG, POPUP_EDIT_TEXT);
    }

    lcd_puts(vx, item_y + 7, value);
  }

  if (it->desc && it->desc[0])
  {
    lcd_set_font(FONT_SMALL);
    lcd_set_color(bg, fg);
    lcd_puts(FS_MARGIN + 5, item_y + 18, it->desc);
  }
}

//-----------------------------------------------------------------------------
static void fs_draw(const menu_inst_t *m, bool full)
{
  if (full)
  {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, FS_BG);
    lcd_draw_rect(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, FS_BORDER);
    lcd_draw_rect(1, 1, LCD_WIDTH - 3, LCD_HEIGHT - 3, FS_BORDER);

    if (m->title)
    {
      lcd_set_font(FONT_LARGE);
      lcd_set_color(FS_BG, FS_TITLE_COLOR);
      lcd_puts((LCD_WIDTH - strlen(m->title) * FW_LARGE) / 2, 10, m->title);
    }

    lcd_fill_rect(FS_MARGIN, FS_TITLE_H - 2, LCD_WIDTH - 2 * FS_MARGIN, 2, FS_BORDER);

    int footer_y = LCD_HEIGHT - FS_FOOTER_H;
    lcd_fill_rect(FS_MARGIN, footer_y, LCD_WIDTH - 2 * FS_MARGIN, 2, FS_BORDER);
    lcd_set_font(FONT_SMALL);
    lcd_set_color(FS_BG, FS_TEXT);
    lcd_puts(FS_MARGIN, footer_y + 5, "UP/DOWN: Navigate  MODE: Select/Edit");
    lcd_puts(FS_MARGIN, footer_y + 15, "LEFT: Back  Edit: arrows set, MODE done");
  }

  lcd_fill_rect(FS_MARGIN, FS_CONTENT_Y, LCD_WIDTH - 2 * FS_MARGIN, FS_CONTENT_H, FS_BG);

  int end = m->scroll + FS_VISIBLE_ITEMS;

  if (end > m->count)
    end = m->count;

  for (int i = m->scroll; i < end; i++)
    fs_draw_item(m, i);

  if (m->count > FS_VISIBLE_ITEMS)
  {
    lcd_set_font(FONT_SMALL);
    lcd_set_color(FS_BG, FS_TEXT);

    if (m->scroll > 0)
      lcd_puts(LCD_WIDTH - 20, FS_CONTENT_Y + 5, "^");

    if (m->scroll + FS_VISIBLE_ITEMS < m->count)
      lcd_puts(LCD_WIDTH - 20, FS_CONTENT_Y + FS_CONTENT_H - 15, "v");
  }
}

//-----------------------------------------------------------------------------
static void ensure_visible(menu_inst_t *m)
{
  if (m->vis <= 0)
    return;

  if (m->sel < m->scroll)
    m->scroll = m->sel;
  else if (m->sel >= m->scroll + m->vis)
    m->scroll = m->sel - m->vis + 1;
}

//-----------------------------------------------------------------------------
static void move_sel(menu_inst_t *m, int dir)
{
  int i = m->sel;

  do
  {
    i += dir;
  } while (i >= 0 && i < m->count && MI_SEPARATOR == m->items[i].kind);

  if (i < 0 || i >= m->count)
    return;

  m->sel = i;
  ensure_visible(m);
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
static void number_adjust(const menu_item_t *it, int delta)
{
  int v = *it->u.number.value + delta;

  if (v < it->u.number.min)
    v = it->u.number.min;
  if (v > it->u.number.max)
    v = it->u.number.max;

  if (v == *it->u.number.value)
    return;

  *it->u.number.value = v;

  if (it->u.number.apply)
    it->u.number.apply(v);

  ui_invalidate_below(); // the screen under the menu is showing the old value
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
static void choice_adjust(const menu_item_t *it, int dir)
{
  int count = it->u.choice.count;
  int v = *it->u.choice.value + dir;

  // Wrap, and recover from an out-of-range value (e.g. stale config)
  v = ((v % count) + count) % count;

  *it->u.choice.value = v;

  if (it->u.choice.on_change)
    it->u.choice.on_change();

  ui_invalidate_below();
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
// How wide the box is and how many rows fit in it, for the text size in force
// now. Split out of the open because the size can change while the menu is up -
// it is changed FROM a menu - and then every open popup has to be laid out
// again around rows that are suddenly taller. See menu_relayout().
static void popup_layout(menu_inst_t *m)
{
  const UiMetrics *u = um();
  int max_vis = (LCD_HEIGHT - 2 * POPUP_MARGIN) / u->item_h;

  m->w = m->is_submenu ? u->sub_width : u->width;
  m->vis = (m->count < max_vis) ? m->count : max_vis;
  m->h = m->vis * u->item_h + 2 * POPUP_MARGIN;

  // A popup is opened at a point that suits what it belongs to - a status-line
  // slot, a parent row - and at the large size it is 64 px wider than the point
  // was chosen for. Pulling it back onto the screen is what keeps a shortcut
  // from opening half off the right edge; x can go negative doing that, which a
  // box drawn from x=0 with a shadow beside it would notice.
  if (m->x + m->w > LCD_WIDTH)
    m->x = LCD_WIDTH - m->w;
  if (m->x < 0)
    m->x = 0;
  if (m->y + m->h > LCD_HEIGHT)
    m->y = LCD_HEIGHT - m->h;
  if (m->y < 0)
    m->y = 0;
}

//-----------------------------------------------------------------------------
static void popup_open_common(const menu_item_t *items, int count, int x, int y,
    bool is_submenu, bool editing)
{
  menu_inst_t *m = alloc_inst();

  if (NULL == m)
    return;

  m->fullscreen = false;
  m->is_submenu = is_submenu;
  m->items = items;
  m->count = count;
  m->x = x;
  m->y = y;
  m->sel = 0;

  popup_layout(m);

  m->sel = first_selectable(m);

  // Straight into editing when the caller asks: a one-row popup over a choice
  // is a shortcut, and making the user press MODE to start turning the value
  // would give the shortcut a step of its own
  m->editing = editing && (MI_CHOICE == m->items[m->sel].kind ||
      MI_NUMBER == m->items[m->sel].kind);
  m->shortcut = editing;

  ui_push(&menu_screen_popup, m);
}

//-----------------------------------------------------------------------------
static void open_submenu(const menu_inst_t *parent, const menu_item_t *it)
{
  int x, y;

  if (parent->fullscreen)
  {
    x = 60;
    y = 60;
  }
  else
  {
    x = parent->x + parent->w - 5;
    y = parent->y + POPUP_MARGIN + (parent->sel - parent->scroll) * um()->item_h;
  }

  popup_open_common(it->u.submenu.items, it->u.submenu.count, x, y, true, false);
}

//-----------------------------------------------------------------------------
void menu_open_fullscreen(const menu_def_t *def)
{
  menu_inst_t *m = alloc_inst();

  if (NULL == m)
    return;

  m->fullscreen = true;
  m->vis = FS_VISIBLE_ITEMS;
  m->title = def->title;
  m->items = def->items;
  m->count = def->count;
  m->sel = first_selectable(m);
  ensure_visible(m);

  ui_push(&menu_screen_fs, m);
}

//-----------------------------------------------------------------------------
// Fullscreen menu that closes like a dialog: MENU and LEFT pop it. The plain
// fullscreen variant is for the launcher root, where MENU has to fall through
// so the system menu can open on top.
void menu_open_dialog(const menu_def_t *def)
{
  menu_inst_t *m = alloc_inst();

  if (NULL == m)
    return;

  m->fullscreen = true;
  m->is_submenu = true; // gives it the submenu close behavior
  m->vis = FS_VISIBLE_ITEMS;
  m->title = def->title;
  m->items = def->items;
  m->count = def->count;
  m->sel = first_selectable(m);
  ensure_visible(m);

  ui_push(&menu_screen_fs, m);
}

//-----------------------------------------------------------------------------
void menu_open_popup(const menu_def_t *def, int x, int y)
{
  popup_open_common(def->items, def->count, x, y, false, false);
}

//-----------------------------------------------------------------------------
// One row of an existing menu, on its own, as a shortcut to that one setting.
// The row is the same object the menu holds, so the shortcut and the menu can
// never disagree about what the setting is called or what it may be.
void menu_open_item_popup(const menu_item_t *item, int x, int y, bool editing)
{
  popup_open_common(item, 1, x, y, false, editing);
}

//-----------------------------------------------------------------------------
// Info page: modal fullscreen text.
//
// It scrolls, so there is no length a page has to stay under - a help text
// that grew into a manual is still readable to its last line. A line that
// starts with INFO_HEAD is a section heading: drawn as a band, named under
// the title while it is the section on screen, and a jump target, which is
// what makes six hundred lines navigable at all.
//
// A page that fits keeps the layout and the contract it always had: roomy
// spacing, no scrollbar, any button closes it.
//-----------------------------------------------------------------------------
static bool info_is_head(const char *line)
{
  return line && INFO_HEAD[0] == line[0];
}

//-----------------------------------------------------------------------------
// One line of a page, however the page supplies them. A generated page owns
// the storage it hands back and only promises it until the next call, so the
// result must be used before asking for another - every caller here does.
static const char *info_line(const info_page_t *page, int index)
{
  if (page->line_fn)
    return page->line_fn(index);

  return page->lines ? page->lines[index] : "";
}

//-----------------------------------------------------------------------------
// The heading that line belongs under, -1 if it is above the first one
static int info_head_at(const info_inst_t *in, int line)
{
  for (int i = line; i >= 0; i--)
  {
    if (info_is_head(info_line(in->page, i)))
      return i;
  }

  return -1;
}

/*
 * WRAPPING, which is what makes a large body possible at all.
 *
 * The pages are written to the width of the 6x8 font - about 46 characters - and
 * lcd_puts does not wrap, it drops the glyphs that fall off the panel. So the
 * first attempt at this only enlarged a page whose every line already fitted the
 * 8x16 column, and since nothing in the firmware is written that narrow, the
 * setting did nothing here: every page stayed small. Truncating instead was never
 * an option - a diagnostic readout cut off mid-number is worse than a small one.
 *
 * So a line that does not fit is broken and continued on the next row. That
 * costs the exact display-row count, which the scrollbar and the end-of-page
 * clamp both need, so it is walked once at open (see info_measure).
 */
#define INFO_CONT_INDENT   8     // continuation rows, so a break reads as one

//-----------------------------------------------------------------------------
// How many characters of `s` go on one row: everything, if it fits; otherwise up
// to the last space at or before the limit, and a hard break at the limit if
// there is no space to use (a long number, a path, a hex dump).
static int info_wrap_at(const char *s, int cols)
{
  int n = (int)strlen(s);

  if (n <= cols)
    return n;

  for (int i = cols; i > 0; i--)
  {
    if (' ' == s[i])
      return i;
  }

  return cols;
}

//-----------------------------------------------------------------------------
// Display rows one logical line takes
static int info_rows_of(const info_inst_t *in, const char *line)
{
  int rows = 0;

  if (!line)
    return 1;

  if (info_is_head(line))
    line++;

  do
  {
    int at = info_wrap_at(line, rows ? in->cols_cont : in->cols);

    line += at;

    while (' ' == *line)
      line++;

    rows++;
  } while (*line && rows < 8);   // a runaway line is clipped, not endless

  return rows;
}

//-----------------------------------------------------------------------------
// One logical line, wrapped. Returns the rows it TAKES, whether or not they were
// all drawn: a row whose glyphs would reach past y_max is counted and skipped, so
// the last line on a screen cannot paint over the key hints under the body, and
// the caller still walks the page by the right number of rows.
static int info_draw_line(const info_inst_t *in, const char *line, int y,
    int y_max)
{
  bool head = info_is_head(line);
  int gh = in->large ? 16 : 8;
  int rows = 0;

  if (!line)
    return 1;

  if (head)
  {
    line++;
    lcd_set_color(INFO_HEAD_BG, INFO_TITLE);
  }

  do
  {
    int cols = rows ? in->cols_cont : in->cols;
    int at = info_wrap_at(line, cols);

    if (y + gh <= y_max)
    {
      int x = INFO_X + (rows ? INFO_CONT_INDENT : 0);
      char buf[64];
      int n = (at > (int)sizeof(buf) - 1) ? (int)sizeof(buf) - 1 : at;

      memcpy(buf, line, n);
      buf[n] = 0;

      if (head)
        lcd_fill_rect(INFO_X - 5, y - 1, INFO_TEXT_R - INFO_X + 10,
            in->large ? 18 : 10, INFO_HEAD_BG);

      lcd_puts(x, y, buf);
    }

    line += at;

    while (' ' == *line)
      line++;

    y += in->line_h;
    rows++;
  } while (*line && rows < 8);

  if (head)
    lcd_set_color(INFO_BG, INFO_FG);

  return rows;
}

//-----------------------------------------------------------------------------
// Thumb size is the share of the page on screen, so a 600-line page says so
// before a single key is pressed. In display rows, not lines: on a wrapped page
// the two are different numbers and it is rows that are on the screen.
static void info_draw_bar(const info_inst_t *in)
{
  int y = in->y0 - 2;
  int h = INFO_BODY_BOTTOM + 9 - y;
  int thumb = h * in->vis / in->rows;
  int pos;

  if (thumb < 8)
    thumb = 8;

  pos = (h - thumb) * in->top / in->max_top;

  lcd_fill_rect(INFO_BAR_X, y, INFO_BAR_W, h, INFO_BAR_BG);
  lcd_fill_rect(INFO_BAR_X, y + pos, INFO_BAR_W, thumb, INFO_BAR_FG);
}

//-----------------------------------------------------------------------------
// Where we are, in the two places that help: the section name beside the
// title, and how far through the page in the footer
static void info_draw_position(const info_inst_t *in)
{
  const char *title = in->page->title ? in->page->title : "";
  int head = in->heads ? info_head_at(in, in->top) : -1;
  const char *name = (head >= 0) ? info_line(in->page, head) + 1 : "";
  int erase_x = INFO_X + (int)strlen(title) * FW_LARGE + 8;
  char buf[8];

  lcd_set_font(FONT_SMALL);
  lcd_set_color(INFO_BG, INFO_DIM);

  lcd_fill_rect(erase_x, INFO_TITLE_Y, INFO_TEXT_R - erase_x, 12, INFO_BG);
  lcd_puts(INFO_TEXT_R - (int)strlen(name) * FW_SMALL, INFO_TITLE_Y + 4, name);

  snprintf(buf, sizeof(buf), "%d%%", 100 * in->top / in->max_top);
  lcd_fill_rect(INFO_TEXT_R - 26, INFO_FOOTER_Y, 26, 8, INFO_BG);
  lcd_puts(INFO_TEXT_R - (int)strlen(buf) * FW_SMALL, INFO_FOOTER_Y, buf);
}

//-----------------------------------------------------------------------------
// Everything that changes when the page scrolls, and nothing else: the frame,
// the title and the key hints are painted once by the full draw
static void info_draw_body(const info_inst_t *in)
{
  int y = in->y0 - 2;

  lcd_fill_rect(11, y, LCD_WIDTH - 22, INFO_BODY_BOTTOM + 9 - y, INFO_BG);

  lcd_set_font(in->large ? FONT_LARGE : FONT_SMALL);
  lcd_set_color(INFO_BG, INFO_FG);

  // Rows, not lines: a wrapped line takes more than one, and the screen holds
  // what it holds. The line that only half fits at the bottom is drawn - its
  // tail is clipped by the body's own rectangle, and half a visible line is
  // what says there is more of the page below.
  for (int i = in->top, row = 0; i < in->count && row < in->vis; i++)
    row += info_draw_line(in, info_line(in->page, i),
        in->y0 + row * in->line_h, INFO_BODY_BOTTOM + 9);

  if (in->max_top > 0)
  {
    info_draw_bar(in);
    info_draw_position(in);
  }
}

//-----------------------------------------------------------------------------
static void info_draw(void *ctx, bool full)
{
  const info_inst_t *in = (const info_inst_t *)ctx;
  const info_page_t *p = in->page;

  if (full)
  {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, INFO_BG);
    lcd_draw_rect(10, 10, LCD_WIDTH - 21, LCD_HEIGHT - 21, INFO_FG);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(INFO_BG, INFO_TITLE);
    lcd_puts(INFO_X, INFO_TITLE_Y, p->title ? p->title : "");

    lcd_set_font(FONT_SMALL);
    lcd_set_color(INFO_BG, INFO_DIM);

    if (in->count <= in->vis)
      lcd_puts(INFO_X, INFO_FOOTER_Y, "Press any button to close");
    else if (in->heads)
      lcd_puts(INFO_X, INFO_FOOTER_Y,
          "U/D line  L/R page  TRIG sect  MODE close");
    else
      lcd_puts(INFO_X, INFO_FOOTER_Y, "U/D line  L/R page  MODE close");
  }

  info_draw_body(in);
}

//-----------------------------------------------------------------------------
static void info_scroll(info_inst_t *in, int delta)
{
  int max = in->count - in->vis;
  int top = in->top + delta;

  if (top > max)
    top = max;

  if (top < 0)
    top = 0;

  if (top == in->top)
    return;

  in->top = top;
  ui_request_redraw();
}

//-----------------------------------------------------------------------------
// To the heading above or below the one on screen, whichever way dir points
static void info_jump_section(info_inst_t *in, int dir)
{
  int i;

  for (i = in->top + dir; i > 0 && i < in->count; i += dir)
  {
    if (info_is_head(info_line(in->page, i)))
      break;
  }

  if (i >= in->count)
    i = in->count - 1; // past the last section: the end of the page

  if (i < 0)
    i = 0;

  info_scroll(in, i - in->top);
}

//-----------------------------------------------------------------------------
static bool info_input(void *ctx, int buttons)
{
  info_inst_t *in = (info_inst_t *)ctx;
  int page = (in->vis > 2) ? in->vis - 2 : 1;

  if (0 == buttons)
    return true;

  // Only a page that scrolls spends the arrows on scrolling; on one that
  // fits, every button still closes it
  if (in->count > in->vis)
  {
    if (buttons & (BTN_UP | BTN_DOWN))
    {
      info_scroll(in, (buttons & BTN_UP) ? -1 : 1);
      return true;
    }

    if (buttons & (BTN_LEFT | BTN_RIGHT))
    {
      info_scroll(in, (buttons & BTN_LEFT) ? -page : page);
      return true;
    }

    if (buttons & (BTN_TRIG_UP | BTN_TRIG_DOWN))
    {
      int dir = (buttons & BTN_TRIG_UP) ? -1 : 1;

      // Without sections there is nothing to jump to, and the pair of keys
      // is more useful as a second page up/down than as nothing at all
      if (in->heads)
        info_jump_section(in, dir);
      else
        info_scroll(in, dir * page);

      return true;
    }
  }

  if (!(buttons & BTN_REPEAT))
    ui_pop();

  return true;
}

static const ui_screen_t info_screen =
{
  .draw   = info_draw,
  .input  = info_input,
  .opaque = true,
  .modal  = true,
};

//-----------------------------------------------------------------------------
// The page's total height in display rows, and the last line that may sit at the
// top - the scroll end, which is the line from which the remaining ones just fill
// a screen. Both counted with wrapping, so both are exact rather than a guess
// that leaves a page scrollable past its own end.
static void info_measure(info_inst_t *in)
{
  int tail = 0;

  in->rows = 0;

  for (int i = 0; i < in->count; i++)
    in->rows += info_rows_of(in, info_line(in->page, i));

  // Backwards from the end until a screen is full: that line is the last useful
  // top. Same answer as count - vis on a page where nothing wraps.
  in->max_top = 0;

  for (int i = in->count - 1; i >= 0; i--)
  {
    tail += info_rows_of(in, info_line(in->page, i));

    if (tail > in->vis)
    {
      in->max_top = i + 1;
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void menu_open_info(const info_page_t *page)
{
  info_inst_t *in = &g_info;

  in->page = page;
  in->count = ((page->lines || page->line_fn) && page->count > 0) ?
      page->count : 0;
  in->top = 0;
  in->heads = 0;
  in->large = (UI_SCALE_LARGE == config.ui_scale) && in->count > 0;
  in->cols = (INFO_TEXT_R - INFO_X) / (in->large ? FW_LARGE : FW_SMALL);
  in->cols_cont = (INFO_TEXT_R - INFO_X - INFO_CONT_INDENT) /
      (in->large ? FW_LARGE : FW_SMALL);
  in->y0 = INFO_Y;
  in->line_h = in->large ? INFO_LINE_H + 8 : INFO_LINE_H;

  // Whole rows only at the large size: the roomy grid's last row has 11 px of
  // clearance under it, which an 8 px glyph fits into and a 16 px one does not -
  // it would land on the key hints, which only a full draw repaints.
  in->vis = in->large ? (INFO_BODY_BOTTOM + 9 - in->y0) / in->line_h :
      (INFO_BODY_BOTTOM - in->y0) / in->line_h + 1;

  info_measure(in);

  // Longer than one screen: the tighter grid, a scrollbar, and the sections
  // it turns out to have. A page that fits is left exactly as it was, body()
  // pages included - those draw on the roomy grid at their own coordinates.
  if (in->rows > in->vis)
  {
    int avail = INFO_BODY_BOTTOM - INFO_SCROLL_Y;

    in->y0 = INFO_SCROLL_Y;
    in->line_h = in->large ? INFO_SCROLL_LINE_H + 8 : INFO_SCROLL_LINE_H;
    in->vis = in->large ? (INFO_BODY_BOTTOM + 9 - in->y0) / in->line_h :
        avail / in->line_h + 1;

    // A page a line or two over the grid is squeezed onto it rather than
    // made to scroll for them; below 9 px the 6x8 font starts to touch. There
    // is no equivalent squeeze at 8x16 - 18 px is already the font plus two.
    if (!in->large && in->rows > in->vis && in->rows <= avail / 9 + 1)
    {
      in->line_h = 9;
      in->vis = in->rows;
    }

    info_measure(in);   // vis changed, so where the page ends changed with it

    for (int i = 0; i < in->count; i++)
    {
      if (info_is_head(info_line(page, i)))
        in->heads++;
    }
  }

  ui_push(&info_screen, in);
}

//-----------------------------------------------------------------------------
void menu_action_info(const void *arg)
{
  menu_open_info((const info_page_t *)arg);
}

//-----------------------------------------------------------------------------
// The text size changed under the menus that are open. Every popup on the stack
// gets laid out again and the whole visible screen is repainted.
//
// Both halves are needed. A popup keeps the geometry it was opened with, so
// without the re-layout its rows draw 6 px taller inside a box that is still the
// old height - the last row lands outside its own background, on top of whatever
// is behind the menu - and the value column is right-aligned to a width the
// labels no longer fit under. And a box that SHRANK leaves its old edge on the
// screen unless what it covered is drawn again, which only the stack can do.
void menu_relayout(void)
{
  for (int i = 0; i < MENU_MAX_OPEN; i++)
  {
    menu_inst_t *m = &g_inst[i];

    if (!m->used || m->fullscreen)
      continue;

    popup_layout(m);
    ensure_visible(m);   // fewer rows fit: the cursor may be off the end now
  }

  ui_request_full_redraw();
}

//-----------------------------------------------------------------------------
// Close the whole popup chain, e.g. after an action that resumes the
// application underneath
void menu_close_popups(void)
{
  while (ui_top_screen() == &menu_screen_popup)
    ui_pop();
}

//-----------------------------------------------------------------------------
static void menu_leave(void *ctx)
{
  ((menu_inst_t *)ctx)->used = false;
}

//-----------------------------------------------------------------------------
// What this popup covers, so that closing it costs a repaint of that and not
// of the instrument behind it. The box plus the two pixels of shadow that
// popup_draw() lays down and to the right of it.
static bool menu_bounds(void *ctx, ui_rect_t *rect)
{
  const menu_inst_t *m = (const menu_inst_t *)ctx;

  if (m->fullscreen)
    return false;

  rect->x = m->x;
  rect->y = m->y;
  rect->w = m->w + 2;
  rect->h = m->h + 2;

  return true;
}

//-----------------------------------------------------------------------------
static void menu_draw(void *ctx, bool full)
{
  menu_inst_t *m = (menu_inst_t *)ctx;

  if (m->fullscreen)
    fs_draw(m, full);
  else
    popup_draw(m, full);
}

//-----------------------------------------------------------------------------
// Input, in two modes.
//
// NAVIGATING: UP/DOWN move the cursor, LEFT goes back one level (any level:
// submenu, dialog - and on the root popup it closes the menu), RIGHT/MODE
// enter things. A number or choice row is ENTERED like anything else - MODE
// (or RIGHT) gives it the edit focus.
//
// EDITING: the arrows belong to the value - LEFT/RIGHT fine step, UP/DOWN
// coarse step - and MODE or MENU hands them back. This exists because the
// old direct-edit model made LEFT ambiguous: on a value row it adjusted,
// everywhere else it went back, so a menu whose cursor stood on a value had
// no way back at all.
static bool menu_input(void *ctx, int buttons)
{
  menu_inst_t *m = (menu_inst_t *)ctx;
  bool repeat = (buttons & BTN_REPEAT);
  const menu_item_t *it = &m->items[m->sel];
  bool editable = (MI_NUMBER == it->kind || MI_CHOICE == it->kind);

  if (m->editing)
  {
    if (!editable)
    {
      m->editing = false; // the item changed under us somehow: recover
    }
    else if (buttons & (BTN_LEFT | BTN_RIGHT | BTN_UP | BTN_DOWN))
    {
      int dir = (buttons & (BTN_RIGHT | BTN_UP)) ? 1 : -1;

      if (MI_NUMBER == it->kind)
      {
        // Vertical = coarse, horizontal = fine: the calibration numbers
        // live in the thousands and one step per press cannot reach them
        int step = (buttons & (BTN_UP | BTN_DOWN)) ?
            it->u.number.step_repeat : it->u.number.step;

        if (repeat)
          step *= 4;

        number_adjust(it, dir * step);
      }
      else if (!repeat)
      {
        choice_adjust(it, dir);
      }

      return true;
    }
    else if (buttons & (BTN_MODE | BTN_MENU))
    {
      if (!repeat)
      {
        if (m->shortcut)
        {
          ui_pop();       // done: the value is already applied
          return true;
        }

        m->editing = false;
        ui_request_redraw();
      }

      return true;
    }

    return true; // stray keys stay ours while editing
  }

  if (buttons & BTN_UP)
  {
    move_sel(m, -1);
    return true;
  }

  if (buttons & BTN_DOWN)
  {
    move_sel(m, 1);
    return true;
  }

  if (buttons & BTN_LEFT)
  {
    // Back, one level, from anywhere - the cursor position no longer matters
    if (repeat)
      return true;

    if (m->is_submenu)
      ui_pop();
    else if (!m->fullscreen)
      menu_close_popups();

    return true;
  }

  if (buttons & BTN_RIGHT)
  {
    if (repeat)
      return true;

    if (MI_SUBMENU == it->kind)
      open_submenu(m, it);
    else if (editable)
    {
      m->editing = true;
      ui_request_redraw();
    }

    return true;
  }

  if (buttons & BTN_MODE)
  {
    if (repeat)
      return true;

    switch (it->kind)
    {
      case MI_SUBMENU:
        open_submenu(m, it);
        break;

      case MI_TOGGLE:
      case MI_TOGGLE_INV:
        *it->u.toggle.value = !*it->u.toggle.value;
        if (it->u.toggle.on_change)
          it->u.toggle.on_change();
        ui_invalidate_below();
        ui_request_redraw();
        break;

      case MI_NUMBER:
      case MI_CHOICE:
        m->editing = true;
        ui_request_redraw();
        break;

      case MI_ACTION:
        // An action is anything at all - an auto-setup, a saved config, a
        // calibration run - so what it left behind it is not knowable from
        // here, and the screen under the menu is repainted whole afterwards
        ui_invalidate_below();

        // May push a dialog or pop this very menu — no instance access after
        it->u.action.fn(it->u.action.arg);
        break;

      default:
        break;
    }

    return true;
  }

  if (buttons & BTN_MENU)
  {
    if (m->fullscreen)
    {
      // A fullscreen DIALOG (menu_open_dialog) closes like any menu. Only
      // the launcher's root menu passes MENU on, so the system menu can
      // open over it - a dialog doing that had no way out at all: MENU
      // stacked a system menu on top and LEFT ignored a non-submenu.
      if (m->is_submenu)
      {
        if (!repeat)
          ui_pop();

        return true;
      }

      return false; // root menu: let the caller open the system menu
    }

    if (!repeat)
      menu_close_popups();

    return true;
  }

  // Popups swallow everything else so stray keys don't reach the app below
  return true;
}
