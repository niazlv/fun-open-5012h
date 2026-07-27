/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DOOM.
 *
 * Not a raycaster that resembles DOOM - the actual engine, running an actual
 * level out of a real WAD: binary space partition traversal, id's perspective
 * wall mapping, visplanes, the original colormaps, and movement with the
 * original's acceleration, friction and 24-unit step height.
 *
 * The level, textures, palette and every projection table are converted on the
 * host by tools/wadpack.py and linked into flash, because none of it could be
 * built at runtime in the RAM this part has. The engine reads them in place
 * through const pointers, so all of the RAM goes to the frame buffer and the
 * per-frame clipping state - see doom/doom.h.
 *
 * The frame buffer sits in the capture SRAM, which is free while the scope is
 * not acquiring; capture is stopped on entry and the oscilloscope re-arms it
 * when it is next opened.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "utils.h"
#include "ui.h"
#include "capture.h"
#include "menu_widget.h"
#include "doom_port.h"
#include "doom/doom.h"

/*- Definitions -------------------------------------------------------------*/
// The status bar is the WAD's own 320x32 STBAR, drawn under the view, which
// leaves exactly one 8-pixel line of the 240-row display for the stats
#define ST_Y                SCREENHEIGHT
#define ST_H                32
#define STATS_Y             (ST_Y + ST_H)

// Where the original puts the three big numbers, relative to the bar
#define ST_AMMOX            44
#define ST_HEALTHX          90
#define ST_ARMORX           221
#define ST_NUMY             3

// The block DOOM may use: main SRAM up to the spare region, which holds the
// logic decoder scratch and the coredump ring and must not be touched
#define DOOM_RAM_BASE       0x20000000u
#define DOOM_RAM_SIZE       (CAPTURE_SPARE_RAM - DOOM_RAM_BASE)

_Static_assert(sizeof(doom_mem_t) <= DOOM_RAM_SIZE,
    "DOOM working set does not fit below the capture spare RAM");

#define HUD_BG              LCD_COLOR(16, 16, 24)
#define HUD_FG              LCD_COLOR(200, 200, 200)
#define HUD_HI              LCD_COLOR(255, 190, 60)

/*- Variables ---------------------------------------------------------------*/
extern const uint8_t doom_assets[];

static bool g_ready;
static bool g_paused;
static bool g_show_stats = true;
static bool g_always_run;

static uint32_t g_last_tic_ms;

static uint16_t g_fps;
static uint32_t g_fps_frames, g_fps_last;
static uint32_t g_render_us, g_blit_us;
static uint32_t g_stats_last;
static char g_statline[64];

// Health, armour and ammo are constants until there is gameplay to change
// them; the widgets that draw them are real, so they will not need touching
static int g_health = 100;
static int g_armor;
static int g_ammo = 50;

/*- Status bar --------------------------------------------------------------*/
typedef struct
{
    uint8_t w, h;
    uint16_t offset;
    uint16_t pad;
} st_glyph_t;

static const uint8_t *g_stbar;
static const st_glyph_t *g_stnumdir;
static const uint8_t *g_stnums;

// One number field at a time, composited here and pushed in a single transfer
static uint8_t g_stbuf[3 * 16 * 16];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// The keyboard is read through two priority encoders and only one key per group
// is visible at a time. LEFT, DOWN and TRIG share a group, so TRIG could never
// be held with LEFT - strafing left was simply unreachable. A modifier has to
// be one of the two keys wired to their own pins: SHIFT strafes, F2 runs.
static void build_ticcmd(ticcmd_t *cmd, int buttons)
{
    bool strafe = (0 != (buttons & BTN_SHIFT));

    cmd->forward = 0;
    cmd->side = 0;
    cmd->turn = 0;
    cmd->run = g_always_run ^ (0 != (buttons & BTN_F2));

    if (buttons & BTN_UP)
        cmd->forward = 1;
    else if (buttons & BTN_DOWN)
        cmd->forward = -1;

    if (buttons & BTN_LEFT)
    {
        if (strafe)
            cmd->side = -1;
        else
            cmd->turn = 1;
    }
    else if (buttons & BTN_RIGHT)
    {
        if (strafe)
            cmd->side = 1;
        else
            cmd->turn = -1;
    }
}

//-----------------------------------------------------------------------------
// A digit, over whatever the bar already had there. The mask is what makes the
// glyph's own background transparent.
static void st_glyph(int gx, int index, int fieldw, int fieldh)
{
    const st_glyph_t *g = &g_stnumdir[index];
    const uint8_t *pixels = g_stnums + g->offset;
    const uint8_t *mask = pixels + g->w * g->h;

    for (int y = 0; y < g->h && y < fieldh; y++)
    {
        for (int x = 0; x < g->w; x++)
        {
            int i = y * g->w + x;

            if (gx + x < 0 || gx + x >= fieldw)
                continue;

            if (mask[i >> 3] & (1 << (i & 7)))
                g_stbuf[y * fieldw + gx + x] = pixels[i];
        }
    }
}

//-----------------------------------------------------------------------------
// The original's numbers are right aligned on their x and stepped by the width
// of a zero, which is why a one sits off centre in its cell
static void st_number(int xright, int digits, int value)
{
    int w = g_stnumdir[0].w;
    int h = g_stnumdir[0].h;
    int fieldw = w * digits;
    int x0 = xright - fieldw;
    int x = fieldw;

    if (NULL == g_stbar || 0 == w)
        return;

    for (int r = 0; r < h; r++)
        memcpy(g_stbuf + r * fieldw, g_stbar + (ST_NUMY + r) * SCREENWIDTH + x0, fieldw);

    if (0 == value)
    {
        x -= w;
        st_glyph(x, 0, fieldw, h);
    }
    else
    {
        while (value > 0 && digits-- > 0)
        {
            x -= w;
            st_glyph(x, value % 10, fieldw, h);
            value /= 10;
        }
    }

    lcd_draw_indexed(x0, ST_Y + ST_NUMY, fieldw, h, g_stbuf, dt_palette);
}

//-----------------------------------------------------------------------------
static void draw_statusbar(void)
{
    if (g_stbar)
    {
        lcd_draw_indexed(0, ST_Y, SCREENWIDTH, ST_H, g_stbar, dt_palette);

        st_number(ST_AMMOX, 3, g_ammo);
        st_number(ST_HEALTHX, 3, g_health);
        st_number(ST_ARMORX, 3, g_armor);
    }
    else
    {
        lcd_fill_rect(0, ST_Y, LCD_WIDTH, ST_H, HUD_BG);
    }
}

//-----------------------------------------------------------------------------
// The stats line is redrawn only when its text changes, and at most a few times
// a second: erasing and repainting it every frame is visible as a flicker at
// the bottom of the screen
static void draw_stats(bool force)
{
    char buf[sizeof(g_statline)];
    uint32_t now = timer_ms();

    if (!g_show_stats)
        return;

    if (!force && now - g_stats_last < 250)
        return;

    g_stats_last = now;

    snprintf(buf, sizeof(buf), "%u.%u fps  %lu+%lu ms  X%ld Y%ld %ld deg",
        g_fps / 10, g_fps % 10,
        (unsigned long)(g_render_us / 1000), (unsigned long)(g_blit_us / 1000),
        (long)(player.x >> FRACBITS), (long)(player.y >> FRACBITS),
        (long)((uint64_t)player.angle * 360 >> 32));

    if (!force && 0 == strcmp(buf, g_statline))
        return;

    // Pad to a fixed width so a shorter line cannot leave the tail of a longer
    // one behind, which would otherwise need an erase and bring the flicker back
    {
        int len = strlen(buf);

        while (len < (int)sizeof(buf) - 1)
            buf[len++] = ' ';

        buf[len] = 0;
    }

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
    lcd_puts(2, STATS_Y, buf);

    memcpy(g_statline, buf, sizeof(g_statline));
}

//-----------------------------------------------------------------------------
// What the pack looked like when it was rejected. The first word says most of
// it: 0xffffffff is flash that was never written - which happens when only the
// first part of the image reaches the device, since all of the code fits below
// the pack and the firmware then runs perfectly without it.
static void draw_no_assets(void)
{
    const char *why = doom_assets_error();
    const uint32_t *words = (const uint32_t *)doom_assets;
    char buf[52];

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    lcd_set_font(FONT_LARGE);
    lcd_set_color(LCD_BLACK_COLOR, LCD_COLOR(220, 60, 40));
    lcd_puts(84, 20, "NO WAD DATA");

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
    snprintf(buf, sizeof(buf), "reason: %s", why ? why : "level data");
    lcd_puts(12, 58, buf);

    lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
    snprintf(buf, sizeof(buf), "pack at %08lx: %08lx %08lx %08lx",
        (unsigned long)(uintptr_t)doom_assets, (unsigned long)words[0],
        (unsigned long)words[1], (unsigned long)words[2]);
    lcd_puts(12, 74, buf);
    lcd_puts(12, 86, "expected  44504b31 00000001 00000019");

    if (why && 'f' == why[0])
    {
        // "flash not written"
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        lcd_puts(12, 110, "Flash reads as erased where the pack");
        lcd_puts(12, 122, "should be. The image is 362 KB and all");
        lcd_puts(12, 134, "the code is in its first 162 KB, so a");
        lcd_puts(12, 146, "short write leaves this exact state.");
        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(12, 166, "Reflash the whole open-5012h.bin and");
        lcd_puts(12, 178, "let the programmer verify it.");
    }
    else
    {
        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(12, 110, "The pack is 208 KB of a 384 KB image, so");
        lcd_puts(12, 122, "builds leave it out unless asked:");
        lcd_set_color(LCD_BLACK_COLOR, HUD_HI);
        lcd_puts(12, 142, "python3 tools/wadpack.py doom1.wad \\");
        lcd_puts(12, 154, "        -o doom/doom_assets.bin");
        lcd_puts(12, 166, "cd make && make DOOM=1");
        lcd_set_color(LCD_BLACK_COLOR, HUD_FG);
        lcd_puts(12, 186, "Any DOOM IWAD works, Freedoom included.");
    }

    lcd_puts(12, 206, "SHIFT+MENU returns to the launcher.");
}

//-----------------------------------------------------------------------------
static void draw_frame(void)
{
    uint32_t start = timer_us();
    uint32_t mid;

    P_RenderPlayerView();
    mid = timer_us();

    lcd_draw_indexed(0, 0, SCREENWIDTH, SCREENHEIGHT, dm->screen, dt_palette);

    g_render_us = mid - start;
    g_blit_us = timer_us() - mid;
}

//-----------------------------------------------------------------------------
void doom_port_init(void)
{
    // The frame buffer lives in the capture ring, so acquisition has to stop
    // before anything is drawn into it
    capture_stop();

    dm = (doom_mem_t *)DOOM_RAM_BASE;

    g_ready = doom_assets_init(doom_assets) && doom_level_load();

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

    if (!g_ready)
    {
        draw_no_assets();
        return;
    }

    g_stbar = doom_asset_find("STBAR", NULL);
    g_stnumdir = doom_asset_find("STNUMDIR", NULL);
    g_stnums = doom_asset_find("STNUMS", NULL);

    R_Init();
    R_InitSegs();
    P_DoorsClear();
    P_SpawnPlayer();

    g_paused = false;
    g_last_tic_ms = timer_ms();
    g_fps = 0;
    g_fps_frames = 0;
    g_fps_last = g_last_tic_ms;
}

//-----------------------------------------------------------------------------
// Paints straight away rather than raising a dirty flag: an overlay can close
// while another one is still open, and only the top screen is ticked, so a
// deferred repaint would not happen until the whole stack unwound
void doom_port_redraw(void)
{
    if (!g_ready)
    {
        draw_no_assets();
        return;
    }

    draw_frame();
    draw_statusbar();

    g_statline[0] = 0;
    draw_stats(true);
}

//-----------------------------------------------------------------------------
void doom_port_task(void)
{
    uint32_t now;
    int tics;

    if (!g_ready)
        return;

    now = timer_ms();

    // The simulation runs at the original's 35 Hz whatever the frame rate is,
    // so walking speed does not depend on how much of the level is in view
    tics = (int)((now - g_last_tic_ms) * TICRATE / 1000);

    if (tics > 0)
    {
        if (tics > 4)
            tics = 4;               // after a long stall, do not fast forward

        g_last_tic_ms = now;

        if (!g_paused)
        {
            ticcmd_t cmd;

            build_ticcmd(&cmd, buttons_state());

            for (int i = 0; i < tics; i++)
            {
                P_PlayerTic(&cmd);
                P_DoorsTic(player.x, player.y, player.z);
            }
        }
    }

    draw_frame();
    g_fps_frames++;

    if (now - g_fps_last >= 1000)
    {
        // One decimal place: below 10 fps the integer alone is too coarse to
        // tell whether a change helped
        g_fps = (uint16_t)(g_fps_frames * 10000 / (now - g_fps_last));
        g_fps_frames = 0;
        g_fps_last = now;
    }

    draw_stats(false);
}

//-----------------------------------------------------------------------------
void doom_port_buttons_handler(int buttons)
{
    // Walking is polled from buttons_state() in the task, because the repeat
    // events are far too coarse to steer with; this only catches the edges of
    // the keys that toggle something
    if (!g_ready)
        return;

    if ((buttons & BTN_MODE) && !(buttons & BTN_REPEAT))
    {
        // Use: opens the door the player is facing
        P_UseLines(player.x, player.y, player.angle);
        return;
    }

    if ((buttons & BTN_STOP) && !(buttons & BTN_REPEAT))
    {
        g_paused = !g_paused;
        g_last_tic_ms = timer_ms();
    }
}

//-----------------------------------------------------------------------------
void doom_port_cleanup(void)
{
    g_ready = false;
    dm = NULL;
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_respawn(const void *arg)
{
    (void)arg;

    if (g_ready)
    {
        // Reloading rebuilds the sector shadow, which is what puts any door
        // that was left open back down
        doom_level_load();
        P_DoorsClear();
        P_SpawnPlayer();
    }

    g_paused = false;
    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "UP / DOWN    - Walk forward / back",
    "LEFT / RIGHT - Turn",
    "SHIFT + L/R  - Strafe",
    "F2           - Run",
    "MODE         - Open the door in front of you",
    "STOP         - Pause",
    "SHIFT+MENU   - Back to the launcher",
    "",
    "The keyboard is read through two priority",
    "encoders and only one key per group is seen",
    "at a time. SHIFT and F2 have their own pins,",
    "which is why the modifiers are on them: any",
    "other key would be unusable with half of the",
    "arrows.",
    "",
    "This is id's renderer on a real WAD level:",
    "BSP traversal, perspective wall mapping,",
    "visplanes and the original colormaps. The",
    "level and textures are converted by",
    "tools/wadpack.py and read out of flash.",
};

static const info_page_t g_help_page =
{
    .title = "DOOM",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_TOGGLE, .label = "Always run",
      .u.toggle = { &g_always_run, NULL } },
    { .kind = MI_TOGGLE, .label = "Show stats",
      .u.toggle = { &g_show_stats, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Restart level",
      .u.action = { action_respawn, NULL } },
};

const menu_def_t doom_port_menu =
{
    .title = "DOOM",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};

// Read-only pages: the system menu shows them under Help, not among the
// settings above
static const menu_item_t g_help_items[] =
{
    { .kind = MI_ACTION, .label = "Controls",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t doom_port_help_menu =
{
    .title = "DOOM",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
