/*
 * DOOM-style raycasting demo
 *
 * One DDA ray per screen column against a const grid map. The distance used
 * for the wall height is the perpendicular distance to the camera plane,
 * which is what removes the fisheye distortion; the previous version divided
 * by the ray's own cosine and bent every corridor.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "utils.h"
#include "ui.h"
#include "menu_widget.h"
#include "doom_port.h"

/*- Definitions -------------------------------------------------------------*/
#define FRAME_MS            30      // cap the frame rate, keep the loop alive

#define MOVE_SPEED          0.12f   // map cells per frame
#define TURN_SPEED          0.09f   // radians per frame

#define FOV_MIN             50
#define FOV_MAX             100
#define FOV_DEFAULT         66

#define MIN_WALL_DIST       0.02f   // clamps the wall height when hugging a wall
#define COLLISION_RADIUS    0.20f

#define CEILING_COLOR       LCD_COLOR(40, 40, 55)
#define FLOOR_COLOR         LCD_COLOR(70, 60, 50)
#define HUD_BG              LCD_COLOR(16, 16, 24)
#define HUD_FG              LCD_COLOR(220, 220, 220)

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    float x, y;         // position in map cells
    float angle;        // radians, 0 = +x
} player_t;

/*- Variables ---------------------------------------------------------------*/
static player_t g_player;
static int g_frame_timer = TIMER_DISABLE;
static int g_fov = FOV_DEFAULT;
static bool g_show_hud = true;
static bool g_paused = false;
static bool g_hud_dirty = true;
static bool g_view_dirty = true;

static uint32_t g_fps_last = 0;
static uint32_t g_fps_frames = 0;
static uint16_t g_fps = 0;
static uint32_t g_frame_ms = 0;

// Wall colors indexed by cell value, 0 is empty
static const uint16_t g_wall_colors[5] =
{
    0,
    LCD_COLOR(170, 170, 170),   // 1 light gray
    LCD_COLOR(160, 70, 60),     // 2 brick
    LCD_COLOR(70, 110, 160),    // 3 blue
    LCD_COLOR(150, 130, 60),    // 4 ochre
};

// The first rows of the level. Everything below row 16 is open floor inside a
// solid border, which is generated rather than stored.
#define MAP_ROWS  17

static const uint8_t g_map_rows[MAP_ROWS][DOOM_MAP_WIDTH] =
{
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,0,0,1},
    {1,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,0,0,0,0,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,0,0,1},
    {1,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Read straight out of flash: the map never changes, so the 4 KB RAM copy the
// first version kept was pure waste
static uint8_t wall_at(int x, int y)
{
    if (x < 0 || x >= DOOM_MAP_WIDTH || y < 0 || y >= DOOM_MAP_HEIGHT)
        return 1;

    if (y < MAP_ROWS)
        return g_map_rows[y][x];

    if (0 == x || DOOM_MAP_WIDTH - 1 == x || DOOM_MAP_HEIGHT - 1 == y)
        return 1;

    return 0;
}

//-----------------------------------------------------------------------------
static uint16_t shade(uint16_t color, float factor)
{
    int r = (int)(((color >> 11) & 0x1F) * factor);
    int g = (int)(((color >> 5) & 0x3F) * factor);
    int b = (int)((color & 0x1F) * factor);

    if (r > 0x1F) r = 0x1F;
    if (g > 0x3F) g = 0x3F;
    if (b > 0x1F) b = 0x1F;

    return (uint16_t)((r << 11) | (g << 5) | b);
}

//-----------------------------------------------------------------------------
static void render_view(void)
{
    float dir_x = cosf(g_player.angle);
    float dir_y = sinf(g_player.angle);

    // Camera plane length sets the horizontal field of view
    float half_fov = (g_fov * (float)M_PI / 180.0f) / 2.0f;
    float plane_len = tanf(half_fov);
    float plane_x = -dir_y * plane_len;
    float plane_y = dir_x * plane_len;

    for (int x = 0; x < DOOM_VIEW_WIDTH; x++)
    {
        float camera_x = 2.0f * x / (float)DOOM_VIEW_WIDTH - 1.0f;
        float ray_x = dir_x + plane_x * camera_x;
        float ray_y = dir_y + plane_y * camera_x;

        int map_x = (int)g_player.x;
        int map_y = (int)g_player.y;

        // A zero component means the ray never crosses that family of grid
        // lines; a huge step keeps it from ever being chosen
        float delta_x = (0.0f == ray_x) ? 1e30f : fabsf(1.0f / ray_x);
        float delta_y = (0.0f == ray_y) ? 1e30f : fabsf(1.0f / ray_y);

        float side_x, side_y;
        int step_x, step_y;

        if (ray_x < 0)
        {
            step_x = -1;
            side_x = (g_player.x - map_x) * delta_x;
        }
        else
        {
            step_x = 1;
            side_x = (map_x + 1.0f - g_player.x) * delta_x;
        }

        if (ray_y < 0)
        {
            step_y = -1;
            side_y = (g_player.y - map_y) * delta_y;
        }
        else
        {
            step_y = 1;
            side_y = (map_y + 1.0f - g_player.y) * delta_y;
        }

        uint8_t cell = 0;
        int side = 0;

        while (0 == cell)
        {
            if (side_x < side_y)
            {
                side_x += delta_x;
                map_x += step_x;
                side = 0;
            }
            else
            {
                side_y += delta_y;
                map_y += step_y;
                side = 1;
            }

            cell = wall_at(map_x, map_y);
        }

        // Distance to the camera plane, not to the eye: this is the fisheye
        // correction, and it is exact rather than an approximation
        float dist = (0 == side) ? (side_x - delta_x) : (side_y - delta_y);

        if (dist < MIN_WALL_DIST)
            dist = MIN_WALL_DIST;

        int line_h = (int)(DOOM_VIEW_HEIGHT / dist);

        if (line_h > DOOM_VIEW_HEIGHT)
            line_h = DOOM_VIEW_HEIGHT;

        int draw_start = (DOOM_VIEW_HEIGHT - line_h) / 2;
        int draw_end = draw_start + line_h;

        if (cell > 4)
            cell = 4;

        // Distance attenuation plus a constant step for the shaded side, so
        // corners stay readable instead of saturating to the ceiling color
        float factor = 1.0f / (1.0f + dist * 0.15f);

        if (side)
            factor *= 0.72f;

        uint16_t color = shade(g_wall_colors[cell], factor);

        if (draw_start > 0)
            lcd_fill_rect(x, 0, 1, draw_start, CEILING_COLOR);

        lcd_fill_rect(x, draw_start, 1, line_h, color);

        if (draw_end < DOOM_VIEW_HEIGHT)
            lcd_fill_rect(x, draw_end, 1, DOOM_VIEW_HEIGHT - draw_end, FLOOR_COLOR);
    }
}

//-----------------------------------------------------------------------------
static void draw_hud(void)
{
    char buf[48];

    lcd_fill_rect(0, DOOM_VIEW_HEIGHT, LCD_WIDTH, DOOM_HUD_HEIGHT, HUD_BG);

    if (!g_show_hud)
        return;

    lcd_set_font(FONT_SMALL);
    lcd_set_color(HUD_BG, HUD_FG);

    snprintf(buf, sizeof(buf), "FPS %d  %lu ms  X%d Y%d", g_fps,
        (unsigned long)g_frame_ms, (int)g_player.x, (int)g_player.y);
    lcd_puts(5, DOOM_VIEW_HEIGHT + 6, buf);

    lcd_puts(5, DOOM_VIEW_HEIGHT + 17,
        "UP/DN move  L/R turn  SHIFT+L/R strafe  MENU");

    if (g_paused)
    {
        lcd_set_color(HUD_BG, LCD_COLOR(255, 200, 0));
        lcd_puts(LCD_WIDTH - 50, DOOM_VIEW_HEIGHT + 6, "PAUSED");
    }
}

//-----------------------------------------------------------------------------
static void reset_player(void)
{
    // Row 8 is an open corridor across the whole level. The original spawn at
    // (5,5) was inside one of the closed 3x3 rooms, with no way out.
    g_player.x = 8.5f;
    g_player.y = 8.5f;
    g_player.angle = 0.0f;
}

//-----------------------------------------------------------------------------
// Move one axis at a time so a glancing hit slides along the wall instead of
// stopping the player dead
static void try_move(float dx, float dy)
{
    float nx = g_player.x + dx;
    float ny = g_player.y + dy;
    float pad_x = (dx > 0) ? COLLISION_RADIUS : -COLLISION_RADIUS;
    float pad_y = (dy > 0) ? COLLISION_RADIUS : -COLLISION_RADIUS;

    if (dx != 0.0f && !wall_at((int)(nx + pad_x), (int)g_player.y))
        g_player.x = nx;

    if (dy != 0.0f && !wall_at((int)g_player.x, (int)(ny + pad_y)))
        g_player.y = ny;
}

//-----------------------------------------------------------------------------
void doom_port_init(void)
{
    reset_player();

    g_paused = false;
    g_hud_dirty = true;
    g_view_dirty = true;
    g_fps = 0;
    g_fps_frames = 0;
    g_fps_last = timer_ms();

    timer_add(&g_frame_timer);
    g_frame_timer = 0;

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
void doom_port_redraw(void)
{
    g_hud_dirty = true;
    g_view_dirty = true;
    g_frame_timer = 0; // repaint on the very next pass
}

//-----------------------------------------------------------------------------
void doom_port_task(void)
{
    uint32_t now;

    if (g_frame_timer != 0)
        return;

    g_frame_timer = FRAME_MS;
    now = timer_ms();

    // While paused the view is only redrawn on demand - which is what repairs
    // the screen after a menu closed over it
    if (!g_paused || g_view_dirty)
    {
        render_view();

        g_frame_ms = timer_ms() - now;
        g_view_dirty = false;
        g_fps_frames++;
    }

    if (now - g_fps_last >= 1000)
    {
        g_fps = g_fps_frames;
        g_fps_frames = 0;
        g_fps_last = now;
        g_hud_dirty = true;
    }

    if (g_hud_dirty)
    {
        draw_hud();
        g_hud_dirty = false;
    }
}

//-----------------------------------------------------------------------------
void doom_port_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    bool strafe = (buttons & BTN_SHIFT);

    if (buttons & (BTN_MODE | BTN_STOP))
    {
        if (!repeat)
        {
            g_paused = !g_paused;
            g_hud_dirty = true;
        }

        return;
    }

    if (g_paused)
        return;

    float dir_x = cosf(g_player.angle);
    float dir_y = sinf(g_player.angle);

    if (buttons & BTN_UP)
        try_move(dir_x * MOVE_SPEED, dir_y * MOVE_SPEED);

    if (buttons & BTN_DOWN)
        try_move(-dir_x * MOVE_SPEED, -dir_y * MOVE_SPEED);

    if (buttons & BTN_LEFT)
    {
        if (strafe)
            try_move(dir_y * MOVE_SPEED, -dir_x * MOVE_SPEED);
        else
            g_player.angle -= TURN_SPEED;
    }

    if (buttons & BTN_RIGHT)
    {
        if (strafe)
            try_move(-dir_y * MOVE_SPEED, dir_x * MOVE_SPEED);
        else
            g_player.angle += TURN_SPEED;
    }

    g_hud_dirty = true;
}

//-----------------------------------------------------------------------------
void doom_port_cleanup(void)
{
    g_frame_timer = TIMER_DISABLE;
    timer_remove(&g_frame_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_respawn(const void *arg)
{
    (void)arg;

    reset_player();
    g_paused = false;
    g_view_dirty = true;
    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "UP/DOWN      - Walk forward / back",
    "LEFT/RIGHT   - Turn",
    "SHIFT+L/R    - Strafe",
    "MODE or STOP - Pause",
    "MENU         - This menu",
    "SHIFT+MENU   - Back to the launcher",
    "",
    "One DDA ray per column against a 64x64",
    "grid. Wall height comes from the distance",
    "to the camera plane, so corridors stay",
    "straight; shading falls off with distance",
    "and the sides facing away are darker.",
};

static const info_page_t g_help_page =
{
    .title = "DOOM Port",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_NUMBER, .label = "Field of view",
      .u.number = { &g_fov, FOV_MIN, FOV_MAX, 2, 10, " deg", NULL } },
    { .kind = MI_TOGGLE, .label = "Show HUD",
      .u.toggle = { &g_show_hud, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Respawn",
      .u.action = { action_respawn, NULL } },
    { .kind = MI_ACTION, .label = "Help",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t doom_port_menu =
{
    .title = "DOOM Port",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};
