/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 3D Cube Renderer
 * Rotating, back-face culled 3D cube demo
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "config.h"
#include "buttons.h"
#include "utils.h"
#include "ui.h"
#include "menu_widget.h"
#include "cube3d.h"

/*- Definitions -------------------------------------------------------------*/
#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)

// Cube rendering area: everything between the title and the status line.
// Drawing is clipped to it, so the cube can never smear over the text.
#define RENDER_X      0
#define RENDER_Y      30
#define RENDER_WIDTH  LCD_WIDTH
#define RENDER_HEIGHT (LCD_HEIGHT - RENDER_Y - 20)

#define STATUS_Y      (LCD_HEIGHT - 14)

// Rotation rate as a percentage of the nominal speed
#define SPEED_MIN     10
#define SPEED_MAX     300
#define SPEED_DEFAULT 100

// Draw style, index into g_style_labels
#define STYLE_SOLID_WIRE  0
#define STYLE_SOLID       1
#define STYLE_WIRE        2

// Cube colors
#define CUBE_EDGE_COLOR     LCD_COLOR(255, 255, 255)  // White edges
#define CUBE_FACE_1_COLOR   LCD_COLOR(255, 0, 0)      // Red
#define CUBE_FACE_2_COLOR   LCD_COLOR(0, 255, 0)      // Green  
#define CUBE_FACE_3_COLOR   LCD_COLOR(0, 0, 255)      // Blue
#define CUBE_FACE_4_COLOR   LCD_COLOR(255, 255, 0)    // Yellow
#define CUBE_FACE_5_COLOR   LCD_COLOR(255, 0, 255)    // Magenta
#define CUBE_FACE_6_COLOR   LCD_COLOR(0, 255, 255)    // Cyan
#define BG_COLOR            LCD_COLOR(0, 0, 0)        // Black background

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float x, y, z;
} Vector3D;

typedef struct {
    int x, y;
} Point2D;

typedef struct {
    Vector3D vertices[8];
    Point2D projected[8];
    float rotation_x, rotation_y, rotation_z;
    float rotation_speed_x, rotation_speed_y, rotation_speed_z;
} Cube3D;

// Sines and cosines of the three rotation angles, computed once per frame
typedef struct {
    float cx, sx, cy, sy, cz, sz;
} rot_t;

/*- Variables ---------------------------------------------------------------*/
static Cube3D g_cube;
static bool g_auto_rotate = true;
static int g_style = STYLE_SOLID_WIRE;
static int g_speed = SPEED_DEFAULT;
static int g_frame_timer = TIMER_DISABLE;

// Frame accounting. paint_ms is how long the cube itself takes; fps is how
// often it actually gets to run. If paint_ms is small but fps is far below
// 1000/paint_ms, the time is going somewhere outside this application.
static uint32_t g_fps_last = 0;
static int g_fps_frames = 0;
static int g_fps = 0;
static uint32_t g_paint_ms = 0;
static int g_loop_max_ms = 0;

// Optimized rendering with dirty rectangles
#define MAX_DIRTY_RECTS 4
typedef struct {
    int x, y, width, height;
    bool active;
} DirtyRect;

static DirtyRect g_dirty_rects[MAX_DIRTY_RECTS];
static int g_dirty_count = 0;
static Point2D g_prev_projected[8];  // Previous frame vertices
static bool g_first_frame = true;

// Cube vertex indices for edges (12 edges total)
static const int cube_edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom face
    {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top face  
    {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Vertical edges
};

// Cube face indices (6 faces, each with 4 vertices)
static const int cube_faces[6][4] = {
    {0, 1, 2, 3}, // Bottom
    {4, 7, 6, 5}, // Top
    {0, 4, 5, 1}, // Front
    {2, 6, 7, 3}, // Back
    {0, 3, 7, 4}, // Left
    {1, 5, 6, 2}  // Right
};

static const int face_colors[6] = {
    CUBE_FACE_1_COLOR, CUBE_FACE_2_COLOR, CUBE_FACE_3_COLOR,
    CUBE_FACE_4_COLOR, CUBE_FACE_5_COLOR, CUBE_FACE_6_COLOR
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static float fast_sin(float angle)
{
    // Normalize angle to [-PI, PI]
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    
    // Use standard math library for accuracy
    return sinf(angle);
}

//-----------------------------------------------------------------------------
static float fast_cos(float angle)
{
    // Normalize angle to [-PI, PI]
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    
    // Use standard math library for accuracy
    return cosf(angle);
}

//-----------------------------------------------------------------------------
// All eight vertices share the same angles, so the six trig calls are hoisted
// out by the caller
static void rotate_point(Vector3D *point, const rot_t *r)
{
    float cos_rx = r->cx, sin_rx = r->sx;
    float cos_ry = r->cy, sin_ry = r->sy;
    float cos_rz = r->cz, sin_rz = r->sz;

    // Rotation around X axis
    float y1 = point->y * cos_rx - point->z * sin_rx;
    float z1 = point->y * sin_rx + point->z * cos_rx;
    point->y = y1;
    point->z = z1;
    
    // Rotation around Y axis  
    float x2 = point->x * cos_ry + point->z * sin_ry;
    float z2 = -point->x * sin_ry + point->z * cos_ry;
    point->x = x2;
    point->z = z2;
    
    // Rotation around Z axis
    float x3 = point->x * cos_rz - point->y * sin_rz;
    float y3 = point->x * sin_rz + point->y * cos_rz;
    point->x = x3;
    point->y = y3;
}

//-----------------------------------------------------------------------------
static Point2D project_3d_to_2d(Vector3D point)
{
    Point2D result;
    float perspective = CUBE3D_DISTANCE / (CUBE3D_DISTANCE + point.z);
    
    result.x = CUBE3D_CENTER_X + (int)(point.x * perspective);
    result.y = CUBE3D_CENTER_Y + (int)(point.y * perspective);
    
    return result;
}

//-----------------------------------------------------------------------------
static void init_cube_vertices(void)
{
    float size = CUBE3D_SIZE / 2.0f;
    
    // Initialize cube vertices (8 corners)
    g_cube.vertices[0] = (Vector3D){-size, -size, -size}; // Bottom-left-back
    g_cube.vertices[1] = (Vector3D){ size, -size, -size}; // Bottom-right-back
    g_cube.vertices[2] = (Vector3D){ size,  size, -size}; // Bottom-right-front
    g_cube.vertices[3] = (Vector3D){-size,  size, -size}; // Bottom-left-front
    g_cube.vertices[4] = (Vector3D){-size, -size,  size}; // Top-left-back
    g_cube.vertices[5] = (Vector3D){ size, -size,  size}; // Top-right-back
    g_cube.vertices[6] = (Vector3D){ size,  size,  size}; // Top-right-front
    g_cube.vertices[7] = (Vector3D){-size,  size,  size}; // Top-left-front
    
    // Initialize rotation
    g_cube.rotation_x = 0.0f;
    g_cube.rotation_y = 0.0f;
    g_cube.rotation_z = 0.0f;
    
    // Set rotation speeds (in radians per frame)
    g_cube.rotation_speed_x = 0.02f;
    g_cube.rotation_speed_y = 0.03f;
    g_cube.rotation_speed_z = 0.01f;
}

//-----------------------------------------------------------------------------
static void update_cube_rotation(void)
{
    if (g_auto_rotate) {
        float scale = g_speed / (float)SPEED_DEFAULT;

        g_cube.rotation_x += g_cube.rotation_speed_x * scale;
        g_cube.rotation_y += g_cube.rotation_speed_y * scale;
        g_cube.rotation_z += g_cube.rotation_speed_z * scale;

        // Keep angles in reasonable range
        if (g_cube.rotation_x > 2 * PI) g_cube.rotation_x -= 2 * PI;
        if (g_cube.rotation_y > 2 * PI) g_cube.rotation_y -= 2 * PI;
        if (g_cube.rotation_z > 2 * PI) g_cube.rotation_z -= 2 * PI;
    }
}

//-----------------------------------------------------------------------------
static void project_cube_vertices(void)
{
    rot_t r;

    r.cx = fast_cos(g_cube.rotation_x); r.sx = fast_sin(g_cube.rotation_x);
    r.cy = fast_cos(g_cube.rotation_y); r.sy = fast_sin(g_cube.rotation_y);
    r.cz = fast_cos(g_cube.rotation_z); r.sz = fast_sin(g_cube.rotation_z);

    for (int i = 0; i < 8; i++) {
        Vector3D rotated = g_cube.vertices[i];
        rotate_point(&rotated, &r);
        g_cube.projected[i] = project_3d_to_2d(rotated);
    }
}

//-----------------------------------------------------------------------------
static void add_dirty_rect(int x, int y, int width, int height)
{
    if (g_dirty_count < MAX_DIRTY_RECTS) {
        g_dirty_rects[g_dirty_count].x = x;
        g_dirty_rects[g_dirty_count].y = y;
        g_dirty_rects[g_dirty_count].width = width;
        g_dirty_rects[g_dirty_count].height = height;
        g_dirty_rects[g_dirty_count].active = true;
        g_dirty_count++;
    }
}

//-----------------------------------------------------------------------------
static bool has_cube_moved(void)
{
    // Check if any vertex has moved
    for (int i = 0; i < 8; i++) {
        if (g_cube.projected[i].x != g_prev_projected[i].x ||
            g_cube.projected[i].y != g_prev_projected[i].y) {
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
static void calculate_dirty_rects(void)
{
    g_dirty_count = 0;
    
    if (g_first_frame) {
        // First frame - mark entire render area as dirty
        add_dirty_rect(RENDER_X, RENDER_Y, RENDER_WIDTH, RENDER_HEIGHT);
        g_first_frame = false;
        return;
    }
    
    // Only calculate dirty rects if cube has actually moved
    if (!has_cube_moved()) {
        return; // No movement, no need to redraw
    }
    
    // Calculate bounding box of movement
    int min_x = LCD_WIDTH, max_x = 0;
    int min_y = LCD_HEIGHT, max_y = 0;
    
    for (int i = 0; i < 8; i++) {
        // Check current position
        if (g_cube.projected[i].x < min_x) min_x = g_cube.projected[i].x;
        if (g_cube.projected[i].x > max_x) max_x = g_cube.projected[i].x;
        if (g_cube.projected[i].y < min_y) min_y = g_cube.projected[i].y;
        if (g_cube.projected[i].y > max_y) max_y = g_cube.projected[i].y;
        
        // Check previous position
        if (g_prev_projected[i].x < min_x) min_x = g_prev_projected[i].x;
        if (g_prev_projected[i].x > max_x) max_x = g_prev_projected[i].x;
        if (g_prev_projected[i].y < min_y) min_y = g_prev_projected[i].y;
        if (g_prev_projected[i].y > max_y) max_y = g_prev_projected[i].y;
    }
    
    // Add some padding for edges
    min_x -= 5; max_x += 5;
    min_y -= 5; max_y += 5;
    
    // Clamp to render area
    if (min_x < RENDER_X) min_x = RENDER_X;
    if (max_x >= RENDER_X + RENDER_WIDTH) max_x = RENDER_X + RENDER_WIDTH - 1;
    if (min_y < RENDER_Y) min_y = RENDER_Y;
    if (max_y >= RENDER_Y + RENDER_HEIGHT) max_y = RENDER_Y + RENDER_HEIGHT - 1;
    
    if (max_x > min_x && max_y > min_y) {
        add_dirty_rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
    }
}

//-----------------------------------------------------------------------------
static void clear_dirty_rects(void)
{
    for (int i = 0; i < g_dirty_count; i++) {
        if (g_dirty_rects[i].active) {
            lcd_fill_rect(g_dirty_rects[i].x, g_dirty_rects[i].y, 
                         g_dirty_rects[i].width, g_dirty_rects[i].height, BG_COLOR);
        }
    }
}

//-----------------------------------------------------------------------------
// Clipped to the render area: the dirty rectangles are clamped to it too, so
// anything painted outside would never be erased again
static void fb_set_pixel(int x, int y, uint16_t color)
{
    if (x >= RENDER_X && x < RENDER_X + RENDER_WIDTH &&
        y >= RENDER_Y && y < RENDER_Y + RENDER_HEIGHT) {
        lcd_draw_pixel(x, y, color);
    }
}

//-----------------------------------------------------------------------------
static void fb_draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    // Simple line drawing using Bresenham's algorithm
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0, y = y0;
    
    while (1) {
        fb_set_pixel(x, y, color);
        
        if (x == x1 && y == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

//-----------------------------------------------------------------------------
static float calculate_face_normal_z(int face_index)
{
    // Calculate Z component of face normal to determine if face is visible
    Point2D *p = g_cube.projected;
    const int *face = cube_faces[face_index];
    
    // Use cross product of two face edges
    int dx1 = p[face[1]].x - p[face[0]].x;
    int dy1 = p[face[1]].y - p[face[0]].y;
    int dx2 = p[face[2]].x - p[face[0]].x;
    int dy2 = p[face[2]].y - p[face[0]].y;
    
    // Z component of cross product (positive = facing viewer)
    return (float)(dx1 * dy2 - dy1 * dx2);
}

//-----------------------------------------------------------------------------
static void fb_fill_triangle(Point2D p1, Point2D p2, Point2D p3, uint16_t color)
{
    // Simple triangle filling using scanline algorithm
    // Sort points by Y coordinate
    if (p1.y > p2.y) { Point2D temp = p1; p1 = p2; p2 = temp; }
    if (p2.y > p3.y) { Point2D temp = p2; p2 = p3; p3 = temp; }
    if (p1.y > p2.y) { Point2D temp = p1; p1 = p2; p2 = temp; }
    
    for (int y = p1.y; y <= p3.y; y++) {
        int x_start, x_end;
        
        if (y <= p2.y) {
            // Upper triangle
            if (p2.y != p1.y) {
                x_start = p1.x + (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y);
            } else {
                x_start = p1.x;
            }
        } else {
            // Lower triangle  
            if (p3.y != p2.y) {
                x_start = p2.x + (p3.x - p2.x) * (y - p2.y) / (p3.y - p2.y);
            } else {
                x_start = p2.x;
            }
        }
        
        if (p3.y != p1.y) {
            x_end = p1.x + (p3.x - p1.x) * (y - p1.y) / (p3.y - p1.y);
        } else {
            x_end = p1.x;
        }
        
        if (x_start > x_end) {
            int temp = x_start;
            x_start = x_end;
            x_end = temp;
        }

        // One windowed transfer per scanline instead of one per pixel: on a
        // bit-banged panel the window setup dominates everything else
        if (y < RENDER_Y || y >= RENDER_Y + RENDER_HEIGHT)
            continue;

        if (x_start < RENDER_X)
            x_start = RENDER_X;

        if (x_end >= RENDER_X + RENDER_WIDTH)
            x_end = RENDER_X + RENDER_WIDTH - 1;

        if (x_start <= x_end)
            lcd_hline(x_start, x_end, y, color);
    }
}

//-----------------------------------------------------------------------------
static void fb_draw_cube_faces(void)
{
    // Draw faces with back-face culling
    for (int i = 0; i < 6; i++) {
        float normal_z = calculate_face_normal_z(i);
        
        if (normal_z > 0) { // Face is visible (facing viewer)
            const int *face = cube_faces[i];
            Point2D *p = g_cube.projected;
            
            // Draw face as two triangles
            fb_fill_triangle(p[face[0]], p[face[1]], p[face[2]], face_colors[i]);
            fb_fill_triangle(p[face[0]], p[face[2]], p[face[3]], face_colors[i]);
        }
    }
}

//-----------------------------------------------------------------------------
static void fb_draw_cube_edges(void)
{
    // Draw all edges
    for (int i = 0; i < 12; i++) {
        Point2D p1 = g_cube.projected[cube_edges[i][0]];
        Point2D p2 = g_cube.projected[cube_edges[i][1]];
        fb_draw_line(p1.x, p1.y, p2.x, p2.y, CUBE_EDGE_COLOR);
    }
}


//-----------------------------------------------------------------------------
// The status line lives outside the render area, so it is only repainted when
// something in it actually changed
// paint = how long this application takes to draw one frame.
// loop  = the longest gap between two main loop passes in the last second.
// If paint is small and loop is large, the time is going somewhere outside
// this application - that is the whole point of showing both.
static void draw_status(void)
{
    char buf[64];

    lcd_fill_rect(0, STATUS_Y - 2, LCD_WIDTH, LCD_HEIGHT - STATUS_Y + 2, BG_COLOR);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(BG_COLOR, g_auto_rotate ? LCD_GREEN_COLOR : LCD_COLOR(255, 200, 0));
    lcd_puts(10, STATUS_Y, g_auto_rotate ? "AUTO" : "MANUAL");

    lcd_set_color(BG_COLOR, LCD_WHITE_COLOR);

    snprintf(buf, sizeof(buf), "%d%%  %d fps  paint %lu ms  loop %d ms",
        g_speed, g_fps, (unsigned long)g_paint_ms, g_loop_max_ms);
    lcd_puts(64, STATUS_Y, buf);
}

//-----------------------------------------------------------------------------
static void draw_static(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);

    lcd_set_font(FONT_LARGE);
    lcd_set_color(BG_COLOR, LCD_WHITE_COLOR);
    lcd_puts(10, 8, "3D CUBE");

    draw_status();
}

//-----------------------------------------------------------------------------
static void reset_view(void)
{
    g_cube.rotation_x = 0.0f;
    g_cube.rotation_y = 0.0f;
    g_cube.rotation_z = 0.0f;
}

//-----------------------------------------------------------------------------
void cube3d_init(void)
{
    init_cube_vertices();

    timer_add(&g_frame_timer);
    g_frame_timer = 50; // ~20 FPS

    g_fps = 0;
    g_fps_frames = 0;
    g_fps_last = timer_ms();
    g_paint_ms = 0;

    draw_static();

    // Initialize dirty rectangle system
    g_dirty_count = 0;
    g_first_frame = true;

    // Initialize previous projected vertices
    for (int i = 0; i < 8; i++) {
        g_prev_projected[i].x = 0;
        g_prev_projected[i].y = 0;
    }
}

//-----------------------------------------------------------------------------
// Project and paint at the current angles, without advancing them
static void paint_cube(void)
{
    project_cube_vertices();
    calculate_dirty_rects();

    if (g_dirty_count > 0) {
        clear_dirty_rects();

        if (g_style != STYLE_WIRE)
            fb_draw_cube_faces();

        if (g_style != STYLE_SOLID)
            fb_draw_cube_edges();
    }

    for (int i = 0; i < 8; i++)
        g_prev_projected[i] = g_cube.projected[i];
}

//-----------------------------------------------------------------------------
// Called when a menu drawn on top of the cube closes. Only the dirty area
// around the cube is repainted every frame, so everything else - title, status
// line and the untouched background - has to be put back here. It paints
// immediately rather than deferring to the next tick: an overlay can close
// while another one is still open, and only the top screen is ticked.
void cube3d_redraw(void)
{
    draw_static();

    g_first_frame = true; // repaint the whole render area
    paint_cube();
}

//-----------------------------------------------------------------------------
void cube3d_task(void)
{
    if (g_frame_timer == 0) {
        uint32_t t0;

        g_frame_timer = 50; // Reset timer for next frame

        update_cube_rotation();

        t0 = timer_ms();
        paint_cube();
        g_paint_ms = timer_ms() - t0;

        g_fps_frames++;

        if (timer_ms() - g_fps_last >= 1000) {
            g_fps = g_fps_frames;
            g_fps_frames = 0;
            g_fps_last = timer_ms();
            g_loop_max_ms = timer_get_max_delta(); // reads and clears
            draw_status();
        }
    }
}

//-----------------------------------------------------------------------------
void cube3d_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    bool status_dirty = false;

    float rotation_step = 0.1f;

    if (buttons & BTN_UP) {
        g_cube.rotation_x += rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_DOWN) {
        g_cube.rotation_x -= rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_LEFT) {
        g_cube.rotation_y -= rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_RIGHT) {
        g_cube.rotation_y += rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_TRIG_UP) {
        g_cube.rotation_z += rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_TRIG_DOWN) {
        g_cube.rotation_z -= rotation_step;
        status_dirty = g_auto_rotate;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_STOP) {
        if (!repeat) {
            g_auto_rotate = !g_auto_rotate;
            status_dirty = true;
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat)
            reset_view();
    }

    // Keep angles in reasonable range
    if (g_cube.rotation_x > 2 * PI) g_cube.rotation_x -= 2 * PI;
    if (g_cube.rotation_x < 0) g_cube.rotation_x += 2 * PI;
    if (g_cube.rotation_y > 2 * PI) g_cube.rotation_y -= 2 * PI;
    if (g_cube.rotation_y < 0) g_cube.rotation_y += 2 * PI;
    if (g_cube.rotation_z > 2 * PI) g_cube.rotation_z -= 2 * PI;
    if (g_cube.rotation_z < 0) g_cube.rotation_z += 2 * PI;

    if (status_dirty)
        draw_status();
}

//-----------------------------------------------------------------------------
void cube3d_cleanup(void)
{
    g_frame_timer = TIMER_DISABLE;
    timer_remove(&g_frame_timer);
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_reset_view(const void *arg)
{
    (void)arg;

    reset_view();
    menu_close_popups(); // resume the demo so the result is visible
}

static const char *const g_help_lines[] =
{
    "UP/DOWN     - Rotate around X",
    "LEFT/RIGHT  - Rotate around Y",
    "TRIG_UP/DN  - Rotate around Z",
    "  any of these switches to manual",
    "STOP        - Auto-rotate on/off",
    "MODE        - Reset the view",
    "MENU        - This menu",
    "SHIFT+MENU  - Back to the launcher",
    "",
    "Only the area the cube moved through is",
    "repainted each frame, which is what keeps",
    "the demo smooth on a bit-banged display.",
};

static const info_page_t g_help_page =
{
    .title = "3D Cube",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const char *const g_style_labels[] = { "Solid+Wire", "Solid", "Wire" };

// The screen is repainted in full when the menu closes, so none of these
// callbacks may draw anything themselves
static const menu_item_t g_menu_items[] =
{
    { .kind = MI_TOGGLE, .label = "Auto-rotate",
      .u.toggle = { &g_auto_rotate, NULL } },
    { .kind = MI_NUMBER, .label = "Speed",
      .u.number = { &g_speed, SPEED_MIN, SPEED_MAX, 10, 25, "%", NULL } },
    { .kind = MI_CHOICE, .label = "Style",
      .u.choice = { &g_style, g_style_labels, ARRAY_SIZE(g_style_labels), NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Reset view",
      .u.action = { action_reset_view, NULL } },
};

const menu_def_t cube3d_menu =
{
    .title = "3D Cube",
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

const menu_def_t cube3d_help_menu =
{
    .title = "3D Cube",
    .items = g_help_items,
    .count = ARRAY_SIZE(g_help_items),
};
