/*
 * 3D Cube Renderer
 * Replaces oscilloscope functionality with 3D rotating cube
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "config.h"
#include "buttons.h"
#include "cube3d.h"
#include "launcher.h"

/*- Definitions -------------------------------------------------------------*/
#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)

// Cube rendering area
#define RENDER_X      0
#define RENDER_Y      30
#define RENDER_WIDTH  LCD_WIDTH
#define RENDER_HEIGHT (LCD_HEIGHT - 80)

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

/*- Variables ---------------------------------------------------------------*/
static Cube3D g_cube;
static bool g_auto_rotate = true;
static int g_frame_timer = TIMER_DISABLE;
static int g_info_timer = TIMER_DISABLE;
static bool g_info_dirty = true;

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
static void rotate_point(Vector3D *point, float rx, float ry, float rz)
{
    float cos_rx = fast_cos(rx), sin_rx = fast_sin(rx);
    float cos_ry = fast_cos(ry), sin_ry = fast_sin(ry);
    float cos_rz = fast_cos(rz), sin_rz = fast_sin(rz);
    
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
        g_cube.rotation_x += g_cube.rotation_speed_x;
        g_cube.rotation_y += g_cube.rotation_speed_y;
        g_cube.rotation_z += g_cube.rotation_speed_z;
        
        // Keep angles in reasonable range
        if (g_cube.rotation_x > 2 * PI) g_cube.rotation_x -= 2 * PI;
        if (g_cube.rotation_y > 2 * PI) g_cube.rotation_y -= 2 * PI;
        if (g_cube.rotation_z > 2 * PI) g_cube.rotation_z -= 2 * PI;
    }
}

//-----------------------------------------------------------------------------
static void project_cube_vertices(void)
{
    for (int i = 0; i < 8; i++) {
        Vector3D rotated = g_cube.vertices[i];
        rotate_point(&rotated, g_cube.rotation_x, g_cube.rotation_y, g_cube.rotation_z);
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
static void fb_set_pixel(int x, int y, uint16_t color)
{
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT) {
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
        
        for (int x = x_start; x <= x_end; x++) {
            fb_set_pixel(x, y, color);
        }
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
static void draw_info_text(void)
{
    lcd_set_font(FONT_LARGE);
    lcd_set_color(BG_COLOR, LCD_WHITE_COLOR);
    
    // Title
    lcd_puts(10, 10, "3D CUBE DEMO");
    
    // Controls info
    lcd_set_font(FONT_SMALL);
    lcd_puts(10, LCD_HEIGHT - 60, "UP/DOWN: Rotation X");
    lcd_puts(10, LCD_HEIGHT - 50, "LEFT/RIGHT: Rotation Y");
    lcd_puts(10, LCD_HEIGHT - 40, "TRIG UP/DOWN: Rotation Z");
    lcd_puts(10, LCD_HEIGHT - 30, "STOP: Toggle Auto-rotate");
    lcd_puts(10, LCD_HEIGHT - 20, "MODE: Reset rotation");
    
    // Auto-rotate status
    lcd_set_color(BG_COLOR, g_auto_rotate ? LCD_GREEN_COLOR : LCD_RED_COLOR);
    lcd_puts(10, LCD_HEIGHT - 10, g_auto_rotate ? "AUTO" : "MANUAL");
    
    lcd_set_font(FONT_LARGE);
}

//-----------------------------------------------------------------------------
void cube3d_init(void)
{
    init_cube_vertices();
    
    // Initialize timers
    timer_add(&g_frame_timer);
    timer_add(&g_info_timer);
    g_frame_timer = 50; // ~20 FPS
    g_info_timer = 500; // Update info every 500ms
    g_info_dirty = true;
    
    // Clear screen
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);
    
    draw_info_text();
    
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
void cube3d_task(void)
{
    if (g_frame_timer == 0) {
        g_frame_timer = 50; // Reset timer for next frame
        
        // Update rotation
        update_cube_rotation();
        
        // Project 3D vertices to 2D screen coordinates
        project_cube_vertices();
        
        // Calculate dirty rectangles based on movement
        calculate_dirty_rects();
        
        // Only redraw if there are dirty areas
        if (g_dirty_count > 0) {
            // Clear only dirty areas
            clear_dirty_rects();
            
            // Draw cube faces first (filled)
            fb_draw_cube_faces();
            
            // Draw cube edges on top (wireframe)
            fb_draw_cube_edges();
        }
        
        // Save current positions for next frame
        for (int i = 0; i < 8; i++) {
            g_prev_projected[i] = g_cube.projected[i];
        }
    }
}

//-----------------------------------------------------------------------------
void cube3d_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    float rotation_step = 0.1f;
    
    if (buttons & BTN_UP) {
        g_cube.rotation_x += rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_DOWN) {
        g_cube.rotation_x -= rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_LEFT) {
        g_cube.rotation_y -= rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_RIGHT) {
        g_cube.rotation_y += rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_TRIG_UP) {
        g_cube.rotation_z += rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_TRIG_DOWN) {
        g_cube.rotation_z -= rotation_step;
        g_auto_rotate = false;
    }
    else if (buttons & BTN_STOP) {
        if (!repeat) {
            g_auto_rotate = !g_auto_rotate;
            // Only redraw the status part to avoid flickering
            lcd_set_color(BG_COLOR, g_auto_rotate ? LCD_GREEN_COLOR : LCD_RED_COLOR);
            lcd_set_font(FONT_SMALL);
            lcd_puts(10, LCD_HEIGHT - 10, g_auto_rotate ? "AUTO " : "MANUAL");
            lcd_set_font(FONT_LARGE);
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat) {
            // Return to menu instead of resetting rotation
            launcher_exit_app();
        }
    }
    
    // Keep angles in reasonable range
    if (g_cube.rotation_x > 2 * PI) g_cube.rotation_x -= 2 * PI;
    if (g_cube.rotation_x < 0) g_cube.rotation_x += 2 * PI;
    if (g_cube.rotation_y > 2 * PI) g_cube.rotation_y -= 2 * PI;
    if (g_cube.rotation_y < 0) g_cube.rotation_y += 2 * PI;
    if (g_cube.rotation_z > 2 * PI) g_cube.rotation_z -= 2 * PI;
    if (g_cube.rotation_z < 0) g_cube.rotation_z += 2 * PI;
}

//-----------------------------------------------------------------------------
void cube3d_cleanup(void)
{
    g_frame_timer = TIMER_DISABLE;
    g_info_timer = TIMER_DISABLE;
}
