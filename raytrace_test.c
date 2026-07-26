/*
 * Simple Ray Tracing Test
 *
 * Renders three shaded spheres one scanline per tick so the main loop keeps
 * running. The camera basis is built once per frame (not per pixel) and each
 * finished line is pushed to the panel as a single windowed transfer.
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
#include "raytrace_test.h"

/*- Definitions -------------------------------------------------------------*/
#define STATUS_H        16
#define IMAGE_H         (LCD_HEIGHT - STATUS_H)  // 224 rows
#define STATUS_Y        (LCD_HEIGHT - STATUS_H + 4)

// Render resolutions: the image is IMAGE_H rows tall, so the scale factor has
// to divide it exactly
#define RES_LOW         0
#define RES_NORMAL      1

#define MAX_SPHERES     3
#define EPSILON         0.001f

#define ROTATE_STEP     0.15f

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    vec3_t center;
    float radius;
    uint16_t color;
} sphere_t;

typedef struct {
    vec3_t origin;
    vec3_t direction;
} ray_t;

typedef struct {
    bool hit;
    float distance;
    vec3_t point;
    vec3_t normal;
    uint16_t color;
} hit_result_t;

// Camera basis for one frame: u right, v up, w backwards
typedef struct {
    vec3_t origin;
    vec3_t u, v, w;
    float viewport_w, viewport_h;
} camera_t;

/*- Variables ---------------------------------------------------------------*/
static sphere_t g_spheres[MAX_SPHERES];
static camera_t g_camera;
static int g_current_line = 0;
static bool g_rendering = false;
static bool g_auto_rotate = false;
static int g_resolution = RES_NORMAL;
static float g_camera_angle = 0.0f;
static float g_frame_angle = 0.0f;   // angle this frame is being rendered with
static uint32_t g_render_start = 0;
static uint32_t g_render_ms = 0;

// One upscaled output row; pushed with a single lcd_draw_buf per LCD line
static uint16_t g_line_buf[LCD_WIDTH];

/*- Forward Declarations ----------------------------------------------------*/
static void start_render(void);
static void draw_status(void);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static int render_scale(void)
{
    return (RES_LOW == g_resolution) ? 4 : 2;
}

//-----------------------------------------------------------------------------
static int render_width(void)
{
    return LCD_WIDTH / render_scale();
}

//-----------------------------------------------------------------------------
static int render_height(void)
{
    return IMAGE_H / render_scale();
}

//-----------------------------------------------------------------------------
static vec3_t vec3_add(vec3_t a, vec3_t b)
{
    return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

//-----------------------------------------------------------------------------
static vec3_t vec3_sub(vec3_t a, vec3_t b)
{
    return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

//-----------------------------------------------------------------------------
static vec3_t vec3_mul(vec3_t v, float s)
{
    return (vec3_t){v.x * s, v.y * s, v.z * s};
}

//-----------------------------------------------------------------------------
static float vec3_dot(vec3_t a, vec3_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

//-----------------------------------------------------------------------------
static vec3_t vec3_cross(vec3_t a, vec3_t b)
{
    return (vec3_t){a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x};
}

//-----------------------------------------------------------------------------
static vec3_t vec3_normalize(vec3_t v)
{
    float len = sqrtf(vec3_dot(v, v));

    if (len > EPSILON)
        return vec3_mul(v, 1.0f / len);

    return (vec3_t){0, 0, 0};
}

//-----------------------------------------------------------------------------
static hit_result_t ray_sphere_intersect(ray_t ray, sphere_t sphere)
{
    hit_result_t result = {false, 0, {0, 0, 0}, {0, 0, 0}, 0};

    vec3_t oc = vec3_sub(ray.origin, sphere.center);
    float a = vec3_dot(ray.direction, ray.direction);
    float b = 2.0f * vec3_dot(oc, ray.direction);
    float c = vec3_dot(oc, oc) - sphere.radius * sphere.radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        float sqrt_discriminant = sqrtf(discriminant);
        float t1 = (-b - sqrt_discriminant) / (2 * a);
        float t2 = (-b + sqrt_discriminant) / (2 * a);

        float t = (t1 > EPSILON) ? t1 : t2;

        if (t > EPSILON) {
            result.hit = true;
            result.distance = t;
            result.point = vec3_add(ray.origin, vec3_mul(ray.direction, t));
            result.normal = vec3_normalize(vec3_sub(result.point, sphere.center));
            result.color = sphere.color;
        }
    }

    return result;
}

//-----------------------------------------------------------------------------
static uint16_t trace_ray(ray_t ray)
{
    hit_result_t closest_hit = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0};

    for (int i = 0; i < MAX_SPHERES; i++) {
        hit_result_t hit = ray_sphere_intersect(ray, g_spheres[i]);

        if (hit.hit && hit.distance < closest_hit.distance)
            closest_hit = hit;
    }

    if (!closest_hit.hit) {
        // Sky gradient
        float t = 0.5f * (ray.direction.y + 1.0f);
        uint8_t sky = (uint8_t)(255 * (0.5f + 0.5f * t));

        return LCD_COLOR(sky / 4, sky / 2, sky);
    }

    vec3_t light_dir = vec3_normalize((vec3_t){-0.5f, -1.0f, -0.5f});
    float light = fmaxf(0.15f, -vec3_dot(closest_hit.normal, light_dir));

    uint8_t r = ((closest_hit.color >> 11) & 0x1F) << 3;
    uint8_t g = ((closest_hit.color >> 5) & 0x3F) << 2;
    uint8_t b = (closest_hit.color & 0x1F) << 3;

    r = (uint8_t)(r * light);
    g = (uint8_t)(g * light);
    b = (uint8_t)(b * light);

    return LCD_COLOR(r, g, b);
}

//-----------------------------------------------------------------------------
// The camera orbits the scene; its basis is constant for a whole frame
static void setup_camera(float angle)
{
    float aspect = (float)render_width() / render_height();
    float fov = 45.0f * (float)M_PI / 180.0f;

    g_camera.viewport_h = 2.0f * tanf(fov / 2.0f);
    g_camera.viewport_w = aspect * g_camera.viewport_h;

    g_camera.origin = (vec3_t){3.0f * cosf(angle), 0.6f, 3.0f * sinf(angle)};

    vec3_t target = {0, 0, 0};
    vec3_t up = {0, 1, 0};

    g_camera.w = vec3_normalize(vec3_sub(g_camera.origin, target)); // backwards
    g_camera.u = vec3_normalize(vec3_cross(up, g_camera.w));        // right
    g_camera.v = vec3_cross(g_camera.w, g_camera.u);                // up
}

//-----------------------------------------------------------------------------
static uint16_t trace_pixel(int x, int y)
{
    float s = (x + 0.5f) / render_width();
    float t = (y + 0.5f) / render_height();

    vec3_t dir = vec3_add(vec3_add(
        vec3_mul(g_camera.u, g_camera.viewport_w * (s - 0.5f)),
        vec3_mul(g_camera.v, g_camera.viewport_h * (0.5f - t))),
        vec3_mul(g_camera.w, -1.0f));

    ray_t ray = { g_camera.origin, vec3_normalize(dir) };

    return trace_ray(ray);
}

//-----------------------------------------------------------------------------
static void setup_scene(void)
{
    g_spheres[0].center = (vec3_t){0.0f, 0.0f, 0.0f};
    g_spheres[0].radius = 0.5f;
    g_spheres[0].color = LCD_COLOR(255, 100, 100);

    g_spheres[1].center = (vec3_t){-1.0f, 0.0f, 1.0f};
    g_spheres[1].radius = 0.3f;
    g_spheres[1].color = LCD_COLOR(100, 255, 100);

    g_spheres[2].center = (vec3_t){1.0f, 0.0f, -1.0f};
    g_spheres[2].radius = 0.4f;
    g_spheres[2].color = LCD_COLOR(100, 100, 255);
}

//-----------------------------------------------------------------------------
// Trace one source line, expand it into the row buffer and blit it once per
// output row: a windowed transfer per row instead of one per pixel
static void render_line(int line)
{
    int scale = render_scale();
    int w = render_width();

    for (int x = 0; x < w; x++) {
        uint16_t color = trace_pixel(x, line);

        for (int i = 0; i < scale; i++)
            g_line_buf[x * scale + i] = color;
    }

    for (int i = 0; i < scale; i++)
        lcd_draw_buf(0, line * scale + i, LCD_WIDTH, 1, g_line_buf);
}

//-----------------------------------------------------------------------------
static void draw_status(void)
{
    char buf[48];

    lcd_fill_rect(0, LCD_HEIGHT - STATUS_H, LCD_WIDTH, STATUS_H, LCD_BLACK_COLOR);

    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);

    if (g_rendering) {
        snprintf(buf, sizeof(buf), "Rendering %d%%",
            100 * g_current_line / render_height());
    }
    else if (g_render_ms) {
        snprintf(buf, sizeof(buf), "%lu ms   MODE: render",
            (unsigned long)g_render_ms);
    }
    else {
        snprintf(buf, sizeof(buf), "MODE: render");
    }

    lcd_puts(5, STATUS_Y, buf);
    lcd_puts(150, STATUS_Y, "LEFT/RIGHT: orbit  MENU: menu");
}

//-----------------------------------------------------------------------------
static void start_render(void)
{
    // The angle is latched for the whole frame: changing it mid-render would
    // tear the image across the scanline the change landed on
    g_frame_angle = g_camera_angle;
    setup_camera(g_frame_angle);

    g_current_line = 0;
    g_rendering = true;
    g_render_start = timer_ms();

    draw_status();
}

//-----------------------------------------------------------------------------
void raytrace_test_init(void)
{
    setup_scene();

    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);

    g_render_ms = 0;

    start_render();
}

//-----------------------------------------------------------------------------
// There is no frame buffer to restore from, so the only way to repair the
// image after a menu covered it is to trace it again
void raytrace_test_redraw(void)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    start_render();
}

//-----------------------------------------------------------------------------
void raytrace_test_task(void)
{
    // One source line per main loop pass: the line itself is the time budget,
    // and buttons keep being sampled between lines
    if (!g_rendering)
        return;

    render_line(g_current_line);
    g_current_line++;

    if (g_current_line >= render_height()) {
        g_rendering = false;
        g_render_ms = timer_ms() - g_render_start;

        draw_status();

        if (g_auto_rotate) {
            g_camera_angle += ROTATE_STEP;
            start_render();
        }
    }
    else if (0 == (g_current_line & 15)) {
        draw_status(); // progress, cheap enough every 16 lines
    }
}

//-----------------------------------------------------------------------------
void raytrace_test_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);

    if (buttons & BTN_LEFT) {
        g_camera_angle -= ROTATE_STEP;

        if (!g_rendering)
            start_render();
    }
    else if (buttons & BTN_RIGHT) {
        g_camera_angle += ROTATE_STEP;

        if (!g_rendering)
            start_render();
    }
    else if (buttons & BTN_MODE) {
        if (!repeat && !g_rendering)
            start_render();
    }
}

//-----------------------------------------------------------------------------
void raytrace_test_cleanup(void)
{
    g_rendering = false;
}

//-----------------------------------------------------------------------------
// Application menu
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
static void action_render(const void *arg)
{
    (void)arg;

    g_rendering = false; // start_render() is called by the redraw that follows
    menu_close_popups();
}

static const char *const g_help_lines[] =
{
    "LEFT/RIGHT - Orbit the camera",
    "MODE       - Trace the frame again",
    "MENU       - This menu",
    "SHIFT+MENU - Back to the launcher",
    "",
    "One scanline is traced per main loop pass,",
    "so the device stays responsive while the",
    "image builds up from the top.",
    "",
    "Low resolution traces a quarter of the",
    "rays and is roughly four times faster.",
    "Auto-orbit re-traces after every frame.",
};

static const info_page_t g_help_page =
{
    .title = "Ray Trace Test",
    .lines = g_help_lines,
    .count = ARRAY_SIZE(g_help_lines),
};

static const char *const g_resolution_labels[] = { "Low", "Normal" };

static const menu_item_t g_menu_items[] =
{
    { .kind = MI_CHOICE, .label = "Resolution",
      .u.choice = { &g_resolution, g_resolution_labels,
                    ARRAY_SIZE(g_resolution_labels), NULL } },
    { .kind = MI_TOGGLE, .label = "Auto-orbit",
      .u.toggle = { &g_auto_rotate, NULL } },
    { .kind = MI_SEPARATOR },
    { .kind = MI_ACTION, .label = "Render now",
      .u.action = { action_render, NULL } },
    { .kind = MI_ACTION, .label = "Help",
      .u.action = { menu_action_info, &g_help_page } },
};

const menu_def_t raytrace_test_menu =
{
    .title = "Ray Trace Test",
    .items = g_menu_items,
    .count = ARRAY_SIZE(g_menu_items),
};
