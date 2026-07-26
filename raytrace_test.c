/*
 * Simple Ray Tracing Test
 * Basic ray tracing demonstration with spheres
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "raytrace_test.h"
#include "launcher.h"

/*- Definitions -------------------------------------------------------------*/
#define RT_WIDTH        160  // Reduced resolution for performance
#define RT_HEIGHT       120
#define RT_SCALE_X      (LCD_WIDTH / RT_WIDTH)
#define RT_SCALE_Y      (LCD_HEIGHT / RT_HEIGHT)

#define MAX_SPHERES     3
#define MAX_DEPTH       3
#define EPSILON         0.001f

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    vec3_t center;
    float radius;
    uint16_t color;
    float reflectivity;
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
    float reflectivity;
} hit_result_t;

/*- Variables ---------------------------------------------------------------*/
static sphere_t g_spheres[MAX_SPHERES];
static int g_render_timer = TIMER_DISABLE;
static int g_current_line = 0;
static bool g_rendering = false;
static float g_camera_angle = 0.0f;

/*- Forward Declarations ----------------------------------------------------*/
static vec3_t vec3_add(vec3_t a, vec3_t b);
static vec3_t vec3_sub(vec3_t a, vec3_t b);
static vec3_t vec3_mul(vec3_t v, float s);
static float vec3_dot(vec3_t a, vec3_t b);
static vec3_t vec3_normalize(vec3_t v);
static hit_result_t ray_sphere_intersect(ray_t ray, sphere_t sphere);
static hit_result_t trace_ray(ray_t ray, int depth);
static uint16_t trace_pixel(int x, int y);
static void setup_scene(void);
static void render_line(int line);

/*- Implementations ---------------------------------------------------------*/

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
static vec3_t vec3_normalize(vec3_t v)
{
    float len = sqrtf(vec3_dot(v, v));
    if (len > EPSILON) {
        return vec3_mul(v, 1.0f / len);
    }
    return (vec3_t){0, 0, 0};
}

//-----------------------------------------------------------------------------
static hit_result_t ray_sphere_intersect(ray_t ray, sphere_t sphere)
{
    hit_result_t result = {false, 0, {0, 0, 0}, {0, 0, 0}, 0, 0};
    
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
            result.reflectivity = sphere.reflectivity;
        }
    }
    
    return result;
}

//-----------------------------------------------------------------------------
static hit_result_t trace_ray(ray_t ray, int depth __attribute__((unused)))
{
    hit_result_t closest_hit = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0};
    
    // Find closest intersection
    for (int i = 0; i < MAX_SPHERES; i++) {
        hit_result_t hit = ray_sphere_intersect(ray, g_spheres[i]);
        if (hit.hit && hit.distance < closest_hit.distance) {
            closest_hit = hit;
        }
    }
    
    if (!closest_hit.hit) {
        // Sky color (gradient)
        float t = 0.5f * (ray.direction.y + 1.0f);
        uint8_t sky_intensity = (uint8_t)(255 * (0.5f + 0.5f * t));
        closest_hit.color = LCD_COLOR(sky_intensity/4, sky_intensity/2, sky_intensity);
        return closest_hit;
    }
    
    // Simple lighting
    vec3_t light_dir = vec3_normalize((vec3_t){-0.5f, -1.0f, -0.5f});
    float light_intensity = fmaxf(0.1f, -vec3_dot(closest_hit.normal, light_dir));
    
    // Extract RGB components
    uint8_t r = ((closest_hit.color >> 11) & 0x1F) << 3;
    uint8_t g = ((closest_hit.color >> 5) & 0x3F) << 2;
    uint8_t b = (closest_hit.color & 0x1F) << 3;
    
    // Apply lighting
    r = (uint8_t)(r * light_intensity);
    g = (uint8_t)(g * light_intensity);
    b = (uint8_t)(b * light_intensity);
    
    closest_hit.color = LCD_COLOR(r, g, b);
    
    return closest_hit;
}

//-----------------------------------------------------------------------------
static uint16_t trace_pixel(int x, int y)
{
    // Camera setup
    float aspect_ratio = (float)RT_WIDTH / RT_HEIGHT;
    float fov = 45.0f * M_PI / 180.0f;
    float viewport_height = 2.0f * tanf(fov / 2.0f);
    float viewport_width = aspect_ratio * viewport_height;
    
    // Camera position (rotating around scene)
    vec3_t camera_pos = {
        3.0f * cosf(g_camera_angle),
        0.0f,
        3.0f * sinf(g_camera_angle)
    };
    
    vec3_t camera_target = {0, 0, 0};
    vec3_t camera_up = {0, 1, 0};
    
    // Camera coordinate system
    vec3_t w = vec3_normalize(vec3_sub(camera_pos, camera_target));
    vec3_t u = vec3_normalize(vec3_sub((vec3_t){0, 0, 0}, 
                              (vec3_t){w.y * camera_up.z - w.z * camera_up.y,
                                       w.z * camera_up.x - w.x * camera_up.z,
                                       w.x * camera_up.y - w.y * camera_up.x}));
    vec3_t v = (vec3_t){u.y * w.z - u.z * w.y,
                        u.z * w.x - u.x * w.z,
                        u.x * w.y - u.y * w.x};
    
    // Ray direction
    float s = (x + 0.5f) / RT_WIDTH;
    float t = (y + 0.5f) / RT_HEIGHT;
    
    vec3_t ray_dir = vec3_normalize(vec3_add(vec3_add(
        vec3_mul(u, viewport_width * (s - 0.5f)),
        vec3_mul(v, viewport_height * (0.5f - t))),
        vec3_mul(w, -1.0f)));
    
    ray_t ray = {camera_pos, ray_dir};
    
    hit_result_t result = trace_ray(ray, 0);
    return result.color;
}

//-----------------------------------------------------------------------------
static void setup_scene(void)
{
    // Sphere 1: Red
    g_spheres[0].center = (vec3_t){0.0f, 0.0f, 0.0f};
    g_spheres[0].radius = 0.5f;
    g_spheres[0].color = LCD_COLOR(255, 100, 100);
    g_spheres[0].reflectivity = 0.3f;
    
    // Sphere 2: Green
    g_spheres[1].center = (vec3_t){-1.0f, 0.0f, 1.0f};
    g_spheres[1].radius = 0.3f;
    g_spheres[1].color = LCD_COLOR(100, 255, 100);
    g_spheres[1].reflectivity = 0.5f;
    
    // Sphere 3: Blue
    g_spheres[2].center = (vec3_t){1.0f, 0.0f, -1.0f};
    g_spheres[2].radius = 0.4f;
    g_spheres[2].color = LCD_COLOR(100, 100, 255);
    g_spheres[2].reflectivity = 0.2f;
}

//-----------------------------------------------------------------------------
static void render_line(int line)
{
    for (int x = 0; x < RT_WIDTH; x++) {
        uint16_t color = trace_pixel(x, line);
        
        // Scale up to LCD resolution
        for (int sy = 0; sy < RT_SCALE_Y; sy++) {
            for (int sx = 0; sx < RT_SCALE_X; sx++) {
                int lcd_x = x * RT_SCALE_X + sx;
                int lcd_y = line * RT_SCALE_Y + sy;
                if (lcd_x < LCD_WIDTH && lcd_y < LCD_HEIGHT) {
                    lcd_draw_pixel(lcd_x, lcd_y, color);
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
static void draw_info(void)
{
    lcd_set_font(FONT_SMALL);
    lcd_set_color(LCD_BLACK_COLOR, LCD_WHITE_COLOR);
    
    lcd_puts(5, 5, "RAY TRACE TEST");
    lcd_puts(5, 15, "LEFT/RIGHT: Rotate camera");
    lcd_puts(5, 25, "MODE: Return to menu");
    
    if (g_rendering) {
        lcd_puts(5, LCD_HEIGHT - 15, "Rendering...");
    } else {
        lcd_puts(5, LCD_HEIGHT - 15, "Press UP to render");
    }
}

//-----------------------------------------------------------------------------
void raytrace_test_init(void)
{
    setup_scene();
    
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_BLACK_COLOR);
    draw_info();
    
    g_current_line = 0;
    g_rendering = false;
    g_camera_angle = 0.0f;
    
    timer_add(&g_render_timer);
    g_render_timer = 10; // Fast rendering
}

//-----------------------------------------------------------------------------
void raytrace_test_task(void)
{
    if (g_render_timer == 0) {
        g_render_timer = 10; // Reset timer
        
        if (g_rendering) {
            render_line(g_current_line);
            g_current_line++;
            
            if (g_current_line >= RT_HEIGHT) {
                g_rendering = false;
                g_current_line = 0;
                draw_info(); // Update status
            }
        }
    }
}

//-----------------------------------------------------------------------------
void raytrace_test_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_LEFT) {
        g_camera_angle -= 0.1f;
        if (!g_rendering) {
            g_rendering = true;
            g_current_line = 0;
            draw_info();
        }
    }
    else if (buttons & BTN_RIGHT) {
        g_camera_angle += 0.1f;
        if (!g_rendering) {
            g_rendering = true;
            g_current_line = 0;
            draw_info();
        }
    }
    else if (buttons & BTN_UP) {
        if (!repeat && !g_rendering) {
            g_rendering = true;
            g_current_line = 0;
            draw_info();
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat) {
            launcher_exit_app();
        }
    }
}

//-----------------------------------------------------------------------------
void raytrace_test_cleanup(void)
{
    g_render_timer = TIMER_DISABLE;
}
