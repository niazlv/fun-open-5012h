/*
 * Advanced 3D Engine
 * Full-featured 3D graphics engine with camera control, scene management,
 * RayCast and RayTrace rendering modes
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "config.h"
#include "buttons.h"
#include "engine3d.h"
#include "launcher.h"

/*- Definitions -------------------------------------------------------------*/
#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)
#define EPSILON 0.001f

// Rendering settings
#define DEFAULT_RENDER_WIDTH    160  // Better resolution
#define DEFAULT_RENDER_HEIGHT   120  // Better resolution
#define DEFAULT_RENDER_SCALE    2    // Normal scale
#define MAX_RAY_DEPTH          2     // Reduced depth for performance
#define LINES_PER_FRAME        2     // Render only 2 lines per frame for better quality

// Camera settings
#define DEFAULT_FOV            (60.0f * DEG_TO_RAD)
#define DEFAULT_NEAR_PLANE     0.1f
#define DEFAULT_FAR_PLANE      100.0f
#define CAMERA_MOVE_SPEED      0.1f
#define CAMERA_ROTATE_SPEED    0.05f

// Colors
#define BG_COLOR               LCD_COLOR(20, 20, 40)
#define WIREFRAME_COLOR        LCD_COLOR(255, 255, 255)
#define NORMAL_COLOR           LCD_COLOR(255, 255, 0)

/*- Variables ---------------------------------------------------------------*/
static engine3d_t g_engine;
static int g_frame_timer = TIMER_DISABLE;
static int g_fps_timer = TIMER_DISABLE;
static uint32_t g_frame_start_time = 0;
static bool g_scene_dirty = true;

// Progressive rendering state
static int g_current_render_line = 0;
static bool g_rendering_in_progress = false;
static bool g_frame_complete = false;

// Rendering buffers
static uint16_t *g_framebuffer = NULL;
static float *g_depth_buffer = NULL;

/*- Forward Declarations ----------------------------------------------------*/
static void setup_default_scene(void);
static void update_camera(void);
static void render_frame(void);
static void render_raycast(void);
static void render_raytrace(void);
static void draw_ui(void);
static void calculate_fps(void);

// Math functions
static vec3_t transform_point(vec3_t point, vec3_t position, vec3_t rotation, vec3_t scale);
static vec3_t world_to_screen(vec3_t world_pos);
static ray_t screen_to_ray(int x, int y);

// Intersection functions
static hit_result_t intersect_sphere(ray_t ray, object_t *obj);
static hit_result_t intersect_cube(ray_t ray, object_t *obj);
static hit_result_t intersect_plane(ray_t ray, object_t *obj);
static hit_result_t intersect_triangle(ray_t ray, object_t *obj);

// Lighting
static vec3_t calculate_lighting(hit_result_t hit, vec3_t view_dir);

/*- Math Utility Functions -------------------------------------------------*/

//-----------------------------------------------------------------------------
vec3_t vec3_add(vec3_t a, vec3_t b)
{
    return (vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

//-----------------------------------------------------------------------------
vec3_t vec3_sub(vec3_t a, vec3_t b)
{
    return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

//-----------------------------------------------------------------------------
vec3_t vec3_mul(vec3_t v, float s)
{
    return (vec3_t){v.x * s, v.y * s, v.z * s};
}

//-----------------------------------------------------------------------------
vec3_t vec3_cross(vec3_t a, vec3_t b)
{
    return (vec3_t){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

//-----------------------------------------------------------------------------
float vec3_dot(vec3_t a, vec3_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

//-----------------------------------------------------------------------------
float vec3_length(vec3_t v)
{
    return sqrtf(vec3_dot(v, v));
}

//-----------------------------------------------------------------------------
vec3_t vec3_normalize(vec3_t v)
{
    float len = vec3_length(v);
    if (len > EPSILON) {
        return vec3_mul(v, 1.0f / len);
    }
    return (vec3_t){0, 0, 0};
}

//-----------------------------------------------------------------------------
vec3_t vec3_rotate_x(vec3_t v, float angle)
{
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    return (vec3_t){
        v.x,
        v.y * cos_a - v.z * sin_a,
        v.y * sin_a + v.z * cos_a
    };
}

//-----------------------------------------------------------------------------
vec3_t vec3_rotate_y(vec3_t v, float angle)
{
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    return (vec3_t){
        v.x * cos_a + v.z * sin_a,
        v.y,
        -v.x * sin_a + v.z * cos_a
    };
}

//-----------------------------------------------------------------------------
vec3_t vec3_rotate_z(vec3_t v, float angle)
{
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    return (vec3_t){
        v.x * cos_a - v.y * sin_a,
        v.x * sin_a + v.y * cos_a,
        v.z
    };
}

/*- Transform Functions -----------------------------------------------------*/

//-----------------------------------------------------------------------------
static vec3_t transform_point(vec3_t point, vec3_t position, vec3_t rotation, vec3_t scale)
{
    // Apply scale
    point.x *= scale.x;
    point.y *= scale.y;
    point.z *= scale.z;
    
    // Apply rotation
    point = vec3_rotate_x(point, rotation.x);
    point = vec3_rotate_y(point, rotation.y);
    point = vec3_rotate_z(point, rotation.z);
    
    // Apply translation
    return vec3_add(point, position);
}

//-----------------------------------------------------------------------------
static vec3_t world_to_screen(vec3_t world_pos)
{
    // Transform to camera space
    vec3_t cam_pos = vec3_sub(world_pos, g_engine.camera.position);
    
    // Apply camera rotation (inverse)
    cam_pos = vec3_rotate_y(cam_pos, -g_engine.camera.rotation.y);
    cam_pos = vec3_rotate_x(cam_pos, -g_engine.camera.rotation.x);
    cam_pos = vec3_rotate_z(cam_pos, -g_engine.camera.rotation.z);
    
    // Perspective projection
    if (cam_pos.z <= g_engine.camera.near_plane) {
        return (vec3_t){-1, -1, -1}; // Behind camera
    }
    
    float aspect = (float)g_engine.render_width / g_engine.render_height;
    float tan_half_fov = tanf(g_engine.camera.fov * 0.5f);
    
    float x = cam_pos.x / (cam_pos.z * tan_half_fov * aspect);
    float y = cam_pos.y / (cam_pos.z * tan_half_fov);
    
    // Convert to screen coordinates
    x = (x + 1.0f) * 0.5f * g_engine.render_width;
    y = (1.0f - y) * 0.5f * g_engine.render_height;
    
    return (vec3_t){x, y, cam_pos.z};
}

//-----------------------------------------------------------------------------
static ray_t screen_to_ray(int x, int y)
{
    ray_t ray;
    ray.origin = g_engine.camera.position;
    
    // Convert screen coordinates to normalized device coordinates
    float ndc_x = (2.0f * x / g_engine.render_width) - 1.0f;
    float ndc_y = 1.0f - (2.0f * y / g_engine.render_height);
    
    // Calculate ray direction
    float aspect = (float)g_engine.render_width / g_engine.render_height;
    float tan_half_fov = tanf(g_engine.camera.fov * 0.5f);
    
    vec3_t dir = {
        ndc_x * tan_half_fov * aspect,
        ndc_y * tan_half_fov,
        -1.0f
    };
    
    // Apply camera rotation
    dir = vec3_rotate_z(dir, g_engine.camera.rotation.z);
    dir = vec3_rotate_x(dir, g_engine.camera.rotation.x);
    dir = vec3_rotate_y(dir, g_engine.camera.rotation.y);
    
    ray.direction = vec3_normalize(dir);
    return ray;
}

/*- Intersection Functions -------------------------------------------------*/

//-----------------------------------------------------------------------------
static hit_result_t intersect_sphere(ray_t ray, object_t *obj)
{
    hit_result_t result = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
    
    vec3_t oc = vec3_sub(ray.origin, obj->position);
    float a = vec3_dot(ray.direction, ray.direction);
    float b = 2.0f * vec3_dot(oc, ray.direction);
    float c = vec3_dot(oc, oc) - obj->sphere.radius * obj->sphere.radius;
    
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
            result.normal = vec3_normalize(vec3_sub(result.point, obj->position));
            result.color = obj->color;
            result.reflectivity = obj->reflectivity;
            result.roughness = obj->roughness;
            result.object = obj;
        }
    }
    
    return result;
}

//-----------------------------------------------------------------------------
static hit_result_t intersect_cube(ray_t ray, object_t *obj)
{
    hit_result_t result = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
    
    // Transform ray to object space
    vec3_t local_origin = vec3_sub(ray.origin, obj->position);
    vec3_t local_dir = ray.direction;
    
    // Apply inverse rotation (simplified for axis-aligned cube)
    // For now, assume no rotation for performance
    
    float half_size = obj->cube.size * 0.5f;
    vec3_t box_min = {-half_size, -half_size, -half_size};
    vec3_t box_max = {half_size, half_size, half_size};
    
    // Ray-box intersection using slab method
    float t_min = 0.0f;
    float t_max = INFINITY;
    
    for (int i = 0; i < 3; i++) {
        float dir_comp = (i == 0) ? local_dir.x : (i == 1) ? local_dir.y : local_dir.z;
        float origin_comp = (i == 0) ? local_origin.x : (i == 1) ? local_origin.y : local_origin.z;
        float box_min_comp = (i == 0) ? box_min.x : (i == 1) ? box_min.y : box_min.z;
        float box_max_comp = (i == 0) ? box_max.x : (i == 1) ? box_max.y : box_max.z;
        
        if (fabsf(dir_comp) < EPSILON) {
            // Ray is parallel to slab
            if (origin_comp < box_min_comp || origin_comp > box_max_comp) {
                return result; // No intersection
            }
        } else {
            float t1 = (box_min_comp - origin_comp) / dir_comp;
            float t2 = (box_max_comp - origin_comp) / dir_comp;
            
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            
            t_min = fmaxf(t_min, t1);
            t_max = fminf(t_max, t2);
            
            if (t_min > t_max) {
                return result; // No intersection
            }
        }
    }
    
    float t = (t_min > EPSILON) ? t_min : t_max;
    if (t > EPSILON) {
        result.hit = true;
        result.distance = t;
        result.point = vec3_add(ray.origin, vec3_mul(ray.direction, t));
        
        // Calculate normal (simplified)
        vec3_t local_hit = vec3_sub(result.point, obj->position);
        vec3_t abs_hit = {fabsf(local_hit.x), fabsf(local_hit.y), fabsf(local_hit.z)};
        
        if (abs_hit.x > abs_hit.y && abs_hit.x > abs_hit.z) {
            result.normal = (vec3_t){(local_hit.x > 0) ? 1.0f : -1.0f, 0, 0};
        } else if (abs_hit.y > abs_hit.z) {
            result.normal = (vec3_t){0, (local_hit.y > 0) ? 1.0f : -1.0f, 0};
        } else {
            result.normal = (vec3_t){0, 0, (local_hit.z > 0) ? 1.0f : -1.0f};
        }
        
        result.color = obj->color;
        result.reflectivity = obj->reflectivity;
        result.roughness = obj->roughness;
        result.object = obj;
    }
    
    return result;
}

//-----------------------------------------------------------------------------
static hit_result_t intersect_plane(ray_t ray, object_t *obj)
{
    hit_result_t result = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
    
    float denom = vec3_dot(obj->plane.normal, ray.direction);
    
    if (fabsf(denom) > EPSILON) {
        vec3_t p0 = vec3_mul(obj->plane.normal, obj->plane.distance);
        vec3_t p0l0 = vec3_sub(p0, ray.origin);
        float t = vec3_dot(p0l0, obj->plane.normal) / denom;
        
        if (t > EPSILON) {
            vec3_t hit_point = vec3_add(ray.origin, vec3_mul(ray.direction, t));
            
            // Check if hit point is within plane bounds
            vec3_t local_hit = vec3_sub(hit_point, obj->position);
            if (fabsf(local_hit.x) <= obj->plane.width * 0.5f && 
                fabsf(local_hit.z) <= obj->plane.height * 0.5f) {
                
                result.hit = true;
                result.distance = t;
                result.point = hit_point;
                result.normal = obj->plane.normal;
                result.color = obj->color;
                result.reflectivity = obj->reflectivity;
                result.roughness = obj->roughness;
                result.object = obj;
            }
        }
    }
    
    return result;
}

//-----------------------------------------------------------------------------
static hit_result_t intersect_triangle(ray_t ray, object_t *obj)
{
    hit_result_t result = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
    
    // Möller-Trumbore intersection algorithm
    vec3_t edge1 = vec3_sub(obj->triangle.v1, obj->triangle.v0);
    vec3_t edge2 = vec3_sub(obj->triangle.v2, obj->triangle.v0);
    vec3_t h = vec3_cross(ray.direction, edge2);
    float a = vec3_dot(edge1, h);
    
    if (fabsf(a) < EPSILON) {
        return result; // Ray is parallel to triangle
    }
    
    float f = 1.0f / a;
    vec3_t s = vec3_sub(ray.origin, obj->triangle.v0);
    float u = f * vec3_dot(s, h);
    
    if (u < 0.0f || u > 1.0f) {
        return result;
    }
    
    vec3_t q = vec3_cross(s, edge1);
    float v = f * vec3_dot(ray.direction, q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return result;
    }
    
    float t = f * vec3_dot(edge2, q);
    
    if (t > EPSILON) {
        result.hit = true;
        result.distance = t;
        result.point = vec3_add(ray.origin, vec3_mul(ray.direction, t));
        result.normal = vec3_normalize(vec3_cross(edge1, edge2));
        result.color = obj->color;
        result.reflectivity = obj->reflectivity;
        result.roughness = obj->roughness;
        result.object = obj;
    }
    
    return result;
}

/*- Ray Tracing Functions --------------------------------------------------*/

//-----------------------------------------------------------------------------
hit_result_t engine3d_cast_ray(ray_t ray)
{
    hit_result_t closest_hit = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
    
    for (int i = 0; i < g_engine.object_count; i++) {
        if (!g_engine.objects[i].enabled) continue;
        
        hit_result_t hit = {false, INFINITY, {0, 0, 0}, {0, 0, 0}, 0, 0, 0, NULL};
        
        switch (g_engine.objects[i].type) {
            case OBJECT_SPHERE:
                hit = intersect_sphere(ray, &g_engine.objects[i]);
                break;
            case OBJECT_CUBE:
                hit = intersect_cube(ray, &g_engine.objects[i]);
                break;
            case OBJECT_PLANE:
                hit = intersect_plane(ray, &g_engine.objects[i]);
                break;
            case OBJECT_TRIANGLE:
                hit = intersect_triangle(ray, &g_engine.objects[i]);
                break;
        }
        
        if (hit.hit && hit.distance < closest_hit.distance) {
            closest_hit = hit;
        }
    }
    
    return closest_hit;
}

//-----------------------------------------------------------------------------
static vec3_t refract_ray(vec3_t incident, vec3_t normal, float n1, float n2)
{
    float n = n1 / n2;
    float cos_i = -vec3_dot(normal, incident);
    float sin_t2 = n * n * (1.0f - cos_i * cos_i);
    
    if (sin_t2 > 1.0f) {
        // Total internal reflection
        return (vec3_t){0, 0, 0};
    }
    
    float cos_t = sqrtf(1.0f - sin_t2);
    return vec3_add(vec3_mul(incident, n), vec3_mul(normal, n * cos_i - cos_t));
}

//-----------------------------------------------------------------------------
hit_result_t engine3d_trace_ray(ray_t ray, int depth)
{
    if (depth <= 0) {
        return (hit_result_t){false, INFINITY, {0, 0, 0}, {0, 0, 0}, BG_COLOR, 0, 0, NULL};
    }
    
    hit_result_t hit = engine3d_cast_ray(ray);
    
    if (!hit.hit) {
        // Sky gradient
        float t = 0.5f * (ray.direction.y + 1.0f);
        uint8_t sky_intensity = (uint8_t)(100 + 155 * t);
        return (hit_result_t){false, INFINITY, {0, 0, 0}, {0, 0, 0}, 
                             LCD_COLOR(sky_intensity/4, sky_intensity/3, sky_intensity), 0, 0, NULL};
    }
    
    // Calculate lighting
    vec3_t view_dir = vec3_mul(ray.direction, -1.0f);
    vec3_t lit_color = calculate_lighting(hit, view_dir);
    
    // Apply lighting to base color
    uint8_t r = ((hit.color >> 11) & 0x1F) << 3;
    uint8_t g = ((hit.color >> 5) & 0x3F) << 2;
    uint8_t b = (hit.color & 0x1F) << 3;
    
    r = (uint8_t)(r * lit_color.x);
    g = (uint8_t)(g * lit_color.y);
    b = (uint8_t)(b * lit_color.z);
    
    // Add emission (glow)
    if (hit.object->emission > 0.0f) {
        uint8_t er = (uint8_t)(255 * hit.object->emission_color.x * hit.object->emission);
        uint8_t eg = (uint8_t)(255 * hit.object->emission_color.y * hit.object->emission);
        uint8_t eb = (uint8_t)(255 * hit.object->emission_color.z * hit.object->emission);
        
        r = (uint8_t)fminf(255, r + er);
        g = (uint8_t)fminf(255, g + eg);
        b = (uint8_t)fminf(255, b + eb);
    }
    
    hit.color = LCD_COLOR(r, g, b);
    
    // Handle transparency and refraction
    if (hit.object->transparency > 0.0f && depth > 1) {
        vec3_t refract_dir = refract_ray(ray.direction, hit.normal, 1.0f, hit.object->refraction_index);
        
        if (vec3_length(refract_dir) > EPSILON) {
            // Refraction ray
            ray_t refract_ray = {
                vec3_add(hit.point, vec3_mul(hit.normal, -EPSILON)),
                refract_dir
            };
            
            hit_result_t refract_hit = engine3d_trace_ray(refract_ray, depth - 1);
            
            // Blend refracted color with surface color
            uint8_t hr = ((hit.color >> 11) & 0x1F) << 3;
            uint8_t hg = ((hit.color >> 5) & 0x3F) << 2;
            uint8_t hb = (hit.color & 0x1F) << 3;
            
            uint8_t tr = ((refract_hit.color >> 11) & 0x1F) << 3;
            uint8_t tg = ((refract_hit.color >> 5) & 0x3F) << 2;
            uint8_t tb = (refract_hit.color & 0x1F) << 3;
            
            float transparency = hit.object->transparency;
            r = (uint8_t)(hr * (1.0f - transparency) + tr * transparency);
            g = (uint8_t)(hg * (1.0f - transparency) + tg * transparency);
            b = (uint8_t)(hb * (1.0f - transparency) + tb * transparency);
            
            hit.color = LCD_COLOR(r, g, b);
        }
    }
    
    // Handle reflections
    if (hit.reflectivity > 0.0f && depth > 1) {
        vec3_t reflect_dir = vec3_sub(ray.direction, 
                                     vec3_mul(hit.normal, 2.0f * vec3_dot(ray.direction, hit.normal)));
        ray_t reflect_ray = {
            vec3_add(hit.point, vec3_mul(hit.normal, EPSILON)),
            reflect_dir
        };
        
        hit_result_t reflect_hit = engine3d_trace_ray(reflect_ray, depth - 1);
        if (reflect_hit.hit || reflect_hit.color != BG_COLOR) {
            // Blend reflection
            uint8_t hr = ((hit.color >> 11) & 0x1F) << 3;
            uint8_t hg = ((hit.color >> 5) & 0x3F) << 2;
            uint8_t hb = (hit.color & 0x1F) << 3;
            
            uint8_t rr = ((reflect_hit.color >> 11) & 0x1F) << 3;
            uint8_t rg = ((reflect_hit.color >> 5) & 0x3F) << 2;
            uint8_t rb = (reflect_hit.color & 0x1F) << 3;
            
            float blend = hit.reflectivity;
            r = (uint8_t)(hr * (1.0f - blend) + rr * blend);
            g = (uint8_t)(hg * (1.0f - blend) + rg * blend);
            b = (uint8_t)(hb * (1.0f - blend) + rb * blend);
            
            hit.color = LCD_COLOR(r, g, b);
        }
    }
    
    return hit;
}

/*- Lighting Functions -----------------------------------------------------*/

//-----------------------------------------------------------------------------
static vec3_t calculate_lighting(hit_result_t hit, vec3_t view_dir)
{
    vec3_t total_light = {0.1f, 0.1f, 0.1f}; // Ambient light
    
    for (int i = 0; i < g_engine.light_count; i++) {
        if (!g_engine.lights[i].enabled) continue;
        
        light_t *light = &g_engine.lights[i];
        vec3_t light_dir;
        float attenuation = 1.0f;
        
        if (light->type == LIGHT_DIRECTIONAL) {
            light_dir = vec3_mul(light->direction, -1.0f);
        } else {
            light_dir = vec3_sub(light->position, hit.point);
            float distance = vec3_length(light_dir);
            light_dir = vec3_normalize(light_dir);
            attenuation = 1.0f / (1.0f + 0.1f * distance + 0.01f * distance * distance);
        }
        
        // Diffuse lighting
        float diffuse = fmaxf(0.0f, vec3_dot(hit.normal, light_dir));
        
        // Specular lighting (Blinn-Phong)
        vec3_t half_dir = vec3_normalize(vec3_add(light_dir, view_dir));
        float specular = powf(fmaxf(0.0f, vec3_dot(hit.normal, half_dir)), 32.0f);
        
        vec3_t light_contribution = {
            light->color.x * light->intensity * attenuation * (diffuse + specular * 0.5f),
            light->color.y * light->intensity * attenuation * (diffuse + specular * 0.5f),
            light->color.z * light->intensity * attenuation * (diffuse + specular * 0.5f)
        };
        
        total_light = vec3_add(total_light, light_contribution);
    }
    
    // Clamp to [0, 1]
    total_light.x = fminf(1.0f, fmaxf(0.0f, total_light.x));
    total_light.y = fminf(1.0f, fmaxf(0.0f, total_light.y));
    total_light.z = fminf(1.0f, fmaxf(0.0f, total_light.z));
    
    return total_light;
}

/*- Rendering Functions ----------------------------------------------------*/

//-----------------------------------------------------------------------------
static void render_raycast_progressive(void)
{
    // Progressive raycast rendering - render only a few lines per frame
    int lines_to_render = LINES_PER_FRAME;
    int end_line = g_current_render_line + lines_to_render;
    if (end_line > g_engine.render_height) {
        end_line = g_engine.render_height;
    }
    
    for (int y = g_current_render_line; y < end_line; y++) {
        for (int x = 0; x < g_engine.render_width; x++) {
            ray_t ray = screen_to_ray(x, y);
            hit_result_t hit = engine3d_cast_ray(ray);
            
            uint16_t color;
            if (hit.hit) {
                vec3_t view_dir = vec3_mul(ray.direction, -1.0f);
                vec3_t lit_color = calculate_lighting(hit, view_dir);
                
                uint8_t r = ((hit.color >> 11) & 0x1F) << 3;
                uint8_t g = ((hit.color >> 5) & 0x3F) << 2;
                uint8_t b = (hit.color & 0x1F) << 3;
                
                r = (uint8_t)(r * lit_color.x);
                g = (uint8_t)(g * lit_color.y);
                b = (uint8_t)(b * lit_color.z);
                
                color = LCD_COLOR(r, g, b);
            } else {
                // Sky gradient
                float t = 0.5f * (ray.direction.y + 1.0f);
                uint8_t sky_intensity = (uint8_t)(100 + 155 * t);
                color = LCD_COLOR(sky_intensity/4, sky_intensity/3, sky_intensity);
            }
            
            // Scale up to LCD resolution
            for (int sy = 0; sy < g_engine.render_scale; sy++) {
                for (int sx = 0; sx < g_engine.render_scale; sx++) {
                    int lcd_x = x * g_engine.render_scale + sx;
                    int lcd_y = y * g_engine.render_scale + sy;
                    if (lcd_x < LCD_WIDTH && lcd_y < LCD_HEIGHT) {
                        lcd_draw_pixel(lcd_x, lcd_y, color);
                    }
                }
            }
        }
    }
    
    g_current_render_line = end_line;
    if (g_current_render_line >= g_engine.render_height) {
        g_frame_complete = true;
        g_rendering_in_progress = false;
        g_current_render_line = 0;
    }
}

//-----------------------------------------------------------------------------
static void render_raytrace_progressive(void)
{
    // Progressive raytracing - render only 2 lines per frame for better performance
    int lines_to_render = (g_engine.render_mode == RENDER_MODE_RAYTRACE) ? 2 : LINES_PER_FRAME;
    int end_line = g_current_render_line + lines_to_render;
    if (end_line > g_engine.render_height) {
        end_line = g_engine.render_height;
    }
    
    for (int y = g_current_render_line; y < end_line; y++) {
        for (int x = 0; x < g_engine.render_width; x++) {
            ray_t ray = screen_to_ray(x, y);
            hit_result_t hit = engine3d_trace_ray(ray, MAX_RAY_DEPTH);
            
            uint16_t color = hit.color;
            
            // Scale up to LCD resolution
            for (int sy = 0; sy < g_engine.render_scale; sy++) {
                for (int sx = 0; sx < g_engine.render_scale; sx++) {
                    int lcd_x = x * g_engine.render_scale + sx;
                    int lcd_y = y * g_engine.render_scale + sy;
                    if (lcd_x < LCD_WIDTH && lcd_y < LCD_HEIGHT) {
                        lcd_draw_pixel(lcd_x, lcd_y, color);
                    }
                }
            }
        }
    }
    
    g_current_render_line = end_line;
    if (g_current_render_line >= g_engine.render_height) {
        g_frame_complete = true;
        g_rendering_in_progress = false;
        g_current_render_line = 0;
    }
}

//-----------------------------------------------------------------------------
static void render_frame(void)
{
    // Start new frame if not currently rendering
    if (!g_rendering_in_progress) {
        g_rendering_in_progress = true;
        g_current_render_line = 0;
        g_frame_complete = false;
    }
    
    // Render progressively
    if (g_engine.render_mode == RENDER_MODE_RAYCAST) {
        render_raycast_progressive();
    } else {
        render_raytrace_progressive();
    }
}

//-----------------------------------------------------------------------------
static void draw_ui(void)
{
    lcd_set_font(FONT_SMALL);
    lcd_set_color(BG_COLOR, LCD_WHITE_COLOR);
    
    // FPS counter in top-left corner
    char fps_str[16];
    snprintf(fps_str, sizeof(fps_str), "%lu FPS", g_engine.current_fps);
    lcd_puts(5, 5, fps_str);
    
    // Render mode
    lcd_puts(5, 15, g_engine.render_mode == RENDER_MODE_RAYCAST ? "RAYCAST" : "RAYTRACE");
    
    // Camera info
    // Formatted from integer tenths: newlib's %f goes through malloc-backed
    // _dtoa, and the heap shares the same scarce TCM as the stack
    char cam_str[64];
    int cx = (int)(g_engine.camera.position.x * 10.0f);
    int cy = (int)(g_engine.camera.position.y * 10.0f);
    int cz = (int)(g_engine.camera.position.z * 10.0f);
    snprintf(cam_str, sizeof(cam_str), "X:%d.%d Y:%d.%d Z:%d.%d",
             cx / 10, cx < 0 ? -cx % 10 : cx % 10,
             cy / 10, cy < 0 ? -cy % 10 : cy % 10,
             cz / 10, cz < 0 ? -cz % 10 : cz % 10);
    lcd_puts(5, LCD_HEIGHT - 25, cam_str);
    
    // Controls
    lcd_puts(5, LCD_HEIGHT - 15, "F1:Mode WASD:Move QE:UpDown");
}

//-----------------------------------------------------------------------------
static void calculate_fps(void)
{
    static uint32_t fps_counter = 0;
    static uint32_t last_fps_update = 0;
    
    g_engine.frame_count++;
    fps_counter++;
    
    // Simple FPS calculation based on frame timer
    if (fps_counter >= 20) { // Update FPS every 20 frames
        // Estimate FPS based on frame timer interval
        if (g_engine.render_mode == RENDER_MODE_RAYCAST) {
            g_engine.current_fps = 1000 / 50; // 50ms per frame = 20 FPS
        } else {
            g_engine.current_fps = 1000 / 200; // 200ms per frame = 5 FPS
        }
        fps_counter = 0;
    }
}

/*- Scene Management Functions ---------------------------------------------*/

//-----------------------------------------------------------------------------
static void setup_default_scene(void)
{
    engine3d_clear_scene();
    
    // Add ground plane (at Y = -2, so objects are above it)
    int ground = engine3d_add_object(OBJECT_PLANE, (vec3_t){0, -2, 0}, (vec3_t){1, 1, 1}, LCD_COLOR(80, 80, 80));
    if (ground >= 0) {
        g_engine.objects[ground].plane.normal = (vec3_t){0, 1, 0};
        g_engine.objects[ground].plane.distance = 2.0f;
        g_engine.objects[ground].plane.width = 20.0f;
        g_engine.objects[ground].plane.height = 20.0f;
        g_engine.objects[ground].reflectivity = 0.3f; // More reflective ground
    }
    
    // 1. Glowing sphere (emission)
    int glow_sphere = engine3d_add_object(OBJECT_SPHERE, (vec3_t){0, 0.5f, -3}, (vec3_t){1, 1, 1}, LCD_COLOR(255, 200, 100));
    if (glow_sphere >= 0) {
        g_engine.objects[glow_sphere].sphere.radius = 0.5f;
        g_engine.objects[glow_sphere].emission = 0.6f;  // Strong glow
        g_engine.objects[glow_sphere].emission_color = (vec3_t){1.0f, 0.8f, 0.4f}; // Warm orange glow
        g_engine.objects[glow_sphere].reflectivity = 0.1f;
    }
    
    // 2. Glass cube (transparent + refractive)
    int glass_cube = engine3d_add_object(OBJECT_CUBE, (vec3_t){-2, 0.5f, -5}, (vec3_t){1, 1, 1}, LCD_COLOR(200, 220, 255));
    if (glass_cube >= 0) {
        g_engine.objects[glass_cube].cube.size = 1.0f;
        g_engine.objects[glass_cube].transparency = 0.8f;      // Very transparent
        g_engine.objects[glass_cube].refraction_index = 1.5f;  // Glass
        g_engine.objects[glass_cube].reflectivity = 0.1f;      // Some reflection
    }
    
    // 3. Mirror sphere (highly reflective)
    int mirror_sphere = engine3d_add_object(OBJECT_SPHERE, (vec3_t){2, 0.3f, -4}, (vec3_t){0.8f, 0.8f, 0.8f}, LCD_COLOR(200, 200, 200));
    if (mirror_sphere >= 0) {
        g_engine.objects[mirror_sphere].sphere.radius = 0.4f;
        g_engine.objects[mirror_sphere].reflectivity = 0.9f;  // Almost perfect mirror
        g_engine.objects[mirror_sphere].roughness = 0.1f;     // Very smooth
    }
    
    // 4. Semi-transparent colored sphere
    int colored_sphere = engine3d_add_object(OBJECT_SPHERE, (vec3_t){-1, 1.0f, -6}, (vec3_t){1, 1, 1}, LCD_COLOR(100, 255, 150));
    if (colored_sphere >= 0) {
        g_engine.objects[colored_sphere].sphere.radius = 0.6f;
        g_engine.objects[colored_sphere].transparency = 0.5f;      // Semi-transparent
        g_engine.objects[colored_sphere].refraction_index = 1.3f;  // Like water
        g_engine.objects[colored_sphere].reflectivity = 0.2f;
    }
    
    // 5. Diamond-like cube (high refraction)
    int diamond_cube = engine3d_add_object(OBJECT_CUBE, (vec3_t){1, 1.2f, -7}, (vec3_t){0.6f, 0.6f, 0.6f}, LCD_COLOR(255, 255, 255));
    if (diamond_cube >= 0) {
        g_engine.objects[diamond_cube].cube.size = 0.8f;
        g_engine.objects[diamond_cube].transparency = 0.9f;      // Very transparent
        g_engine.objects[diamond_cube].refraction_index = 2.4f;  // Diamond
        g_engine.objects[diamond_cube].reflectivity = 0.3f;      // Some reflection
    }
    
    // Add lights
    engine3d_add_light(LIGHT_DIRECTIONAL, (vec3_t){0, 0, 0}, (vec3_t){-0.5f, -1.0f, -0.5f}, 
                      (vec3_t){1.0f, 1.0f, 0.9f}, 0.8f);
    engine3d_add_light(LIGHT_POINT, (vec3_t){3, 3, -2}, (vec3_t){0, 0, 0}, 
                      (vec3_t){0.8f, 0.8f, 1.0f}, 0.6f);
}

//-----------------------------------------------------------------------------
void engine3d_clear_scene(void)
{
    g_engine.object_count = 0;
    g_engine.light_count = 0;
    memset(g_engine.objects, 0, sizeof(g_engine.objects));
    memset(g_engine.lights, 0, sizeof(g_engine.lights));
}

//-----------------------------------------------------------------------------
int engine3d_add_object(object_type_t type, vec3_t position, vec3_t scale, uint16_t color)
{
    if (g_engine.object_count >= ENGINE3D_MAX_OBJECTS) {
        return -1;
    }
    
    int index = g_engine.object_count++;
    object_t *obj = &g_engine.objects[index];
    
    obj->type = type;
    obj->position = position;
    obj->rotation = (vec3_t){0, 0, 0};
    obj->scale = scale;
    obj->color = color;
    obj->reflectivity = 0.0f;
    obj->roughness = 0.5f;
    obj->enabled = true;
    
    // Initialize advanced material properties
    obj->transparency = 0.0f;           // Opaque by default
    obj->refraction_index = 1.0f;       // Air by default
    obj->emission = 0.0f;               // No glow by default
    obj->emission_color = (vec3_t){1.0f, 1.0f, 1.0f}; // White emission
    
    // Set default properties based on type
    switch (type) {
        case OBJECT_CUBE:
            obj->cube.size = 1.0f;
            break;
        case OBJECT_SPHERE:
            obj->sphere.radius = 0.5f;
            break;
        case OBJECT_PLANE:
            obj->plane.normal = (vec3_t){0, 1, 0};
            obj->plane.distance = 0.0f;
            obj->plane.width = 10.0f;
            obj->plane.height = 10.0f;
            break;
        case OBJECT_TRIANGLE:
            obj->triangle.v0 = (vec3_t){-0.5f, -0.5f, 0};
            obj->triangle.v1 = (vec3_t){0.5f, -0.5f, 0};
            obj->triangle.v2 = (vec3_t){0, 0.5f, 0};
            break;
    }
    
    return index;
}

//-----------------------------------------------------------------------------
int engine3d_add_light(light_type_t type, vec3_t position, vec3_t direction, vec3_t color, float intensity)
{
    if (g_engine.light_count >= ENGINE3D_MAX_LIGHTS) {
        return -1;
    }
    
    int index = g_engine.light_count++;
    light_t *light = &g_engine.lights[index];
    
    light->type = type;
    light->position = position;
    light->direction = vec3_normalize(direction);
    light->color = color;
    light->intensity = intensity;
    light->enabled = true;
    
    return index;
}

/*- Camera Control Functions -----------------------------------------------*/

//-----------------------------------------------------------------------------
void engine3d_set_camera_position(vec3_t position)
{
    g_engine.camera.position = position;
}

//-----------------------------------------------------------------------------
void engine3d_set_camera_rotation(vec3_t rotation)
{
    g_engine.camera.rotation = rotation;
}

//-----------------------------------------------------------------------------
void engine3d_move_camera(vec3_t delta)
{
    // Transform movement relative to camera orientation
    vec3_t forward = {0, 0, -1};
    vec3_t right = {1, 0, 0};
    vec3_t up = {0, 1, 0};
    
    // Apply camera rotation to movement vectors
    forward = vec3_rotate_y(forward, g_engine.camera.rotation.y);
    forward = vec3_rotate_x(forward, g_engine.camera.rotation.x);
    
    right = vec3_rotate_y(right, g_engine.camera.rotation.y);
    
    vec3_t movement = vec3_add(vec3_add(
        vec3_mul(forward, delta.z),
        vec3_mul(right, delta.x)),
        vec3_mul(up, delta.y));
    
    g_engine.camera.position = vec3_add(g_engine.camera.position, movement);
}

//-----------------------------------------------------------------------------
void engine3d_rotate_camera(vec3_t delta)
{
    g_engine.camera.rotation = vec3_add(g_engine.camera.rotation, delta);
    
    // Clamp pitch to avoid gimbal lock
    if (g_engine.camera.rotation.x > PI * 0.49f) {
        g_engine.camera.rotation.x = PI * 0.49f;
    }
    if (g_engine.camera.rotation.x < -PI * 0.49f) {
        g_engine.camera.rotation.x = -PI * 0.49f;
    }
}

/*- Main Engine Functions --------------------------------------------------*/

//-----------------------------------------------------------------------------
void engine3d_init(void)
{
    // Initialize engine state
    memset(&g_engine, 0, sizeof(g_engine));
    
    // Set default camera
    g_engine.camera.position = (vec3_t){0, 0, 0};
    g_engine.camera.rotation = (vec3_t){0, 0, 0};
    g_engine.camera.fov = DEFAULT_FOV;
    g_engine.camera.near_plane = DEFAULT_NEAR_PLANE;
    g_engine.camera.far_plane = DEFAULT_FAR_PLANE;
    
    // Set rendering settings
    g_engine.render_mode = RENDER_MODE_RAYCAST;
    g_engine.render_width = DEFAULT_RENDER_WIDTH;
    g_engine.render_height = DEFAULT_RENDER_HEIGHT;
    g_engine.render_scale = DEFAULT_RENDER_SCALE;
    g_engine.camera_speed = CAMERA_MOVE_SPEED;
    
    // Initialize timers
    timer_add(&g_frame_timer);
    timer_add(&g_fps_timer);
    g_frame_timer = 50; // ~20 FPS for raycast, slower for raytrace
    g_fps_timer = 100;  // Update FPS every 100ms
    
    // Setup scene
    setup_default_scene();
    
    // Clear screen
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);
    
    g_scene_dirty = true;
}

//-----------------------------------------------------------------------------
void engine3d_task(void)
{
    if (g_frame_timer == 0) {
        // Much faster timer for progressive rendering - allows system to breathe
        g_frame_timer = 10; // 10ms = 100Hz, but only renders a few lines each time
        
        render_frame();
        
        // Only calculate FPS when frame is complete
        if (g_frame_complete) {
            calculate_fps();
        }
    }
    
    if (g_fps_timer == 0) {
        g_fps_timer = 100;
        draw_ui();
    }
}

//-----------------------------------------------------------------------------
void engine3d_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    // Toggle render mode with F1
    if (buttons & BTN_F1) {
        if (!repeat) {
            g_engine.render_mode = (g_engine.render_mode == RENDER_MODE_RAYCAST) ? 
                                  RENDER_MODE_RAYTRACE : RENDER_MODE_RAYCAST;
            
            // Clear screen when switching modes
            lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);
        }
    }
    
    // Camera movement (WASD-like controls)
    vec3_t move_delta = {0, 0, 0};
    vec3_t rotate_delta = {0, 0, 0};
    
    if (buttons & BTN_UP) {        // W - Forward
        move_delta.z = -g_engine.camera_speed;
    }
    if (buttons & BTN_DOWN) {      // S - Backward
        move_delta.z = g_engine.camera_speed;
    }
    if (buttons & BTN_LEFT) {      // A - Left
        move_delta.x = -g_engine.camera_speed;
    }
    if (buttons & BTN_RIGHT) {     // D - Right
        move_delta.x = g_engine.camera_speed;
    }
    if (buttons & BTN_TRIG_UP) {   // Q - Up
        move_delta.y = g_engine.camera_speed;
    }
    if (buttons & BTN_TRIG_DOWN) { // E - Down
        move_delta.y = -g_engine.camera_speed;
    }
    
    // Camera rotation
    if (buttons & BTN_SHIFT) {
        // Hold shift for rotation mode
        if (buttons & BTN_UP) {
            rotate_delta.x = -CAMERA_ROTATE_SPEED;
        }
        if (buttons & BTN_DOWN) {
            rotate_delta.x = CAMERA_ROTATE_SPEED;
        }
        if (buttons & BTN_LEFT) {
            rotate_delta.y = -CAMERA_ROTATE_SPEED;
        }
        if (buttons & BTN_RIGHT) {
            rotate_delta.y = CAMERA_ROTATE_SPEED;
        }
        
        engine3d_rotate_camera(rotate_delta);
        move_delta = (vec3_t){0, 0, 0}; // Don't move when rotating
    }
    
    // Apply movement
    if (move_delta.x != 0 || move_delta.y != 0 || move_delta.z != 0) {
        engine3d_move_camera(move_delta);
    }
    
    // Return to menu
    if (buttons & BTN_MODE) {
        if (!repeat) {
            launcher_exit_app();
        }
    }
}

//-----------------------------------------------------------------------------
void engine3d_set_render_mode(render_mode_t mode)
{
    g_engine.render_mode = mode;
}

//-----------------------------------------------------------------------------
void engine3d_cleanup(void)
{
    g_frame_timer = TIMER_DISABLE;
    g_fps_timer = TIMER_DISABLE;
}
