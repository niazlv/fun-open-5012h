/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Advanced 3D Engine
 * Full-featured 3D graphics engine with camera control, scene management,
 * RayCast and RayTrace rendering modes
 */

#ifndef _ENGINE3D_H_
#define _ENGINE3D_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define ENGINE3D_MAX_OBJECTS    10
#define ENGINE3D_MAX_LIGHTS     3

// Rendering modes
typedef enum {
    RENDER_MODE_RAYCAST = 0,
    RENDER_MODE_RAYTRACE = 1
} render_mode_t;

// Object types
typedef enum {
    OBJECT_CUBE = 0,
    OBJECT_SPHERE = 1,
    OBJECT_TRIANGLE = 2,
    OBJECT_PLANE = 3
} object_type_t;

// Light types
typedef enum {
    LIGHT_DIRECTIONAL = 0,
    LIGHT_POINT = 1
} light_type_t;

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    float x, y;
} vec2_t;

typedef struct {
    vec3_t position;
    vec3_t rotation;    // Euler angles in radians
    float fov;          // Field of view in radians
    float near_plane;
    float far_plane;
} camera_t;

typedef struct {
    light_type_t type;
    vec3_t position;    // For point lights
    vec3_t direction;   // For directional lights
    vec3_t color;       // RGB values 0.0-1.0
    float intensity;
    bool enabled;
} light_t;

typedef struct {
    object_type_t type;
    vec3_t position;
    vec3_t rotation;
    vec3_t scale;
    uint16_t color;
    float reflectivity;
    float roughness;
    bool enabled;
    
    // Advanced material properties
    float transparency;     // 0.0 = opaque, 1.0 = fully transparent
    float refraction_index; // 1.0 = air, 1.5 = glass, 2.4 = diamond
    float emission;         // 0.0 = no glow, 1.0 = full glow
    vec3_t emission_color;  // RGB emission color (0.0-1.0)
    
    // Object-specific data
    union {
        struct {
            float size;
        } cube;
        
        struct {
            float radius;
        } sphere;
        
        struct {
            vec3_t v0, v1, v2;  // Triangle vertices
        } triangle;
        
        struct {
            vec3_t normal;
            float distance;
            float width, height;
        } plane;
    };
} object_t;

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
    float roughness;
    object_t *object;
} hit_result_t;

typedef struct {
    camera_t camera;
    object_t objects[ENGINE3D_MAX_OBJECTS];
    light_t lights[ENGINE3D_MAX_LIGHTS];
    int object_count;
    int light_count;
    render_mode_t render_mode;
    
    // Performance counters
    uint32_t frame_count;
    uint32_t last_fps_time;
    uint32_t current_fps;
    
    // Camera movement
    vec3_t camera_velocity;
    float camera_speed;
    float mouse_sensitivity;
    
    // Rendering settings
    int render_width;
    int render_height;
    int render_scale;
    bool wireframe_mode;
    bool show_normals;
    
} engine3d_t;

/*- Prototypes --------------------------------------------------------------*/
void engine3d_init(void);
void engine3d_task(void);
void engine3d_buttons_handler(int buttons);
void engine3d_cleanup(void);
void engine3d_redraw(void);

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t engine3d_menu;
extern const menu_def_t engine3d_help_menu;

// Scene management
void engine3d_clear_scene(void);
int engine3d_add_object(object_type_t type, vec3_t position, vec3_t scale, uint16_t color);
int engine3d_add_light(light_type_t type, vec3_t position, vec3_t direction, vec3_t color, float intensity);

// Camera control
void engine3d_set_camera_position(vec3_t position);
void engine3d_set_camera_rotation(vec3_t rotation);
void engine3d_move_camera(vec3_t delta);
void engine3d_rotate_camera(vec3_t delta);

// Rendering
void engine3d_set_render_mode(render_mode_t mode);

// Math utilities
vec3_t vec3_add(vec3_t a, vec3_t b);
vec3_t vec3_sub(vec3_t a, vec3_t b);
vec3_t vec3_mul(vec3_t v, float s);
vec3_t vec3_cross(vec3_t a, vec3_t b);
float vec3_dot(vec3_t a, vec3_t b);
float vec3_length(vec3_t v);
vec3_t vec3_normalize(vec3_t v);
vec3_t vec3_rotate_x(vec3_t v, float angle);
vec3_t vec3_rotate_y(vec3_t v, float angle);
vec3_t vec3_rotate_z(vec3_t v, float angle);

// Ray tracing
hit_result_t engine3d_cast_ray(ray_t ray);
hit_result_t engine3d_trace_ray(ray_t ray, int depth);

#endif // _ENGINE3D_H_
