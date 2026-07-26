/*
 * DOOM Port for GD32F407VE Platform
 * Simplified DOOM engine adapted for embedded systems
 */

#ifndef _DOOM_PORT_H_
#define _DOOM_PORT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "common.h"
#include "lcd.h"
#include "buttons.h"

/*- Definitions -------------------------------------------------------------*/
// Screen dimensions - adapted for LCD
#define DOOM_SCREEN_WIDTH   320
#define DOOM_SCREEN_HEIGHT  200
#define DOOM_SCALE_FACTOR   1

// Simplified DOOM constants
#define DOOM_TICRATE        35
#define DOOM_PI             3.14159265359f
#define DOOM_PI2            (DOOM_PI * 2.0f)

// Fixed point math (simplified)
#ifndef FRACBITS
#define FRACBITS            16
#endif
#ifndef FRACUNIT
#define FRACUNIT            (1 << FRACBITS)
#endif
#ifndef _FIXED_T_DEFINED
#define _FIXED_T_DEFINED
typedef int32_t fixed_t;
#endif

// Map dimensions
#define DOOM_MAP_WIDTH      64
#define DOOM_MAP_HEIGHT     64
#define DOOM_WALL_HEIGHT    64
#define DOOM_PLAYER_HEIGHT  32

// Colors (16-bit RGB565)
#define DOOM_COLOR_BLACK    0x0000
#define DOOM_COLOR_WHITE    0xFFFF
#define DOOM_COLOR_RED      0xF800
#define DOOM_COLOR_GREEN    0x07E0
#define DOOM_COLOR_BLUE     0x001F
#define DOOM_COLOR_YELLOW   0xFFE0
#define DOOM_COLOR_CYAN     0x07FF
#define DOOM_COLOR_MAGENTA  0xF81F
#define DOOM_COLOR_GRAY     0x8410
#define DOOM_COLOR_DARKGRAY 0x4208

// Wall colors for different wall types
#define DOOM_WALL_COLOR_1   0x8410  // Gray
#define DOOM_WALL_COLOR_2   0x4208  // Dark gray
#define DOOM_WALL_COLOR_3   0x2104  // Very dark gray
#define DOOM_WALL_COLOR_4   0x6318  // Brown-ish

/*- Types -------------------------------------------------------------------*/
typedef struct {
    fixed_t x, y;
    fixed_t angle;
    int32_t sector;
} doom_player_t;

typedef struct {
    uint8_t wall_type;  // 0 = empty, 1-4 = different wall types
} doom_cell_t;

typedef struct {
    doom_cell_t cells[DOOM_MAP_WIDTH][DOOM_MAP_HEIGHT];
} doom_map_t;

typedef struct {
    doom_player_t player;
    doom_map_t map;
    uint32_t game_tics;
    bool running;
    bool paused;
    
    // Rendering state
    fixed_t cos_table[360];
    fixed_t sin_table[360];
    
    // Performance counters
    uint32_t frame_time;
    uint32_t fps_counter;
    uint32_t last_fps_time;
    uint16_t current_fps;
} doom_game_t;

typedef struct {
    int16_t x1, y1, x2, y2;
    uint16_t color;
    int16_t height;
} doom_wall_t;

/*- Prototypes --------------------------------------------------------------*/
// Main DOOM functions
void doom_port_init(void);
void doom_port_task(void);
void doom_port_buttons_handler(int buttons);
void doom_port_cleanup(void);

// Game logic
void doom_init_game(void);
void doom_update_game(void);
void doom_render_frame(void);

// Map functions
void doom_init_map(void);
uint8_t doom_get_wall_at(int32_t x, int32_t y);
bool doom_is_wall(int32_t x, int32_t y);

// Player functions
void doom_init_player(void);
void doom_update_player(void);
void doom_move_player(fixed_t forward, fixed_t strafe);
void doom_rotate_player(fixed_t angle_delta);

// Rendering functions
void doom_render_walls(void);
void doom_render_floor_ceiling(void);
void doom_render_hud(void);
void doom_clear_screen(void);

// Math functions
fixed_t doom_fixed_mul(fixed_t a, fixed_t b);
fixed_t doom_fixed_div(fixed_t a, fixed_t b);
fixed_t doom_cos(int32_t angle);
fixed_t doom_sin(int32_t angle);
int32_t doom_normalize_angle(int32_t angle);

// Utility functions
uint16_t doom_shade_color(uint16_t color, int32_t distance);
void doom_draw_vertical_line(int16_t x, int16_t y1, int16_t y2, uint16_t color);

/*- Global Variables --------------------------------------------------------*/
extern doom_game_t doom_game;

#endif // _DOOM_PORT_H_
