/*
 * DOOM Port for GD32F407VE Platform
 * Simplified DOOM engine adapted for embedded systems
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom_port.h"
#include "timer.h"
#include "utils.h"
#include <stdio.h>
#include <math.h>

/*- Variables ---------------------------------------------------------------*/
doom_game_t doom_game;
static uint32_t doom_timer_ticks = 0;

/*- Helper Functions --------------------------------------------------------*/
static inline fixed_t doom_abs(fixed_t x) {
    return x < 0 ? -x : x;
}

/*- Local Functions ---------------------------------------------------------*/

// Simple test map - walls form a maze-like structure
static const uint8_t doom_test_map[DOOM_MAP_HEIGHT][DOOM_MAP_WIDTH] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,0,0,1},
    {1,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,0,1},
    {1,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,0,0,1},
    {1,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,4,0,0,0,2,0,0,0,2,0,0,0,3,0,0,0,3,0,0,0,4,0,0,0,0,1},
    {1,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,4,4,0,0,0,2,2,2,2,2,0,0,0,3,3,3,3,3,0,0,0,4,4,4,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    // ... continue pattern for remaining rows
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void doom_port_init(void)
{
    doom_init_game();
}

//-----------------------------------------------------------------------------
void doom_port_task(void)
{
    if (!doom_game.running)
        return;
        
    doom_update_game();
    doom_render_frame();
}

//-----------------------------------------------------------------------------
void doom_port_buttons_handler(int buttons)
{
    if (buttons & BTN_STOP) {
        doom_game.running = false;
        return;
    }
    
    if (buttons & BTN_MODE) {
        doom_game.paused = !doom_game.paused;
        return;
    }
    
    if (doom_game.paused)
        return;
    
    // Movement controls
    fixed_t move_speed = FRACUNIT / 8;  // Movement speed
    fixed_t rot_speed = 5;              // Rotation speed in degrees
    
    if (buttons & BTN_UP) {
        doom_move_player(move_speed, 0);
    }
    if (buttons & BTN_DOWN) {
        doom_move_player(-move_speed, 0);
    }
    if (buttons & BTN_LEFT) {
        if (buttons & BTN_SHIFT) {
            // Strafe left
            doom_move_player(0, -move_speed);
        } else {
            // Turn left
            doom_rotate_player(-rot_speed);
        }
    }
    if (buttons & BTN_RIGHT) {
        if (buttons & BTN_SHIFT) {
            // Strafe right
            doom_move_player(0, move_speed);
        } else {
            // Turn right
            doom_rotate_player(rot_speed);
        }
    }
}

//-----------------------------------------------------------------------------
void doom_port_cleanup(void)
{
    doom_game.running = false;
}

//-----------------------------------------------------------------------------
void doom_init_game(void)
{
    memset(&doom_game, 0, sizeof(doom_game));
    
    // Initialize trigonometric tables
    for (int i = 0; i < 360; i++) {
        float rad = (float)i * DOOM_PI / 180.0f;
        doom_game.cos_table[i] = (fixed_t)(cosf(rad) * FRACUNIT);
        doom_game.sin_table[i] = (fixed_t)(sinf(rad) * FRACUNIT);
    }
    
    doom_init_map();
    doom_init_player();
    
    doom_game.running = true;
    doom_game.paused = false;
    doom_game.game_tics = 0;
    doom_game.last_fps_time = timer_ms();
}

//-----------------------------------------------------------------------------
void doom_update_game(void)
{
    if (doom_game.paused)
        return;
        
    doom_game.game_tics++;
    doom_update_player();
    
    // Update FPS counter
    uint32_t current_time = timer_ms();
    doom_game.fps_counter++;
    
    if (current_time - doom_game.last_fps_time >= 1000) {
        doom_game.current_fps = doom_game.fps_counter;
        doom_game.fps_counter = 0;
        doom_game.last_fps_time = current_time;
    }
}

//-----------------------------------------------------------------------------
void doom_render_frame(void)
{
    uint32_t start_time = timer_ms();
    
    doom_clear_screen();
    doom_render_floor_ceiling();
    doom_render_walls();
    doom_render_hud();
    
    doom_game.frame_time = timer_ms() - start_time;
}

//-----------------------------------------------------------------------------
void doom_init_map(void)
{
    // Copy test map to game map
    for (int y = 0; y < DOOM_MAP_HEIGHT; y++) {
        for (int x = 0; x < DOOM_MAP_WIDTH; x++) {
            if (y < 17) {  // Only copy the defined rows
                doom_game.map.cells[x][y].wall_type = doom_test_map[y][x];
            } else {
                // Fill remaining rows with walls on edges, empty inside
                if (x == 0 || x == DOOM_MAP_WIDTH-1 || y == 0 || y == DOOM_MAP_HEIGHT-1) {
                    doom_game.map.cells[x][y].wall_type = 1;
                } else {
                    doom_game.map.cells[x][y].wall_type = 0;
                }
            }
        }
    }
}

//-----------------------------------------------------------------------------
uint8_t doom_get_wall_at(int32_t x, int32_t y)
{
    if (x < 0 || x >= DOOM_MAP_WIDTH || y < 0 || y >= DOOM_MAP_HEIGHT)
        return 1;  // Treat out-of-bounds as walls
        
    return doom_game.map.cells[x][y].wall_type;
}

//-----------------------------------------------------------------------------
bool doom_is_wall(int32_t x, int32_t y)
{
    return doom_get_wall_at(x, y) != 0;
}

//-----------------------------------------------------------------------------
void doom_init_player(void)
{
    // Start player in an open area
    doom_game.player.x = 5 * FRACUNIT;
    doom_game.player.y = 5 * FRACUNIT;
    doom_game.player.angle = 0;  // Facing east
    doom_game.player.sector = 0;
}

//-----------------------------------------------------------------------------
void doom_update_player(void)
{
    // Player update logic (if needed)
}

//-----------------------------------------------------------------------------
void doom_move_player(fixed_t forward, fixed_t strafe)
{
    fixed_t new_x = doom_game.player.x;
    fixed_t new_y = doom_game.player.y;
    
    if (forward != 0) {
        fixed_t cos_a = doom_cos(doom_game.player.angle);
        fixed_t sin_a = doom_sin(doom_game.player.angle);
        
        new_x += doom_fixed_mul(forward, cos_a);
        new_y += doom_fixed_mul(forward, sin_a);
    }
    
    if (strafe != 0) {
        fixed_t cos_a = doom_cos(doom_game.player.angle + 90);
        fixed_t sin_a = doom_sin(doom_game.player.angle + 90);
        
        new_x += doom_fixed_mul(strafe, cos_a);
        new_y += doom_fixed_mul(strafe, sin_a);
    }
    
    // Collision detection
    int32_t map_x = new_x >> FRACBITS;
    int32_t map_y = new_y >> FRACBITS;
    
    if (!doom_is_wall(map_x, map_y)) {
        doom_game.player.x = new_x;
        doom_game.player.y = new_y;
    }
}

//-----------------------------------------------------------------------------
void doom_rotate_player(fixed_t angle_delta)
{
    doom_game.player.angle += angle_delta;
    doom_game.player.angle = doom_normalize_angle(doom_game.player.angle);
}

//-----------------------------------------------------------------------------
void doom_render_walls(void)
{
    // Improved raycasting renderer with better performance
    for (int x = 0; x < DOOM_SCREEN_WIDTH; x++) {
        // Calculate ray angle with fisheye correction
        fixed_t ray_angle = doom_game.player.angle + 
                           ((x - DOOM_SCREEN_WIDTH/2) * 60) / DOOM_SCREEN_WIDTH;
        ray_angle = doom_normalize_angle(ray_angle);
        
        // Cast ray using DDA algorithm for better performance
        fixed_t ray_cos = doom_cos(ray_angle);
        fixed_t ray_sin = doom_sin(ray_angle);
        
        // DDA variables
        int32_t map_x = doom_game.player.x >> FRACBITS;
        int32_t map_y = doom_game.player.y >> FRACBITS;
        
        fixed_t delta_dist_x = abs(doom_fixed_div(FRACUNIT, ray_cos));
        fixed_t delta_dist_y = abs(doom_fixed_div(FRACUNIT, ray_sin));
        
        fixed_t side_dist_x, side_dist_y;
        int32_t step_x, step_y;
        
        // Calculate step and initial side_dist
        if (ray_cos < 0) {
            step_x = -1;
            side_dist_x = doom_fixed_mul(doom_game.player.x - (map_x << FRACBITS), delta_dist_x);
        } else {
            step_x = 1;
            side_dist_x = doom_fixed_mul(((map_x + 1) << FRACBITS) - doom_game.player.x, delta_dist_x);
        }
        
        if (ray_sin < 0) {
            step_y = -1;
            side_dist_y = doom_fixed_mul(doom_game.player.y - (map_y << FRACBITS), delta_dist_y);
        } else {
            step_y = 1;
            side_dist_y = doom_fixed_mul(((map_y + 1) << FRACBITS) - doom_game.player.y, delta_dist_y);
        }
        
        // Perform DDA
        uint8_t wall_type = 0;
        int32_t side = 0; // 0 if x-side, 1 if y-side
        
        while (wall_type == 0) {
            // Jump to next map square, either in x-direction, or in y-direction
            if (side_dist_x < side_dist_y) {
                side_dist_x += delta_dist_x;
                map_x += step_x;
                side = 0;
            } else {
                side_dist_y += delta_dist_y;
                map_y += step_y;
                side = 1;
            }
            
            // Check if ray has hit a wall
            wall_type = doom_get_wall_at(map_x, map_y);
        }
        
        // Calculate distance
        fixed_t perp_wall_dist;
        if (side == 0) {
            perp_wall_dist = doom_fixed_div((map_x << FRACBITS) - doom_game.player.x + 
                                          ((1 - step_x) << (FRACBITS - 1)), ray_cos);
        } else {
            perp_wall_dist = doom_fixed_div((map_y << FRACBITS) - doom_game.player.y + 
                                          ((1 - step_y) << (FRACBITS - 1)), ray_sin);
        }
        
        // Calculate wall height
        int32_t line_height = doom_fixed_div(DOOM_SCREEN_HEIGHT << FRACBITS, perp_wall_dist) >> FRACBITS;
        
        // Calculate lowest and highest pixel to fill in current stripe
        int32_t draw_start = (-line_height / 2) + (DOOM_SCREEN_HEIGHT / 2);
        if (draw_start < 0) draw_start = 0;
        
        int32_t draw_end = (line_height / 2) + (DOOM_SCREEN_HEIGHT / 2);
        if (draw_end >= DOOM_SCREEN_HEIGHT) draw_end = DOOM_SCREEN_HEIGHT - 1;
        
        // Choose wall color based on type and side
        uint16_t wall_color;
        switch (wall_type) {
            case 1: wall_color = DOOM_WALL_COLOR_1; break;
            case 2: wall_color = DOOM_WALL_COLOR_2; break;
            case 3: wall_color = DOOM_WALL_COLOR_3; break;
            case 4: wall_color = DOOM_WALL_COLOR_4; break;
            default: wall_color = DOOM_COLOR_GRAY; break;
        }
        
        // Make y-side walls darker for depth effect
        if (side == 1) {
            wall_color = doom_shade_color(wall_color, 2);
        }
        
        // Apply distance shading
        int32_t shade_level = perp_wall_dist >> (FRACBITS - 2);
        wall_color = doom_shade_color(wall_color, shade_level);
        
        doom_draw_vertical_line(x, draw_start, draw_end, wall_color);
    }
}

//-----------------------------------------------------------------------------
void doom_render_floor_ceiling(void)
{
    // Simple floor and ceiling
    lcd_fill_rect(0, 0, DOOM_SCREEN_WIDTH, DOOM_SCREEN_HEIGHT/2, DOOM_COLOR_DARKGRAY);  // Ceiling
    lcd_fill_rect(0, DOOM_SCREEN_HEIGHT/2, DOOM_SCREEN_WIDTH, DOOM_SCREEN_HEIGHT/2, DOOM_COLOR_GRAY);  // Floor
}

//-----------------------------------------------------------------------------
void doom_render_hud(void)
{
    char fps_str[16];
    char pos_str[32];
    
    // FPS counter
    sprintf(fps_str, "FPS: %d", doom_game.current_fps);
    lcd_set_color(DOOM_COLOR_BLACK, DOOM_COLOR_WHITE);
    lcd_set_font(FONT_SMALL);
    lcd_puts(5, 5, fps_str);
    
    // Player position
    sprintf(pos_str, "X:%ld Y:%ld A:%ld", 
            (long)(doom_game.player.x >> FRACBITS),
            (long)(doom_game.player.y >> FRACBITS),
            (long)doom_game.player.angle);
    lcd_puts(5, 20, pos_str);
    
    // Pause indicator
    if (doom_game.paused) {
        lcd_set_color(DOOM_COLOR_BLACK, DOOM_COLOR_YELLOW);
        lcd_puts(DOOM_SCREEN_WIDTH/2 - 20, DOOM_SCREEN_HEIGHT/2, "PAUSED");
    }
}

//-----------------------------------------------------------------------------
void doom_clear_screen(void)
{
    // Screen is cleared by floor/ceiling rendering
}

//-----------------------------------------------------------------------------
fixed_t doom_fixed_mul(fixed_t a, fixed_t b)
{
    return (fixed_t)(((int64_t)a * b) >> FRACBITS);
}

//-----------------------------------------------------------------------------
fixed_t doom_fixed_div(fixed_t a, fixed_t b)
{
    if (b == 0) return 0;
    return (fixed_t)(((int64_t)a << FRACBITS) / b);
}

//-----------------------------------------------------------------------------
fixed_t doom_cos(int32_t angle)
{
    angle = doom_normalize_angle(angle);
    return doom_game.cos_table[angle];
}

//-----------------------------------------------------------------------------
fixed_t doom_sin(int32_t angle)
{
    angle = doom_normalize_angle(angle);
    return doom_game.sin_table[angle];
}

//-----------------------------------------------------------------------------
int32_t doom_normalize_angle(int32_t angle)
{
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

//-----------------------------------------------------------------------------
uint16_t doom_shade_color(uint16_t color, int32_t distance)
{
    if (distance < 4) return color;
    if (distance < 8) {
        // Darken by 25%
        uint16_t r = (color >> 11) & 0x1F;
        uint16_t g = (color >> 5) & 0x3F;
        uint16_t b = color & 0x1F;
        r = (r * 3) / 4;
        g = (g * 3) / 4;
        b = (b * 3) / 4;
        return (r << 11) | (g << 5) | b;
    }
    if (distance < 16) {
        // Darken by 50%
        uint16_t r = (color >> 11) & 0x1F;
        uint16_t g = (color >> 5) & 0x3F;
        uint16_t b = color & 0x1F;
        r = r / 2;
        g = g / 2;
        b = b / 2;
        return (r << 11) | (g << 5) | b;
    }
    // Very dark for distant walls
    return DOOM_COLOR_DARKGRAY;
}

//-----------------------------------------------------------------------------
void doom_draw_vertical_line(int16_t x, int16_t y1, int16_t y2, uint16_t color)
{
    if (x < 0 || x >= DOOM_SCREEN_WIDTH) return;
    if (y1 > y2) {
        int16_t temp = y1;
        y1 = y2;
        y2 = temp;
    }
    if (y1 < 0) y1 = 0;
    if (y2 >= DOOM_SCREEN_HEIGHT) y2 = DOOM_SCREEN_HEIGHT - 1;
    
    for (int16_t y = y1; y <= y2; y++) {
        lcd_draw_pixel(x, y, color);
    }
}
