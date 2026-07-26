/*
 * Simple Snake Game
 * Classic snake game implementation
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "gd32f4xx.h"
#include "lcd.h"
#include "timer.h"
#include "buttons.h"
#include "snake_game.h"
#include "launcher.h"

/*- Definitions -------------------------------------------------------------*/
#define GRID_SIZE           10
#define GRID_WIDTH          (LCD_WIDTH / GRID_SIZE)
#define GRID_HEIGHT         (LCD_HEIGHT / GRID_SIZE)
#define MAX_SNAKE_LENGTH    (GRID_WIDTH * GRID_HEIGHT)

// Colors
#define BG_COLOR            LCD_COLOR(0, 0, 0)        // Black
#define SNAKE_COLOR         LCD_COLOR(0, 255, 0)      // Green
#define SNAKE_HEAD_COLOR    LCD_COLOR(0, 200, 0)      // Dark green
#define FOOD_COLOR          LCD_COLOR(255, 0, 0)      // Red
#define WALL_COLOR          LCD_COLOR(128, 128, 128)  // Gray
#define TEXT_COLOR          LCD_COLOR(255, 255, 255)  // White

/*- Types -------------------------------------------------------------------*/
typedef enum {
    DIR_UP = 0,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT
} direction_t;

typedef struct {
    int x, y;
} point_t;

typedef struct {
    point_t segments[MAX_SNAKE_LENGTH];
    int length;
    direction_t direction;
    direction_t next_direction;
    bool alive;
} snake_t;

typedef struct {
    snake_t snake;
    point_t food;
    int score;
    bool game_over;
    bool game_started;
    bool paused;
} game_state_t;

/*- Variables ---------------------------------------------------------------*/
static game_state_t g_game;
static int g_game_timer = TIMER_DISABLE;

/*- Forward Declarations ----------------------------------------------------*/
static void reset_game(void);
static void spawn_food(void);
static void update_snake(void);
static void check_collisions(void);
static void check_food_collision(void);
static void draw_grid(void);
static void draw_snake(void);
static void draw_food(void);
static void draw_ui(void);
static bool is_valid_position(int x, int y);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void reset_game(void)
{
    // Initialize snake in center
    g_game.snake.length = 3;
    g_game.snake.direction = DIR_RIGHT;
    g_game.snake.next_direction = DIR_RIGHT;
    g_game.snake.alive = true;
    
    int start_x = GRID_WIDTH / 2;
    int start_y = GRID_HEIGHT / 2;
    
    for (int i = 0; i < g_game.snake.length; i++) {
        g_game.snake.segments[i].x = start_x - i;
        g_game.snake.segments[i].y = start_y;
    }
    
    g_game.score = 0;
    g_game.game_over = false;
    g_game.game_started = false;
    g_game.paused = false;
    
    spawn_food();
}

//-----------------------------------------------------------------------------
static bool is_valid_position(int x, int y)
{
    // Check bounds
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) {
        return false;
    }
    
    // Check if position is occupied by snake
    for (int i = 0; i < g_game.snake.length; i++) {
        if (g_game.snake.segments[i].x == x && g_game.snake.segments[i].y == y) {
            return false;
        }
    }
    
    return true;
}

//-----------------------------------------------------------------------------
static void spawn_food(void)
{
    int attempts = 0;
    do {
        g_game.food.x = rand() % GRID_WIDTH;
        g_game.food.y = rand() % GRID_HEIGHT;
        attempts++;
    } while (!is_valid_position(g_game.food.x, g_game.food.y) && attempts < 100);
}

//-----------------------------------------------------------------------------
static void update_snake(void)
{
    if (!g_game.snake.alive || !g_game.game_started || g_game.paused) return;
    
    // Update direction
    g_game.snake.direction = g_game.snake.next_direction;
    
    // Calculate new head position
    point_t new_head = g_game.snake.segments[0];
    
    switch (g_game.snake.direction) {
        case DIR_UP:    new_head.y--; break;
        case DIR_DOWN:  new_head.y++; break;
        case DIR_LEFT:  new_head.x--; break;
        case DIR_RIGHT: new_head.x++; break;
    }
    
    // Move snake body
    for (int i = g_game.snake.length - 1; i > 0; i--) {
        g_game.snake.segments[i] = g_game.snake.segments[i - 1];
    }
    
    // Set new head
    g_game.snake.segments[0] = new_head;
}

//-----------------------------------------------------------------------------
static void check_collisions(void)
{
    if (!g_game.snake.alive || !g_game.game_started) return;
    
    point_t head = g_game.snake.segments[0];
    
    // Check wall collision
    if (head.x < 0 || head.x >= GRID_WIDTH || head.y < 0 || head.y >= GRID_HEIGHT) {
        g_game.snake.alive = false;
        g_game.game_over = true;
        return;
    }
    
    // Check self collision
    for (int i = 1; i < g_game.snake.length; i++) {
        if (g_game.snake.segments[i].x == head.x && g_game.snake.segments[i].y == head.y) {
            g_game.snake.alive = false;
            g_game.game_over = true;
            return;
        }
    }
}

//-----------------------------------------------------------------------------
static void check_food_collision(void)
{
    if (!g_game.snake.alive || !g_game.game_started) return;
    
    point_t head = g_game.snake.segments[0];
    
    if (head.x == g_game.food.x && head.y == g_game.food.y) {
        // Grow snake
        if (g_game.snake.length < MAX_SNAKE_LENGTH) {
            g_game.snake.length++;
        }
        
        g_game.score += 10;
        spawn_food();
    }
}

//-----------------------------------------------------------------------------
static void draw_grid(void)
{
    // Clear screen
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, BG_COLOR);
    
    // Draw grid lines (optional, for debugging)
    /*
    for (int x = 0; x <= GRID_WIDTH; x++) {
        lcd_draw_line(x * GRID_SIZE, 0, x * GRID_SIZE, LCD_HEIGHT, WALL_COLOR);
    }
    for (int y = 0; y <= GRID_HEIGHT; y++) {
        lcd_draw_line(0, y * GRID_SIZE, LCD_WIDTH, y * GRID_SIZE, WALL_COLOR);
    }
    */
}

//-----------------------------------------------------------------------------
static void draw_snake(void)
{
    for (int i = 0; i < g_game.snake.length; i++) {
        int x = g_game.snake.segments[i].x * GRID_SIZE;
        int y = g_game.snake.segments[i].y * GRID_SIZE;
        
        uint16_t color = (i == 0) ? SNAKE_HEAD_COLOR : SNAKE_COLOR;
        
        lcd_fill_rect(x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2, color);
        
        if (i == 0) {
            // Draw eyes on head
            lcd_draw_pixel(x + 2, y + 2, BG_COLOR);
            lcd_draw_pixel(x + GRID_SIZE - 3, y + 2, BG_COLOR);
        }
    }
}

//-----------------------------------------------------------------------------
static void draw_food(void)
{
    int x = g_game.food.x * GRID_SIZE;
    int y = g_game.food.y * GRID_SIZE;
    
    lcd_fill_rect(x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2, FOOD_COLOR);
    
    // Draw food pattern
    lcd_draw_pixel(x + GRID_SIZE/2, y + GRID_SIZE/2, LCD_COLOR(255, 255, 0));
}

//-----------------------------------------------------------------------------
static void draw_ui(void)
{
    lcd_set_font(FONT_SMALL);
    lcd_set_color(BG_COLOR, TEXT_COLOR);
    
    if (!g_game.game_started) {
        lcd_set_font(FONT_LARGE);
        lcd_puts(LCD_WIDTH/2 - 30, LCD_HEIGHT/2 - 40, "SNAKE");
        lcd_set_font(FONT_SMALL);
        lcd_puts(LCD_WIDTH/2 - 50, LCD_HEIGHT/2 - 20, "UP/DOWN/LEFT/RIGHT: Move");
        lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 - 10, "STOP: Start game");
        lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2, "MODE: Return to menu");
    } else {
        // Score
        char score_str[16];
        sprintf(score_str, "Score: %d", g_game.score);
        lcd_puts(5, 5, score_str);
        
        // Length
        char length_str[16];
        sprintf(length_str, "Length: %d", g_game.snake.length);
        lcd_puts(5, 15, length_str);
        
        if (g_game.paused) {
            lcd_set_font(FONT_LARGE);
            lcd_puts(LCD_WIDTH/2 - 30, LCD_HEIGHT/2 - 10, "PAUSED");
            lcd_set_font(FONT_SMALL);
            lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 + 10, "STOP: Resume");
        } else if (g_game.game_over) {
            lcd_set_font(FONT_LARGE);
            lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 - 20, "GAME OVER");
            lcd_set_font(FONT_SMALL);
            lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2, "STOP: Restart");
            lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 + 10, "MODE: Return to menu");
        } else {
            lcd_puts(LCD_WIDTH - 80, 5, "STOP: Pause");
        }
    }
}

//-----------------------------------------------------------------------------
void snake_game_init(void)
{
    reset_game();
    
    timer_add(&g_game_timer);
    g_game_timer = 200; // Slower game speed
    
    // Draw initial state
    draw_grid();
    draw_ui();
}

//-----------------------------------------------------------------------------
void snake_game_task(void)
{
    if (g_game_timer == 0) {
        g_game_timer = 200; // Reset timer
        
        if (g_game.game_started && !g_game.game_over && !g_game.paused) {
            update_snake();
            check_collisions();
            check_food_collision();
        }
        
        // Redraw everything
        draw_grid();
        if (g_game.game_started) {
            draw_snake();
            draw_food();
        }
        draw_ui();
    }
}

//-----------------------------------------------------------------------------
void snake_game_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_UP) {
        if (g_game.game_started && g_game.snake.direction != DIR_DOWN) {
            g_game.snake.next_direction = DIR_UP;
        }
    }
    else if (buttons & BTN_DOWN) {
        if (g_game.game_started && g_game.snake.direction != DIR_UP) {
            g_game.snake.next_direction = DIR_DOWN;
        }
    }
    else if (buttons & BTN_LEFT) {
        if (g_game.game_started && g_game.snake.direction != DIR_RIGHT) {
            g_game.snake.next_direction = DIR_LEFT;
        }
    }
    else if (buttons & BTN_RIGHT) {
        if (g_game.game_started && g_game.snake.direction != DIR_LEFT) {
            g_game.snake.next_direction = DIR_RIGHT;
        }
    }
    else if (buttons & BTN_STOP) {
        if (!repeat) {
            if (!g_game.game_started) {
                g_game.game_started = true;
            } else if (g_game.game_over) {
                reset_game();
            } else {
                g_game.paused = !g_game.paused;
            }
        }
    }
    else if (buttons & BTN_MODE) {
        if (!repeat) {
            launcher_exit_app();
        }
    }
}

//-----------------------------------------------------------------------------
void snake_game_cleanup(void)
{
    g_game_timer = TIMER_DISABLE;
}
