/*
 * Simple Flappy Bird Game
 * Basic side-scrolling bird game
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
#include "flappy_bird.h"
#include "launcher.h"

/*- Definitions -------------------------------------------------------------*/
#define BIRD_SIZE           8
#define BIRD_X              50
#define PIPE_WIDTH          20
#define PIPE_GAP            60
#define PIPE_SPEED          2
#define GRAVITY             0.5f
#define JUMP_STRENGTH       -8.0f
#define MAX_PIPES           4

#define GROUND_HEIGHT       20
#define SKY_HEIGHT          (LCD_HEIGHT - GROUND_HEIGHT)

// Colors
#define SKY_COLOR           LCD_COLOR(135, 206, 235)  // Sky blue
#define GROUND_COLOR        LCD_COLOR(222, 184, 135)  // Tan
#define BIRD_COLOR          LCD_COLOR(255, 255, 0)    // Yellow
#define PIPE_COLOR          LCD_COLOR(0, 128, 0)      // Green
#define SCORE_COLOR         LCD_COLOR(255, 255, 255)  // White

/*- Types -------------------------------------------------------------------*/
typedef struct {
    float x, y;
    float velocity_y;
    bool alive;
} bird_t;

typedef struct {
    float x;
    int gap_y;
    bool passed;
} pipe_t;

typedef struct {
    bird_t bird;
    pipe_t pipes[MAX_PIPES];
    int score;
    bool game_over;
    bool game_started;
    float scroll_offset;
} game_state_t;

/*- Variables ---------------------------------------------------------------*/
static game_state_t g_game;
static int g_game_timer = TIMER_DISABLE;

/*- Forward Declarations ----------------------------------------------------*/
static void reset_game(void);
static void update_bird(void);
static void update_pipes(void);
static void check_collisions(void);
static void draw_background(void);
static void draw_bird(void);
static void draw_pipes(void);
static void draw_ui(void);
static void spawn_pipe(int index);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void reset_game(void)
{
    g_game.bird.x = BIRD_X;
    g_game.bird.y = LCD_HEIGHT / 2;
    g_game.bird.velocity_y = 0;
    g_game.bird.alive = true;
    
    g_game.score = 0;
    g_game.game_over = false;
    g_game.game_started = false;
    g_game.scroll_offset = 0;
    
    // Initialize pipes
    for (int i = 0; i < MAX_PIPES; i++) {
        g_game.pipes[i].x = LCD_WIDTH + i * (LCD_WIDTH / 2);
        g_game.pipes[i].gap_y = SKY_HEIGHT / 2;
        g_game.pipes[i].passed = false;
    }
}

//-----------------------------------------------------------------------------
static void spawn_pipe(int index)
{
    g_game.pipes[index].x = LCD_WIDTH;
    g_game.pipes[index].gap_y = 40 + (rand() % (SKY_HEIGHT - PIPE_GAP - 80));
    g_game.pipes[index].passed = false;
}

//-----------------------------------------------------------------------------
static void update_bird(void)
{
    if (!g_game.bird.alive) return;
    
    // Apply gravity
    g_game.bird.velocity_y += GRAVITY;
    g_game.bird.y += g_game.bird.velocity_y;
    
    // Check bounds
    if (g_game.bird.y < 0) {
        g_game.bird.y = 0;
        g_game.bird.velocity_y = 0;
    }
    
    if (g_game.bird.y > SKY_HEIGHT - BIRD_SIZE) {
        g_game.bird.y = SKY_HEIGHT - BIRD_SIZE;
        g_game.bird.alive = false;
        g_game.game_over = true;
    }
}

//-----------------------------------------------------------------------------
static void update_pipes(void)
{
    if (!g_game.game_started) return;
    
    for (int i = 0; i < MAX_PIPES; i++) {
        g_game.pipes[i].x -= PIPE_SPEED;
        
        // Check if pipe passed bird
        if (!g_game.pipes[i].passed && g_game.pipes[i].x + PIPE_WIDTH < BIRD_X) {
            g_game.pipes[i].passed = true;
            g_game.score++;
        }
        
        // Respawn pipe when it goes off screen
        if (g_game.pipes[i].x < -PIPE_WIDTH) {
            spawn_pipe(i);
        }
    }
}

//-----------------------------------------------------------------------------
static void check_collisions(void)
{
    if (!g_game.bird.alive || !g_game.game_started) return;
    
    for (int i = 0; i < MAX_PIPES; i++) {
        // Check if bird is in pipe's X range
        if (g_game.bird.x + BIRD_SIZE > g_game.pipes[i].x && 
            g_game.bird.x < g_game.pipes[i].x + PIPE_WIDTH) {
            
            // Check if bird is in gap
            if (g_game.bird.y < g_game.pipes[i].gap_y || 
                g_game.bird.y + BIRD_SIZE > g_game.pipes[i].gap_y + PIPE_GAP) {
                g_game.bird.alive = false;
                g_game.game_over = true;
            }
        }
    }
}

//-----------------------------------------------------------------------------
static void draw_background(void)
{
    // Sky
    lcd_fill_rect(0, 0, LCD_WIDTH, SKY_HEIGHT, SKY_COLOR);
    
    // Ground
    lcd_fill_rect(0, SKY_HEIGHT, LCD_WIDTH, GROUND_HEIGHT, GROUND_COLOR);
    
    // Ground pattern
    for (int x = 0; x < LCD_WIDTH; x += 10) {
        lcd_draw_pixel(x, SKY_HEIGHT, LCD_COLOR(139, 69, 19));
    }
}

//-----------------------------------------------------------------------------
static void draw_bird(void)
{
    if (!g_game.bird.alive) return;
    
    int x = (int)g_game.bird.x;
    int y = (int)g_game.bird.y;
    
    // Draw bird as a simple rectangle
    lcd_fill_rect(x, y, BIRD_SIZE, BIRD_SIZE, BIRD_COLOR);
    
    // Bird outline
    lcd_draw_rect(x, y, BIRD_SIZE - 1, BIRD_SIZE - 1, LCD_COLOR(255, 165, 0));
    
    // Simple eye
    lcd_draw_pixel(x + 2, y + 2, LCD_BLACK_COLOR);
}

//-----------------------------------------------------------------------------
static void draw_pipes(void)
{
    for (int i = 0; i < MAX_PIPES; i++) {
        int x = (int)g_game.pipes[i].x;
        
        if (x > -PIPE_WIDTH && x < LCD_WIDTH) {
            // Top pipe
            lcd_fill_rect(x, 0, PIPE_WIDTH, g_game.pipes[i].gap_y, PIPE_COLOR);
            
            // Bottom pipe
            lcd_fill_rect(x, g_game.pipes[i].gap_y + PIPE_GAP, PIPE_WIDTH, 
                         SKY_HEIGHT - g_game.pipes[i].gap_y - PIPE_GAP, PIPE_COLOR);
            
            // Pipe outlines
            lcd_draw_rect(x, 0, PIPE_WIDTH - 1, g_game.pipes[i].gap_y - 1, 
                         LCD_COLOR(0, 100, 0));
            lcd_draw_rect(x, g_game.pipes[i].gap_y + PIPE_GAP, PIPE_WIDTH - 1, 
                         SKY_HEIGHT - g_game.pipes[i].gap_y - PIPE_GAP - 1, 
                         LCD_COLOR(0, 100, 0));
        }
    }
}

//-----------------------------------------------------------------------------
static void draw_ui(void)
{
    lcd_set_font(FONT_LARGE);
    lcd_set_color(SKY_COLOR, SCORE_COLOR);
    
    if (!g_game.game_started) {
        lcd_puts(LCD_WIDTH/2 - 60, LCD_HEIGHT/2 - 40, "FLAPPY BIRD");
        lcd_set_font(FONT_SMALL);
        lcd_puts(LCD_WIDTH/2 - 50, LCD_HEIGHT/2 - 20, "Press UP to start");
        lcd_puts(LCD_WIDTH/2 - 50, LCD_HEIGHT/2 - 10, "MODE: Return to menu");
    } else {
        // Score
        char score_str[16];
        sprintf(score_str, "Score: %d", g_game.score);
        lcd_puts(10, 10, score_str);
        
        if (g_game.game_over) {
            lcd_puts(LCD_WIDTH/2 - 40, LCD_HEIGHT/2 - 20, "GAME OVER");
            lcd_set_font(FONT_SMALL);
            lcd_puts(LCD_WIDTH/2 - 50, LCD_HEIGHT/2, "Press UP to restart");
            lcd_puts(LCD_WIDTH/2 - 50, LCD_HEIGHT/2 + 10, "MODE: Return to menu");
        } else {
            lcd_set_font(FONT_SMALL);
            lcd_puts(10, 25, "UP: Jump");
        }
    }
}

//-----------------------------------------------------------------------------
void flappy_bird_init(void)
{
    reset_game();
    
    timer_add(&g_game_timer);
    g_game_timer = 50; // 20 FPS
    
    // Clear screen and draw initial state
    draw_background();
    draw_ui();
}

//-----------------------------------------------------------------------------
void flappy_bird_task(void)
{
    if (g_game_timer == 0) {
        g_game_timer = 50; // Reset timer
        
        if (g_game.game_started && !g_game.game_over) {
            update_bird();
            update_pipes();
            check_collisions();
        }
        
        // Redraw everything
        draw_background();
        draw_pipes();
        draw_bird();
        draw_ui();
    }
}

//-----------------------------------------------------------------------------
void flappy_bird_buttons_handler(int buttons)
{
    bool repeat = (buttons & BTN_REPEAT);
    
    if (buttons & BTN_UP) {
        if (!repeat) {
            if (!g_game.game_started) {
                g_game.game_started = true;
            } else if (g_game.game_over) {
                reset_game();
            } else if (g_game.bird.alive) {
                g_game.bird.velocity_y = JUMP_STRENGTH;
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
void flappy_bird_cleanup(void)
{
    g_game_timer = TIMER_DISABLE;
}
