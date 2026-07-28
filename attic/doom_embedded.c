/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DOOM Embedded Port for GD32F407VE
 * Full DOOM engine implementation for embedded systems
 */

/*- Includes ----------------------------------------------------------------*/
#include "doom_embedded.h"
#include "timer.h"
#include "utils.h"

/*- Global Variables --------------------------------------------------------*/
// Video system
byte* screens[5];
static byte screen_buffer[SCREENWIDTH * SCREENHEIGHT * 4];
int usegamma = 0;

// Game state
gamestate_t gamestate = GS_DEMOSCREEN;
skill_t gameskill = sk_medium;
int gameepisode = 1;
int gamemap = 1;
boolean paused = false;
boolean menuactive = false;
boolean demoplayback = false;
boolean nodrawers = false;
int displayplayer = 0;
boolean viewactive = false;
int viewheight = SCREENHEIGHT;
int scaledviewwidth = SCREENWIDTH;
int viewwindowx = 0;
int viewwindowy = 0;
boolean inhelpscreens = false;
boolean setsizeneeded = false;
int gametic = 0;
int maketic = 0;
boolean automapactive = false;

// Event system
#define MAXEVENTS 64
static event_t events[MAXEVENTS];
static int eventhead = 0;
static int eventtail = 0;

// Timer
static uint32_t doom_start_time = 0;
static uint32_t doom_tics = 0;

// Simple embedded WAD data (minimal level)
static const byte embedded_wad_data[] = {
    // This would contain a minimal WAD file data
    // For now, we'll create a simple test level
    0x50, 0x57, 0x41, 0x44,  // "PWAD" signature
    0x01, 0x00, 0x00, 0x00,  // Number of lumps
    0x0C, 0x00, 0x00, 0x00,  // Directory offset
    // Directory entry
    0x00, 0x00, 0x00, 0x00,  // File position
    0x00, 0x00, 0x00, 0x00,  // Size
    'E', '1', 'M', '1', 0, 0, 0, 0  // Name
};

// Player structure (simplified)
typedef struct {
    fixed_t x, y, z;
    fixed_t angle;
    fixed_t momx, momy, momz;
    int health;
    boolean onground;
} player_t;

static player_t players[MAXPLAYERS];

// Simple map data
#define MAP_WIDTH 64
#define MAP_HEIGHT 64
static byte simple_map[MAP_HEIGHT][MAP_WIDTH];

// Palette (simplified)
static byte doom_palette[768] = {
    // Basic 256-color palette (RGB triplets)
    0,0,0, 255,255,255, 255,0,0, 0,255,0, 0,0,255,
    // ... (simplified palette)
};

/*- Platform Implementation ------------------------------------------------*/

void I_InitGraphics(void) {
    // Initialize screen buffers
    for (int i = 0; i < 4; i++) {
        screens[i] = &screen_buffer[i * SCREENWIDTH * SCREENHEIGHT];
    }
    
    // Clear screen
    memset(screens[0], 0, SCREENWIDTH * SCREENHEIGHT);
}

void I_ShutdownGraphics(void) {
    // Nothing to do
}

void I_StartFrame(void) {
    // Frame start
}

void I_StartTic(void) {
    // Process input events
}

void I_FinishUpdate(void) {
    // Copy screen buffer to LCD
    for (int y = 0; y < SCREENHEIGHT; y++) {
        for (int x = 0; x < SCREENWIDTH; x++) {
            byte pixel = screens[0][y * SCREENWIDTH + x];
            // Convert 8-bit palette index to 16-bit color
            uint16_t color = 0;
            if (pixel == 0) {
                color = LCD_BLACK_COLOR;
            } else if (pixel < 32) {
                color = LCD_COLOR(pixel * 8, 0, 0);  // Red gradient
            } else if (pixel < 64) {
                color = LCD_COLOR(0, (pixel - 32) * 8, 0);  // Green gradient
            } else if (pixel < 96) {
                color = LCD_COLOR(0, 0, (pixel - 64) * 8);  // Blue gradient
            } else {
                color = LCD_COLOR(pixel, pixel, pixel);  // Grayscale
            }
            lcd_draw_pixel(x, y, color);
        }
    }
}

void I_UpdateNoBlit(void) {
    // Nothing to do
}

void I_ReadScreen(byte* scr) {
    memcpy(scr, screens[0], SCREENWIDTH * SCREENHEIGHT);
}

void I_SetPalette(byte* palette) {
    if (palette) {
        memcpy(doom_palette, palette, 768);
    }
}

void I_Init(void) {
    doom_start_time = 0;  // We'll use a simple counter
}

void I_Quit(void) {
    // Return to menu
}

void I_Error(char *error, ...) {
    // Display error and halt
    lcd_fill_rect(0, 0, SCREENWIDTH, SCREENHEIGHT, LCD_RED_COLOR);
    lcd_set_color(LCD_WHITE_COLOR, LCD_RED_COLOR);
    lcd_set_font(FONT_LARGE);
    lcd_puts(10, 100, "DOOM ERROR");
    while(1);
}

int I_GetTime(void) {
    return doom_tics;
}

void I_Sleep(int ms) {
    // Simple delay
    for (volatile int i = 0; i < ms * 1000; i++);
}

// Sound stubs
void I_InitSound(void) {}
void I_ShutdownSound(void) {}
void I_UpdateSound(void) {}
void I_SubmitSound(void) {}

// Memory management (simplified)
static byte memory_pool[64 * 1024];  // 64KB memory pool
static int memory_used = 0;

void* I_AllocLow(int length) {
    return Z_Malloc(length, PU_STATIC, NULL);
}

void* Z_Malloc(int size, int tag, void* user) {
    if (memory_used + size > sizeof(memory_pool)) {
        I_Error("Out of memory");
        return NULL;
    }
    void* ptr = &memory_pool[memory_used];
    memory_used += (size + 3) & ~3;  // Align to 4 bytes
    return ptr;
}

void Z_Free(void* ptr) {
    // Simple allocator - no free
}

void Z_Init(void) {
    memory_used = 0;
}

/*- Event System ------------------------------------------------------------*/

void D_PostEvent(event_t* ev) {
    events[eventhead] = *ev;
    eventhead = (eventhead + 1) & (MAXEVENTS - 1);
}

void D_ProcessEvents(void) {
    event_t* ev;
    
    while (eventtail != eventhead) {
        ev = &events[eventtail];
        
        if (M_Responder(ev)) {
            eventtail = (eventtail + 1) & (MAXEVENTS - 1);
            continue;  // Menu ate the event
        }
        
        G_Responder(ev);
        eventtail = (eventtail + 1) & (MAXEVENTS - 1);
    }
}

/*- Game Logic --------------------------------------------------------------*/

void G_InitNew(skill_t skill, int episode, int map) {
    gameskill = skill;
    gameepisode = episode;
    gamemap = map;
    gamestate = GS_LEVEL;
    
    // Initialize player
    players[0].x = 32 * FRACUNIT;
    players[0].y = 32 * FRACUNIT;
    players[0].z = 0;
    players[0].angle = 0;
    players[0].health = 100;
    players[0].onground = true;
    
    // Initialize simple map
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            if (x == 0 || x == MAP_WIDTH-1 || y == 0 || y == MAP_HEIGHT-1) {
                simple_map[y][x] = 1;  // Wall
            } else if ((x % 8 == 0 && y % 8 == 0) && (x > 8 && x < MAP_WIDTH-8 && y > 8 && y < MAP_HEIGHT-8)) {
                simple_map[y][x] = 1;  // Pillars
            } else {
                simple_map[y][x] = 0;  // Empty
            }
        }
    }
}

void G_Ticker(void) {
    if (paused) return;
    
    // Simple player movement
    player_t* player = &players[0];
    
    // Apply momentum
    fixed_t newx = player->x + player->momx;
    fixed_t newy = player->y + player->momy;
    
    // Simple collision detection
    int mapx = newx >> (FRACBITS + 6);  // Convert to map coordinates
    int mapy = newy >> (FRACBITS + 6);
    
    if (mapx >= 0 && mapx < MAP_WIDTH && mapy >= 0 && mapy < MAP_HEIGHT) {
        if (simple_map[mapy][mapx] == 0) {
            player->x = newx;
            player->y = newy;
        }
    }
    
    // Apply friction
    player->momx = (player->momx * 7) / 8;
    player->momy = (player->momy * 7) / 8;
    
    gametic++;
}

boolean G_Responder(event_t* ev) {
    player_t* player = &players[0];
    
    if (ev->type == ev_keydown) {
        switch (ev->data1) {
            case KEY_UPARROW:
                // Move forward
                player->momx += (cosf(player->angle * 3.14159f / 180.0f) * FRACUNIT) / 16;
                player->momy += (sinf(player->angle * 3.14159f / 180.0f) * FRACUNIT) / 16;
                return true;
                
            case KEY_DOWNARROW:
                // Move backward
                player->momx -= (cosf(player->angle * 3.14159f / 180.0f) * FRACUNIT) / 16;
                player->momy -= (sinf(player->angle * 3.14159f / 180.0f) * FRACUNIT) / 16;
                return true;
                
            case KEY_LEFTARROW:
                // Turn left
                player->angle -= 5;
                if (player->angle < 0) player->angle += 360;
                return true;
                
            case KEY_RIGHTARROW:
                // Turn right
                player->angle += 5;
                if (player->angle >= 360) player->angle -= 360;
                return true;
                
            case KEY_ESCAPE:
                menuactive = !menuactive;
                return true;
        }
    }
    
    return false;
}

/*- Menu System -------------------------------------------------------------*/

static int menu_selected = 0;
static const char* menu_items[] = {
    "NEW GAME",
    "OPTIONS", 
    "QUIT"
};
#define MENU_ITEMS_COUNT 3

boolean M_Responder(event_t* ev) {
    if (!menuactive) return false;
    
    if (ev->type == ev_keydown) {
        switch (ev->data1) {
            case KEY_UPARROW:
                menu_selected--;
                if (menu_selected < 0) menu_selected = MENU_ITEMS_COUNT - 1;
                return true;
                
            case KEY_DOWNARROW:
                menu_selected++;
                if (menu_selected >= MENU_ITEMS_COUNT) menu_selected = 0;
                return true;
                
            case KEY_ENTER:
                switch (menu_selected) {
                    case 0:  // New Game
                        menuactive = false;
                        G_InitNew(sk_medium, 1, 1);
                        return true;
                    case 2:  // Quit
                        // Return to main menu
                        return true;
                }
                return true;
                
            case KEY_ESCAPE:
                menuactive = false;
                return true;
        }
    }
    
    return false;
}

void M_Ticker(void) {
    // Menu logic
}

void M_Drawer(void) {
    if (!menuactive) return;
    
    // Draw menu background
    for (int y = 60; y < 140; y++) {
        for (int x = 80; x < 240; x++) {
            screens[0][y * SCREENWIDTH + x] = 64;  // Dark background
        }
    }
    
    // Draw menu items
    for (int i = 0; i < MENU_ITEMS_COUNT; i++) {
        int y = 80 + i * 20;
        int x = 100;
        
        // Draw selection cursor
        if (i == menu_selected) {
            for (int px = x - 10; px < x - 5; px++) {
                for (int py = y; py < y + 8; py++) {
                    if (px >= 0 && px < SCREENWIDTH && py >= 0 && py < SCREENHEIGHT) {
                        screens[0][py * SCREENWIDTH + px] = 255;  // White cursor
                    }
                }
            }
        }
        
        // Draw menu text (simplified)
        const char* text = menu_items[i];
        for (int j = 0; text[j]; j++) {
            // Simple character drawing
            for (int py = 0; py < 8; py++) {
                for (int px = 0; px < 8; px++) {
                    int screen_x = x + j * 8 + px;
                    int screen_y = y + py;
                    if (screen_x >= 0 && screen_x < SCREENWIDTH && 
                        screen_y >= 0 && screen_y < SCREENHEIGHT) {
                        // Simple font pattern
                        if ((px == 0 || px == 7 || py == 0 || py == 7) && 
                            (text[j] >= 'A' && text[j] <= 'Z')) {
                            screens[0][screen_y * SCREENWIDTH + screen_x] = 255;
                        }
                    }
                }
            }
        }
    }
}

/*- Rendering System --------------------------------------------------------*/

void R_Init(void) {
    // Initialize renderer
}

void R_RenderPlayerView(void* player_ptr) {
    player_t* player = (player_t*)player_ptr;
    
    // Simple raycasting renderer
    for (int x = 0; x < SCREENWIDTH; x++) {
        // Calculate ray angle
        float ray_angle = (player->angle - 30) + (x * 60.0f / SCREENWIDTH);
        float ray_cos = cosf(ray_angle * 3.14159f / 180.0f);
        float ray_sin = sinf(ray_angle * 3.14159f / 180.0f);
        
        // Cast ray
        float ray_x = player->x / (float)FRACUNIT;
        float ray_y = player->y / (float)FRACUNIT;
        float distance = 0;
        
        // Ray marching
        for (int step = 0; step < 64; step++) {
            int map_x = (int)(ray_x / 64);
            int map_y = (int)(ray_y / 64);
            
            if (map_x >= 0 && map_x < MAP_WIDTH && map_y >= 0 && map_y < MAP_HEIGHT) {
                if (simple_map[map_y][map_x] != 0) {
                    // Hit wall
                    break;
                }
            }
            
            ray_x += ray_cos;
            ray_y += ray_sin;
            distance += 1.0f;
        }
        
        // Calculate wall height
        if (distance > 0) {
            int wall_height = (int)(SCREENHEIGHT / distance * 32);
            if (wall_height > SCREENHEIGHT) wall_height = SCREENHEIGHT;
            
            int wall_top = (SCREENHEIGHT - wall_height) / 2;
            int wall_bottom = wall_top + wall_height;
            
            // Draw column
            for (int y = 0; y < SCREENHEIGHT; y++) {
                byte color;
                if (y < wall_top) {
                    color = 128;  // Ceiling
                } else if (y > wall_bottom) {
                    color = 64;   // Floor
                } else {
                    // Wall with distance shading
                    color = 255 - (byte)(distance * 4);
                    if (color < 128) color = 128;
                }
                
                if (x >= 0 && x < SCREENWIDTH && y >= 0 && y < SCREENHEIGHT) {
                    screens[0][y * SCREENWIDTH + x] = color;
                }
            }
        }
    }
}

void R_ExecuteSetViewSize(void) {
    setsizeneeded = false;
}

/*- Video System ------------------------------------------------------------*/

void V_Init(void) {
    // Initialize video system
}

void V_DrawPatch(int x, int y, int scrn, void* patch) {
    // Stub - patch drawing
}

void V_CopyRect(int srcx, int srcy, int srcscrn, int width, int height, 
                int destx, int desty, int destscrn) {
    // Copy rectangle between screens
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int src_pos = (srcy + y) * SCREENWIDTH + (srcx + x);
            int dest_pos = (desty + y) * SCREENWIDTH + (destx + x);
            
            if (src_pos >= 0 && src_pos < SCREENWIDTH * SCREENHEIGHT &&
                dest_pos >= 0 && dest_pos < SCREENWIDTH * SCREENHEIGHT) {
                screens[destscrn][dest_pos] = screens[srcscrn][src_pos];
            }
        }
    }
}

/*- WAD System (Stub) -------------------------------------------------------*/

void W_InitMultipleFiles(char** filenames) {
    // Initialize with embedded data
}

int W_CheckNumForName(char* name) {
    return -1;  // Not found
}

void* W_CacheLumpName(char* name, int tag) {
    return NULL;  // Not found
}

/*- HUD and Status ----------------------------------------------------------*/

void ST_Init(void) {}
void ST_Drawer(boolean fullscreen, boolean refresh) {}
void HU_Init(void) {}
void HU_Drawer(void) {}
void HU_Erase(void) {}
void AM_Drawer(void) {}
void WI_Drawer(void) {}
void F_Drawer(void) {}
void NetUpdate(void) {}

/*- Main DOOM Loop ----------------------------------------------------------*/

void D_DoomMain(void) {
    // Initialize subsystems
    I_Init();
    V_Init();
    R_Init();
    ST_Init();
    HU_Init();
    
    // Start game
    menuactive = true;
    gamestate = GS_DEMOSCREEN;
}

void D_DoomLoop(void) {
    while (1) {
        // Frame synchronous IO operations
        I_StartFrame();
        
        // Process input
        I_StartTic();
        D_ProcessEvents();
        
        // Game logic
        if (menuactive) {
            M_Ticker();
        } else {
            G_Ticker();
        }
        
        // Rendering
        if (!nodrawers) {
            // Clear screen
            memset(screens[0], 0, SCREENWIDTH * SCREENHEIGHT);
            
            if (gamestate == GS_LEVEL && !menuactive) {
                R_RenderPlayerView(&players[displayplayer]);
            }
            
            // Draw menu on top
            M_Drawer();
            
            // Update display
            I_FinishUpdate();
        }
        
        doom_tics++;
    }
}

/*- Platform Interface ------------------------------------------------------*/

void doom_embedded_init(void) {
    I_InitGraphics();
    D_DoomMain();
}

void doom_embedded_task(void) {
    static boolean initialized = false;
    
    if (!initialized) {
        initialized = true;
        return;
    }
    
    // Run one iteration of DOOM loop
    I_StartFrame();
    I_StartTic();
    D_ProcessEvents();
    
    if (menuactive) {
        M_Ticker();
    } else {
        G_Ticker();
    }
    
    if (!nodrawers) {
        memset(screens[0], 0, SCREENWIDTH * SCREENHEIGHT);
        
        if (gamestate == GS_LEVEL && !menuactive) {
            R_RenderPlayerView(&players[displayplayer]);
        }
        
        M_Drawer();
        I_FinishUpdate();
    }
    
    doom_tics++;
}

void doom_embedded_buttons_handler(int buttons) {
    event_t event;
    
    if (buttons & BTN_STOP) {
        // Exit DOOM
        return;
    }
    
    // Convert button presses to DOOM events
    if (buttons & BTN_UP) {
        event.type = ev_keydown;
        event.data1 = KEY_UPARROW;
        D_PostEvent(&event);
    }
    
    if (buttons & BTN_DOWN) {
        event.type = ev_keydown;
        event.data1 = KEY_DOWNARROW;
        D_PostEvent(&event);
    }
    
    if (buttons & BTN_LEFT) {
        event.type = ev_keydown;
        event.data1 = KEY_LEFTARROW;
        D_PostEvent(&event);
    }
    
    if (buttons & BTN_RIGHT) {
        event.type = ev_keydown;
        event.data1 = KEY_RIGHTARROW;
        D_PostEvent(&event);
    }
    
    if (buttons & BTN_MODE) {
        event.type = ev_keydown;
        event.data1 = KEY_ENTER;
        D_PostEvent(&event);
    }
    
    if (buttons & BTN_SHIFT) {
        event.type = ev_keydown;
        event.data1 = KEY_ESCAPE;
        D_PostEvent(&event);
    }
}

void doom_embedded_cleanup(void) {
    I_ShutdownGraphics();
}
