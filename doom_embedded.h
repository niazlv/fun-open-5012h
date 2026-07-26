/*
 * DOOM Embedded Port for GD32F407VE
 * Full DOOM engine adapted for embedded systems
 */

#ifndef _DOOM_EMBEDDED_H_
#define _DOOM_EMBEDDED_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "common.h"
#include "lcd.h"
#include "buttons.h"

/*- DOOM Core Includes ------------------------------------------------------*/
// We'll include the essential DOOM headers
typedef unsigned char byte;
typedef int boolean;
typedef int fixed_t;

/*- Platform Definitions ---------------------------------------------------*/
#define SCREENWIDTH     320
#define SCREENHEIGHT    200
#define MAXPLAYERS      1       // Single player only for embedded

// Fixed point math
#define FRACBITS        16
#define FRACUNIT        (1<<FRACBITS)

// Boolean values
#ifndef true
#define true    1
#define false   0
#endif

/*- DOOM Key Definitions ----------------------------------------------------*/
#define KEY_RIGHTARROW  0xae
#define KEY_LEFTARROW   0xac
#define KEY_UPARROW     0xad
#define KEY_DOWNARROW   0xaf
#define KEY_ESCAPE      27
#define KEY_ENTER       13
#define KEY_TAB         9
#define KEY_F1          (0x80+0x3b)
#define KEY_F2          (0x80+0x3c)
#define KEY_RSHIFT      (0x80+0x36)
#define KEY_RCTRL       (0x80+0x1d)
#define KEY_RALT        (0x80+0x38)
#define KEY_LALT        KEY_RALT
#define KEY_FIRE        KEY_RCTRL
#define KEY_USE         ' '

/*- Event System ------------------------------------------------------------*/
typedef enum {
    ev_keydown,
    ev_keyup,
    ev_mouse,
    ev_joystick
} evtype_t;

typedef struct {
    evtype_t type;
    int data1;      // keys / mouse/joystick buttons
    int data2;      // mouse/joystick x move
    int data3;      // mouse/joystick y move
} event_t;

/*- Game States -------------------------------------------------------------*/
typedef enum {
    GS_LEVEL,
    GS_INTERMISSION,
    GS_FINALE,
    GS_DEMOSCREEN
} gamestate_t;

/*- Skill Levels ------------------------------------------------------------*/
typedef enum {
    sk_baby,
    sk_easy,
    sk_medium,
    sk_hard,
    sk_nightmare
} skill_t;

/*- Video System ------------------------------------------------------------*/
extern byte* screens[5];
extern int usegamma;

/*- Game Variables ----------------------------------------------------------*/
extern gamestate_t gamestate;
extern skill_t gameskill;
extern int gameepisode;
extern int gamemap;
extern boolean paused;
extern boolean menuactive;
extern boolean demoplayback;

/*- Function Prototypes -----------------------------------------------------*/
// Platform interface
void I_InitGraphics(void);
void I_ShutdownGraphics(void);
void I_StartFrame(void);
void I_StartTic(void);
void I_FinishUpdate(void);
void I_UpdateNoBlit(void);
void I_ReadScreen(byte* scr);
void I_SetPalette(byte* palette);

// System interface
void I_Init(void);
void I_Quit(void);
void I_Error(char *error, ...);
int I_GetTime(void);
void I_Sleep(int ms);

// Sound interface (stub for embedded)
void I_InitSound(void);
void I_ShutdownSound(void);
void I_UpdateSound(void);
void I_SubmitSound(void);

// Memory management
void* I_AllocLow(int length);
void* Z_Malloc(int size, int tag, void* user);
void Z_Free(void* ptr);
void Z_Init(void);

// Event handling
void D_PostEvent(event_t* ev);
void D_ProcessEvents(void);

// Main DOOM functions
void D_DoomMain(void);
void D_DoomLoop(void);

// Game functions
void G_InitNew(skill_t skill, int episode, int map);
void G_Ticker(void);
boolean G_Responder(event_t* ev);

// Menu functions
boolean M_Responder(event_t* ev);
void M_Ticker(void);
void M_Drawer(void);

// Rendering functions
void R_Init(void);
void R_RenderPlayerView(void* player);
void R_ExecuteSetViewSize(void);

// Video functions
void V_Init(void);
void V_DrawPatch(int x, int y, int scrn, void* patch);
void V_CopyRect(int srcx, int srcy, int srcscrn, int width, int height, int destx, int desty, int destscrn);

// WAD file system
void W_InitMultipleFiles(char** filenames);
int W_CheckNumForName(char* name);
void* W_CacheLumpName(char* name, int tag);

// Status bar
void ST_Init(void);
void ST_Drawer(boolean fullscreen, boolean refresh);

// HUD
void HU_Init(void);
void HU_Drawer(void);
void HU_Erase(void);

// Automap
void AM_Drawer(void);
extern boolean automapactive;

// Intermission
void WI_Drawer(void);

// Finale
void F_Drawer(void);

// Network (stub)
void NetUpdate(void);

// Platform-specific embedded functions
void doom_embedded_init(void);
void doom_embedded_task(void);
void doom_embedded_buttons_handler(int buttons);
void doom_embedded_cleanup(void);

/*- Memory Tags -------------------------------------------------------------*/
#define PU_STATIC       1
#define PU_SOUND        2
#define PU_MUSIC        3
#define PU_DAVE         4
#define PU_LEVEL        50
#define PU_LEVSPEC      51
#define PU_CACHE        101

/*- Global Variables --------------------------------------------------------*/
extern boolean nodrawers;
extern int displayplayer;
extern boolean viewactive;
extern int viewheight;
extern int scaledviewwidth;
extern int viewwindowx;
extern int viewwindowy;
extern boolean inhelpscreens;
extern boolean setsizeneeded;
extern int gametic;
extern int maketic;

#endif // _DOOM_EMBEDDED_H_
