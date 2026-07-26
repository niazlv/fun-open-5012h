/*
 * Full DOOM Port for GD32F407VE
 * Complete DOOM engine using original architecture
 * Based on linuxdoom-1.10 source code
 */

#ifndef _DOOM_FULL_PORT_H_
#define _DOOM_FULL_PORT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "common.h"
#include "lcd.h"
#include "buttons.h"
#include "timer.h"

/*- DOOM Core Definitions ---------------------------------------------------*/
// Version
#define VERSION 110

// Screen dimensions
#define SCREENWIDTH     320
#define SCREENHEIGHT    200
#define MAXPLAYERS      1

// Fixed point math
#ifndef FRACBITS
#define FRACBITS        16
#endif
#ifndef FRACUNIT
#define FRACUNIT        (1<<FRACBITS)
#endif

// Game states
typedef enum {
    GS_LEVEL,
    GS_INTERMISSION,
    GS_FINALE,
    GS_DEMOSCREEN
} gamestate_t;

// Skill levels
typedef enum {
    sk_baby,
    sk_easy,
    sk_medium,
    sk_hard,
    sk_nightmare
} skill_t;

// Game modes
typedef enum {
    shareware,
    registered,
    commercial,
    retail,
    indetermined
} GameMode_t;

// Basic types
typedef unsigned char byte;
typedef int boolean;
#ifndef _FIXED_T_DEFINED
#define _FIXED_T_DEFINED
typedef int fixed_t;
#endif

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
#define KEY_F3          (0x80+0x3d)
#define KEY_F4          (0x80+0x3e)
#define KEY_F5          (0x80+0x3f)
#define KEY_F6          (0x80+0x40)
#define KEY_F7          (0x80+0x41)
#define KEY_F8          (0x80+0x42)
#define KEY_F9          (0x80+0x43)
#define KEY_F10         (0x80+0x44)
#define KEY_F11         (0x80+0x57)
#define KEY_F12         (0x80+0x58)
#define KEY_BACKSPACE   127
#define KEY_PAUSE       0xff
#define KEY_EQUALS      0x3d
#define KEY_MINUS       0x2d
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

/*- Weapons -----------------------------------------------------------------*/
typedef enum {
    wp_fist,
    wp_pistol,
    wp_shotgun,
    wp_chaingun,
    wp_missile,
    wp_plasma,
    wp_bfg,
    wp_chainsaw,
    wp_supershotgun,
    NUMWEAPONS,
    wp_nochange
} weapontype_t;

/*- Ammunition --------------------------------------------------------------*/
typedef enum {
    am_clip,    // Pistol / chaingun ammo
    am_shell,   // Shotgun / double barreled shotgun
    am_cell,    // Plasma rifle, BFG
    am_misl,    // Missile launcher
    NUMAMMO,
    am_noammo   // Unlimited for chainsaw / fist
} ammotype_t;

/*- Power-ups ---------------------------------------------------------------*/
typedef enum {
    pw_invulnerability,
    pw_strength,
    pw_invisibility,
    pw_ironfeet,
    pw_allmap,
    pw_infrared,
    NUMPOWERS
} powertype_t;

/*- Key cards ---------------------------------------------------------------*/
typedef enum {
    it_bluecard,
    it_yellowcard,
    it_redcard,
    it_blueskull,
    it_yellowskull,
    it_redskull,
    NUMCARDS
} card_t;

/*- Memory Management -------------------------------------------------------*/
#define PU_STATIC       1
#define PU_SOUND        2
#define PU_MUSIC        3
#define PU_DAVE         4
#define PU_LEVEL        50
#define PU_LEVSPEC      51
#define PU_CACHE        101

/*- Math Constants ----------------------------------------------------------*/
#define TICRATE         35
#define PI              3.141592657

/*- Angle System ------------------------------------------------------------*/
#define ANG45           0x20000000
#define ANG90           0x40000000
#define ANG180          0x80000000
#define ANG270          0xc0000000
#define ANGLETOFINESHIFT 19
#define FINEANGLES      8192
#define FINEMASK        (FINEANGLES-1)

typedef unsigned angle_t;

/*- Map Structures ----------------------------------------------------------*/
typedef struct {
    fixed_t x, y;
} vertex_t;

typedef struct {
    vertex_t* v1;
    vertex_t* v2;
    fixed_t dx;
    fixed_t dy;
    short flags;
    short special;
    short tag;
    short sidenum[2];
    fixed_t bbox[4];
    int slopetype;
    struct sector_s* frontsector;
    struct sector_s* backsector;
    int validcount;
    void* specialdata;
} line_t;

typedef struct {
    fixed_t textureoffset;
    fixed_t rowoffset;
    char toptexture[8];
    char bottomtexture[8];
    char midtexture[8];
    struct sector_s* sector;
} side_t;

typedef struct sector_s {
    fixed_t floorheight;
    fixed_t ceilingheight;
    char floorpic[8];
    char ceilingpic[8];
    short lightlevel;
    short special;
    short tag;
    int soundtraversed;
    struct mobj_s* soundtarget;
    int blockbox[4];
    struct mobj_s* thinglist;
    void* specialdata;
    int linecount;
    struct line_s** lines;
} sector_t;

/*- Thing/Object System -----------------------------------------------------*/
typedef enum {
    MT_PLAYER,
    MT_POSSESSED,
    MT_SHOTGUY,
    MT_VILE,
    MT_FIRE,
    MT_UNDEAD,
    MT_TRACER,
    MT_SMOKE,
    MT_FATSO,
    MT_FATSHOT,
    MT_CHAINGUY,
    MT_TROOP,
    MT_SERGEANT,
    MT_SHADOWS,
    MT_HEAD,
    MT_BRUISER,
    MT_BRUISERSHOT,
    MT_KNIGHT,
    MT_SKULL,
    MT_SPIDER,
    MT_BABY,
    MT_CYBORG,
    MT_PAIN,
    MT_WOLFSS,
    MT_KEEN,
    MT_BOSSBRAIN,
    MT_BOSSSPIT,
    MT_BOSSTARGET,
    MT_SPAWNSHOT,
    MT_SPAWNFIRE,
    MT_BARREL,
    MT_TROOPSHOT,
    MT_HEADSHOT,
    MT_ROCKET,
    MT_PLASMA,
    MT_BFG,
    MT_ARACHPLAZ,
    MT_PUFF,
    MT_BLOOD,
    MT_TFOG,
    MT_IFOG,
    MT_TELEPORTMAN,
    MT_EXTRABFG,
    MT_MISC0,
    MT_MISC1,
    MT_MISC2,
    MT_MISC3,
    MT_MISC4,
    MT_MISC5,
    MT_MISC6,
    MT_MISC7,
    MT_MISC8,
    MT_MISC9,
    MT_MISC10,
    MT_MISC11,
    MT_MISC12,
    MT_INV,
    MT_MISC13,
    MT_INS,
    MT_MISC14,
    MT_MISC15,
    MT_MISC16,
    MT_MEGA,
    MT_CLIP,
    MT_MISC17,
    MT_MISC18,
    MT_MISC19,
    MT_MISC20,
    MT_MISC21,
    MT_MISC22,
    MT_MISC23,
    MT_MISC24,
    MT_MISC25,
    MT_CHAINGUN,
    MT_MISC26,
    MT_MISC27,
    MT_MISC28,
    MT_SHOTGUN,
    MT_SUPERSHOTGUN,
    MT_MISC29,
    MT_MISC30,
    MT_MISC31,
    MT_MISC32,
    MT_MISC33,
    MT_MISC34,
    MT_MISC35,
    MT_MISC36,
    MT_MISC37,
    MT_MISC38,
    MT_MISC39,
    MT_MISC40,
    MT_MISC41,
    MT_MISC42,
    MT_MISC43,
    MT_MISC44,
    MT_MISC45,
    MT_MISC46,
    MT_MISC47,
    MT_MISC48,
    MT_MISC49,
    MT_MISC50,
    MT_MISC51,
    MT_MISC52,
    MT_MISC53,
    MT_MISC54,
    MT_MISC55,
    MT_MISC56,
    MT_MISC57,
    MT_MISC58,
    MT_MISC59,
    MT_MISC60,
    MT_MISC61,
    MT_MISC62,
    MT_MISC63,
    MT_MISC64,
    MT_MISC65,
    MT_MISC66,
    MT_MISC67,
    MT_MISC68,
    MT_MISC69,
    MT_MISC70,
    MT_MISC71,
    MT_MISC72,
    MT_MISC73,
    MT_MISC74,
    MT_MISC75,
    MT_MISC76,
    MT_MISC77,
    MT_MISC78,
    MT_MISC79,
    MT_MISC80,
    MT_MISC81,
    MT_MISC82,
    MT_MISC83,
    MT_MISC84,
    MT_MISC85,
    MT_MISC86,
    NUMMOBJTYPES
} mobjtype_t;

/*- Global Variables --------------------------------------------------------*/
extern byte* screens[5];
extern int usegamma;
extern gamestate_t gamestate;
extern skill_t gameskill;
extern int gameepisode;
extern int gamemap;
extern boolean paused;
extern boolean menuactive;
extern boolean demoplayback;
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
extern boolean automapactive;

/*- Function Prototypes -----------------------------------------------------*/

// Platform interface (i_*.c equivalents)
void I_InitGraphics(void);
void I_ShutdownGraphics(void);
void I_StartFrame(void);
void I_StartTic(void);
void I_FinishUpdate(void);
void I_UpdateNoBlit(void);
void I_ReadScreen(byte* scr);
void I_SetPalette(byte* palette);
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
void M_Init(void);

// Rendering functions
void R_Init(void);
void R_RenderPlayerView(void* player);
void R_ExecuteSetViewSize(void);

// Video functions
void V_Init(void);
void V_DrawPatch(int x, int y, int scrn, void* patch);
void V_CopyRect(int srcx, int srcy, int srcscrn, int width, int height, 
                int destx, int desty, int destscrn);

// WAD file system
void W_InitMultipleFiles(char** filenames);
int W_CheckNumForName(char* name);
void* W_CacheLumpName(char* name, int tag);
int W_GetNumForName(char* name);
void* W_CacheLumpNum(int lump, int tag);

// Status bar
void ST_Init(void);
void ST_Drawer(boolean fullscreen, boolean refresh);
void ST_Ticker(void);

// HUD
void HU_Init(void);
void HU_Drawer(void);
void HU_Erase(void);
void HU_Ticker(void);

// Automap
void AM_Drawer(void);
void AM_Ticker(void);
boolean AM_Responder(event_t* ev);

// Intermission
void WI_Drawer(void);
void WI_Ticker(void);

// Finale
void F_Drawer(void);
void F_Ticker(void);

// Network (stub)
void NetUpdate(void);

// Platform-specific embedded functions
void doom_full_port_init(void);
void doom_full_port_task(void);
void doom_full_port_buttons_handler(int buttons);
void doom_full_port_cleanup(void);

// Math functions
fixed_t FixedMul(fixed_t a, fixed_t b);
fixed_t FixedDiv(fixed_t a, fixed_t b);
int R_PointToAngle(fixed_t x, fixed_t y);
fixed_t R_PointToDist(fixed_t x, fixed_t y);

// Tables
extern fixed_t finesine[10240];
extern fixed_t finecosine[10240];
extern angle_t tantoangle[2049];

// Embedded WAD data
extern const byte embedded_doom_wad[];
extern const int embedded_doom_wad_size;

#endif // _DOOM_FULL_PORT_H_
