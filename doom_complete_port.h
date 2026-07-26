/*
 * Complete DOOM Port for GD32F407VE
 * Full implementation based on linuxdoom-1.10 source code
 * This is a REAL DOOM port, not a simplified version
 */

#ifndef _DOOM_COMPLETE_PORT_H_
#define _DOOM_COMPLETE_PORT_H_

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

/*- DOOM Core Definitions (from doomdef.h) ---------------------------------*/
#define VERSION 110

// Screen dimensions
#define SCREENWIDTH     320
#define SCREENHEIGHT    200
#define MAXPLAYERS      4

// Fixed point math
#define FRACBITS        16
#define FRACUNIT        (1<<FRACBITS)

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
typedef int fixed_t;
typedef unsigned angle_t;

#ifndef true
#define true    1
#define false   0
#endif

// Tic rate
#define TICRATE 35

/*- Key definitions (from doomdef.h) ---------------------------------------*/
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

/*- Event system (from d_event.h) ------------------------------------------*/
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

/*- Weapons (from d_items.h) -----------------------------------------------*/
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

/*- Player states (from d_player.h) ----------------------------------------*/
typedef enum {
    PST_LIVE,    // Playing or camping
    PST_DEAD,    // Dead on the ground, view follows killer
    PST_REBORN   // Ready to restart/respawn
} playerstate_t;

/*- Cheat flags -------------------------------------------------------------*/
typedef enum {
    CF_NOCLIP = 1,      // No clipping, walk through barriers
    CF_GODMODE = 2,     // No damage, no health loss
    CF_NOMOMENTUM = 4   // Not really a cheat, just a debug aid
} cheat_t;

/*- Map structures (from r_defs.h) -----------------------------------------*/
typedef struct {
    fixed_t x;
    fixed_t y;
} vertex_t;

// Forward declarations
struct line_s;
struct sector_s;
struct mobj_s;

/*- Thing/Object structure (from p_mobj.h) ---------------------------------*/
typedef struct mobj_s {
    // Position
    fixed_t x, y, z;
    
    // Orientation
    angle_t angle;
    
    // Movement
    fixed_t momx, momy, momz;
    
    // Sprite
    spritenum_t sprite;
    int frame;
    
    // Interaction info
    fixed_t radius;
    fixed_t height;
    int flags;
    
    // Health
    int health;
    
    // Movement direction, movement generation
    int movedir;
    int movecount;
    
    // Thing being chased/attacked
    struct mobj_s* target;
    
    // Reaction time
    int reactiontime;
    
    // If >0, the target will be chased
    int threshold;
    
    // Additional info record for player avatars only
    struct player_s* player;
    
    // Player number last looked for
    int lastlook;
    
    // For nightmare respawn
    int spawnpoint;
    
    // Thing type
    mobjtype_t type;
    
    // Info for drawing
    int state;
    int tics;
    
    // Sector links
    struct sector_s* subsector;
    
    // Links in sector
    struct mobj_s* snext;
    struct mobj_s* sprev;
    
    // Links in blockmap
    struct mobj_s* bnext;
    struct mobj_s* bprev;
} mobj_t;

typedef struct {
    fixed_t textureoffset;
    fixed_t rowoffset;
    short toptexture;
    short bottomtexture;
    short midtexture;
    struct sector_s* sector;
} side_t;

typedef struct sector_s {
    fixed_t floorheight;
    fixed_t ceilingheight;
    short floorpic;
    short ceilingpic;
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

typedef enum {
    ST_HORIZONTAL,
    ST_VERTICAL,
    ST_POSITIVE,
    ST_NEGATIVE
} slopetype_t;

typedef struct line_s {
    vertex_t* v1;
    vertex_t* v2;
    fixed_t dx;
    fixed_t dy;
    short flags;
    short special;
    short tag;
    short sidenum[2];
    fixed_t bbox[4];
    slopetype_t slopetype;
    sector_t* frontsector;
    sector_t* backsector;
    int validcount;
    void* specialdata;
} line_t;

/*- Thing/Object system (from p_mobj.h) ------------------------------------*/
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
    NUMMOBJTYPES
} mobjtype_t;

// Sprite numbers
typedef enum {
    SPR_TROO,
    SPR_SHTG,
    SPR_PUNG,
    SPR_PISG,
    SPR_PISF,
    SPR_SHTF,
    SPR_SHT2,
    SPR_CHGG,
    SPR_CHGF,
    SPR_MISG,
    SPR_MISF,
    SPR_SAWG,
    SPR_PLSG,
    SPR_PLSF,
    SPR_BFGG,
    SPR_BFGF,
    SPR_BLUD,
    SPR_PUFF,
    SPR_BAL1,
    SPR_BAL2,
    SPR_PLSS,
    SPR_PLSE,
    SPR_MISL,
    SPR_BFS1,
    SPR_BFE1,
    SPR_BFE2,
    SPR_TFOG,
    SPR_IFOG,
    SPR_PLAY,
    SPR_POSS,
    SPR_SPOS,
    SPR_VILE,
    SPR_FIRE,
    SPR_FATB,
    SPR_FBXP,
    SPR_SKEL,
    SPR_MANF,
    SPR_FATT,
    SPR_CPOS,
    SPR_SARG,
    SPR_HEAD,
    SPR_BAL7,
    SPR_BOSS,
    SPR_BOS2,
    SPR_SKUL,
    SPR_SPID,
    SPR_BSPI,
    SPR_APLS,
    SPR_APBX,
    SPR_CYBR,
    SPR_PAIN,
    NUMSPRITES
} spritenum_t;

/*- Player structure (from d_player.h) -------------------------------------*/
typedef struct {
    int forwardmove;
    int sidemove;
    short angleturn;
    short consistancy;
    byte chatchar;
    byte buttons;
} ticcmd_t;

// Player sprite definition
typedef struct {
    int state;
    int tics;
    fixed_t sx;
    fixed_t sy;
} pspdef_t;

#define NUMPSPRITES 2

typedef struct player_s {
    struct mobj_s* mo;
    playerstate_t playerstate;
    ticcmd_t cmd;
    
    // Determine POV, including viewpoint bobbing during movement
    fixed_t viewz;
    fixed_t viewheight;
    fixed_t deltaviewheight;
    fixed_t bob;
    
    // This is only used between levels, mo->health is used during levels
    int health;
    int armorpoints;
    int armortype;
    
    // Power ups. invinc and invis are tic counters
    int powers[NUMPOWERS];
    boolean cards[NUMCARDS];
    boolean backpack;
    
    // Frags, kills of other players
    int frags[MAXPLAYERS];
    weapontype_t readyweapon;
    weapontype_t pendingweapon;
    
    boolean weaponowned[NUMWEAPONS];
    int ammo[NUMAMMO];
    int maxammo[NUMAMMO];
    
    // True if button down last tic
    int attackdown;
    int usedown;
    
    // Bit flags, for cheats and debug
    int cheats;
    
    // Refired shots are less accurate
    int refire;
    
    // For intermission stats
    int killcount;
    int itemcount;
    int secretcount;
    
    // Hint messages
    char* message;
    
    // For screen flashing (red or bright)
    int damagecount;
    int bonuscount;
    
    // Who did damage (NULL for floors/ceilings)
    struct mobj_s* attacker;
    
    // So gun flashes light up areas
    int extralight;
    
    // Current PLAYPAL, can be set to REDCOLORMAP for pain, etc
    int fixedcolormap;
    
    // Player skin colorshift, 0-3 for which color to draw player
    int colormap;
    
    // Overlay view sprites (gun, etc)
    pspdef_t psprites[NUMPSPRITES];
    
    // True if secret level has been done
    boolean didsecret;
} player_t;

/*- Math constants ----------------------------------------------------------*/
#define PI 3.141592657
#define ANG45   0x20000000
#define ANG90   0x40000000
#define ANG180  0x80000000
#define ANG270  0xc0000000
#define ANGLETOFINESHIFT 19
#define FINEANGLES 8192
#define FINEMASK (FINEANGLES-1)

/*- Memory management -------------------------------------------------------*/
#define PU_STATIC       1
#define PU_SOUND        2
#define PU_MUSIC        3
#define PU_DAVE         4
#define PU_LEVEL        50
#define PU_LEVSPEC      51
#define PU_CACHE        101

/*- Global variables --------------------------------------------------------*/
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
extern player_t players[MAXPLAYERS];
extern GameMode_t gamemode;

/*- Math tables -------------------------------------------------------------*/
extern fixed_t finesine[10240];
extern fixed_t finecosine[10240];
extern angle_t tantoangle[2049];

/*- Function prototypes -----------------------------------------------------*/

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
void G_BuildTiccmd(ticcmd_t* cmd);

// Menu functions
boolean M_Responder(event_t* ev);
void M_Ticker(void);
void M_Drawer(void);
void M_Init(void);

// Rendering functions
void R_Init(void);
void R_RenderPlayerView(player_t* player);
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

// Play subsystem
void P_Init(void);
void P_Ticker(void);

// Math functions
fixed_t FixedMul(fixed_t a, fixed_t b);
fixed_t FixedDiv(fixed_t a, fixed_t b);
int R_PointToAngle(fixed_t x, fixed_t y);
fixed_t R_PointToDist(fixed_t x, fixed_t y);

// Platform-specific embedded functions
void doom_complete_port_init(void);
void doom_complete_port_task(void);
void doom_complete_port_buttons_handler(int buttons);
void doom_complete_port_cleanup(void);

// Embedded map data
extern const byte embedded_e1m1_map[];
extern const int embedded_e1m1_map_size;

#endif // _DOOM_COMPLETE_PORT_H_
