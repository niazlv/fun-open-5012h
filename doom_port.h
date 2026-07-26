/*
 * DOOM-style raycasting demo for the GD32F407VE platform
 *
 * A textbook DDA raycaster: one ray per screen column, perpendicular wall
 * distance (so no fisheye), and one windowed LCD transfer per column band.
 * The map is a const table in flash, not a RAM copy.
 */

#ifndef _DOOM_PORT_H_
#define _DOOM_PORT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "buttons.h"
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define DOOM_VIEW_WIDTH     LCD_WIDTH
#define DOOM_HUD_HEIGHT     30
#define DOOM_VIEW_HEIGHT    (LCD_HEIGHT - DOOM_HUD_HEIGHT)

#define DOOM_MAP_WIDTH      64
#define DOOM_MAP_HEIGHT     64

/*- Variables ---------------------------------------------------------------*/
extern const menu_def_t doom_port_menu;

/*- Prototypes --------------------------------------------------------------*/
void doom_port_init(void);
void doom_port_task(void);
void doom_port_buttons_handler(int buttons);
void doom_port_cleanup(void);
void doom_port_redraw(void);

#endif // _DOOM_PORT_H_
