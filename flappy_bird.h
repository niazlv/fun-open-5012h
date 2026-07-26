/*
 * Simple Flappy Bird Game
 * Basic side-scrolling bird game
 */

#ifndef _FLAPPY_BIRD_H_
#define _FLAPPY_BIRD_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
extern const menu_def_t flappy_bird_menu;

/*- Prototypes --------------------------------------------------------------*/
void flappy_bird_init(void);
void flappy_bird_task(void);
void flappy_bird_buttons_handler(int buttons);
void flappy_bird_cleanup(void);
void flappy_bird_redraw(void);

#endif // _FLAPPY_BIRD_H_
