/*
 * Simple Snake Game
 * Classic snake game implementation
 */

#ifndef _SNAKE_GAME_H_
#define _SNAKE_GAME_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Variables ---------------------------------------------------------------*/
extern const menu_def_t snake_game_menu;

/*- Prototypes --------------------------------------------------------------*/
void snake_game_init(void);
void snake_game_task(void);
void snake_game_buttons_handler(int buttons);
void snake_game_cleanup(void);
void snake_game_redraw(void);

#endif // _SNAKE_GAME_H_
