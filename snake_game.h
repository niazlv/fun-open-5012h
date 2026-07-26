/*
 * Simple Snake Game
 * Classic snake game implementation
 */

#ifndef _SNAKE_GAME_H_
#define _SNAKE_GAME_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Prototypes --------------------------------------------------------------*/
void snake_game_init(void);
void snake_game_task(void);
void snake_game_buttons_handler(int buttons);
void snake_game_cleanup(void);

#endif // _SNAKE_GAME_H_
