/*
 * Simple Flappy Bird Game
 * Basic side-scrolling bird game
 */

#ifndef _FLAPPY_BIRD_H_
#define _FLAPPY_BIRD_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Prototypes --------------------------------------------------------------*/
void flappy_bird_init(void);
void flappy_bird_task(void);
void flappy_bird_buttons_handler(int buttons);
void flappy_bird_cleanup(void);

#endif // _FLAPPY_BIRD_H_
