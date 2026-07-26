/*
 * 3D Cube Renderer
 * Replaces oscilloscope functionality with 3D rotating cube
 */

#ifndef _CUBE3D_H_
#define _CUBE3D_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define CUBE3D_CENTER_X    (LCD_WIDTH / 2)
#define CUBE3D_CENTER_Y    (LCD_HEIGHT / 2)
#define CUBE3D_SIZE        80
#define CUBE3D_DISTANCE    200

/*- Prototypes --------------------------------------------------------------*/
void cube3d_init(void);
void cube3d_task(void);
void cube3d_buttons_handler(int buttons);
void cube3d_cleanup(void);

#endif // _CUBE3D_H_
