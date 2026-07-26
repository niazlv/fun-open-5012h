/*
 * Input translation layer: key remapping and sticky shift mode.
 * Sits between the raw button events and the UI stack.
 */

#ifndef _INPUT_H_
#define _INPUT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Prototypes --------------------------------------------------------------*/
void input_init(void);
int input_translate(int buttons);

void shift_mode_task(void);
bool shift_mode_is_active(void);
void shift_mode_reset(void);

#endif // _INPUT_H_
