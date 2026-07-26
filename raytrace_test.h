/*
 * Simple Ray Tracing Test
 * Basic ray tracing demonstration with spheres
 */

#ifndef _RAYTRACE_TEST_H_
#define _RAYTRACE_TEST_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Prototypes --------------------------------------------------------------*/
void raytrace_test_init(void);
void raytrace_test_task(void);
void raytrace_test_buttons_handler(int buttons);
void raytrace_test_cleanup(void);

#endif // _RAYTRACE_TEST_H_
