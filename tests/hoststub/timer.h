/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Host test stub */
#ifndef _HOSTSTUB_TIMER_H_
#define _HOSTSTUB_TIMER_H_

#include <stdint.h>

#define TIMER_DISABLE  (-1)

void timer_add(int *timer);
void timer_remove(int *timer);
uint32_t timer_ms(void);

#endif
