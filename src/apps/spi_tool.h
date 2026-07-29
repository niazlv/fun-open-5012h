/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SPI flash loader - the host-facing half of getting data onto the serial part
 */

#ifndef _SPI_TOOL_H_
#define _SPI_TOOL_H_

/*- Prototypes --------------------------------------------------------------*/
void spi_tool_init(void);
void spi_tool_task(void);
void spi_tool_buttons_handler(int buttons);
void spi_tool_cleanup(void);
void spi_tool_redraw(void);

#endif // _SPI_TOOL_H_
