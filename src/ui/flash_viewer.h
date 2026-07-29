/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Flash Memory Viewer
 * Interactive viewer for flash memory contents with structure analysis
 */

#ifndef _FLASH_VIEWER_H_
#define _FLASH_VIEWER_H_

#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define FLASH_START_ADDR        0x08000000
// Fallback only: the real size comes from the device's flash size register,
// and reads past the end of the array bus-fault
#define FLASH_SIZE              (512 * 1024)

#define FLASH_MAX_SECTIONS      12

typedef enum {
    FLASH_VIEW_MAP = 0,     // one cell per sector: what is used, and where
    FLASH_VIEW_HEX,
    FLASH_VIEW_ASCII,
    FLASH_VIEW_STRUCTURE,
    FLASH_VIEW_THUMB,
    FLASH_VIEW_CONFIG,      // the settings store, decoded rather than dumped
    FLASH_VIEW_COUNT
} flash_view_mode_t;

// Address spaces the viewer will read. Deliberately a whitelist and not a free
// pointer: an unmapped address bus-faults, and a peripheral register can have
// side effects just from being read - popping a FIFO, clearing a status bit -
// so "read any address" would turn the viewer into a way to break the running
// scope. Everything here is a plain array and safe to walk.
//
// SPI is the odd one: it is not in the address space at all, so its bytes come
// back through a command on the bus rather than off a pointer. The viewer does
// not care - it asks a source for a byte - but it is why the read path is a
// function and not a cast.
typedef enum {
    MEM_REGION_FLASH = 0,
    MEM_REGION_SRAM,
    MEM_REGION_TCM,
    MEM_REGION_SYSTEM,      // bootloader ROM, and the id/size registers past it
    MEM_REGION_SPI,         // the 8 MB serial NOR part, read over SPI0
    MEM_REGION_COUNT
} mem_region_t;

typedef struct {
    const char* name;
    uint32_t address;
    uint32_t size;
    const char* description;
} flash_section_t;

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t flash_viewer_menu;
extern const menu_def_t flash_viewer_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void flash_viewer_init(void);
void flash_viewer_task(void);
void flash_viewer_buttons_handler(int buttons);
void flash_viewer_cleanup(void);
void flash_viewer_redraw(void);

const flash_section_t* flash_get_sections(int* count);

#endif // _FLASH_VIEWER_H_
