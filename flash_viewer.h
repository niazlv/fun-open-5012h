/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
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
    FLASH_VIEW_HEX = 0,
    FLASH_VIEW_ASCII,
    FLASH_VIEW_STRUCTURE,
    FLASH_VIEW_THUMB,
    FLASH_VIEW_COUNT
} flash_view_mode_t;

typedef struct {
    const char* name;
    uint32_t address;
    uint32_t size;
    const char* description;
} flash_section_t;

/*- Variables ---------------------------------------------------------------*/
extern const menu_def_t flash_viewer_menu;

/*- Prototypes --------------------------------------------------------------*/
void flash_viewer_init(void);
void flash_viewer_task(void);
void flash_viewer_buttons_handler(int buttons);
void flash_viewer_cleanup(void);
void flash_viewer_redraw(void);

const flash_section_t* flash_get_sections(int* count);

#endif // _FLASH_VIEWER_H_
