/*
 * Flash Memory Viewer
 * Interactive viewer for flash memory contents with structure analysis
 */

#ifndef _FLASH_VIEWER_H_
#define _FLASH_VIEWER_H_

#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define FLASH_START_ADDR        0x08000000
#define FLASH_SIZE              (512 * 1024)   // GD32F407VE ("VE" = 512KB); reads past this bus-fault
#define FLASH_PAGE_SIZE         16              // 16 bytes per line
#define FLASH_LINES_PER_PAGE    15              // Lines visible on screen
#define FLASH_BYTES_PER_PAGE    (FLASH_PAGE_SIZE * FLASH_LINES_PER_PAGE)

typedef enum {
    FLASH_VIEW_HEX = 0,
    FLASH_VIEW_ASCII,
    FLASH_VIEW_STRUCTURE,
    FLASH_VIEW_DISASM,
    FLASH_VIEW_COUNT
} flash_view_mode_t;

typedef struct {
    const char* name;
    uint32_t address;
    uint32_t size;
    const char* description;
} flash_section_t;

/*- Prototypes --------------------------------------------------------------*/
void flash_viewer_init(void);
void flash_viewer_task(void);
void flash_viewer_buttons_handler(int buttons);
void flash_viewer_cleanup(void);
bool flash_viewer_is_active(void);

// Memory analysis functions
void flash_analyze_structure(void);
const flash_section_t* flash_get_sections(int* count);
bool flash_is_valid_instruction(uint32_t addr);
bool flash_is_ascii_printable(uint8_t byte);

#endif // _FLASH_VIEWER_H_
