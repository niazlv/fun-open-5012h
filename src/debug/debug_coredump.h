/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Debug Coredump System
 * Provides stack trace and error analysis functionality
 */

#ifndef _DEBUG_COREDUMP_H_
#define _DEBUG_COREDUMP_H_

#include <stdint.h>
#include <stdbool.h>
#include "menu_widget.h"

/*- Definitions -------------------------------------------------------------*/
#define MAX_STACK_TRACE_DEPTH   16
#define MAX_ERROR_MESSAGE_LEN   64
#define MAX_COREDUMP_ENTRIES    6

// Identifies a live ring in retained SRAM. Changing the layout of
// coredump_store_t must change this, or a stale ring from the previous
// firmware would be read back as valid.
#define COREDUMP_MAGIC          0xC0DE0002ul

typedef enum {
    ERROR_TYPE_TIMER_OVERFLOW,
    ERROR_TYPE_HARD_FAULT,
    ERROR_TYPE_GENERAL_ERROR,
    ERROR_TYPE_ASSERTION_FAILED,
    ERROR_TYPE_MEMORY_ERROR,
    ERROR_TYPE_UNKNOWN
} error_type_t;

typedef struct {
    uint32_t pc;        // Program Counter
    uint32_t lr;        // Link Register
    uint32_t sp;        // Stack Pointer
    uint32_t function;  // Function address (if available)
} stack_frame_t;

typedef struct {
    error_type_t type;
    char message[MAX_ERROR_MESSAGE_LEN];
    uint32_t boot_id;       // which power-on / reset recorded this
    uint32_t timestamp;
    uint32_t error_pc;
    uint32_t error_lr;
    uint32_t error_sp;
    
    // CPU registers at time of error
    uint32_t r0, r1, r2, r3, r12;
    uint32_t psr;
    
    // Fault status registers
    uint32_t cfsr, hfsr, dfsr, afsr;
    uint32_t mmar, bfar;
    
    // Stack trace
    stack_frame_t stack_trace[MAX_STACK_TRACE_DEPTH];
    int stack_depth;
    
    // Additional context
    uint32_t stack_free;
} coredump_entry_t;

// The whole ring, kept in retained SRAM (COREDUMP_RAM_BASE, see capture.h).
// Nothing is linked into that region, so the startup code neither loads nor
// zeroes it and the contents survive a reset; magic + crc tell a live ring
// apart from whatever the SRAM happened to hold after a cold start.
typedef struct {
    uint32_t magic;
    uint32_t boot_id;
    int32_t  count;
    int32_t  write_index;
    coredump_entry_t entries[MAX_COREDUMP_ENTRIES];
    uint32_t crc;           // over everything above, must stay last
} coredump_store_t;

/*- Variables ---------------------------------------------------------------*/
// Settings for the system menu, help pages for its Help section
extern const menu_def_t coredump_menu;
extern const menu_def_t coredump_help_menu;

/*- Prototypes --------------------------------------------------------------*/
void debug_coredump_init(void);
void debug_coredump_capture(error_type_t type, const char* message,
                           uint32_t pc, uint32_t lr, uint32_t sp);
void debug_coredump_capture_hard_fault(uint32_t lr, uint32_t msp, uint32_t psp);
int debug_coredump_get_count(void);
void debug_coredump_clear_all(void);

// Incremented on every boot that finds a valid ring; entries carry the value
// they were recorded under, so the viewer can tell "this session" from
// "before the last reset"
uint32_t debug_coredump_boot_id(void);
bool debug_coredump_ring_retained(void);

// True once debug_coredump_init() has run. Before that the ring is whatever
// the SRAM happened to hold, and a dump written into it is either wiped by
// store_reset() on a cold start or filed under the previous boot's id - so a
// caller that can run that early (config_init() does, main.c brings the store
// up first) has to ask rather than assume.
bool debug_coredump_ready(void);

// Application entry points, see launcher.c
void coredump_app_init(void);
void coredump_app_task(void);
void coredump_app_buttons_handler(int buttons);
void coredump_app_cleanup(void);
void coredump_app_redraw(void);

// Macro for easy error capture with automatic PC/LR/SP
#define COREDUMP_CAPTURE(type, msg) do { \
    uint32_t pc, lr, sp; \
    __asm volatile ("mov %0, pc" : "=r" (pc)); \
    __asm volatile ("mov %0, lr" : "=r" (lr)); \
    __asm volatile ("mov %0, sp" : "=r" (sp)); \
    debug_coredump_capture(type, msg, pc, lr, sp); \
} while(0)

#endif // _DEBUG_COREDUMP_H_
