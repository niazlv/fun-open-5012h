/*
 * Debug Coredump System
 * Provides stack trace and error analysis functionality
 */

#ifndef _DEBUG_COREDUMP_H_
#define _DEBUG_COREDUMP_H_

#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define MAX_STACK_TRACE_DEPTH   16
#define MAX_ERROR_MESSAGE_LEN   64
#define MAX_COREDUMP_ENTRIES    8

// Flash storage configuration
#define COREDUMP_FLASH_SECTOR   11              // Use sector 11 (last 128KB sector)
#define COREDUMP_FLASH_ADDR     0x080E0000      // Start of sector 11
#define COREDUMP_MAGIC          0xDEADBEEF      // Magic number for validation

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
    uint32_t heap_free;
    uint32_t stack_free;
    int active_timers;
    uint32_t system_state;
} coredump_entry_t;

typedef struct {
    uint32_t magic;                             // Magic number for validation
    uint32_t count;                             // Number of stored entries
    uint32_t write_index;                       // Next write position
    uint32_t crc32;                             // CRC32 of the data
    coredump_entry_t entries[MAX_COREDUMP_ENTRIES];
} coredump_flash_storage_t;

/*- Prototypes --------------------------------------------------------------*/
void debug_coredump_init(void);
void debug_coredump_capture(error_type_t type, const char* message, 
                           uint32_t pc, uint32_t lr, uint32_t sp);
void debug_coredump_capture_hard_fault(uint32_t lr, uint32_t msp, uint32_t psp);
void debug_coredump_show_viewer(void);
void debug_coredump_viewer_buttons_handler(int buttons);
bool debug_coredump_viewer_is_active(void);
void debug_coredump_viewer_task(void);
int debug_coredump_get_count(void);
void debug_coredump_clear_all(void);

// Application wrapper functions for menu system
void coredump_app_init(void);
void coredump_app_task(void);
void coredump_app_buttons_handler(int buttons);
void coredump_app_cleanup(void);

// Macro for easy error capture with automatic PC/LR/SP
#define COREDUMP_CAPTURE(type, msg) do { \
    uint32_t pc, lr, sp; \
    __asm volatile ("mov %0, pc" : "=r" (pc)); \
    __asm volatile ("mov %0, lr" : "=r" (lr)); \
    __asm volatile ("mov %0, sp" : "=r" (sp)); \
    debug_coredump_capture(type, msg, pc, lr, sp); \
} while(0)

#endif // _DEBUG_COREDUMP_H_
