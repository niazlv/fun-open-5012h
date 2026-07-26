/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 *
 * Test file header for coredump functionality
 */

#ifndef _TEST_COREDUMP_H_
#define _TEST_COREDUMP_H_

/*- Prototypes --------------------------------------------------------------*/
void test_timer_overflow(void);
void test_general_error(void);
void test_memory_error(void);
void test_assertion_failure(void);
void test_hard_fault(void);
void test_show_coredump_viewer(void);
void test_coredump_menu(void);
void test_coredump_buttons_handler(int buttons);

#endif // _TEST_COREDUMP_H_
