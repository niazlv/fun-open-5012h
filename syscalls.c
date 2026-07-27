/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * System call stubs for embedded systems
 * Provides minimal implementations for newlib system calls
 */

#include <sys/stat.h>
#include <errno.h>
#include <stdint.h>

// Heap management. Heap grows up from _end (end of .bss in TCM), the stack
// grows down from _stack_top (top of TCM); keep them from colliding.
#define STACK_RESERVE (8 * 1024)

extern char _end;
extern char _stack_top;
static char *heap_end = 0;

void *_sbrk(int incr)
{
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }

    if ((uintptr_t)heap_end + incr > (uintptr_t)&_stack_top - STACK_RESERVE) {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev_heap_end = heap_end;
    heap_end += incr;

    return (void *)prev_heap_end;
}

// Process management stubs
int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1);
}

// File I/O stubs
int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
