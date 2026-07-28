/*
 * Copyright (c) 2026 Niaz Leushkin <niazlv03@gmail.com>
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Input translation layer: key remapping and sticky shift mode.
 * Sits between the raw button events and the UI stack.
 */

#ifndef _INPUT_H_
#define _INPUT_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
// Stored in config.key_mapping to mean "this key does nothing". Zero cannot
// carry that meaning: it is what a config saved before the editor existed
// holds, and what the defaults reset to, and both of those mean "not set, the
// key is itself".
#define KEY_MAP_NONE   (1u << 31)

/*- Prototypes --------------------------------------------------------------*/
void input_init(void);
int input_translate(int buttons);

// Raw passthrough, for the editor's "press a key" prompt: while it is on,
// input_translate hands the buttons over untouched, so a capture reads the
// key that was physically pressed rather than what it currently acts as.
void input_capture_set(bool enable);

// The remappable keys, in the order the editor lists them
int         key_remap_count(void);
const char *key_remap_name(int index);
uint32_t    key_remap_button(int index);
int         key_remap_index_of(uint32_t button);

// What the key at index does now: its own button, another key's button, or 0
// for a key that has been turned off
uint32_t key_remap_target(int index);
void     key_remap_set(int index, uint32_t target);
void     key_remap_reset(void);
bool     key_remap_is_default(void);

void shift_mode_task(void);
bool shift_mode_is_active(void);
void shift_mode_reset(void);

#endif // _INPUT_H_
