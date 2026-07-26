/*
 * Context Menu System
 * Windows-style context menu with system information and settings
 */

#ifndef _CONTEXT_MENU_H_
#define _CONTEXT_MENU_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define CONTEXT_MENU_MAX_ITEMS    8
#define CONTEXT_MENU_MAX_SUBITEMS 6
#define CONTEXT_MENU_WIDTH        180
#define CONTEXT_MENU_ITEM_HEIGHT  20
#define CONTEXT_MENU_MARGIN       2

/*- Types -------------------------------------------------------------------*/
typedef enum {
    CONTEXT_MENU_ITEM_NORMAL = 0,
    CONTEXT_MENU_ITEM_SUBMENU,
    CONTEXT_MENU_ITEM_SEPARATOR,
    CONTEXT_MENU_ITEM_INFO
} context_menu_item_type_t;

typedef struct context_menu_item_s {
    context_menu_item_type_t type;
    const char *text;
    void (*action)(void);
    struct context_menu_item_s *submenu;
    int submenu_count;
} context_menu_item_t;

typedef struct {
    context_menu_item_t *items;
    int item_count;
    int selected_item;
    int x, y;
    int width, height;
    bool visible;
    bool dirty;
    bool in_submenu;
    context_menu_item_t *current_submenu;
    int submenu_selected;
    int submenu_x, submenu_y;
    bool modal_dialog_active;
} context_menu_t;

/*- Prototypes --------------------------------------------------------------*/
void context_menu_init(void);
void context_menu_show(int x, int y);
void context_menu_hide(void);
void context_menu_task(void);
void context_menu_buttons_handler(int buttons);
bool context_menu_is_visible(void);
bool context_menu_is_modal_active(void);
void context_menu_close_modal(void);

// Menu actions
void context_menu_action_exit_app(void);
void context_menu_action_trigger_settings(void);
void context_menu_action_general_settings(void);
void context_menu_action_reboot_firmware(void);
void context_menu_action_reboot_system(void);
void context_menu_action_device_info(void);
void context_menu_action_system_info(void);

// General settings submenu actions
void context_menu_action_brightness_settings(void);
void context_menu_action_key_remapping(void);
void context_menu_action_shift_mode_toggle(void);
void context_menu_action_key_bindings(void);

// Settings UI system
typedef enum {
    SETTINGS_UI_NONE = 0,
    SETTINGS_UI_BRIGHTNESS,
    SETTINGS_UI_KEY_BINDINGS,
    SETTINGS_UI_KEY_REMAPPING
} settings_ui_mode_t;

typedef struct {
    settings_ui_mode_t current_mode;
    bool active;
    bool dirty;
    int close_timer;
    int selected_item;
    int brightness_value;
    bool brightness_changed;
    bool key_remap_mode;
    int key_remap_index;
} settings_ui_t;

void settings_ui_init(void);
void settings_ui_show(settings_ui_mode_t mode);
void settings_ui_hide(void);
void settings_ui_task(void);
void settings_ui_buttons_handler(int buttons);
bool settings_ui_is_active(void);

// Shift mode functions
void shift_mode_init(void);
void shift_mode_task(void);
void shift_mode_handle_button(int button);
bool shift_mode_is_active(void);
void shift_mode_draw_indicator(void);

// Key remapping functions
void key_remapping_init(void);
uint32_t key_remapping_translate(uint32_t button);

#endif // _CONTEXT_MENU_H_
