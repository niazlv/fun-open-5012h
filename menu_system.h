/*
 * Menu System for Multiple Applications
 * Scalable menu system for selecting different sub-applications
 */

#ifndef _MENU_SYSTEM_H_
#define _MENU_SYSTEM_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*- Definitions -------------------------------------------------------------*/
#define MAX_MENU_ITEMS      12
#define MENU_ITEM_HEIGHT    30
#define MENU_MARGIN         10
#define MENU_SCROLL_SPEED   5

/*- Types -------------------------------------------------------------------*/
typedef enum {
    APP_MENU = 0,
    APP_3D_GRAPHICS,
    APP_3D_ENGINE,
    APP_RAYTRACE_TEST,
    APP_FLAPPY_BIRD,
    APP_SNAKE,
    APP_DOOM_PORT,
    APP_DOOM_FULL,
    APP_OSCILLOSCOPE,
    APP_COREDUMP_VIEWER,
    APP_FLASH_VIEWER,
    APP_COUNT
} app_id_t;

typedef struct {
    app_id_t id;
    const char *name;
    const char *description;
    void (*init_func)(void);
    void (*task_func)(void);
    void (*buttons_func)(int buttons);
    void (*cleanup_func)(void);
} menu_item_t;

typedef struct {
    menu_item_t items[MAX_MENU_ITEMS];
    int item_count;
    int selected_item;
    int scroll_offset;
    app_id_t current_app;
    bool in_menu;
    bool menu_dirty;
} menu_system_t;

/*- Prototypes --------------------------------------------------------------*/
void menu_system_init(void);
void menu_system_task(void);
void menu_system_buttons_handler(int buttons);
void menu_system_register_app(app_id_t id, const char *name, const char *description,
                             void (*init_func)(void), void (*task_func)(void),
                             void (*buttons_func)(int), void (*cleanup_func)(void));
void menu_system_show(void);
void menu_system_hide(void);
void menu_system_launch_app(app_id_t app_id);
void menu_system_return_to_menu(void);
app_id_t menu_system_get_current_app(void);
bool menu_system_is_in_menu(void);

#endif // _MENU_SYSTEM_H_
