#include "os_mode.h"
#include "eeconfig.h"

typedef union {
    uint32_t raw;
    struct {
        bool os_mode : 1;  // 0 = Windows, 1 = Mac
    };
} user_config_t;

static user_config_t user_config;

#define EEPROM_ADDR_USER_CONFIG 0x00  // safe free address in EEPROM

void os_mode_init(void) {
    user_config.raw = eeconfig_read_user();
    if (user_config.raw == 0xFFFFFFFF) {  // sanity check, default to windows
        user_config.os_mode = 0;
        eeconfig_update_user(user_config.raw);
    }
    os_mode_swap_ctrl_gui();
}

void os_mode_toggle(void) {
    user_config.os_mode = !user_config.os_mode;
    eeconfig_update_user(user_config.raw);
    os_mode_swap_ctrl_gui();
}

void os_mode_swap_ctrl_gui(void) {
    bool is_mac_os = user_config.os_mode == OS_MAC;
    keymap_config.swap_lctl_lgui = is_mac_os;
}

os_mode_t os_mode_get(void) {
    return user_config.os_mode ? OS_MAC : OS_WINDOWS;
}
