#include "os_mode.h"
#include "eeconfig.h"

bool is_mac = false;

void os_mode_init(os_variant_t detected_os) {
    is_mac = (detected_os == OS_MACOS || detected_os == OS_IOS);
    os_mode_swap_ctrl_gui();
}

void os_mode_toggle(void) {
    is_mac = !is_mac;
    os_mode_swap_ctrl_gui();
}

void os_mode_swap_ctrl_gui(void) {
    keymap_config.swap_lctl_lgui = is_mac;
}

bool is_mac_os(void) {
    return is_mac;
}
