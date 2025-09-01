#include "magickey.h"
#include "os_mode.h"

void handle_magic_key(
    magic_key_config_t config,
    uint16_t keycode,
    keyrecord_t *record
) {
    if (keycode != config.trigger) return;

    bool is_mac_os = os_mode_get() == OS_MAC;
    uint16_t key = (is_mac_os && config.mac_os_key) ? config.mac_os_key : config.default_key;
    uint8_t mods = (is_mac_os && config.mac_os_mods) ? config.mac_os_mods : config.default_mods;

    if (record->event.pressed) {
        if(mods) {
            add_mods(mods);
            send_keyboard_report();
        }
        register_code(key);
    } else {
        unregister_code(key);
        if(mods) {
            del_mods(mods);
            send_keyboard_report();
        }
    }
}
