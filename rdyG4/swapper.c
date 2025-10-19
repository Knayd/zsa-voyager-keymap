#include "swapper.h"

void update_swapper(
    bool *active,
    uint16_t cmdish,
    uint16_t tabish,
    uint16_t trigger,
    uint16_t keycode,
    keyrecord_t *record
) {
    if (keycode == trigger) {
        if (record->event.pressed) {
            if (!*active) {
                *active = true;
                register_code(cmdish);
            }
            register_code(tabish);
        } else {
            unregister_code(tabish);
            // Don't unregister cmdish until some other key is hit or released.
        }
    } else if (*active) {
        if(is_swapper_cancel_key(keycode)) {
            register_code(KC_ESCAPE);
            unregister_code(KC_ESCAPE);
        }
        if(keycode == KC_MS_BTN1) {
            tap_code(keycode);
            wait_ms(2); // Wait so only click gets processed and avoid double selection
        }
        unregister_code(cmdish);
        *active = false;
    }
}

