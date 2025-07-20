#include "jumplines.h"

void handle_jump_line_key(
    uint16_t trigger,
    uint16_t direction_key,
    uint16_t keycode,
    keyrecord_t *record
) {
    if (record->event.pressed && keycode == trigger) {
        for (int i = 0; i < 10; i++) {
            tap_code(direction_key);
        }
    }
}
