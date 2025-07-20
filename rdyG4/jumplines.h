#pragma once

#include QMK_KEYBOARD_H

void handle_jump_line_key(
    uint16_t trigger,
    uint16_t direction,
    uint16_t keycode,
    keyrecord_t *record
);
