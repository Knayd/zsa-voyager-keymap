#pragma once

#include QMK_KEYBOARD_H
#include "quantum.h"

typedef struct {
    uint16_t trigger;
    uint16_t default_key;
    uint8_t default_mods;  // Optional
    uint16_t mac_os_key;
    uint8_t mac_os_mods;  // Optional
} magic_key_config_t;

void handle_magic_key(
    magic_key_config_t config,
    uint16_t keycode,
    keyrecord_t *record
);

// Some keys, like mouse buttons, need a delay after applying mods for the whole thing to work correctly
bool is_key_with_delayed_mods(uint16_t keycode);
