#pragma once

#include QMK_KEYBOARD_H

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
