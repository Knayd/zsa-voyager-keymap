#pragma once
#include "quantum.h"

void os_mode_init(os_variant_t);

void os_mode_toggle(void);

void os_mode_swap_ctrl_gui(void);

bool is_mac_os(void);
