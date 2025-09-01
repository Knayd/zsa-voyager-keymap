#pragma once
#include "quantum.h"

typedef enum {
    OS_WINDOWS = 0,
    OS_MAC = 1
} os_mode_t;

void os_mode_init(void);

void os_mode_toggle(void);

os_mode_t os_mode_get(void);
