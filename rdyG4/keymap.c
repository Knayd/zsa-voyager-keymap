#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#include "print.h"
#include "swapper.h"
#include "nshot_mod.h"
#include "magickey.h"
#include "jumplines.h"
#include "os_mode.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif
#define LA_NAV MO(1)

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  HSV_0_255_255,
  HSV_74_255_255,
  HSV_152_255_255,
  HSV_41_255_255,
  HSV_135_255_255,
  HSV_0_0_255,
  HSV_114_255_255,
  HSV_234_255_255,
  HSV_217_249_239,
  HSV_191_247_228,
  HSV_60_247_227,
  ST_MACRO_0,
  ST_MACRO_1,
  DRAG_SCROLL,

  SW_WIN,  // Switch to next window (alt-tab)
  // Custom oneshot mod implementation with no timers.
  OS_SHFT,
  OS_CTRL,
  OS_ALT,
  OS_GUI,

  // Magic keys
  DLT_WRD, // Delete word
  NEXT_TAB,
  PREV_TAB,
  // Win manager keys
  WIN_LEFT,
  WIN_RIGHT,
  WIN_UP,
  WIN_DOWN,
  WIN_FULL,
  DESK_LEFT,
  DESK_RIGHT,
  MON_LEFT,
  MON_RIGHT,
  // Custom page up/down
  JUMP_UP,
  JUMP_DOWN,
  // OS mode
  OS_TOGGLE,
};

nshot_state_t  nshot_states[] = {
//| trigger  | modbit            | swap-to          | max | roll into | State         | ## | timer | keydown? | //roll-in action |
//|----------|-------------------|------------------|-----|-----------|---------------|----|-------|----------|------------------|
    {OS_SHFT,  MOD_BIT(KC_LSFT),  MOD_BIT(KC_LSFT),   1,   true,       os_up_unqueued,  0,   0,     false},    // S-a
    {OS_CTRL,  MOD_BIT(KC_LCTL),  MOD_BIT(KC_LGUI),   1,   true,       os_up_unqueued,  0,   0,     false},    // C-a
    {OS_ALT,   MOD_BIT(KC_LALT),  MOD_BIT(KC_LALT),   1,   true,       os_up_unqueued,  0,   0,     false},    // A-a
    {OS_GUI,   MOD_BIT(KC_LGUI),  MOD_BIT(KC_LCTL),   1,   true,       os_up_unqueued,  0,   0,     false},    // G-a
};

uint8_t NUM_NSHOT_STATES = sizeof(nshot_states) / sizeof(nshot_state_t);

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           TD(DANCE_0),    KC_7,           KC_8,           KC_9,           KC_0,           TD(DANCE_1),
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           TD(DANCE_2),
    KC_BSPC,        MT(MOD_LGUI, KC_A),MT(MOD_LALT, KC_S),MT(MOD_LSFT, KC_D),MT(MOD_LCTL, KC_F),KC_G,                                           KC_H,           MT(MOD_LCTL, KC_J),MT(MOD_LSFT, KC_K),MT(MOD_LALT, KC_L),MT(MOD_LGUI, KC_QUOTE),KC_SCLN,
    MO(7),          KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       CW_TOGG,
                                                    LA_NAV,         LT(4, KC_ENTER),                                      KC_LEFT_SHIFT,  LT(2, KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN3,                                     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, LCTL(KC_W),     SW_WIN,         PREV_TAB,       NEXT_TAB,       KC_MS_BTN2,                                     JUMP_UP,        KC_HOME,        KC_UP,          KC_END,         KC_TRANSPARENT, KC_TRANSPARENT,
    DLT_WRD,        OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,  KC_MS_BTN1,                                     JUMP_DOWN,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT,
    LCTL(LSFT(KC_Z)),KC_PC_UNDO,     KC_PC_CUT,      KC_PC_COPY,     KC_PC_PASTE,    DRAG_SCROLL,                                    TD(DANCE_3),    LALT(LCTL(KC_L)),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_EQUAL,       KC_GRAVE,       KC_AMPR,        KC_PIPE,        KC_PLUS,        KC_TRANSPARENT,
    KC_TRANSPARENT, OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,        KC_CIRC,                                        KC_ASTR,        KC_LPRN,        KC_LCBR,        KC_LBRC,        KC_MINUS,       KC_TRANSPARENT,
    KC_TRANSPARENT, KC_BSLS,        KC_TRANSPARENT, ST_MACRO_0,     ST_MACRO_1,     KC_TRANSPARENT,                                 KC_TILD,        KC_RPRN,        KC_RCBR,        KC_RBRC,        KC_UNDS,        KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_DYNAMIC_TAPPING_TERM_DOWN,QK_DYNAMIC_TAPPING_TERM_UP,QK_DYNAMIC_TAPPING_TERM_PRINT,                                KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, OS_TOGGLE,
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_PLAY_PAUSE,                                KC_F12,         KC_F7,          KC_F8,          KC_F9,          KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,          KC_TRANSPARENT,                                 KC_F11,         KC_F4,          KC_F5,          KC_F6,          KC_TRANSPARENT, KC_TRANSPARENT,
    KC_AUDIO_MUTE,  KC_TRANSPARENT, KC_TRANSPARENT, LCTL(LSFT(KC_C)),LCTL(LSFT(KC_V)),KC_TRANSPARENT,                                 KC_F10,         KC_F1,          KC_F2,          KC_F3,          KC_TRANSPARENT, EE_CLR,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_EQUAL,       KC_7,           KC_8,           KC_9,           KC_PLUS,        KC_TRANSPARENT,
    KC_TRANSPARENT, OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,        KC_TRANSPARENT,                                 KC_ASTR,        KC_4,           KC_5,           KC_6,           KC_MINUS,       KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_DOT,         KC_1,           KC_2,           KC_3,           KC_SLASH,       KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_0
  ),
  [5] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        HSV_0_255_255,  HSV_74_255_255, HSV_152_255_255,HSV_41_255_255, HSV_135_255_255,QK_BOOT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 HSV_0_0_255,    HSV_114_255_255,HSV_234_255_255,HSV_217_249_239,HSV_191_247_228,TD(DANCE_4),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, HSV_60_247_227, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [6] = LAYOUT_voyager(
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          TD(DANCE_5),    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                                    KC_NO,          KC_NO,                                          KC_NO,          KC_NO
  ),
  [7] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, DESK_LEFT,      DESK_RIGHT,     KC_TRANSPARENT,                                 KC_TRANSPARENT, MON_LEFT,       WIN_UP,         MON_RIGHT,      KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, WIN_LEFT,       WIN_DOWN,       WIN_RIGHT,      KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, WIN_FULL,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};

const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] PROGMEM = LAYOUT(
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R',
  'L', 'L', 'L', 'L', 'L', 'L', 'R', 'R', 'R', 'R', 'R', 'R',
  '*', '*', '*', '*'
);

extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
  os_mode_init();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [6] = { {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (!keyboard_config.disable_layer_led) {
    switch (biton32(layer_state)) {
      case 6:
        set_layer_color(6);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  if (nshot_states[0].state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][4], RGB_RED);
  }

  if (nshot_states[1].state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][5], RGB_RED);
  }

  if (nshot_states[2].state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][3], RGB_RED);
  }

  if (nshot_states[3].state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][2], RGB_RED);
  }

  return true;
}

bool is_nshot_cancel_key(uint16_t keycode) {
    switch (keycode) {
    case LA_NAV:
        return true;
    default:
        return false;
    }
}

bool is_nshot_ignored_key(uint16_t keycode) {
    switch (keycode) {
    case LA_NAV:
    case OS_SHFT:
    case OS_CTRL:
    case OS_ALT:
    case OS_GUI:
        return true;
    default:
        return false;
    }
}

bool is_swapper_cancel_key(uint16_t keycode) {
    switch (keycode) {
    case KC_ESCAPE:
        return true;
    default:
        return false;
    }
}

bool sw_win_active = false;
extern bool set_scrolling;
extern bool navigator_turbo;
extern bool navigator_aim;


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[6];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_6);
        tap_code16(KC_6);
        tap_code16(KC_6);
    }
    if(state->count > 3) {
        tap_code16(KC_6);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_6); break;
        case DOUBLE_TAP: layer_move(6); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_6); register_code16(KC_6);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_6); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_6); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_PSCR);
        tap_code16(KC_PSCR);
        tap_code16(KC_PSCR);
    }
    if(state->count > 3) {
        tap_code16(KC_PSCR);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_PSCR); break;
        case DOUBLE_TAP: register_code16(KC_F16); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_PSCR); register_code16(KC_PSCR);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_PSCR); break;
        case DOUBLE_TAP: unregister_code16(KC_F16); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_PSCR); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_DELETE);
        tap_code16(KC_DELETE);
        tap_code16(KC_DELETE);
    }
    if(state->count > 3) {
        tap_code16(KC_DELETE);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_DELETE); break;
        case DOUBLE_TAP: layer_move(5); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_DELETE); register_code16(KC_DELETE);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_DELETE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_DELETE); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(LGUI(KC_RIGHT)));
        tap_code16(LCTL(LGUI(KC_RIGHT)));
        tap_code16(LCTL(LGUI(KC_RIGHT)));
    }
    if(state->count > 3) {
        tap_code16(LCTL(LGUI(KC_RIGHT)));
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(LCTL(LGUI(KC_RIGHT))); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_BSLS)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(LGUI(KC_RIGHT))); register_code16(LCTL(LGUI(KC_RIGHT)));
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(LCTL(LGUI(KC_RIGHT))); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_BSLS)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(LGUI(KC_RIGHT))); break;
    }
    dance_state[3].step = 0;
}
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case DOUBLE_TAP: layer_move(0); break;
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
    }
    dance_state[4].step = 0;
}
void dance_5_finished(tap_dance_state_t *state, void *user_data);
void dance_5_reset(tap_dance_state_t *state, void *user_data);

void dance_5_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case DOUBLE_TAP: layer_move(0); break;
    }
}

void dance_5_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
    }
    dance_state[5].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_5_finished, dance_5_reset),
};

layer_state_t layer_state_set_user(layer_state_t state) {
  // Nav + Sym = Func
  state =  update_tri_layer_state(state, 1, 2, 3);
  return state;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    #ifdef CONSOLE_ENABLE
        uprintf("KL: kc: 0x%04X, col: %2u, row: %2u, pressed: %u, time: %5u, int: %u, count: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed, record->event.time, record->tap.interrupted, record->tap.count);
    #endif

    process_nshot_state(keycode, record, false);

    update_swapper(
        &sw_win_active, KC_LEFT_ALT, KC_TAB, SW_WIN,
        keycode, record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = DLT_WRD,
            .default_key = KC_BSPC,
            .default_mods = MOD_LCTL,
            .mac_os_mods = MOD_LALT
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = NEXT_TAB,
            .default_key = KC_PGDN,
            .default_mods = MOD_LCTL,
            .mac_os_key = KC_RBRC,
            .mac_os_mods = MOD_LSFT | MOD_LCTL
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = PREV_TAB,
            .default_key = KC_PAGE_UP,
            .default_mods = MOD_LCTL,
            .mac_os_key = KC_LBRC,
            .mac_os_mods = MOD_LSFT | MOD_LCTL
        },
        keycode,
        record
    );


    // Custom page up/down
    handle_jump_line_key(JUMP_UP, KC_UP, keycode, record);
    handle_jump_line_key(JUMP_DOWN, KC_DOWN, keycode, record);

    // Win manager keys

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI,
            .mac_os_mods = MOD_LGUI | MOD_LALT,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI,
            .mac_os_mods = MOD_LGUI | MOD_LALT,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_UP,
            .default_key = KC_UP,
            .default_mods = MOD_LGUI,
            .mac_os_mods = MOD_LGUI | MOD_LALT,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_DOWN,
            .default_key = KC_DOWN,
            .default_mods = MOD_LGUI,
            .mac_os_mods = MOD_LGUI | MOD_LALT,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_FULL,
            .default_key = KC_UP,
            .default_mods = MOD_LGUI,
            .mac_os_mods = MOD_LGUI | MOD_LALT,
            .mac_os_key = KC_ENTER,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = DESK_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI | MOD_LCTL,
            .mac_os_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = DESK_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI | MOD_LCTL,
            .mac_os_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = MON_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI | MOD_LSFT,
            .mac_os_mods = MOD_LGUI | MOD_LALT | MOD_LCTL,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = MON_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI | MOD_LSFT,
            .mac_os_mods = MOD_LGUI | MOD_LALT | MOD_LCTL,
        },
        keycode,
        record
    );

  switch (keycode) {
  case QK_MODS ... QK_MODS_MAX:
    // Mouse keys with modifiers work inconsistently across operating systems, this makes sure that modifiers are always
    // applied to the mouse key that was pressed.
    if (IS_MOUSE_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode))) {
    if (record->event.pressed) {
        add_mods(QK_MODS_GET_MODS(keycode));
        send_keyboard_report();
        wait_ms(2);
        register_code(QK_MODS_GET_BASIC_KEYCODE(keycode));
        return false;
      } else {
        wait_ms(2);
        del_mods(QK_MODS_GET_MODS(keycode));
      }
    }
    break;
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_MINUS)SS_DELAY(100)  SS_LSFT(SS_TAP(X_DOT)));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_EQUAL)SS_DELAY(100)  SS_LSFT(SS_TAP(X_DOT)));
    }
    break;

    case OS_TOGGLE:
    if (record->event.pressed) {
        os_mode_toggle();
        return false;
    }

    case DRAG_SCROLL:
      if (record->event.pressed) {
        set_scrolling = true;
        if (!host_keyboard_led_state().caps_lock) {
            tap_code(KC_CAPS);
        }
      } else {
        set_scrolling = false;
        if (host_keyboard_led_state().caps_lock) {
            tap_code(KC_CAPS);
        }
      }
      return false;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_0_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,255,255);
      }
      return false;
    case HSV_74_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(74,255,255);
      }
      return false;
    case HSV_152_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(152,255,255);
      }
      return false;
    case HSV_41_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(41,255,255);
      }
      return false;
    case HSV_135_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(135,255,255);
      }
      return false;
    case HSV_0_0_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,0,255);
      }
      return false;
    case HSV_114_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(114,255,255);
      }
      return false;
    case HSV_234_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(234,255,255);
      }
      return false;
    case HSV_217_249_239:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(217,249,239);
      }
      return false;
    case HSV_191_247_228:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(191,247,228);
      }
      return false;
    case HSV_60_247_227:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(60,247,227);
      }
      return false;
  }
  return true;
}
