#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#include "print.h"
#include "swapper.h"
#include "oneshot.h"
#include "magickey.h"
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
  ST_MACRO_0,
  ST_MACRO_1,

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
};

oneshot_state os_shft_state = os_up_unqueued;
oneshot_state os_ctrl_state = os_up_unqueued;
oneshot_state os_alt_state = os_up_unqueued;
oneshot_state os_gui_state = os_up_unqueued;

enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           TD(DANCE_0),    KC_7,           KC_8,           KC_9,           KC_0,           TD(DANCE_1),
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           TD(DANCE_2),
    KC_BSPC,        KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_QUOTE,       KC_SCLN,
    MO(8),          KC_Z,           KC_X,           KC_C,           KC_V,           KC_B,                                           KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       KC_LEFT_SHIFT,
                                                    LA_NAV,         LT(4, KC_ENTER),                                      OS_SHFT,  LT(2, KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, LCTL(KC_W),     SW_WIN,         PREV_TAB,       NEXT_TAB,       KC_TRANSPARENT,                                 KC_PAGE_UP,     KC_HOME,        KC_UP,          KC_END,         KC_TRANSPARENT, KC_TRANSPARENT,
    DLT_WRD,        OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,  KC_TRANSPARENT,                                 KC_PGDN,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT,
    LCTL(LSFT(KC_Z)),KC_PC_UNDO,     KC_PC_CUT,      KC_PC_COPY,     KC_PC_PASTE,    KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
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
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_DYNAMIC_TAPPING_TERM_DOWN,QK_DYNAMIC_TAPPING_TERM_UP,QK_DYNAMIC_TAPPING_TERM_PRINT,                                KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_PLAY_PAUSE,                                KC_F12,         KC_F7,          KC_F8,          KC_F9,          KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,          KC_TRANSPARENT,                                 KC_F11,         KC_F4,          KC_F5,          KC_F6,          KC_TRANSPARENT, KC_TRANSPARENT,
    KC_AUDIO_MUTE,  KC_TRANSPARENT, KC_TRANSPARENT, LCTL(LSFT(KC_C)),LCTL(LSFT(KC_V)),KC_TRANSPARENT,                                 KC_F10,         KC_F1,          KC_F2,          KC_F3,          KC_TRANSPARENT, KC_TRANSPARENT,
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
    KC_TRANSPARENT, KC_MS_ACCEL0,   KC_MS_ACCEL1,   KC_MS_ACCEL2,   KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_MS_WH_LEFT,  KC_MS_WH_RIGHT, KC_MS_WH_DOWN,  KC_MS_WH_UP,    KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_UP,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, OS_GUI,         OS_ALT,         OS_SHFT,        OS_CTRL,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_MS_BTN3,     KC_TRANSPARENT,
    KC_TRANSPARENT, KC_PC_UNDO,     KC_PC_CUT,      KC_PC_COPY,     KC_PC_PASTE,    KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_MS_BTN2,     KC_MS_BTN1
  ),
  [6] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        HSV_0_255_255,  HSV_74_255_255, HSV_152_255_255,HSV_41_255_255, HSV_135_255_255,QK_BOOT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 HSV_0_0_255,    HSV_114_255_255,HSV_234_255_255,HSV_217_249_239,HSV_191_247_228,TD(DANCE_3),
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [7] = LAYOUT_voyager(
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          TD(DANCE_4),    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                                    KC_NO,          KC_NO,                                          KC_NO,          KC_NO
  ),
  [8] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, MON_LEFT,       WIN_UP,         MON_RIGHT,      KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, WIN_FULL,       DESK_LEFT,      DESK_RIGHT,     KC_TRANSPARENT,                                 KC_TRANSPARENT, WIN_LEFT,       WIN_DOWN,       WIN_RIGHT,      KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};


const uint16_t PROGMEM combo0[] = { KC_R, KC_F, COMBO_END};
const uint16_t PROGMEM combo1[] = { OS_SHFT, LT(4, KC_ENTER), COMBO_END};
const uint16_t PROGMEM combo2[] = { KC_D, KC_E, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_LEFT_CTRL),
    COMBO(combo1, CW_TOGG),
    COMBO(combo2, KC_LEFT_SHIFT),
};


extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [7] = { {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255} },

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
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 7:
      set_layer_color(7);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }

  if (os_shft_state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][4], RGB_RED);
  }

  if (os_ctrl_state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][5], RGB_RED);
  }

  if (os_alt_state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][3], RGB_RED);
  }

  if (os_gui_state != os_up_unqueued) {
    rgb_matrix_set_color(g_led_config.matrix_co[2][2], RGB_RED);
  }
  return true;
}

bool is_oneshot_cancel_key(uint16_t keycode) {
    switch (keycode) {
    case LA_NAV:
        return true;
    default:
        return false;
    }
}

bool is_oneshot_ignored_key(uint16_t keycode) {
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

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    #ifdef CONSOLE_ENABLE
        uprintf("KL: kc: 0x%04X, col: %2u, row: %2u, pressed: %u, time: %5u, int: %u, count: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed, record->event.time, record->tap.interrupted, record->tap.count);
    #endif

    update_swapper(
        &sw_win_active, KC_LEFT_ALT, KC_TAB, SW_WIN,
        keycode, record
    );

    update_oneshot(
        &os_shft_state, KC_LSFT, OS_SHFT,
        keycode, record
    );
    update_oneshot(
        &os_ctrl_state, KC_LCTL, OS_CTRL,
        keycode, record
    );
    update_oneshot(
        &os_alt_state, KC_LALT, OS_ALT,
        keycode, record
    );
    update_oneshot(
        &os_gui_state, KC_LGUI, OS_GUI,
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

    // Win manager keys

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_UP,
            .default_key = KC_UP,
            .default_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_DOWN,
            .default_key = KC_DOWN,
            .default_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = WIN_FULL,
            .default_key = KC_UP,
            .default_mods = MOD_LGUI,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = DESK_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI | MOD_LCTL,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = DESK_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI | MOD_LCTL,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = MON_LEFT,
            .default_key = KC_LEFT,
            .default_mods = MOD_LGUI | MOD_LSFT,
        },
        keycode,
        record
    );

    handle_magic_key(
        (magic_key_config_t){
            .trigger = MON_RIGHT,
            .default_key = KC_RIGHT,
            .default_mods = MOD_LGUI | MOD_LSFT,
        },
        keycode,
        record
    );

  switch (keycode) {
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
  }
  return true;
}

layer_state_t layer_state_set_user(layer_state_t state) {
  // Nav + Sym = Func
  state =  update_tri_layer_state(state, 1, 2, 3);
  // Nav + Num = Mouse
  state =  update_tri_layer_state(state, 1, 4, 5);
  return state;
}

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

static tap dance_state[5];

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
        case DOUBLE_TAP: layer_move(7); break;
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
        case DOUBLE_TAP: layer_move(6); break;
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
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case DOUBLE_TAP: layer_move(0); break;
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
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

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_4_finished, dance_4_reset),
};
