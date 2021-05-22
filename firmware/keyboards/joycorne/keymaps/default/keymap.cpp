/*
Copyright 2018 <Pierre Constantineau>

3-Clause BSD License

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "keymap.h"
#include "bluetooth.h"

#if KEYBOARD_SIDE == LEFT
#define THUMBSTICK_READ_INTERVAL_MS 50
#define THUMBSTICK_X_PIN 31
#define THUMBSTICK_Y_PIN 2

#define THUMBSTICK_X_FLIP true
#define THUMBSTICK_Y_FLIP false

#define THUMBSTICK_DEAD_ZONE 90 // Values below this are ignored (deadzone)
#define THUMBSTICK_SPEED 256

// Implicit and derived constants
#define THUMBSTICK_RANGE_START 0
#define THUMBSTICK_RANGE_STOP 1023
#define THUMBSTICK_RANGE_CENTER (THUMBSTICK_RANGE_STOP - THUMBSTICK_RANGE_START + 1) / 2
#define THUMBSTICK_RANGE_MOVEMENT (THUMBSTICK_RANGE_CENTER - THUMBSTICK_DEAD_ZONE)

// Axis-level wrapper to read raw value
uint16_t thumbstick_get_axis_distance(uint16_t pin)
{
    uint16_t analogValue = analogRead(pin);
    // Compute direction
    bool directionIsPositive = (analogValue > THUMBSTICK_RANGE_CENTER);
    // Compute distance from the center
    uint16_t distance = directionIsPositive ? (analogValue - THUMBSTICK_RANGE_CENTER) : (THUMBSTICK_RANGE_CENTER - analogValue);
    // Compute component (range of [0 to 1023])
    return directionIsPositive ? distance : -(uint16_t)distance;
}

int16_t thumbstick_get_mouse_speed(int16_t distance)
{
    int16_t maxSpeed;
    uint16_t abs_distance = abs(distance);
    // if (distance > THUMBSTICK_FINE_ZONE) {
    //     maxSpeed = THUMBSTICK_SPEED;
    // } else
    if (abs_distance > THUMBSTICK_DEAD_ZONE)
    {
        maxSpeed = THUMBSTICK_SPEED; // can be higher or lower
    }
    else
    {
        return 0;
    }
    return (float)maxSpeed * distance / THUMBSTICK_RANGE_CENTER;
}

// Axis-level wrapper to get corresponding mouse move
int16_t thumbstick_get_axis_mouse_move(int16_t pin, bool flip_axis)
{
    int16_t axis_distance;
    if (flip_axis)
    {
        axis_distance = -thumbstick_get_axis_distance(pin);
    }
    else
    {
        axis_distance = thumbstick_get_axis_distance(pin);
    }
    blehid.begin();
    return thumbstick_get_mouse_speed(axis_distance);
}

// Interval between systick event
extern "C"
{

    void SysTick_Handler(void)
    {
        int16_t x_move;
        int16_t y_move;

        x_move = thumbstick_get_axis_mouse_move(THUMBSTICK_X_PIN, THUMBSTICK_X_FLIP);
        y_move = thumbstick_get_axis_mouse_move(THUMBSTICK_Y_PIN, THUMBSTICK_Y_FLIP);

        bt_getBLEHid().mouseMove(x_move, y_move);
    }
} // extern C
/* Qwerty
 * ,-----------------------------------------.
 * | Tab  |   Q  |   W     |   E  |   R  |   T  |
 * |------+------+------+------+------+------|
 * | Ctrl |   A  |   S     |   D  |   F  |   G  |
 * |------+------+------+------+------+------|
 * | Shift|   Z  |   X     |   C  |   V  |   B  |
 * |------+------+------+------+------+------|
 * |      |      | KC_LALT |  GUI | L(1) |Space |
 * `-----------------------------------------'
 */

std::array<std::array<Key, MATRIX_COLS>, MATRIX_ROWS> matrix =
    {KEYMAP(
        KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T,
        KC_LCTL, KC_A, KC_S, KC_D, KC_F, KC_G,
        KC_LSFT, KC_Z, KC_X, KC_C, KC_V, KC_B,
        KC_LALT, KC_LGUI, LAYER_1, KC_SPC)};

void setupKeymap()
{

    /* Layer 1 (Raise)
 * ,-----------------------------------------.
 * | ESC  |   1  |   2  |   3  |   4  |   5  |
 * |------+------+------+------+------+------|
 * | CTRL |      |      |      |      |      |
 * |------+------+------+------+------+------|
 * | Shift|      |      |      |      |      |
 * |------+------+------+------+------+------|
 * |      |      |      |  GUI | L(1) |Space |
 * `-----------------------------------------'
 */
    uint32_t layer1[MATRIX_ROWS][MATRIX_COLS] =
        KEYMAP(
            KC_ESC, PRINT_BATTERY, BLEPROFILE_1, KC_3, KC_4, KC_5,
            PRINT_INFO, OUT_AUTO, OUT_USB, OUT_BT, KC_NO, KC_NO,
            PRINT_BLE, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
            KC_LALT, KC_LGUI, LAYER_1, KC_SPC);

    /* Layer 2 (lower)
 * ,-----------------------------------------.
 * |  Esc |   !  |   @  |   #  |   $  |   %  |
 * |------+------+------+------+------+------|
 * | Ctrl |      |      |      |      |      |
 * |------+------+------+------+------+------|
 * | Shift|      |      |      |      |      |
 * |------+------+------+------+------+------|
 * |      |      |      | GUI  | L(1) |Space |
 * `-----------------------------------------'
 */
    uint32_t layer2[MATRIX_ROWS][MATRIX_COLS] =
        KEYMAP(
            KC_ESC, KC_EXLM, KC_AT, KC_HASH, KC_DLR, KC_PERC,
            KC_LCTL, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
            KC_LSFT, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
            KC_LALT, KC_LGUI, LAYER_1, KC_SPC);
    /*
     * add the other layers on the regular presses.
     */
    for (int row = 0; row < MATRIX_ROWS; ++row)
    {
        for (int col = 0; col < MATRIX_COLS; ++col)
        {
            matrix[row][col].addActivation(_L1, Method::PRESS, layer1[row][col]);
            matrix[row][col].addActivation(_L2, Method::PRESS, layer2[row][col]);
        }
    }

    pinMode(THUMBSTICK_X_PIN, INPUT);
    pinMode(THUMBSTICK_Y_PIN, INPUT);
    /* Input parameter is number of ticks between interrupts handler i.e SysTick_Handler
   * 1000 ms --> F_CPU            ticks
   * T    ms --> (F_CPU/1000)*T   ticks
   *
   * Note: Since systick is 24-bit timer, the max tick value is 0xFFFFFF, F_CPU = 64 Mhz
   * --> our Tmax = 0xFFFFFF/64000 ~ 262 ms
   */

    SysTick_Config( (F_CPU/1000)*THUMBSTICK_READ_INTERVAL_MS);
}
#endif // left

#if KEYBOARD_SIDE == RIGHT

/* Qwerty
 * ,-----------------------------------------.
 * |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+-------------|
 * |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------|------|
 * |   N  |   M  |   ,  |   .  |   /  |Enter |
 * |------+------+------+------+------+------|
 * | Space| L(2) | Left | Down |  Up  |Right |
 * `-----------------------------------------'
 */

std::array<std::array<Key, MATRIX_COLS>, MATRIX_ROWS> matrix =
    {KEYMAP(
        KC_Y, KC_U, KC_I, KC_O, KC_P, KC_BSPACE,
        KC_H, KC_J, KC_K, KC_L, KC_SCOLON, KC_QUOTE,
        KC_N, KC_M, KC_COMMA, KC_DOT, KC_SLSH, KC_RSFT,
        KC_ENT, LAYER_2, KC_RALT, KC_RGUI)};

void setupKeymap()
{

    /* Layer 1 (Raise)
 * ,-----------------------------------------.
 * |   Y  |   U  |   I  |   O  |   P  | Del  |
 * |------+------+------+------+-------------|
 * |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------|------|
 * |   N  |   M  |   ,  |   .  |   /  |Enter |
 * |------+------+------+------+------+------|
 * | Space| L(2) | Left |      |      |      |
 * `-----------------------------------------'
 */
    uint32_t layer1[MATRIX_ROWS][MATRIX_COLS] =
        KEYMAP(
            KC_6, KC_7, KC_8, KC_9, KC_0, KC_BSPC,
            KC_LEFT, KC_DOWN, KC_UP, KC_RIGHT, KC_NO, KC_NO,
            KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO,
            KC_SPC, LAYER_2, KC_RALT, KC_RGUI);

    /* Layer 2 (lower)
 * ,-----------------------------------------.
 * |   ^  |   &  |   *  |   (  |   )  | Bspc |
 * |------+------+------+------+-------------|
 * |   -  |   =  |   {  |   }  |   |  |  `   |
 * |------+------+------+------+------|------|
 * |   _  |   +  |   [  |   ]  |   \  |  ~   |
 * |------+------+------+------+------+------|
 * | Space| L(2) |  Alt |      |      |      |
 * `-----------------------------------------'
 */
    uint32_t layer2[MATRIX_ROWS][MATRIX_COLS] =
        KEYMAP(
            KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
            KC_MINS, KC_EQL, KC_LCBR, KC_RCBR, KC_PIPE, KC_GRV,
            KC_UNDS, KC_PLUS, KC_LBRC, KC_RBRC, KC_BSLS, KC_TILD,
            KC_ENT, LAYER_2, KC_RALT, KC_RGUI);
    /*
     * add the other layers
     */
    for (int row = 0; row < MATRIX_ROWS; ++row)
    {
        for (int col = 0; col < MATRIX_COLS; ++col)
        {
            matrix[row][col].addActivation(_L1, Method::PRESS, layer1[row][col]);
            matrix[row][col].addActivation(_L2, Method::PRESS, layer2[row][col]);
        }
    }
}

#endif
