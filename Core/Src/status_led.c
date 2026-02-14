///* Core/Src/status_led.c */
//#include "status_led.h"
//#include "stm32f4xx_hal.h"
//
//static board_type_t board;
//
///* ---------- Pinout (your exact spec) ---------- */
//#if defined(BOARD_BABY)
//    #define STATUS1_PORT GPIOA
//    #define STATUS1_PIN  GPIO_PIN_2
//    #define STATUS2_PORT GPIOA
//    #define STATUS2_PIN  GPIO_PIN_3
//
//    #define LED_ON(pin)  HAL_GPIO_WritePin(pin ## _PORT, pin ## _PIN, GPIO_PIN_RESET)
//    #define LED_OFF(pin) HAL_GPIO_WritePin(pin ## _PORT, pin ## _PIN, GPIO_PIN_SET)
//
//#elif defined(BOARD_MAMA) || defined(BOARD_PAPA)
//    #define STATUS_PORT GPIOA
//    #define STATUS_PIN  GPIO_PIN_2
//
//    #define LED_ON(pin)  HAL_GPIO_WritePin(pin ## _PORT, pin ## _PIN, GPIO_PIN_SET)
//    #define LED_OFF(pin) HAL_GPIO_WritePin(pin ## _PORT, pin ## _PIN, GPIO_PIN_RESET)
//
//#else
//    #error "Board type not defined"
//#endif
//
//void status_led_init(board_type_t type)
//{
//    board = type;
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull  = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//
//#if defined(BOARD_BABY)
//    GPIO_InitStruct.Pin = STATUS1_PIN;
//    HAL_GPIO_Init(STATUS1_PORT, &GPIO_InitStruct);
//    GPIO_InitStruct.Pin = STATUS2_PIN;
//    HAL_GPIO_Init(STATUS2_PORT, &GPIO_InitStruct);
//
//    LED_OFF(STATUS1);
//    LED_OFF(STATUS2);
//
//#elif defined(BOARD_MAMA) || defined(BOARD_PAPA)
//    GPIO_InitStruct.Pin = STATUS_PIN;
//    HAL_GPIO_Init(STATUS_PORT, &GPIO_InitStruct);
//    LED_OFF(STATUS);
//#endif
//}
//
///* -------------------------------------------------------------------- */
//void status_led_set(status_t s)
//{
//    switch (s) {
//        case STATUS_OFF:
//#if defined(BOARD_BABY)
//            LED_OFF(STATUS1);
//            LED_OFF(STATUS2);
//#else
//            LED_OFF(STATUS);
//#endif
//            break;
//
//        case STATUS_BOOT:
//        case STATUS_ERROR:
//#if defined(BOARD_BABY)
//            LED_ON(STATUS1);
//            LED_OFF(STATUS2);
//#else
//            LED_ON(STATUS);
//#endif
//            break;
//
//        case STATUS_APP_LOADED:
//#if defined(BOARD_BABY)
//            LED_OFF(STATUS1);
//            LED_ON(STATUS2);
//#else
//            LED_ON(STATUS);
//#endif
//            break;
//
//        case STATUS_APP_RUNNING:
//#if defined(BOARD_BABY)
//            LED_ON(STATUS1);
//            LED_OFF(STATUS2);
//#else
//            LED_ON(STATUS);
//#endif
//            break;
//    }
//}
//
///* -------------------------------------------------------------------- */
//void status_led_blink(status_t s, uint16_t ms)
//{
//    static uint32_t timer = 0;
//    static status_t current = STATUS_OFF;
//    static uint8_t phase = 0;
//
//    if (s != current) {
//        current = s;
//        timer   = HAL_GetTick();
//        phase   = 0;
//        status_led_set(STATUS_OFF);
//    }
//
//    uint32_t now = HAL_GetTick();
//    if (now - timer >= ms) {
//        timer = now;
//        ++phase;
//
//        switch (current) {
//            case STATUS_BOOT:
//                status_led_set((phase % 2) ? STATUS_APP_RUNNING : STATUS_OFF);
//                break;
//            case STATUS_ERROR:
//                status_led_set((phase % 6 < 3) ? STATUS_APP_RUNNING : STATUS_OFF);
//                break;
//            default:
//                break;
//        }
//    }
//}
//
//void status_led_data_activity(void){
//	HAL_GPIO_TogglePin(STATUS2_PORT, STATUS2_PIN);
//
//}


#include "status_led.h"
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Private Types
// ---------------------------------------------------------------------------
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    bool          active_high;
} led_gpio_t;

typedef struct {
    uint8_t       num_leds;        // 1 or 2
    led_gpio_t    led1;
    led_gpio_t    led2;
} board_config_t;

// ---------------------------------------------------------------------------
// Board Configurations (based on your exact hardware)
// ---------------------------------------------------------------------------
static const board_config_t board_configs[] = {
    [BOARD_TYPE_BABY] = {
        .num_leds = 2,
        .led1 = { .port = GPIOA, .pin = GPIO_PIN_2, .active_high = false },
        .led2 = { .port = GPIOA, .pin = GPIO_PIN_3, .active_high = false }
    },
    [BOARD_TYPE_RESERVED_01] = { .num_leds = 0 }, // invalid/fallback

    [BOARD_TYPE_MAMA] = {
        .num_leds = 1,
        .led1 = { .port = GPIOA, .pin = GPIO_PIN_2, .active_high = true }
        // no led2
    },
    [BOARD_TYPE_PAPA] = { .num_leds = 0 } /* Papa cancelled — reserved */
};

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------
static struct {
    board_type_t           board_type;
    const board_config_t*  cfg;

    status_led_mode_t      mode;
    uint32_t               mode_timer;

    // Data activity overlay state machine
    bool                   data_activity_active;
    uint32_t               da_start_ms;
    uint8_t                da_step;        // 0=off, 1=on, 2=off (for single LED invert case)
    bool                   was_led_on_before_da;  // for single-LED resume
} g = {0};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline uint32_t millis(void) { return HAL_GetTick(); }

static inline void led_write(const led_gpio_t* led, bool on)
{
    if (!led->port) return;
    HAL_GPIO_WritePin(led->port, led->pin,
        (led->active_high) ? (on ? GPIO_PIN_SET : GPIO_PIN_RESET)
                           : (on ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

static bool get_current_main_pattern_state(void)
{
    uint32_t t = millis();

    switch (g.mode) {
        case LED_MODE_BOOTING:
            return false;

        case LED_MODE_NOAPP:
        	uint32_t cycle = t % 4000;
        	return (cycle < 250);  // ON for first 250ms of 2s

        case LED_MODE_STOPPED: {
            uint32_t cycle = t % 1500;
            return (cycle < 250);  // ON for first 250ms of 2s
        }

        case LED_MODE_RUNNING: {
            uint32_t cycle = t % 500;
            return (cycle < 250);  // 50% duty, 2 Hz
        }

        default:
            return false;
    }
}

static void update_leds(void)
{
    bool main_pattern = get_current_main_pattern_state();

    // Handle dataActivity overlay
    if (g.data_activity_active) {
        uint32_t elapsed = millis() - g.da_start_ms;

        if (g.cfg->num_leds == 2) {
            // LED2: dedicated data activity → simple 50ms ON pulse
            bool led2_on = (elapsed < 50);
            led_write(&g.cfg->led2, led2_on);

            // LED1: continues normal pattern
            led_write(&g.cfg->led1, main_pattern);

            if (elapsed >= 150) {
                g.data_activity_active = false;
            }
        }
        else if (g.cfg->num_leds == 1) {
            // Single LED board (MAMA): smart overlay
            bool output;

            if (elapsed < 10) {
                // Phase 0: Force OFF
                output = false;
                g.da_step = 0;
            }
            else if (elapsed < 40) {
                // Phase 1: Force ON
                output = true;
                g.da_step = 1;
            }
            else if (elapsed < 80) {
                // Phase 2: Force OFF again
                output = false;
                g.da_step = 2;
            }
            else {
                // Done → resume normal pattern
                g.data_activity_active = false;
                output = main_pattern;
            }

            led_write(&g.cfg->led1, output);
        }
    }
    else {
        // Normal operation — no data activity
        led_write(&g.cfg->led1, main_pattern);
        if (g.cfg->num_leds == 2) {
            led_write(&g.cfg->led2, false);  // LED2 off when idle
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void status_led_init(board_type_t boardType)
{
    if (boardType >= sizeof(board_configs)/sizeof(board_configs[0]) ||
        board_configs[boardType].num_leds == 0) {
        boardType = BOARD_TYPE_MAMA;  // safe fallback
    }

    g.board_type = boardType;
    g.cfg        = &board_configs[boardType];
    g.mode       = LED_MODE_BOOTING;
    g.data_activity_active = false;

    // Enable GPIOA clock (all LEDs on PA)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = g.cfg->led1.pin;
    HAL_GPIO_Init(g.cfg->led1.port, &GPIO_InitStruct);

    if (g.cfg->num_leds == 2) {
        GPIO_InitStruct.Pin = g.cfg->led2.pin;
        HAL_GPIO_Init(g.cfg->led2.port, &GPIO_InitStruct);
        led_write(&g.cfg->led2, false);
    }

    update_leds();
}

void status_led_set_mode(status_led_mode_t mode)
{
    if (mode != g.mode) {
        g.mode = mode;
        g.mode_timer = millis();  // optional phase reset
    }
}

status_led_mode_t status_led_get_mode(void)
{
    return g.mode;
}

void status_led_data_activity(void){
//    if (g.data_activity_active) {
//        // Already in progress — extend/retrigger slightly
//        g.da_start_ms = millis();
//        return;
//    }
//
//    g.data_activity_active = true;
//    g.da_start_ms = millis();
//    g.was_led_on_before_da = get_current_main_pattern_state();
//

	if (g.data_activity_active) {
		return;               // ← This is the complete fix!
	}

	g.data_activity_active = true;
	g.da_start_ms = millis();

	if (g.cfg->num_leds == 1) {
		g.was_led_on_before_da = get_current_main_pattern_state();
	}

}


void status_led_task(void)
{
    update_leds();
}
