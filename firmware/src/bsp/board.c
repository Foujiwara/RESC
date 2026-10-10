#include "board.h"
#include "gpio.h"
#include "hw_config.h"

void board_power_hold_early(void)
{
#if BOARD_PWR_HOLD_PIN != HW_PIN_NONE
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    __DSB();
    gpio_out(BOARD_PWR_HOLD_PIN, true);
#endif
}

void board_gate_enable_raw(bool on)
{
#if BOARD_GD_EN_PIN != HW_PIN_NONE
    gpio_write(BOARD_GD_EN_PIN, BOARD_GD_EN_ACTIVE_HIGH ? on : !on);
#else
    (void)on;
#endif
}

void board_gate_enable(bool on) { board_gate_enable_raw(on); }

bool board_gate_fault(void)
{
#if BOARD_GD_FAULT_PIN != HW_PIN_NONE
    bool v = gpio_read(BOARD_GD_FAULT_PIN);
    return BOARD_GD_FAULT_ACTIVE_LOW ? !v : v;
#else
    return false;
#endif
}

void board_led(bool green, bool red)
{
    gpio_write(BOARD_LED_GREEN_PIN, green);
    gpio_write(BOARD_LED_RED_PIN, red);
}

void board_phase_filter(bool on) { gpio_write(BOARD_PHASE_FILTER_PIN, on); }

void board_power_off(void)
{
#if BOARD_PWR_HOLD_PIN != HW_PIN_NONE
    gpio_write(BOARD_PWR_HOLD_PIN, false);
#endif
}

bool board_power_key(void)
{
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
    return !gpio_read(BOARD_PWR_KEY_PIN);
#else
    return false;
#endif
}

void board_init(void)
{
    gpio_clock_all();

    /* Sorties PWM forcees a l'etat bas (MOSFET bloques) avant la configuration de TIM1 */
    const uint8_t pwm[6] = { BOARD_PWM_AH_PIN, BOARD_PWM_BH_PIN, BOARD_PWM_CH_PIN,
                             BOARD_PWM_AL_PIN, BOARD_PWM_BL_PIN, BOARD_PWM_CL_PIN };
    for (int k = 0; k < 6; k++) {
        bool off = (k < 3) ? !BOARD_PWM_HS_ACTIVE_HIGH : !BOARD_PWM_LS_ACTIVE_HIGH;
        gpio_out(pwm[k], off);
    }

#if BOARD_GD_EN_PIN != HW_PIN_NONE
    gpio_out(BOARD_GD_EN_PIN, !BOARD_GD_EN_ACTIVE_HIGH);
#endif
#if BOARD_GD_FAULT_PIN != HW_PIN_NONE
    gpio_cfg(BOARD_GD_FAULT_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
#endif
#if BOARD_GD_DCCAL_PIN != HW_PIN_NONE
    gpio_out(BOARD_GD_DCCAL_PIN, false);
#endif
    gpio_out(BOARD_LED_GREEN_PIN, false);
    gpio_out(BOARD_LED_RED_PIN, false);
    gpio_out(BOARD_PHASE_FILTER_PIN, false);
    gpio_out(BOARD_CS_FILTER_PIN, false);
    gpio_out(BOARD_AUX_PIN, false);
    gpio_out(BOARD_SENSOR_5V_PIN, false);
#if BOARD_PWR_KEY_PIN != HW_PIN_NONE
    gpio_write(BOARD_PWR_KEY_PIN, true);
    gpio_cfg(BOARD_PWR_KEY_PIN, GPIO_MODE_OUT, true, 0, GPIO_PUPD_UP, 0);   /* open-drain + tirage */
#endif
    /* Capteurs : halls en entree avec tirage */
    gpio_cfg(BOARD_HALL1_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
    gpio_cfg(BOARD_HALL2_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
    gpio_cfg(BOARD_HALL3_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
}

const uint8_t *board_uid(void) { return (const uint8_t *)UID_BASE; }

uint8_t board_uid_hash(void)
{
    /* Identifiant CAN par defaut derive de l'UID (meme principe que la reference) */
    const uint8_t *u = board_uid();
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < 12; i++) {
        c ^= u[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0x82F63B78u & (uint32_t)-(int32_t)(c & 1));
    }
    return (uint8_t)((~c) & 0x7F);
}
