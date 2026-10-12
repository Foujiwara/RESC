/*
 * inputs.c — Entrees des applications.
 *
 * Servo : TIM4 en mode « entree PWM » sur la broche servo (CH1) : le compteur (1 MHz)
 *         est remis a zero sur chaque front montant (CCR1 = periode) et CCR2 capture
 *         le front descendant (largeur d'impulsion). Lu par scrutation a 1 kHz : la
 *         derniere impulsion suffit, aucune interruption.
 *         Indisponible si TIM4 sert au codeur (BOARD_ENC_TIM == 4).
 * ADC    : voies externes 1 et 2, deja converties en continu par motor_hw (voies lentes).
 * Port UART : boutons (actifs a 0, tirage interne) ou capteur de pedalage en quadrature,
 *         quand les applications n'utilisent pas la liaison serie sur ces broches.
 */
#include "inputs.h"
#include "gpio.h"
#include "hw_config.h"
#include "motor_hw.h"

#define SERVO_TIM_OK (BOARD_SERVO_PIN != HW_PIN_NONE && BOARD_ENC_TIM != 4)

static bool s_servo;
static bool s_servo_out;
static in_uart_use_t s_uart_use;
static float s_servo_ms;
static uint32_t s_servo_t;
static uint32_t s_ms_count;
static uint8_t s_pas_state;

void inputs_init(bool servo, in_uart_use_t uart_use, bool servo_pin_owned)
{
    s_servo_out = false;
    s_servo = servo && SERVO_TIM_OK && !servo_pin_owned;
    s_uart_use = uart_use;
#if SERVO_TIM_OK
    if (s_servo) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        __DSB();
        gpio_cfg(BOARD_SERVO_PIN, GPIO_MODE_AF, false, 1, GPIO_PUPD_DOWN, 2);
        TIM4->CR1 = 0;
        TIM4->PSC = (uint16_t)(2u * APB1_HZ / 1000000u - 1u);   /* timers APB1 : 2 x 42 MHz -> 1 MHz */
        TIM4->ARR = 0xFFFF;
        TIM4->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (2u << TIM_CCMR1_CC2S_Pos) |
                      (3u << TIM_CCMR1_IC1F_Pos) | (3u << TIM_CCMR1_IC2F_Pos);
        TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;   /* CH2 : front descendant */
        TIM4->SMCR = (5u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos); /* TI1FP1, remise a zero */
        TIM4->EGR = TIM_EGR_UG;
        TIM4->SR = 0;
        TIM4->CR1 = TIM_CR1_CEN;
    } else
#endif
    if (!servo_pin_owned) {
        gpio_cfg(BOARD_SERVO_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
    }
    if (uart_use != IN_UART_COMM) {
        gpio_cfg(BOARD_UART_TX_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
        gpio_cfg(BOARD_UART_RX_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
    }
}

void inputs_servo_out_init(void)
{
#if SERVO_TIM_OK
    s_servo = false;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    __DSB();
    TIM4->CR1 = 0;
    TIM4->SMCR = 0;
    TIM4->CCER = 0;
    TIM4->PSC = (uint16_t)(2u * APB1_HZ / 1000000u - 1u);   /* 1 MHz */
    TIM4->ARR = 20000u - 1u;                                 /* 50 Hz */
    TIM4->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCR1 = 1500u;
    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    gpio_cfg(BOARD_SERVO_PIN, GPIO_MODE_AF, false, 1, GPIO_PUPD_NONE, 2);
    s_servo_out = true;
#endif
}

void inputs_servo_out_set(float pos)
{
#if SERVO_TIM_OK
    if (!s_servo_out) return;
    if (pos < 0.0f) pos = 0.0f;
    if (pos > 1.0f) pos = 1.0f;
    TIM4->CCR1 = (uint32_t)(1000.0f + 1000.0f * pos);
#else
    (void)pos;
#endif
}

bool inputs_servo_ms(float *ms, uint32_t now_ms)
{
#if SERVO_TIM_OK
    if (s_servo && (TIM4->SR & TIM_SR_CC1IF)) {
        uint32_t period = TIM4->CCR1;           /* lecture : efface CC1IF */
        uint32_t width = TIM4->CCR2;
        if (period > 2500u && width >= 500u && width <= 2600u) {
            s_servo_ms = (float)width * 1e-3f;
            s_servo_t = now_ms;
            s_ms_count++;
        }
    }
#endif
    *ms = s_servo_ms;
    return s_ms_count > 0 && now_ms - s_servo_t < 100u;
}

bool inputs_servo_level(void) { return gpio_read(BOARD_SERVO_PIN); }

float inputs_adc_v(int n)
{
    uint8_t ch = n == 2 ? BOARD_ADC_EXT2_CH : BOARD_ADC_EXT1_CH;
    if (ch == HW_ADC_CH_NONE) return 0.0f;
    /* moyenne sur 16 echantillons : un parasite isole (moteur lance, pont coupe) ne depasse plus
     * la zone morte de l'accelerateur */
    return motor_hw_slow_filt(ch) * (BOARD_VREF_V / 4095.0f);
}

bool inputs_btn_cruise(void) { return s_uart_use == IN_UART_BUTTONS && !gpio_read(BOARD_UART_TX_PIN); }
bool inputs_btn_reverse(void) { return s_uart_use == IN_UART_BUTTONS && !gpio_read(BOARD_UART_RX_PIN); }

int32_t inputs_pas_poll(void)
{
    if (s_uart_use != IN_UART_PAS) return 0;
    /* decodage en quadrature (sequence de Gray 00 -> 01 -> 11 -> 10) */
    static const int8_t tab[16] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };
    uint8_t s = (uint8_t)((gpio_read(BOARD_UART_TX_PIN) ? 1u : 0u) | (gpio_read(BOARD_UART_RX_PIN) ? 2u : 0u));
    int32_t d = tab[(s_pas_state << 2) | s];
    s_pas_state = s;
    return d;
}
