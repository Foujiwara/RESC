/* gpio.h — Acces GPIO a partir des codes de broche HW_Pxx (hw_types.h). */
#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32.h"
#include "hw_types.h"

#define GPIO_MODE_IN   0
#define GPIO_MODE_OUT  1
#define GPIO_MODE_AF   2
#define GPIO_MODE_AN   3
#define GPIO_PUPD_NONE 0
#define GPIO_PUPD_UP   1
#define GPIO_PUPD_DOWN 2

static inline GPIO_TypeDef *gpio_port(uint8_t pin)
{
    return (GPIO_TypeDef *)(GPIOA_BASE + 0x400u * HW_PIN_PORT(pin));
}

static inline void gpio_clock_all(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    __DSB();
}

/* mode, sortie open-drain, vitesse (0..3), tirage, fonction alternative */
static inline void gpio_cfg(uint8_t pin, uint32_t mode, bool od, uint32_t speed, uint32_t pupd, uint32_t af)
{
    if (pin == HW_PIN_NONE) return;
    GPIO_TypeDef *g = gpio_port(pin);
    uint32_t n = HW_PIN_NUM(pin);
    if (mode == GPIO_MODE_AF) {
        volatile uint32_t *afr = &g->AFR[n >> 3];
        *afr = (*afr & ~(0xFu << ((n & 7) * 4))) | (af << ((n & 7) * 4));
    }
    g->OTYPER = (g->OTYPER & ~(1u << n)) | ((od ? 1u : 0u) << n);
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (n * 2))) | (speed << (n * 2));
    g->PUPDR = (g->PUPDR & ~(3u << (n * 2))) | (pupd << (n * 2));
    g->MODER = (g->MODER & ~(3u << (n * 2))) | (mode << (n * 2));
}

static inline void gpio_write(uint8_t pin, bool v)
{
    if (pin == HW_PIN_NONE) return;
    gpio_port(pin)->BSRR = v ? (1u << HW_PIN_NUM(pin)) : (1u << (HW_PIN_NUM(pin) + 16));
}

static inline bool gpio_read(uint8_t pin)
{
    if (pin == HW_PIN_NONE) return false;
    return (gpio_port(pin)->IDR >> HW_PIN_NUM(pin)) & 1u;
}

static inline void gpio_out(uint8_t pin, bool initial)
{
    gpio_write(pin, initial);
    gpio_cfg(pin, GPIO_MODE_OUT, false, 2, GPIO_PUPD_NONE, 0);
}

/* Broche analogique d'une voie ADC (IN0-7 PA0-7, IN8-9 PB0-1, IN10-15 PC0-5) */
static inline uint8_t adc_ch_pin(uint8_t ch)
{
    if (ch <= 7) return (uint8_t)(HW_PA0 + ch);
    if (ch <= 9) return (uint8_t)(HW_PB0 + (ch - 8));
    if (ch <= 15) return (uint8_t)(HW_PC0 + (ch - 10));
    return HW_PIN_NONE;
}

#endif
