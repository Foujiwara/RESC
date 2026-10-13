/* stm32.h — En-tetes CMSIS du STM32F405 (ST, Apache-2.0) */
#ifndef STM32_H
#define STM32_H
#ifndef STM32F405xx
#define STM32F405xx
#endif
#include "stm32f4xx.h"
#include "platform.h"

#define SYSCLK_HZ   168000000u
#define APB1_HZ     42000000u
#define APB2_HZ     84000000u
#define TIM1_CLK_HZ 168000000u

/* Priorites NVIC (4 bits de preemption, 0 = plus prioritaire) */
#define PRIO_BRK      0
#define PRIO_ADC_FOC  1
#define PRIO_FAULT_IO 0
#define PRIO_SYSTICK  PLAT_CTRL_PRIO   /* couche de controle (1 kHz) : voir plat_ctrl_lock() */
#define PRIO_CAN      6
#define PRIO_UART     6
#define PRIO_USB      7

#endif
