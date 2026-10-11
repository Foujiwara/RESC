/* motor_hw.h — TIM1 (pont triphase) + ADC1/2/3 (mesures synchrones) + ISR FOC */
#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdbool.h>
#include <stdint.h>

void motor_hw_init(float f_loop, uint8_t ctl_mode);   /* ctl_mode 1 : boucle a 2 x f_pwm */
void motor_hw_set_freq(float f_loop, uint8_t ctl_mode);       /* pont coupe uniquement */
float motor_hw_freq(void);
void motor_hw_set_awd(const float off[3], float i_trip, float i_scale);
uint16_t motor_hw_slow_raw(uint8_t ch);     /* derniere valeur d'une voie lente (0..17) */
uint32_t motor_hw_isr_cycles(void);
uint32_t motor_hw_isr_cycles_max(void);
uint32_t motor_hw_overruns(void);
void motor_hw_reset_stats(void);
#include "abs_enc.h"
/* sensor_mode 1 (codeur) : ABI sur le port Hall, ou codeur absolu selon port (m_sensor_port_mode) */
void motor_hw_sensor_config(uint8_t sensor_mode, uint32_t encoder_counts, uint8_t port, const abs_enc_sincos_t *sc);

void motor_hw_set_hall_samples(uint8_t extra);

#endif
