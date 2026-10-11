/*
 * dump_defaults.c — Ecrit en JSON les valeurs par defaut reelles du firmware pour une carte
 * (defauts + reglages prudents de la carte + bornes materielles) et les limites materielles.
 * Utilise par build.py pour que la description envoyee a l'outil (commande 250) affiche les
 * vraies valeurs et plages du firmware compile.
 */
#include <stdio.h>
#include <string.h>
#include "resc_conf.h"
#include "vesc_conf.h"
#include "hw_config.h"

bool plat_conf_write(uint32_t s, const uint8_t *d, uint32_t n) { (void)s; (void)d; (void)n; return true; }
const uint8_t *plat_conf_read(uint32_t s, uint32_t *n) { (void)s; *n = 0; return 0; }
uint8_t plat_default_controller_id(void) { return 0; }
void plat_pwm_set_freq(float f, uint8_t m) { (void)f; (void)m; }
void plat_sensor_config(uint8_t m, uint32_t c, uint8_t p, const abs_enc_sincos_t *s) { (void)m; (void)c; (void)p; (void)s; }
void plat_hall_samples(uint8_t e) { (void)e; }
void foc_hw_phase_filter(bool on) { (void)on; }
void foc_hw_kill(void) { }

static void dump(const char *name, bool app, const conf_field_t *t, uint16_t n, bool last)
{
    printf("  \"%s\": {\n", name);
    for (uint16_t k = 0; k < n; k++)
        printf("    \"%s\": %.9g%s\n", t[k].name, (double)resc_conf_field_get(app, &t[k]), k + 1 < n ? "," : "");
    printf("  }%s\n", last ? "" : ",");
}

int main(void)
{
    resc_conf_defaults_mc(&g_mcconf);
    resc_conf_defaults_app(&g_appconf);
    printf("{\n  \"board\": \"%s\",\n", BOARD_NAME);
    /* capacites de la carte pour les choix qui en dependent (tools/resc_conf/board_enum.py) */
    printf("  \"caps\": [\"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\"],\n",
           BOARD_CS_TOPOLOGY == HW_CS_INLINE_3 ? "inline" : "",
           BOARD_CS_TOPOLOGY != HW_CS_LOWSIDE_2 ? "three" : "",
           (BOARD_SPI_SCK_PIN != HW_PIN_NONE && BOARD_SPI_MISO_PIN != HW_PIN_NONE) ? "spi" : "",
           (BOARD_SERVO_PIN != HW_PIN_NONE && BOARD_ENC_TIM != 4) ? "servo" : "",
           (BOARD_ADC_EXT1_CH != HW_ADC_CH_NONE && BOARD_ADC_EXT2_CH != HW_ADC_CH_NONE) ? "adc2" : "",
           (BOARD_IMU_SDA_PIN != HW_PIN_NONE && BOARD_IMU_SCL_PIN != HW_PIN_NONE) ? "imu" : "",
           (BOARD_I2C_SDA_PIN != HW_PIN_NONE && BOARD_I2C_SCL_PIN != HW_PIN_NONE) ? "i2c" : "",
           (BOARD_UART_RX_PIN != HW_PIN_NONE && BOARD_ADC_EXT1_CH != HW_ADC_CH_NONE) ? "rs485" : "");
    printf("  \"limits\": {\"i_motor\": %g, \"i_input\": %g, \"i_abs\": %g, \"vbus_min\": %g, \"vbus_max\": %g, "
           "\"erpm\": %g, \"f_pwm_min\": %g, \"f_pwm_max\": %g, \"duty_max\": %g, \"t_fet\": %g, \"cap\": %g},\n",
           (double)HWLIM_I_MOTOR_A, (double)HWLIM_I_INPUT_A, (double)HWLIM_I_ABS_TRIP_A, (double)HWLIM_VBUS_MIN_V,
           (double)HWLIM_VBUS_MAX_V, (double)HWLIM_ERPM, (double)HWLIM_F_PWM_MIN_HZ, (double)HWLIM_F_PWM_MAX_HZ,
           (double)HWLIM_DUTY_MAX_PERMILLE / 1000.0, (double)HWLIM_T_FET_C, (double)resc_current_cap_frac());
    dump("mc", false, vesc_mcconf_fields, vesc_mcconf_nfields, false);
    dump("app", true, vesc_appconf_fields, vesc_appconf_nfields, true);
    printf("}\n");
    return 0;
}
