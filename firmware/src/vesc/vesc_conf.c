/* GENERE par tools/resc_conf/gen_resc_conf.py depuis les XML de firmware/conf - ne pas editer. */
#include <stddef.h>
#include "vesc_conf.h"
#include "vesc_buffer.h"
#include "vesc_compat_defaults.h"

/* Profil RESC : listes reduites aux choix pris en charge. Position affichee <-> valeur interne
 * (valeur du format 7.00) ; pour les champs de bits, bit affiche <-> bit interne. */
static uint8_t map_out(uint8_t v, const uint8_t *k, uint8_t n) { for (uint8_t j = 0; j < n; j++) if (k[j] == v) return j; return 0; }
static uint8_t map_in(uint8_t j, const uint8_t *k, uint8_t n) { return j < n ? k[j] : k[0]; }
static uint8_t map_bits_out(uint8_t v, const uint8_t *k, uint8_t n) { uint8_t o = 0; for (uint8_t j = 0; j < n; j++) if (v & (1u << k[j])) o |= (uint8_t)(1u << j); return o; }
static uint8_t map_bits_in(uint8_t o, const uint8_t *k, uint8_t n) { uint8_t v = 0; for (uint8_t j = 0; j < n; j++) if (o & (1u << j)) v |= (uint8_t)(1u << k[j]); return v; }
static const uint8_t KEEP_APP_ADC_CONF_BUTTONS[] = { 0, 1, 2 };
static const uint8_t KEEP_CAN_STATUS_MSGS_R1[] = { 0, 1, 2, 3, 4, 5 };
static const uint8_t KEEP_CAN_STATUS_MSGS_R2[] = { 0, 1, 2, 3, 4, 5 };
static const uint8_t KEEP_FOC_HFI_AMB_MODE[] = { 1, 2 };
static const uint8_t KEEP_FOC_OFFSETS_CAL_MODE[] = { 0 };
static const uint8_t KEEP_L_ADDITIONAL_FAULTS[] = { 0, 1, 3 };
static const uint8_t KEEP_M_OUT_AUX_MODE[] = { 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12 };
static const uint8_t KEEP_SP_PID_LOOP_RATE[] = { 0, 1, 2, 3, 4, 5 };

int32_t vesc700_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c)
{
    int32_t i = 0;
    vb_put_u32(buf, VESC700_MCCONF_SIGNATURE, &i);
    buf[i++] = (uint8_t)c->pwm_mode;
    buf[i++] = (uint8_t)c->comm_mode;
    buf[i++] = (uint8_t)c->motor_type;
    buf[i++] = (uint8_t)c->sensor_mode;
    vb_put_f32_auto(buf, c->l_current_max, &i);
    vb_put_f32_auto(buf, c->l_current_min, &i);
    vb_put_f32_auto(buf, c->l_in_current_max, &i);
    vb_put_f32_auto(buf, c->l_in_current_min, &i);
    vb_put_f16(buf, c->l_in_current_map_start, 10000, &i);
    vb_put_f16(buf, c->l_in_current_map_filter, 10000, &i);
    vb_put_f32_auto(buf, c->l_abs_current_max, &i);
    vb_put_f32_auto(buf, c->l_min_erpm, &i);
    vb_put_f32_auto(buf, c->l_max_erpm, &i);
    vb_put_f16(buf, c->l_erpm_start, 10000, &i);
    vb_put_f32_auto(buf, c->l_max_erpm_fbrake, &i);
    vb_put_f32_auto(buf, c->l_max_erpm_fbrake_cc, &i);
    vb_put_f16(buf, c->l_min_vin, 10, &i);
    vb_put_f16(buf, c->l_max_vin, 10, &i);
    vb_put_f16(buf, c->l_battery_cut_start, 10, &i);
    vb_put_f16(buf, c->l_battery_cut_end, 10, &i);
    vb_put_f16(buf, c->l_battery_regen_cut_start, 10, &i);
    vb_put_f16(buf, c->l_battery_regen_cut_end, 10, &i);
    buf[i++] = (uint8_t)c->l_slow_abs_current;
    buf[i++] = (uint8_t)c->l_temp_fet_start;
    buf[i++] = (uint8_t)c->l_temp_fet_end;
    buf[i++] = (uint8_t)c->l_temp_motor_start;
    buf[i++] = (uint8_t)c->l_temp_motor_end;
    vb_put_f16(buf, c->l_temp_accel_dec, 10000, &i);
    vb_put_f16(buf, c->l_min_duty, 10000, &i);
    vb_put_f16(buf, c->l_max_duty, 10000, &i);
    vb_put_f32_auto(buf, c->l_watt_max, &i);
    vb_put_f32_auto(buf, c->l_watt_min, &i);
    vb_put_f16(buf, c->l_current_max_scale, 10000, &i);
    vb_put_f16(buf, c->l_current_min_scale, 10000, &i);
    vb_put_f16(buf, c->l_duty_start, 10000, &i);
    buf[i++] = (uint8_t)c->l_additional_faults;
    vb_put_f32_auto(buf, c->sl_min_erpm, &i);
    vb_put_f32_auto(buf, c->sl_min_erpm_cycle_int_limit, &i);
    vb_put_f32_auto(buf, c->sl_max_fullbreak_current_dir_change, &i);
    vb_put_f16(buf, c->sl_cycle_int_limit, 10, &i);
    vb_put_f16(buf, c->sl_phase_advance_at_br, 10000, &i);
    vb_put_f32_auto(buf, c->sl_cycle_int_rpm_br, &i);
    vb_put_f32_auto(buf, c->sl_bemf_coupling_k, &i);
    buf[i++] = (uint8_t)c->hall_table_0;
    buf[i++] = (uint8_t)c->hall_table_1;
    buf[i++] = (uint8_t)c->hall_table_2;
    buf[i++] = (uint8_t)c->hall_table_3;
    buf[i++] = (uint8_t)c->hall_table_4;
    buf[i++] = (uint8_t)c->hall_table_5;
    buf[i++] = (uint8_t)c->hall_table_6;
    buf[i++] = (uint8_t)c->hall_table_7;
    vb_put_f32_auto(buf, c->hall_sl_erpm, &i);
    vb_put_f32_auto(buf, c->foc_current_kp, &i);
    vb_put_f32_auto(buf, c->foc_current_ki, &i);
    vb_put_f32_auto(buf, c->foc_f_zv, &i);
    vb_put_f32_auto(buf, c->foc_dt_us, &i);
    buf[i++] = (uint8_t)c->foc_encoder_inverted;
    vb_put_f32_auto(buf, c->foc_encoder_offset, &i);
    vb_put_f32_auto(buf, c->foc_encoder_ratio, &i);
    buf[i++] = (uint8_t)c->foc_sensor_mode;
    vb_put_f32_auto(buf, c->foc_pll_kp, &i);
    vb_put_f32_auto(buf, c->foc_pll_ki, &i);
    vb_put_f32_auto(buf, c->foc_motor_l, &i);
    vb_put_f32_auto(buf, c->foc_motor_ld_lq_diff, &i);
    vb_put_f32_auto(buf, c->foc_motor_r, &i);
    vb_put_f32_auto(buf, c->foc_motor_flux_linkage, &i);
    vb_put_f32_auto(buf, c->foc_observer_gain, &i);
    vb_put_f32_auto(buf, c->foc_observer_gain_slow, &i);
    vb_put_f16(buf, c->foc_observer_offset, 1000, &i);
    vb_put_f32_auto(buf, c->foc_duty_dowmramp_kp, &i);
    vb_put_f32_auto(buf, c->foc_duty_dowmramp_ki, &i);
    vb_put_f16(buf, c->foc_start_curr_dec, 10000, &i);
    vb_put_f32_auto(buf, c->foc_start_curr_dec_rpm, &i);
    vb_put_f32_auto(buf, c->foc_openloop_rpm, &i);
    vb_put_f16(buf, c->foc_openloop_rpm_low, 1000, &i);
    vb_put_f16(buf, c->foc_sl_openloop_hyst, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time_lock, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time_ramp, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_boost_q, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_max_q, 100, &i);
    buf[i++] = (uint8_t)c->foc_hall_table_0;
    buf[i++] = (uint8_t)c->foc_hall_table_1;
    buf[i++] = (uint8_t)c->foc_hall_table_2;
    buf[i++] = (uint8_t)c->foc_hall_table_3;
    buf[i++] = (uint8_t)c->foc_hall_table_4;
    buf[i++] = (uint8_t)c->foc_hall_table_5;
    buf[i++] = (uint8_t)c->foc_hall_table_6;
    buf[i++] = (uint8_t)c->foc_hall_table_7;
    vb_put_f32_auto(buf, c->foc_hall_interp_erpm, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm_start, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm, &i);
    buf[i++] = (uint8_t)c->foc_control_sample_mode;
    buf[i++] = (uint8_t)c->foc_current_sample_mode;
    buf[i++] = (uint8_t)c->foc_sat_comp_mode;
    vb_put_f16(buf, c->foc_sat_comp, 1000, &i);
    buf[i++] = (uint8_t)c->foc_temp_comp;
    vb_put_f16(buf, c->foc_temp_comp_base_temp, 100, &i);
    vb_put_f16(buf, c->foc_current_filter_const, 10000, &i);
    buf[i++] = (uint8_t)c->foc_cc_decoupling;
    buf[i++] = (uint8_t)c->foc_observer_type;
    buf[i++] = (uint8_t)c->foc_hfi_amb_mode;
    vb_put_f16(buf, c->foc_hfi_amb_current, 10, &i);
    buf[i++] = (uint8_t)c->foc_hfi_amb_tres;
    vb_put_f16(buf, c->foc_hfi_voltage_start, 10, &i);
    vb_put_f16(buf, c->foc_hfi_voltage_run, 10, &i);
    vb_put_f16(buf, c->foc_hfi_voltage_max, 10, &i);
    vb_put_f16(buf, c->foc_hfi_gain, 1000, &i);
    vb_put_f16(buf, c->foc_hfi_max_err, 1000, &i);
    vb_put_f16(buf, c->foc_hfi_hyst, 100, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm_hfi, &i);
    vb_put_f32_auto(buf, c->foc_hfi_reset_erpm, &i);
    vb_put_u16(buf, c->foc_hfi_start_samples, &i);
    vb_put_f32_auto(buf, c->foc_hfi_obs_ovr_sec, &i);
    buf[i++] = (uint8_t)c->foc_hfi_samples;
    buf[i++] = (uint8_t)c->foc_offsets_cal_mode;
    vb_put_f32_auto(buf, c->foc_offsets_current_0, &i);
    vb_put_f32_auto(buf, c->foc_offsets_current_1, &i);
    vb_put_f32_auto(buf, c->foc_offsets_current_2, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_0, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_1, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_2, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_0, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_1, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_2, 10000, &i);
    buf[i++] = (uint8_t)c->foc_phase_filter_enable;
    buf[i++] = (uint8_t)c->foc_phase_filter_disable_fault;
    vb_put_f32_auto(buf, c->foc_phase_filter_max_erpm, &i);
    buf[i++] = (uint8_t)c->foc_mtpa_mode;
    vb_put_f32_auto(buf, c->foc_fw_current_max, &i);
    vb_put_f16(buf, c->foc_fw_duty_start, 10000, &i);
    vb_put_f16(buf, c->foc_fw_ramp_time, 1000, &i);
    vb_put_f16(buf, c->foc_fw_q_current_factor, 10000, &i);
    vb_put_f16(buf, c->foc_fw_backoff, 1000, &i);
    buf[i++] = (uint8_t)c->foc_speed_soure;
    buf[i++] = (uint8_t)c->foc_short_ls_on_zero_duty;
    vb_put_f16(buf, c->foc_overmod_factor, 10000, &i);
    vb_put_f16(buf, c->foc_mag_vd_max, 10000, &i);
    buf[i++] = (uint8_t)c->sp_pid_loop_rate;
    vb_put_f32_auto(buf, c->s_pid_kp, &i);
    vb_put_f32_auto(buf, c->s_pid_ki, &i);
    vb_put_f32_auto(buf, c->s_pid_kd, &i);
    vb_put_f16(buf, c->s_pid_kd_filter, 10000, &i);
    vb_put_f32_auto(buf, c->s_pid_min_erpm, &i);
    buf[i++] = (uint8_t)c->s_pid_allow_braking;
    vb_put_f32_auto(buf, c->s_pid_ramp_erpms_s, &i);
    buf[i++] = (uint8_t)c->s_pid_speed_source;
    vb_put_f32_auto(buf, c->p_pid_kp, &i);
    vb_put_f32_auto(buf, c->p_pid_ki, &i);
    vb_put_f32_auto(buf, c->p_pid_kd, &i);
    vb_put_f32_auto(buf, c->p_pid_kd_proc, &i);
    vb_put_f16(buf, c->p_pid_kd_filter, 10000, &i);
    vb_put_f32_auto(buf, c->p_pid_ang_div, &i);
    vb_put_f16(buf, c->p_pid_gain_dec_angle, 10, &i);
    vb_put_f32_auto(buf, c->p_pid_offset, &i);
    vb_put_f16(buf, c->cc_startup_boost_duty, 10000, &i);
    vb_put_f32_auto(buf, c->cc_min_current, &i);
    vb_put_f32_auto(buf, c->cc_gain, &i);
    vb_put_f16(buf, c->cc_ramp_step_max, 10000, &i);
    vb_put_i32(buf, c->m_fault_stop_time_ms, &i);
    vb_put_f16(buf, c->m_duty_ramp_step, 10000, &i);
    vb_put_f32_auto(buf, c->m_current_backoff_gain, &i);
    vb_put_u32(buf, c->m_encoder_counts, &i);
    vb_put_f16(buf, c->m_encoder_sin_amp, 1000, &i);
    vb_put_f16(buf, c->m_encoder_cos_amp, 1000, &i);
    vb_put_f16(buf, c->m_encoder_sin_offset, 1000, &i);
    vb_put_f16(buf, c->m_encoder_cos_offset, 1000, &i);
    vb_put_f16(buf, c->m_encoder_sincos_filter_constant, 1000, &i);
    vb_put_f16(buf, c->m_encoder_sincos_phase_correction, 1000, &i);
    buf[i++] = (uint8_t)c->m_sensor_port_mode;
    buf[i++] = (uint8_t)c->m_invert_direction;
    buf[i++] = (uint8_t)c->m_drv8301_oc_mode;
    buf[i++] = (uint8_t)c->m_drv8301_oc_adj;
    vb_put_f32_auto(buf, c->m_bldc_f_sw_min, &i);
    vb_put_f32_auto(buf, c->m_bldc_f_sw_max, &i);
    vb_put_f32_auto(buf, c->m_dc_f_sw, &i);
    vb_put_f32_auto(buf, c->m_ntc_motor_beta, &i);
    buf[i++] = (uint8_t)c->m_out_aux_mode;
    buf[i++] = (uint8_t)c->m_motor_temp_sens_type;
    vb_put_f32_auto(buf, c->m_ptc_motor_coeff, &i);
    vb_put_f16(buf, c->m_ntcx_ptcx_res, 0.1, &i);
    vb_put_f16(buf, c->m_ntcx_ptcx_temp_base, 10, &i);
    buf[i++] = (uint8_t)c->m_hall_extra_samples;
    buf[i++] = (uint8_t)c->m_batt_filter_const;
    buf[i++] = (uint8_t)c->si_motor_poles;
    vb_put_f32_auto(buf, c->si_gear_ratio, &i);
    vb_put_f32_auto(buf, c->si_wheel_diameter, &i);
    buf[i++] = (uint8_t)c->si_battery_type;
    buf[i++] = (uint8_t)c->si_battery_cells;
    vb_put_f32_auto(buf, c->si_battery_ah, &i);
    vb_put_f32_auto(buf, c->si_motor_nl_current, &i);
    buf[i++] = (uint8_t)c->bms_type;
    buf[i++] = (uint8_t)c->bms_limit_mode;
    buf[i++] = (uint8_t)c->bms_t_limit_start;
    buf[i++] = (uint8_t)c->bms_t_limit_end;
    vb_put_f16(buf, c->bms_soc_limit_start, 1000, &i);
    vb_put_f16(buf, c->bms_soc_limit_end, 1000, &i);
    vb_put_f16(buf, c->bms_vmin_limit_start, 1000, &i);
    vb_put_f16(buf, c->bms_vmin_limit_end, 1000, &i);
    vb_put_f16(buf, c->bms_vmax_limit_start, 1000, &i);
    vb_put_f16(buf, c->bms_vmax_limit_end, 1000, &i);
    buf[i++] = (uint8_t)c->bms_fwd_can_mode;
    return i;
}

bool vesc700_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c)
{
    int32_t i = 0;
    if (len < 488) return false;
    if (vb_get_u32(buf, &i) != VESC700_MCCONF_SIGNATURE) return false;
    c->pwm_mode = (uint8_t)buf[i++];
    c->comm_mode = (uint8_t)buf[i++];
    c->motor_type = (uint8_t)buf[i++];
    c->sensor_mode = (uint8_t)buf[i++];
    c->l_current_max = vb_get_f32_auto(buf, &i);
    c->l_current_min = vb_get_f32_auto(buf, &i);
    c->l_in_current_max = vb_get_f32_auto(buf, &i);
    c->l_in_current_min = vb_get_f32_auto(buf, &i);
    c->l_in_current_map_start = vb_get_f16(buf, 10000, &i);
    c->l_in_current_map_filter = vb_get_f16(buf, 10000, &i);
    c->l_abs_current_max = vb_get_f32_auto(buf, &i);
    c->l_min_erpm = vb_get_f32_auto(buf, &i);
    c->l_max_erpm = vb_get_f32_auto(buf, &i);
    c->l_erpm_start = vb_get_f16(buf, 10000, &i);
    c->l_max_erpm_fbrake = vb_get_f32_auto(buf, &i);
    c->l_max_erpm_fbrake_cc = vb_get_f32_auto(buf, &i);
    c->l_min_vin = vb_get_f16(buf, 10, &i);
    c->l_max_vin = vb_get_f16(buf, 10, &i);
    c->l_battery_cut_start = vb_get_f16(buf, 10, &i);
    c->l_battery_cut_end = vb_get_f16(buf, 10, &i);
    c->l_battery_regen_cut_start = vb_get_f16(buf, 10, &i);
    c->l_battery_regen_cut_end = vb_get_f16(buf, 10, &i);
    c->l_slow_abs_current = (uint8_t)buf[i++];
    c->l_temp_fet_start = (float)buf[i++];
    c->l_temp_fet_end = (float)buf[i++];
    c->l_temp_motor_start = (float)buf[i++];
    c->l_temp_motor_end = (float)buf[i++];
    c->l_temp_accel_dec = vb_get_f16(buf, 10000, &i);
    c->l_min_duty = vb_get_f16(buf, 10000, &i);
    c->l_max_duty = vb_get_f16(buf, 10000, &i);
    c->l_watt_max = vb_get_f32_auto(buf, &i);
    c->l_watt_min = vb_get_f32_auto(buf, &i);
    c->l_current_max_scale = vb_get_f16(buf, 10000, &i);
    c->l_current_min_scale = vb_get_f16(buf, 10000, &i);
    c->l_duty_start = vb_get_f16(buf, 10000, &i);
    c->l_additional_faults = (uint8_t)buf[i++];
    c->sl_min_erpm = vb_get_f32_auto(buf, &i);
    c->sl_min_erpm_cycle_int_limit = vb_get_f32_auto(buf, &i);
    c->sl_max_fullbreak_current_dir_change = vb_get_f32_auto(buf, &i);
    c->sl_cycle_int_limit = vb_get_f16(buf, 10, &i);
    c->sl_phase_advance_at_br = vb_get_f16(buf, 10000, &i);
    c->sl_cycle_int_rpm_br = vb_get_f32_auto(buf, &i);
    c->sl_bemf_coupling_k = vb_get_f32_auto(buf, &i);
    c->hall_table_0 = (float)buf[i++];
    c->hall_table_1 = (float)buf[i++];
    c->hall_table_2 = (float)buf[i++];
    c->hall_table_3 = (float)buf[i++];
    c->hall_table_4 = (float)buf[i++];
    c->hall_table_5 = (float)buf[i++];
    c->hall_table_6 = (float)buf[i++];
    c->hall_table_7 = (float)buf[i++];
    c->hall_sl_erpm = vb_get_f32_auto(buf, &i);
    c->foc_current_kp = vb_get_f32_auto(buf, &i);
    c->foc_current_ki = vb_get_f32_auto(buf, &i);
    c->foc_f_zv = vb_get_f32_auto(buf, &i);
    c->foc_dt_us = vb_get_f32_auto(buf, &i);
    c->foc_encoder_inverted = (uint8_t)buf[i++];
    c->foc_encoder_offset = vb_get_f32_auto(buf, &i);
    c->foc_encoder_ratio = vb_get_f32_auto(buf, &i);
    c->foc_sensor_mode = (uint8_t)buf[i++];
    c->foc_pll_kp = vb_get_f32_auto(buf, &i);
    c->foc_pll_ki = vb_get_f32_auto(buf, &i);
    c->foc_motor_l = vb_get_f32_auto(buf, &i);
    c->foc_motor_ld_lq_diff = vb_get_f32_auto(buf, &i);
    c->foc_motor_r = vb_get_f32_auto(buf, &i);
    c->foc_motor_flux_linkage = vb_get_f32_auto(buf, &i);
    c->foc_observer_gain = vb_get_f32_auto(buf, &i);
    c->foc_observer_gain_slow = vb_get_f32_auto(buf, &i);
    c->foc_observer_offset = vb_get_f16(buf, 1000, &i);
    c->foc_duty_dowmramp_kp = vb_get_f32_auto(buf, &i);
    c->foc_duty_dowmramp_ki = vb_get_f32_auto(buf, &i);
    c->foc_start_curr_dec = vb_get_f16(buf, 10000, &i);
    c->foc_start_curr_dec_rpm = vb_get_f32_auto(buf, &i);
    c->foc_openloop_rpm = vb_get_f32_auto(buf, &i);
    c->foc_openloop_rpm_low = vb_get_f16(buf, 1000, &i);
    c->foc_sl_openloop_hyst = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time_lock = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time_ramp = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_boost_q = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_max_q = vb_get_f16(buf, 100, &i);
    c->foc_hall_table_0 = (float)buf[i++];
    c->foc_hall_table_1 = (float)buf[i++];
    c->foc_hall_table_2 = (float)buf[i++];
    c->foc_hall_table_3 = (float)buf[i++];
    c->foc_hall_table_4 = (float)buf[i++];
    c->foc_hall_table_5 = (float)buf[i++];
    c->foc_hall_table_6 = (float)buf[i++];
    c->foc_hall_table_7 = (float)buf[i++];
    c->foc_hall_interp_erpm = vb_get_f32_auto(buf, &i);
    c->foc_sl_erpm_start = vb_get_f32_auto(buf, &i);
    c->foc_sl_erpm = vb_get_f32_auto(buf, &i);
    c->foc_control_sample_mode = (uint8_t)buf[i++];
    c->foc_current_sample_mode = (uint8_t)buf[i++];
    c->foc_sat_comp_mode = (uint8_t)buf[i++];
    c->foc_sat_comp = vb_get_f16(buf, 1000, &i);
    c->foc_temp_comp = (uint8_t)buf[i++];
    c->foc_temp_comp_base_temp = vb_get_f16(buf, 100, &i);
    c->foc_current_filter_const = vb_get_f16(buf, 10000, &i);
    c->foc_cc_decoupling = (uint8_t)buf[i++];
    c->foc_observer_type = (uint8_t)buf[i++];
    c->foc_hfi_amb_mode = (uint8_t)buf[i++];
    c->foc_hfi_amb_current = vb_get_f16(buf, 10, &i);
    c->foc_hfi_amb_tres = (float)buf[i++];
    c->foc_hfi_voltage_start = vb_get_f16(buf, 10, &i);
    c->foc_hfi_voltage_run = vb_get_f16(buf, 10, &i);
    c->foc_hfi_voltage_max = vb_get_f16(buf, 10, &i);
    c->foc_hfi_gain = vb_get_f16(buf, 1000, &i);
    c->foc_hfi_max_err = vb_get_f16(buf, 1000, &i);
    c->foc_hfi_hyst = vb_get_f16(buf, 100, &i);
    c->foc_sl_erpm_hfi = vb_get_f32_auto(buf, &i);
    c->foc_hfi_reset_erpm = vb_get_f32_auto(buf, &i);
    c->foc_hfi_start_samples = vb_get_u16(buf, &i);
    c->foc_hfi_obs_ovr_sec = vb_get_f32_auto(buf, &i);
    c->foc_hfi_samples = (uint8_t)buf[i++];
    c->foc_offsets_cal_mode = (uint8_t)buf[i++];
    c->foc_offsets_current_0 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_current_1 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_current_2 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_voltage_0 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_1 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_2 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_0 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_1 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_2 = vb_get_f16(buf, 10000, &i);
    c->foc_phase_filter_enable = (uint8_t)buf[i++];
    c->foc_phase_filter_disable_fault = (uint8_t)buf[i++];
    c->foc_phase_filter_max_erpm = vb_get_f32_auto(buf, &i);
    c->foc_mtpa_mode = (uint8_t)buf[i++];
    c->foc_fw_current_max = vb_get_f32_auto(buf, &i);
    c->foc_fw_duty_start = vb_get_f16(buf, 10000, &i);
    c->foc_fw_ramp_time = vb_get_f16(buf, 1000, &i);
    c->foc_fw_q_current_factor = vb_get_f16(buf, 10000, &i);
    c->foc_fw_backoff = vb_get_f16(buf, 1000, &i);
    c->foc_speed_soure = (uint8_t)buf[i++];
    c->foc_short_ls_on_zero_duty = (uint8_t)buf[i++];
    c->foc_overmod_factor = vb_get_f16(buf, 10000, &i);
    c->foc_mag_vd_max = vb_get_f16(buf, 10000, &i);
    c->sp_pid_loop_rate = (uint8_t)buf[i++];
    c->s_pid_kp = vb_get_f32_auto(buf, &i);
    c->s_pid_ki = vb_get_f32_auto(buf, &i);
    c->s_pid_kd = vb_get_f32_auto(buf, &i);
    c->s_pid_kd_filter = vb_get_f16(buf, 10000, &i);
    c->s_pid_min_erpm = vb_get_f32_auto(buf, &i);
    c->s_pid_allow_braking = (uint8_t)buf[i++];
    c->s_pid_ramp_erpms_s = vb_get_f32_auto(buf, &i);
    c->s_pid_speed_source = (uint8_t)buf[i++];
    c->p_pid_kp = vb_get_f32_auto(buf, &i);
    c->p_pid_ki = vb_get_f32_auto(buf, &i);
    c->p_pid_kd = vb_get_f32_auto(buf, &i);
    c->p_pid_kd_proc = vb_get_f32_auto(buf, &i);
    c->p_pid_kd_filter = vb_get_f16(buf, 10000, &i);
    c->p_pid_ang_div = vb_get_f32_auto(buf, &i);
    c->p_pid_gain_dec_angle = vb_get_f16(buf, 10, &i);
    c->p_pid_offset = vb_get_f32_auto(buf, &i);
    c->cc_startup_boost_duty = vb_get_f16(buf, 10000, &i);
    c->cc_min_current = vb_get_f32_auto(buf, &i);
    c->cc_gain = vb_get_f32_auto(buf, &i);
    c->cc_ramp_step_max = vb_get_f16(buf, 10000, &i);
    c->m_fault_stop_time_ms = vb_get_i32(buf, &i);
    c->m_duty_ramp_step = vb_get_f16(buf, 10000, &i);
    c->m_current_backoff_gain = vb_get_f32_auto(buf, &i);
    c->m_encoder_counts = vb_get_u32(buf, &i);
    c->m_encoder_sin_amp = vb_get_f16(buf, 1000, &i);
    c->m_encoder_cos_amp = vb_get_f16(buf, 1000, &i);
    c->m_encoder_sin_offset = vb_get_f16(buf, 1000, &i);
    c->m_encoder_cos_offset = vb_get_f16(buf, 1000, &i);
    c->m_encoder_sincos_filter_constant = vb_get_f16(buf, 1000, &i);
    c->m_encoder_sincos_phase_correction = vb_get_f16(buf, 1000, &i);
    c->m_sensor_port_mode = (uint8_t)buf[i++];
    c->m_invert_direction = (uint8_t)buf[i++];
    c->m_drv8301_oc_mode = (uint8_t)buf[i++];
    c->m_drv8301_oc_adj = (float)buf[i++];
    c->m_bldc_f_sw_min = vb_get_f32_auto(buf, &i);
    c->m_bldc_f_sw_max = vb_get_f32_auto(buf, &i);
    c->m_dc_f_sw = vb_get_f32_auto(buf, &i);
    c->m_ntc_motor_beta = vb_get_f32_auto(buf, &i);
    c->m_out_aux_mode = (uint8_t)buf[i++];
    c->m_motor_temp_sens_type = (uint8_t)buf[i++];
    c->m_ptc_motor_coeff = vb_get_f32_auto(buf, &i);
    c->m_ntcx_ptcx_res = vb_get_f16(buf, 0.1, &i);
    c->m_ntcx_ptcx_temp_base = vb_get_f16(buf, 10, &i);
    c->m_hall_extra_samples = (float)buf[i++];
    c->m_batt_filter_const = (float)buf[i++];
    c->si_motor_poles = (float)buf[i++];
    c->si_gear_ratio = vb_get_f32_auto(buf, &i);
    c->si_wheel_diameter = vb_get_f32_auto(buf, &i);
    c->si_battery_type = (uint8_t)buf[i++];
    c->si_battery_cells = (float)buf[i++];
    c->si_battery_ah = vb_get_f32_auto(buf, &i);
    c->si_motor_nl_current = vb_get_f32_auto(buf, &i);
    c->bms_type = (uint8_t)buf[i++];
    c->bms_limit_mode = (uint8_t)buf[i++];
    c->bms_t_limit_start = (float)buf[i++];
    c->bms_t_limit_end = (float)buf[i++];
    c->bms_soc_limit_start = vb_get_f16(buf, 1000, &i);
    c->bms_soc_limit_end = vb_get_f16(buf, 1000, &i);
    c->bms_vmin_limit_start = vb_get_f16(buf, 1000, &i);
    c->bms_vmin_limit_end = vb_get_f16(buf, 1000, &i);
    c->bms_vmax_limit_start = vb_get_f16(buf, 1000, &i);
    c->bms_vmax_limit_end = vb_get_f16(buf, 1000, &i);
    c->bms_fwd_can_mode = (uint8_t)buf[i++];
    return true;
}

int32_t rescp_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c)
{
    int32_t i = 0;
    vb_put_u32(buf, g_rescp_mcconf_sig, &i);
    buf[i++] = (uint8_t)c->motor_type;
    vb_put_f32_auto(buf, c->l_current_max, &i);
    vb_put_f32_auto(buf, c->l_current_min, &i);
    vb_put_f32_auto(buf, c->l_in_current_max, &i);
    vb_put_f32_auto(buf, c->l_in_current_min, &i);
    vb_put_f16(buf, c->l_in_current_map_start, 10000, &i);
    vb_put_f16(buf, c->l_in_current_map_filter, 10000, &i);
    vb_put_f32_auto(buf, c->l_abs_current_max, &i);
    vb_put_f32_auto(buf, c->l_min_erpm, &i);
    vb_put_f32_auto(buf, c->l_max_erpm, &i);
    vb_put_f16(buf, c->l_erpm_start, 10000, &i);
    vb_put_f16(buf, c->l_min_vin, 10, &i);
    vb_put_f16(buf, c->l_max_vin, 10, &i);
    vb_put_f16(buf, c->l_battery_cut_start, 10, &i);
    vb_put_f16(buf, c->l_battery_cut_end, 10, &i);
    vb_put_f16(buf, c->l_battery_regen_cut_start, 10, &i);
    vb_put_f16(buf, c->l_battery_regen_cut_end, 10, &i);
    buf[i++] = (uint8_t)c->l_slow_abs_current;
    buf[i++] = (uint8_t)c->l_temp_fet_start;
    buf[i++] = (uint8_t)c->l_temp_fet_end;
    buf[i++] = (uint8_t)c->l_temp_motor_start;
    buf[i++] = (uint8_t)c->l_temp_motor_end;
    vb_put_f16(buf, c->l_temp_accel_dec, 10000, &i);
    vb_put_f16(buf, c->l_min_duty, 10000, &i);
    vb_put_f16(buf, c->l_max_duty, 10000, &i);
    vb_put_f32_auto(buf, c->l_watt_max, &i);
    vb_put_f32_auto(buf, c->l_watt_min, &i);
    vb_put_f16(buf, c->l_current_max_scale, 10000, &i);
    vb_put_f16(buf, c->l_current_min_scale, 10000, &i);
    vb_put_f16(buf, c->l_duty_start, 10000, &i);
    buf[i++] = map_bits_out((uint8_t)c->l_additional_faults, KEEP_L_ADDITIONAL_FAULTS, 3);
    vb_put_f32_auto(buf, c->foc_current_kp, &i);
    vb_put_f32_auto(buf, c->foc_current_ki, &i);
    vb_put_f32_auto(buf, c->foc_f_zv, &i);
    vb_put_f32_auto(buf, c->foc_dt_us, &i);
    buf[i++] = (uint8_t)c->foc_encoder_inverted;
    vb_put_f32_auto(buf, c->foc_encoder_offset, &i);
    vb_put_f32_auto(buf, c->foc_encoder_ratio, &i);
    buf[i++] = (uint8_t)c->foc_sensor_mode;
    vb_put_f32_auto(buf, c->foc_pll_kp, &i);
    vb_put_f32_auto(buf, c->foc_pll_ki, &i);
    vb_put_f32_auto(buf, c->foc_motor_l, &i);
    vb_put_f32_auto(buf, c->foc_motor_ld_lq_diff, &i);
    vb_put_f32_auto(buf, c->foc_motor_r, &i);
    vb_put_f32_auto(buf, c->foc_motor_flux_linkage, &i);
    vb_put_f32_auto(buf, c->foc_observer_gain, &i);
    vb_put_f32_auto(buf, c->foc_observer_gain_slow, &i);
    vb_put_f16(buf, c->foc_observer_offset, 1000, &i);
    vb_put_f32_auto(buf, c->foc_duty_dowmramp_kp, &i);
    vb_put_f32_auto(buf, c->foc_duty_dowmramp_ki, &i);
    vb_put_f16(buf, c->foc_start_curr_dec, 10000, &i);
    vb_put_f32_auto(buf, c->foc_start_curr_dec_rpm, &i);
    vb_put_f32_auto(buf, c->foc_openloop_rpm, &i);
    vb_put_f16(buf, c->foc_openloop_rpm_low, 1000, &i);
    vb_put_f16(buf, c->foc_sl_openloop_hyst, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time_lock, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time_ramp, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_time, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_boost_q, 100, &i);
    vb_put_f16(buf, c->foc_sl_openloop_max_q, 100, &i);
    buf[i++] = (uint8_t)c->foc_hall_table_0;
    buf[i++] = (uint8_t)c->foc_hall_table_1;
    buf[i++] = (uint8_t)c->foc_hall_table_2;
    buf[i++] = (uint8_t)c->foc_hall_table_3;
    buf[i++] = (uint8_t)c->foc_hall_table_4;
    buf[i++] = (uint8_t)c->foc_hall_table_5;
    buf[i++] = (uint8_t)c->foc_hall_table_6;
    buf[i++] = (uint8_t)c->foc_hall_table_7;
    vb_put_f32_auto(buf, c->foc_hall_interp_erpm, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm_start, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm, &i);
    buf[i++] = map_out((uint8_t)c->foc_control_sample_mode, g_keepb_foc_control_sample_mode, g_keepb_foc_control_sample_mode_n);
    buf[i++] = map_out((uint8_t)c->foc_current_sample_mode, g_keepb_foc_current_sample_mode, g_keepb_foc_current_sample_mode_n);
    buf[i++] = (uint8_t)c->foc_sat_comp_mode;
    vb_put_f16(buf, c->foc_sat_comp, 1000, &i);
    buf[i++] = (uint8_t)c->foc_temp_comp;
    vb_put_f16(buf, c->foc_temp_comp_base_temp, 100, &i);
    vb_put_f16(buf, c->foc_current_filter_const, 10000, &i);
    buf[i++] = (uint8_t)c->foc_cc_decoupling;
    buf[i++] = (uint8_t)c->foc_observer_type;
    buf[i++] = map_out((uint8_t)c->foc_hfi_amb_mode, KEEP_FOC_HFI_AMB_MODE, 2);
    vb_put_f16(buf, c->foc_hfi_amb_current, 10, &i);
    buf[i++] = (uint8_t)c->foc_hfi_amb_tres;
    vb_put_f16(buf, c->foc_hfi_voltage_start, 10, &i);
    vb_put_f16(buf, c->foc_hfi_voltage_run, 10, &i);
    vb_put_f16(buf, c->foc_hfi_voltage_max, 10, &i);
    vb_put_f16(buf, c->foc_hfi_gain, 1000, &i);
    vb_put_f16(buf, c->foc_hfi_max_err, 1000, &i);
    vb_put_f16(buf, c->foc_hfi_hyst, 100, &i);
    vb_put_f32_auto(buf, c->foc_sl_erpm_hfi, &i);
    vb_put_f32_auto(buf, c->foc_hfi_reset_erpm, &i);
    vb_put_u16(buf, c->foc_hfi_start_samples, &i);
    vb_put_f32_auto(buf, c->foc_hfi_obs_ovr_sec, &i);
    buf[i++] = (uint8_t)c->foc_hfi_samples;
    buf[i++] = map_bits_out((uint8_t)c->foc_offsets_cal_mode, KEEP_FOC_OFFSETS_CAL_MODE, 1);
    vb_put_f32_auto(buf, c->foc_offsets_current_0, &i);
    vb_put_f32_auto(buf, c->foc_offsets_current_1, &i);
    vb_put_f32_auto(buf, c->foc_offsets_current_2, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_0, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_1, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_2, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_0, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_1, 10000, &i);
    vb_put_f16(buf, c->foc_offsets_voltage_undriven_2, 10000, &i);
    buf[i++] = (uint8_t)c->foc_phase_filter_enable;
    buf[i++] = (uint8_t)c->foc_phase_filter_disable_fault;
    vb_put_f32_auto(buf, c->foc_phase_filter_max_erpm, &i);
    buf[i++] = (uint8_t)c->foc_mtpa_mode;
    vb_put_f32_auto(buf, c->foc_fw_current_max, &i);
    vb_put_f16(buf, c->foc_fw_duty_start, 10000, &i);
    vb_put_f16(buf, c->foc_fw_ramp_time, 1000, &i);
    vb_put_f16(buf, c->foc_fw_q_current_factor, 10000, &i);
    vb_put_f16(buf, c->foc_fw_backoff, 1000, &i);
    buf[i++] = (uint8_t)c->foc_speed_soure;
    buf[i++] = (uint8_t)c->foc_short_ls_on_zero_duty;
    vb_put_f16(buf, c->foc_overmod_factor, 10000, &i);
    vb_put_f16(buf, c->foc_mag_vd_max, 10000, &i);
    buf[i++] = map_out((uint8_t)c->sp_pid_loop_rate, KEEP_SP_PID_LOOP_RATE, 6);
    vb_put_f32_auto(buf, c->s_pid_kp, &i);
    vb_put_f32_auto(buf, c->s_pid_ki, &i);
    vb_put_f32_auto(buf, c->s_pid_kd, &i);
    vb_put_f16(buf, c->s_pid_kd_filter, 10000, &i);
    vb_put_f32_auto(buf, c->s_pid_min_erpm, &i);
    buf[i++] = (uint8_t)c->s_pid_allow_braking;
    vb_put_f32_auto(buf, c->s_pid_ramp_erpms_s, &i);
    buf[i++] = (uint8_t)c->s_pid_speed_source;
    vb_put_f32_auto(buf, c->p_pid_kp, &i);
    vb_put_f32_auto(buf, c->p_pid_ki, &i);
    vb_put_f32_auto(buf, c->p_pid_kd, &i);
    vb_put_f32_auto(buf, c->p_pid_kd_proc, &i);
    vb_put_f16(buf, c->p_pid_kd_filter, 10000, &i);
    vb_put_f32_auto(buf, c->p_pid_ang_div, &i);
    vb_put_f16(buf, c->p_pid_gain_dec_angle, 10, &i);
    vb_put_f32_auto(buf, c->p_pid_offset, &i);
    vb_put_f32_auto(buf, c->cc_min_current, &i);
    vb_put_i32(buf, c->m_fault_stop_time_ms, &i);
    vb_put_u32(buf, c->m_encoder_counts, &i);
    buf[i++] = (uint8_t)c->m_invert_direction;
    vb_put_f32_auto(buf, c->m_ntc_motor_beta, &i);
    buf[i++] = map_out((uint8_t)c->m_out_aux_mode, KEEP_M_OUT_AUX_MODE, 12);
    buf[i++] = (uint8_t)c->m_motor_temp_sens_type;
    vb_put_f32_auto(buf, c->m_ptc_motor_coeff, &i);
    vb_put_f16(buf, c->m_ntcx_ptcx_res, 0.1, &i);
    vb_put_f16(buf, c->m_ntcx_ptcx_temp_base, 10, &i);
    buf[i++] = (uint8_t)c->m_hall_extra_samples;
    buf[i++] = (uint8_t)c->m_batt_filter_const;
    buf[i++] = (uint8_t)c->si_motor_poles;
    vb_put_f32_auto(buf, c->si_gear_ratio, &i);
    vb_put_f32_auto(buf, c->si_wheel_diameter, &i);
    buf[i++] = (uint8_t)c->si_battery_type;
    buf[i++] = (uint8_t)c->si_battery_cells;
    vb_put_f32_auto(buf, c->si_battery_ah, &i);
    vb_put_f32_auto(buf, c->si_motor_nl_current, &i);
    buf[i++] = (uint8_t)c->foc_motor_kind;
    vb_put_f32_auto(buf, c->foc_acim_lm, &i);
    vb_put_f32_auto(buf, c->foc_acim_lr, &i);
    vb_put_f32_auto(buf, c->foc_acim_rr, &i);
    vb_put_f16(buf, c->foc_acim_im, 100, &i);
    vb_put_f32_auto(buf, c->foc_acim_base_erpm, &i);
    vb_put_f32_auto(buf, c->foc_resc_cc_bw, &i);
    vb_put_f16(buf, c->foc_resc_rl_frac, 10000, &i);
    return i;
}

bool rescp_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c)
{
    int32_t i = 0;
    if (len < 408) return false;
    if (vb_get_u32(buf, &i) != g_rescp_mcconf_sig) return false;
    c->motor_type = (uint8_t)buf[i++];
    c->l_current_max = vb_get_f32_auto(buf, &i);
    c->l_current_min = vb_get_f32_auto(buf, &i);
    c->l_in_current_max = vb_get_f32_auto(buf, &i);
    c->l_in_current_min = vb_get_f32_auto(buf, &i);
    c->l_in_current_map_start = vb_get_f16(buf, 10000, &i);
    c->l_in_current_map_filter = vb_get_f16(buf, 10000, &i);
    c->l_abs_current_max = vb_get_f32_auto(buf, &i);
    c->l_min_erpm = vb_get_f32_auto(buf, &i);
    c->l_max_erpm = vb_get_f32_auto(buf, &i);
    c->l_erpm_start = vb_get_f16(buf, 10000, &i);
    c->l_min_vin = vb_get_f16(buf, 10, &i);
    c->l_max_vin = vb_get_f16(buf, 10, &i);
    c->l_battery_cut_start = vb_get_f16(buf, 10, &i);
    c->l_battery_cut_end = vb_get_f16(buf, 10, &i);
    c->l_battery_regen_cut_start = vb_get_f16(buf, 10, &i);
    c->l_battery_regen_cut_end = vb_get_f16(buf, 10, &i);
    c->l_slow_abs_current = (uint8_t)buf[i++];
    c->l_temp_fet_start = (float)buf[i++];
    c->l_temp_fet_end = (float)buf[i++];
    c->l_temp_motor_start = (float)buf[i++];
    c->l_temp_motor_end = (float)buf[i++];
    c->l_temp_accel_dec = vb_get_f16(buf, 10000, &i);
    c->l_min_duty = vb_get_f16(buf, 10000, &i);
    c->l_max_duty = vb_get_f16(buf, 10000, &i);
    c->l_watt_max = vb_get_f32_auto(buf, &i);
    c->l_watt_min = vb_get_f32_auto(buf, &i);
    c->l_current_max_scale = vb_get_f16(buf, 10000, &i);
    c->l_current_min_scale = vb_get_f16(buf, 10000, &i);
    c->l_duty_start = vb_get_f16(buf, 10000, &i);
    c->l_additional_faults = (uint8_t)map_bits_in(buf[i++], KEEP_L_ADDITIONAL_FAULTS, 3);
    c->foc_current_kp = vb_get_f32_auto(buf, &i);
    c->foc_current_ki = vb_get_f32_auto(buf, &i);
    c->foc_f_zv = vb_get_f32_auto(buf, &i);
    c->foc_dt_us = vb_get_f32_auto(buf, &i);
    c->foc_encoder_inverted = (uint8_t)buf[i++];
    c->foc_encoder_offset = vb_get_f32_auto(buf, &i);
    c->foc_encoder_ratio = vb_get_f32_auto(buf, &i);
    c->foc_sensor_mode = (uint8_t)buf[i++];
    c->foc_pll_kp = vb_get_f32_auto(buf, &i);
    c->foc_pll_ki = vb_get_f32_auto(buf, &i);
    c->foc_motor_l = vb_get_f32_auto(buf, &i);
    c->foc_motor_ld_lq_diff = vb_get_f32_auto(buf, &i);
    c->foc_motor_r = vb_get_f32_auto(buf, &i);
    c->foc_motor_flux_linkage = vb_get_f32_auto(buf, &i);
    c->foc_observer_gain = vb_get_f32_auto(buf, &i);
    c->foc_observer_gain_slow = vb_get_f32_auto(buf, &i);
    c->foc_observer_offset = vb_get_f16(buf, 1000, &i);
    c->foc_duty_dowmramp_kp = vb_get_f32_auto(buf, &i);
    c->foc_duty_dowmramp_ki = vb_get_f32_auto(buf, &i);
    c->foc_start_curr_dec = vb_get_f16(buf, 10000, &i);
    c->foc_start_curr_dec_rpm = vb_get_f32_auto(buf, &i);
    c->foc_openloop_rpm = vb_get_f32_auto(buf, &i);
    c->foc_openloop_rpm_low = vb_get_f16(buf, 1000, &i);
    c->foc_sl_openloop_hyst = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time_lock = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time_ramp = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_time = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_boost_q = vb_get_f16(buf, 100, &i);
    c->foc_sl_openloop_max_q = vb_get_f16(buf, 100, &i);
    c->foc_hall_table_0 = (float)buf[i++];
    c->foc_hall_table_1 = (float)buf[i++];
    c->foc_hall_table_2 = (float)buf[i++];
    c->foc_hall_table_3 = (float)buf[i++];
    c->foc_hall_table_4 = (float)buf[i++];
    c->foc_hall_table_5 = (float)buf[i++];
    c->foc_hall_table_6 = (float)buf[i++];
    c->foc_hall_table_7 = (float)buf[i++];
    c->foc_hall_interp_erpm = vb_get_f32_auto(buf, &i);
    c->foc_sl_erpm_start = vb_get_f32_auto(buf, &i);
    c->foc_sl_erpm = vb_get_f32_auto(buf, &i);
    c->foc_control_sample_mode = (uint8_t)map_in(buf[i++], g_keepb_foc_control_sample_mode, g_keepb_foc_control_sample_mode_n);
    c->foc_current_sample_mode = (uint8_t)map_in(buf[i++], g_keepb_foc_current_sample_mode, g_keepb_foc_current_sample_mode_n);
    c->foc_sat_comp_mode = (uint8_t)buf[i++];
    c->foc_sat_comp = vb_get_f16(buf, 1000, &i);
    c->foc_temp_comp = (uint8_t)buf[i++];
    c->foc_temp_comp_base_temp = vb_get_f16(buf, 100, &i);
    c->foc_current_filter_const = vb_get_f16(buf, 10000, &i);
    c->foc_cc_decoupling = (uint8_t)buf[i++];
    c->foc_observer_type = (uint8_t)buf[i++];
    c->foc_hfi_amb_mode = (uint8_t)map_in(buf[i++], KEEP_FOC_HFI_AMB_MODE, 2);
    c->foc_hfi_amb_current = vb_get_f16(buf, 10, &i);
    c->foc_hfi_amb_tres = (float)buf[i++];
    c->foc_hfi_voltage_start = vb_get_f16(buf, 10, &i);
    c->foc_hfi_voltage_run = vb_get_f16(buf, 10, &i);
    c->foc_hfi_voltage_max = vb_get_f16(buf, 10, &i);
    c->foc_hfi_gain = vb_get_f16(buf, 1000, &i);
    c->foc_hfi_max_err = vb_get_f16(buf, 1000, &i);
    c->foc_hfi_hyst = vb_get_f16(buf, 100, &i);
    c->foc_sl_erpm_hfi = vb_get_f32_auto(buf, &i);
    c->foc_hfi_reset_erpm = vb_get_f32_auto(buf, &i);
    c->foc_hfi_start_samples = vb_get_u16(buf, &i);
    c->foc_hfi_obs_ovr_sec = vb_get_f32_auto(buf, &i);
    c->foc_hfi_samples = (uint8_t)buf[i++];
    c->foc_offsets_cal_mode = (uint8_t)map_bits_in(buf[i++], KEEP_FOC_OFFSETS_CAL_MODE, 1);
    c->foc_offsets_current_0 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_current_1 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_current_2 = vb_get_f32_auto(buf, &i);
    c->foc_offsets_voltage_0 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_1 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_2 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_0 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_1 = vb_get_f16(buf, 10000, &i);
    c->foc_offsets_voltage_undriven_2 = vb_get_f16(buf, 10000, &i);
    c->foc_phase_filter_enable = (uint8_t)buf[i++];
    c->foc_phase_filter_disable_fault = (uint8_t)buf[i++];
    c->foc_phase_filter_max_erpm = vb_get_f32_auto(buf, &i);
    c->foc_mtpa_mode = (uint8_t)buf[i++];
    c->foc_fw_current_max = vb_get_f32_auto(buf, &i);
    c->foc_fw_duty_start = vb_get_f16(buf, 10000, &i);
    c->foc_fw_ramp_time = vb_get_f16(buf, 1000, &i);
    c->foc_fw_q_current_factor = vb_get_f16(buf, 10000, &i);
    c->foc_fw_backoff = vb_get_f16(buf, 1000, &i);
    c->foc_speed_soure = (uint8_t)buf[i++];
    c->foc_short_ls_on_zero_duty = (uint8_t)buf[i++];
    c->foc_overmod_factor = vb_get_f16(buf, 10000, &i);
    c->foc_mag_vd_max = vb_get_f16(buf, 10000, &i);
    c->sp_pid_loop_rate = (uint8_t)map_in(buf[i++], KEEP_SP_PID_LOOP_RATE, 6);
    c->s_pid_kp = vb_get_f32_auto(buf, &i);
    c->s_pid_ki = vb_get_f32_auto(buf, &i);
    c->s_pid_kd = vb_get_f32_auto(buf, &i);
    c->s_pid_kd_filter = vb_get_f16(buf, 10000, &i);
    c->s_pid_min_erpm = vb_get_f32_auto(buf, &i);
    c->s_pid_allow_braking = (uint8_t)buf[i++];
    c->s_pid_ramp_erpms_s = vb_get_f32_auto(buf, &i);
    c->s_pid_speed_source = (uint8_t)buf[i++];
    c->p_pid_kp = vb_get_f32_auto(buf, &i);
    c->p_pid_ki = vb_get_f32_auto(buf, &i);
    c->p_pid_kd = vb_get_f32_auto(buf, &i);
    c->p_pid_kd_proc = vb_get_f32_auto(buf, &i);
    c->p_pid_kd_filter = vb_get_f16(buf, 10000, &i);
    c->p_pid_ang_div = vb_get_f32_auto(buf, &i);
    c->p_pid_gain_dec_angle = vb_get_f16(buf, 10, &i);
    c->p_pid_offset = vb_get_f32_auto(buf, &i);
    c->cc_min_current = vb_get_f32_auto(buf, &i);
    c->m_fault_stop_time_ms = vb_get_i32(buf, &i);
    c->m_encoder_counts = vb_get_u32(buf, &i);
    c->m_invert_direction = (uint8_t)buf[i++];
    c->m_ntc_motor_beta = vb_get_f32_auto(buf, &i);
    c->m_out_aux_mode = (uint8_t)map_in(buf[i++], KEEP_M_OUT_AUX_MODE, 12);
    c->m_motor_temp_sens_type = (uint8_t)buf[i++];
    c->m_ptc_motor_coeff = vb_get_f32_auto(buf, &i);
    c->m_ntcx_ptcx_res = vb_get_f16(buf, 0.1, &i);
    c->m_ntcx_ptcx_temp_base = vb_get_f16(buf, 10, &i);
    c->m_hall_extra_samples = (float)buf[i++];
    c->m_batt_filter_const = (float)buf[i++];
    c->si_motor_poles = (float)buf[i++];
    c->si_gear_ratio = vb_get_f32_auto(buf, &i);
    c->si_wheel_diameter = vb_get_f32_auto(buf, &i);
    c->si_battery_type = (uint8_t)buf[i++];
    c->si_battery_cells = (float)buf[i++];
    c->si_battery_ah = vb_get_f32_auto(buf, &i);
    c->si_motor_nl_current = vb_get_f32_auto(buf, &i);
    c->foc_motor_kind = (uint8_t)buf[i++];
    c->foc_acim_lm = vb_get_f32_auto(buf, &i);
    c->foc_acim_lr = vb_get_f32_auto(buf, &i);
    c->foc_acim_rr = vb_get_f32_auto(buf, &i);
    c->foc_acim_im = vb_get_f16(buf, 100, &i);
    c->foc_acim_base_erpm = vb_get_f32_auto(buf, &i);
    c->foc_resc_cc_bw = vb_get_f32_auto(buf, &i);
    c->foc_resc_rl_frac = vb_get_f16(buf, 10000, &i);
    return true;
}

#ifdef RESC_CONF_PROFILE_RESC
int32_t vesc_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c) { return rescp_mcconf_serialize(buf, c); }
bool vesc_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c) { return rescp_mcconf_deserialize(buf, len, c); }
#else
int32_t vesc_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c) { return vesc700_mcconf_serialize(buf, c); }
bool vesc_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c) { return vesc700_mcconf_deserialize(buf, len, c); }
#endif

void vesc_mcconf_defaults(vesc_mcconf_t *c)
{
    c->pwm_mode = MCCONF_PWM_MODE;
    c->comm_mode = MCCONF_COMM_MODE;
    c->motor_type = MCCONF_DEFAULT_MOTOR_TYPE;
    c->sensor_mode = MCCONF_SENSOR_MODE;
    c->l_current_max = MCCONF_L_CURRENT_MAX;
    c->l_current_min = MCCONF_L_CURRENT_MIN;
    c->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
    c->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
    c->l_in_current_map_start = MCCONF_L_IN_CURRENT_MAP_START;
    c->l_in_current_map_filter = MCCONF_L_IN_CURRENT_MAP_FILTER;
    c->l_abs_current_max = MCCONF_L_MAX_ABS_CURRENT;
    c->l_min_erpm = MCCONF_L_RPM_MIN;
    c->l_max_erpm = MCCONF_L_RPM_MAX;
    c->l_erpm_start = MCCONF_L_RPM_START;
    c->l_max_erpm_fbrake = MCCONF_L_CURR_MAX_RPM_FBRAKE;
    c->l_max_erpm_fbrake_cc = MCCONF_L_CURR_MAX_RPM_FBRAKE_CC;
    c->l_min_vin = MCCONF_L_MIN_VOLTAGE;
    c->l_max_vin = MCCONF_L_MAX_VOLTAGE;
    c->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
    c->l_battery_cut_end = MCCONF_L_BATTERY_CUT_END;
    c->l_battery_regen_cut_start = MCCONF_L_BATTERY_REGEN_CUT_START;
    c->l_battery_regen_cut_end = MCCONF_L_BATTERY_REGEN_CUT_END;
    c->l_slow_abs_current = MCCONF_L_SLOW_ABS_OVERCURRENT;
    c->l_temp_fet_start = MCCONF_L_LIM_TEMP_FET_START;
    c->l_temp_fet_end = MCCONF_L_LIM_TEMP_FET_END;
    c->l_temp_motor_start = MCCONF_L_LIM_TEMP_MOTOR_START;
    c->l_temp_motor_end = MCCONF_L_LIM_TEMP_MOTOR_END;
    c->l_temp_accel_dec = MCCONF_L_LIM_TEMP_ACCEL_DEC;
    c->l_min_duty = MCCONF_L_MIN_DUTY;
    c->l_max_duty = MCCONF_L_MAX_DUTY;
    c->l_watt_max = MCCONF_L_WATT_MAX;
    c->l_watt_min = MCCONF_L_WATT_MIN;
    c->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
    c->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
    c->l_duty_start = MCCONF_L_DUTY_START;
    c->l_additional_faults = MCCONF_L_ADDITIONAL_FAULTS;
    c->sl_min_erpm = MCCONF_SL_MIN_RPM;
    c->sl_min_erpm_cycle_int_limit = MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT;
    c->sl_max_fullbreak_current_dir_change = MCCONF_SL_MAX_FB_CURR_DIR_CHANGE;
    c->sl_cycle_int_limit = MCCONF_SL_CYCLE_INT_LIMIT;
    c->sl_phase_advance_at_br = MCCONF_SL_PHASE_ADVANCE_AT_BR;
    c->sl_cycle_int_rpm_br = MCCONF_SL_CYCLE_INT_BR;
    c->sl_bemf_coupling_k = MCCONF_SL_BEMF_COUPLING_K;
    c->hall_table_0 = MCCONF_HALL_TAB_0;
    c->hall_table_1 = MCCONF_HALL_TAB_1;
    c->hall_table_2 = MCCONF_HALL_TAB_2;
    c->hall_table_3 = MCCONF_HALL_TAB_3;
    c->hall_table_4 = MCCONF_HALL_TAB_4;
    c->hall_table_5 = MCCONF_HALL_TAB_5;
    c->hall_table_6 = MCCONF_HALL_TAB_6;
    c->hall_table_7 = MCCONF_HALL_TAB_7;
    c->hall_sl_erpm = MCCONF_HALL_ERPM;
    c->foc_current_kp = MCCONF_FOC_CURRENT_KP;
    c->foc_current_ki = MCCONF_FOC_CURRENT_KI;
    c->foc_f_zv = MCCONF_FOC_F_ZV;
    c->foc_dt_us = MCCONF_FOC_DT_US;
    c->foc_encoder_inverted = MCCONF_FOC_ENCODER_INVERTED;
    c->foc_encoder_offset = MCCONF_FOC_ENCODER_OFFSET;
    c->foc_encoder_ratio = MCCONF_FOC_ENCODER_RATIO;
    c->foc_sensor_mode = MCCONF_FOC_SENSOR_MODE;
    c->foc_pll_kp = MCCONF_FOC_PLL_KP;
    c->foc_pll_ki = MCCONF_FOC_PLL_KI;
    c->foc_motor_l = MCCONF_FOC_MOTOR_L;
    c->foc_motor_ld_lq_diff = MCCONF_FOC_MOTOR_LD_LQ_DIFF;
    c->foc_motor_r = MCCONF_FOC_MOTOR_R;
    c->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
    c->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN;
    c->foc_observer_gain_slow = MCCONF_FOC_OBSERVER_GAIN_SLOW;
    c->foc_observer_offset = MCCONF_FOC_OBSERVER_OFFSET;
    c->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
    c->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
    c->foc_start_curr_dec = MCCONF_FOC_START_CURR_DEC;
    c->foc_start_curr_dec_rpm = MCCONF_FOC_START_CURR_DEC_RPM;
    c->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
    c->foc_openloop_rpm_low = MCCONF_FOC_OPENLOOP_RPM_LOW;
    c->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST;
    c->foc_sl_openloop_time_lock = MCCONF_FOC_SL_OPENLOOP_T_LOCK;
    c->foc_sl_openloop_time_ramp = MCCONF_FOC_SL_OPENLOOP_T_RAMP;
    c->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME;
    c->foc_sl_openloop_boost_q = MCCONF_FOC_SL_OPENLOOP_BOOST_Q;
    c->foc_sl_openloop_max_q = MCCONF_FOC_SL_OPENLOOP_MAX_Q;
    c->foc_hall_table_0 = MCCONF_FOC_HALL_TAB_0;
    c->foc_hall_table_1 = MCCONF_FOC_HALL_TAB_1;
    c->foc_hall_table_2 = MCCONF_FOC_HALL_TAB_2;
    c->foc_hall_table_3 = MCCONF_FOC_HALL_TAB_3;
    c->foc_hall_table_4 = MCCONF_FOC_HALL_TAB_4;
    c->foc_hall_table_5 = MCCONF_FOC_HALL_TAB_5;
    c->foc_hall_table_6 = MCCONF_FOC_HALL_TAB_6;
    c->foc_hall_table_7 = MCCONF_FOC_HALL_TAB_7;
    c->foc_hall_interp_erpm = MCCONF_FOC_HALL_INTERP_ERPM;
    c->foc_sl_erpm_start = MCCONF_FOC_SL_ERPM_START;
    c->foc_sl_erpm = MCCONF_FOC_SL_ERPM;
    c->foc_control_sample_mode = MCCONF_FOC_CONTROL_SAMPLE_MODE;
    c->foc_current_sample_mode = MCCONF_FOC_CURRENT_SAMPLE_MODE;
    c->foc_sat_comp_mode = MCCONF_FOC_SAT_COMP_MODE;
    c->foc_sat_comp = MCCONF_FOC_SAT_COMP;
    c->foc_temp_comp = MCCONF_FOC_TEMP_COMP;
    c->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP;
    c->foc_current_filter_const = MCCONF_FOC_CURRENT_FILTER_CONST;
    c->foc_cc_decoupling = MCCONF_FOC_CC_DECOUPLING;
    c->foc_observer_type = MCCONF_FOC_OBSERVER_TYPE;
    c->foc_hfi_amb_mode = MCCONF_FOC_HFI_AMB_MODE;
    c->foc_hfi_amb_current = MCCONF_FOC_HFI_AMB_CURRENT;
    c->foc_hfi_amb_tres = MCCONF_FOC_HFI_AMB_TRES;
    c->foc_hfi_voltage_start = MCCONF_FOC_HFI_VOLTAGE_START;
    c->foc_hfi_voltage_run = MCCONF_FOC_HFI_VOLTAGE_RUN;
    c->foc_hfi_voltage_max = MCCONF_FOC_HFI_VOLTAGE_MAX;
    c->foc_hfi_gain = MCCONF_FOC_HFI_GAIN;
    c->foc_hfi_max_err = MCCONF_FOC_HFI_MAX_ERR;
    c->foc_hfi_hyst = MCCONF_FOC_HFI_HYST;
    c->foc_sl_erpm_hfi = MCCONF_FOC_SL_ERPM_HFI;
    c->foc_hfi_reset_erpm = MCCONF_FOC_HFI_RESET_ERPM;
    c->foc_hfi_start_samples = MCCONF_FOC_HFI_START_SAMPLES;
    c->foc_hfi_obs_ovr_sec = MCCONF_FOC_HFI_OBS_OVR_SEC;
    c->foc_hfi_samples = MCCONF_FOC_HFI_SAMPLES;
    c->foc_offsets_cal_mode = MCCONF_FOC_OFFSETS_CAL_MODE;
    c->foc_offsets_current_0 = MCCONF_FOC_OFFSETS_CURRENT_0;
    c->foc_offsets_current_1 = MCCONF_FOC_OFFSETS_CURRENT_1;
    c->foc_offsets_current_2 = MCCONF_FOC_OFFSETS_CURRENT_2;
    c->foc_offsets_voltage_0 = MCCONF_FOC_OFFSETS_VOLTAGE_0;
    c->foc_offsets_voltage_1 = MCCONF_FOC_OFFSETS_VOLTAGE_1;
    c->foc_offsets_voltage_2 = MCCONF_FOC_OFFSETS_VOLTAGE_2;
    c->foc_offsets_voltage_undriven_0 = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
    c->foc_offsets_voltage_undriven_1 = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
    c->foc_offsets_voltage_undriven_2 = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;
    c->foc_phase_filter_enable = MCCONF_FOC_PHASE_FILTER_ENABLE;
    c->foc_phase_filter_disable_fault = MCCONF_FOC_PHASE_FILTER_DISABLE_FAULT;
    c->foc_phase_filter_max_erpm = MCCONF_FOC_PHASE_FILTER_MAX_ERPM;
    c->foc_mtpa_mode = MCCONF_FOC_MTPA_MODE;
    c->foc_fw_current_max = MCCONF_FOC_FW_CURRENT_MAX;
    c->foc_fw_duty_start = MCCONF_FOC_FW_DUTY_START;
    c->foc_fw_ramp_time = MCCONF_FOC_FW_RAMP_TIME;
    c->foc_fw_q_current_factor = MCCONF_FOC_FW_Q_CURRENT_FACTOR;
    c->foc_fw_backoff = MCCONF_FOC_FW_BACKOFF;
    c->foc_speed_soure = MCCONF_FOC_SPEED_SOURCE;
    c->foc_short_ls_on_zero_duty = MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY;
    c->foc_overmod_factor = MCCONF_FOC_OVERMOD_FACTOR;
    c->foc_mag_vd_max = MCCONF_FOC_MAG_VD_MAX;
    c->sp_pid_loop_rate = MCCONF_SP_PID_LOOP_RATE;
    c->s_pid_kp = MCCONF_S_PID_KP;
    c->s_pid_ki = MCCONF_S_PID_KI;
    c->s_pid_kd = MCCONF_S_PID_KD;
    c->s_pid_kd_filter = MCCONF_S_PID_KD_FILTER;
    c->s_pid_min_erpm = MCCONF_S_PID_MIN_RPM;
    c->s_pid_allow_braking = MCCONF_S_PID_ALLOW_BRAKING;
    c->s_pid_ramp_erpms_s = MCCONF_S_PID_RAMP_ERPMS_S;
    c->s_pid_speed_source = MCCONF_S_PID_SPEED_SOURCE;
    c->p_pid_kp = MCCONF_P_PID_KP;
    c->p_pid_ki = MCCONF_P_PID_KI;
    c->p_pid_kd = MCCONF_P_PID_KD;
    c->p_pid_kd_proc = MCCONF_P_PID_KD_PROC;
    c->p_pid_kd_filter = MCCONF_P_PID_KD_FILTER;
    c->p_pid_ang_div = MCCONF_P_PID_ANG_DIV;
    c->p_pid_gain_dec_angle = MCCONF_P_PID_GAIN_DEC_ANGLE;
    c->p_pid_offset = MCCONF_P_PID_OFFSET;
    c->cc_startup_boost_duty = MCCONF_CC_STARTUP_BOOST_DUTY;
    c->cc_min_current = MCCONF_CC_MIN_CURRENT;
    c->cc_gain = MCCONF_CC_GAIN;
    c->cc_ramp_step_max = MCCONF_CC_RAMP_STEP;
    c->m_fault_stop_time_ms = MCCONF_M_FAULT_STOP_TIME;
    c->m_duty_ramp_step = MCCONF_M_RAMP_STEP;
    c->m_current_backoff_gain = MCCONF_M_CURRENT_BACKOFF_GAIN;
    c->m_encoder_counts = MCCONF_M_ENCODER_COUNTS;
    c->m_encoder_sin_amp = MCCONF_M_ENCODER_SIN_AMP;
    c->m_encoder_cos_amp = MCCONF_M_ENCODER_COS_AMP;
    c->m_encoder_sin_offset = MCCONF_M_ENCODER_SIN_OFFSET;
    c->m_encoder_cos_offset = MCCONF_M_ENCODER_COS_OFFSET;
    c->m_encoder_sincos_filter_constant = MCCONF_M_ENCODER_SINCOS_FILTER;
    c->m_encoder_sincos_phase_correction = MCCONF_M_ENCODER_SINCOS_PHASE;
    c->m_sensor_port_mode = MCCONF_M_SENSOR_PORT_MODE;
    c->m_invert_direction = MCCONF_M_INVERT_DIRECTION;
    c->m_drv8301_oc_mode = MCCONF_M_DRV8301_OC_MODE;
    c->m_drv8301_oc_adj = MCCONF_M_DRV8301_OC_ADJ;
    c->m_bldc_f_sw_min = MCCONF_M_BLDC_F_SW_MIN;
    c->m_bldc_f_sw_max = MCCONF_M_BLDC_F_SW_MAX;
    c->m_dc_f_sw = MCCONF_M_DC_F_SW;
    c->m_ntc_motor_beta = MCCONF_M_NTC_MOTOR_BETA;
    c->m_out_aux_mode = MCCONF_M_OUT_AUX_MODE;
    c->m_motor_temp_sens_type = MCCONF_M_MOTOR_TEMP_SENS_TYPE;
    c->m_ptc_motor_coeff = MCCONF_M_PTC_MOTOR_COEFF;
    c->m_ntcx_ptcx_res = MCCONF_M_NTCX_PTCX_RES;
    c->m_ntcx_ptcx_temp_base = MCCONF_M_NTCX_PTCX_BASE_TEMP;
    c->m_hall_extra_samples = MCCONF_M_HALL_EXTRA_SAMPLES;
    c->m_batt_filter_const = MCCONF_M_BATT_FILTER_CONST;
    c->si_motor_poles = MCCONF_SI_MOTOR_POLES;
    c->si_gear_ratio = MCCONF_SI_GEAR_RATIO;
    c->si_wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
    c->si_battery_type = MCCONF_SI_BATTERY_TYPE;
    c->si_battery_cells = MCCONF_SI_BATTERY_CELLS;
    c->si_battery_ah = MCCONF_SI_BATTERY_AH;
    c->si_motor_nl_current = MCCONF_SI_MOTOR_NL_CURRENT;
    c->bms_type = MCCONF_BMS_TYPE;
    c->bms_limit_mode = MCCONF_BMS_LIMIT_MODE;
    c->bms_t_limit_start = MCCONF_BMS_T_LIMIT_START;
    c->bms_t_limit_end = MCCONF_BMS_T_LIMIT_END;
    c->bms_soc_limit_start = MCCONF_BMS_SOC_LIMIT_START;
    c->bms_soc_limit_end = MCCONF_BMS_SOC_LIMIT_END;
    c->bms_vmin_limit_start = MCCONF_BMS_VMIN_LIMIT_START;
    c->bms_vmin_limit_end = MCCONF_BMS_VMIN_LIMIT_END;
    c->bms_vmax_limit_start = MCCONF_BMS_VMAX_LIMIT_START;
    c->bms_vmax_limit_end = MCCONF_BMS_VMAX_LIMIT_END;
    c->bms_fwd_can_mode = MCCONF_BMS_FWD_CAN_MODE;
#ifndef MCCONF_FOC_MOTOR_KIND
#define MCCONF_FOC_MOTOR_KIND 0
#endif
    c->foc_motor_kind = (uint8_t)(MCCONF_FOC_MOTOR_KIND);
#ifndef MCCONF_FOC_ACIM_LM
#define MCCONF_FOC_ACIM_LM 0
#endif
    c->foc_acim_lm = (float)(MCCONF_FOC_ACIM_LM);
#ifndef MCCONF_FOC_ACIM_LR
#define MCCONF_FOC_ACIM_LR 0
#endif
    c->foc_acim_lr = (float)(MCCONF_FOC_ACIM_LR);
#ifndef MCCONF_FOC_ACIM_RR
#define MCCONF_FOC_ACIM_RR 0
#endif
    c->foc_acim_rr = (float)(MCCONF_FOC_ACIM_RR);
#ifndef MCCONF_FOC_ACIM_IM
#define MCCONF_FOC_ACIM_IM 0
#endif
    c->foc_acim_im = (float)(MCCONF_FOC_ACIM_IM);
#ifndef MCCONF_FOC_ACIM_BASE_ERPM
#define MCCONF_FOC_ACIM_BASE_ERPM 0
#endif
    c->foc_acim_base_erpm = (float)(MCCONF_FOC_ACIM_BASE_ERPM);
#ifndef MCCONF_FOC_RESC_CC_BW
#define MCCONF_FOC_RESC_CC_BW 1000
#endif
    c->foc_resc_cc_bw = (float)(MCCONF_FOC_RESC_CC_BW);
#ifndef MCCONF_FOC_RESC_RL_FRAC
#define MCCONF_FOC_RESC_RL_FRAC 0.5
#endif
    c->foc_resc_rl_frac = (float)(MCCONF_FOC_RESC_RL_FRAC);
#ifdef RESC_CONF_PROFILE_RESC
    /* defauts propres au profil RESC (valeurs du XML RESC) */
    c->foc_observer_offset = (float)(0);
    c->foc_cc_decoupling = (uint8_t)(3);
    c->foc_observer_type = (uint8_t)(7);
#endif
}

const conf_field_t vesc_mcconf_fields[] = {
    { 0x581A7C75u, (uint16_t)offsetof(vesc_mcconf_t, pwm_mode), 1, "pwm_mode" },
    { 0xD8EAEAD4u, (uint16_t)offsetof(vesc_mcconf_t, comm_mode), 1, "comm_mode" },
    { 0x409E7640u, (uint16_t)offsetof(vesc_mcconf_t, motor_type), 1, "motor_type" },
    { 0xBA2ED604u, (uint16_t)offsetof(vesc_mcconf_t, sensor_mode), 1, "sensor_mode" },
    { 0x2ADF8C09u, (uint16_t)offsetof(vesc_mcconf_t, l_current_max), 0, "l_current_max" },
    { 0x16D2B350u, (uint16_t)offsetof(vesc_mcconf_t, l_current_min), 0, "l_current_min" },
    { 0xCFB53D1Cu, (uint16_t)offsetof(vesc_mcconf_t, l_in_current_max), 0, "l_in_current_max" },
    { 0xF3B80245u, (uint16_t)offsetof(vesc_mcconf_t, l_in_current_min), 0, "l_in_current_min" },
    { 0xBB542676u, (uint16_t)offsetof(vesc_mcconf_t, l_in_current_map_start), 0, "l_in_current_map_start" },
    { 0xBB8138D6u, (uint16_t)offsetof(vesc_mcconf_t, l_in_current_map_filter), 0, "l_in_current_map_filter" },
    { 0x490B0D57u, (uint16_t)offsetof(vesc_mcconf_t, l_abs_current_max), 0, "l_abs_current_max" },
    { 0x1F76575Fu, (uint16_t)offsetof(vesc_mcconf_t, l_min_erpm), 0, "l_min_erpm" },
    { 0xF916F8BEu, (uint16_t)offsetof(vesc_mcconf_t, l_max_erpm), 0, "l_max_erpm" },
    { 0xB6E24E66u, (uint16_t)offsetof(vesc_mcconf_t, l_erpm_start), 0, "l_erpm_start" },
    { 0xDB19F477u, (uint16_t)offsetof(vesc_mcconf_t, l_max_erpm_fbrake), 0, "l_max_erpm_fbrake" },
    { 0x47CF4A6Eu, (uint16_t)offsetof(vesc_mcconf_t, l_max_erpm_fbrake_cc), 0, "l_max_erpm_fbrake_cc" },
    { 0xEEE7AA48u, (uint16_t)offsetof(vesc_mcconf_t, l_min_vin), 0, "l_min_vin" },
    { 0xED144A07u, (uint16_t)offsetof(vesc_mcconf_t, l_max_vin), 0, "l_max_vin" },
    { 0x57289AE2u, (uint16_t)offsetof(vesc_mcconf_t, l_battery_cut_start), 0, "l_battery_cut_start" },
    { 0x4DC3D561u, (uint16_t)offsetof(vesc_mcconf_t, l_battery_cut_end), 0, "l_battery_cut_end" },
    { 0x431ABF01u, (uint16_t)offsetof(vesc_mcconf_t, l_battery_regen_cut_start), 0, "l_battery_regen_cut_start" },
    { 0x37722C0Cu, (uint16_t)offsetof(vesc_mcconf_t, l_battery_regen_cut_end), 0, "l_battery_regen_cut_end" },
    { 0xF207B663u, (uint16_t)offsetof(vesc_mcconf_t, l_slow_abs_current), 1, "l_slow_abs_current" },
    { 0x43C03F0Fu, (uint16_t)offsetof(vesc_mcconf_t, l_temp_fet_start), 0, "l_temp_fet_start" },
    { 0xD7C4FC67u, (uint16_t)offsetof(vesc_mcconf_t, l_temp_fet_end), 0, "l_temp_fet_end" },
    { 0x65328E88u, (uint16_t)offsetof(vesc_mcconf_t, l_temp_motor_start), 0, "l_temp_motor_start" },
    { 0xE2B55EDDu, (uint16_t)offsetof(vesc_mcconf_t, l_temp_motor_end), 0, "l_temp_motor_end" },
    { 0xF34E361Cu, (uint16_t)offsetof(vesc_mcconf_t, l_temp_accel_dec), 0, "l_temp_accel_dec" },
    { 0xDC3337C6u, (uint16_t)offsetof(vesc_mcconf_t, l_min_duty), 0, "l_min_duty" },
    { 0x3A539827u, (uint16_t)offsetof(vesc_mcconf_t, l_max_duty), 0, "l_max_duty" },
    { 0x63563040u, (uint16_t)offsetof(vesc_mcconf_t, l_watt_max), 0, "l_watt_max" },
    { 0x5F5B0F19u, (uint16_t)offsetof(vesc_mcconf_t, l_watt_min), 0, "l_watt_min" },
    { 0x7E8AAFBAu, (uint16_t)offsetof(vesc_mcconf_t, l_current_max_scale), 0, "l_current_max_scale" },
    { 0xA9611DFBu, (uint16_t)offsetof(vesc_mcconf_t, l_current_min_scale), 0, "l_current_min_scale" },
    { 0x2A9AFEB6u, (uint16_t)offsetof(vesc_mcconf_t, l_duty_start), 0, "l_duty_start" },
    { 0xFF1F255Fu, (uint16_t)offsetof(vesc_mcconf_t, l_additional_faults), 1, "l_additional_faults" },
    { 0xF0BBBA37u, (uint16_t)offsetof(vesc_mcconf_t, sl_min_erpm), 0, "sl_min_erpm" },
    { 0xB7A87BE1u, (uint16_t)offsetof(vesc_mcconf_t, sl_min_erpm_cycle_int_limit), 0, "sl_min_erpm_cycle_int_limit" },
    { 0x8A1F02B4u, (uint16_t)offsetof(vesc_mcconf_t, sl_max_fullbreak_current_dir_change), 0, "sl_max_fullbreak_current_dir_change" },
    { 0x6DD6D14Eu, (uint16_t)offsetof(vesc_mcconf_t, sl_cycle_int_limit), 0, "sl_cycle_int_limit" },
    { 0xFD54BF0Eu, (uint16_t)offsetof(vesc_mcconf_t, sl_phase_advance_at_br), 0, "sl_phase_advance_at_br" },
    { 0x9552FF36u, (uint16_t)offsetof(vesc_mcconf_t, sl_cycle_int_rpm_br), 0, "sl_cycle_int_rpm_br" },
    { 0x486F9960u, (uint16_t)offsetof(vesc_mcconf_t, sl_bemf_coupling_k), 0, "sl_bemf_coupling_k" },
    { 0x2530DEE8u, (uint16_t)offsetof(vesc_mcconf_t, hall_table_0), 0, "hall_table_0" },
    { 0x5237EE7Eu, (uint16_t)offsetof(vesc_mcconf_t, hall_table_1), 0, "hall_table_1" },
    { 0xCB3EBFC4u, (uint16_t)offsetof(vesc_mcconf_t, hall_table_2), 0, "hall_table_2" },
    { 0xBC398F52u, (uint16_t)offsetof(vesc_mcconf_t, hall_table_3), 0, "hall_table_3" },
    { 0x225D1AF1u, (uint16_t)offsetof(vesc_mcconf_t, hall_table_4), 0, "hall_table_4" },
    { 0x555A2A67u, (uint16_t)offsetof(vesc_mcconf_t, hall_table_5), 0, "hall_table_5" },
    { 0xCC537BDDu, (uint16_t)offsetof(vesc_mcconf_t, hall_table_6), 0, "hall_table_6" },
    { 0xBB544B4Bu, (uint16_t)offsetof(vesc_mcconf_t, hall_table_7), 0, "hall_table_7" },
    { 0x661F60D4u, (uint16_t)offsetof(vesc_mcconf_t, hall_sl_erpm), 0, "hall_sl_erpm" },
    { 0x064181FCu, (uint16_t)offsetof(vesc_mcconf_t, foc_current_kp), 0, "foc_current_kp" },
    { 0x622A293Cu, (uint16_t)offsetof(vesc_mcconf_t, foc_current_ki), 0, "foc_current_ki" },
    { 0x2F60EB90u, (uint16_t)offsetof(vesc_mcconf_t, foc_f_zv), 0, "foc_f_zv" },
    { 0xF3769033u, (uint16_t)offsetof(vesc_mcconf_t, foc_dt_us), 0, "foc_dt_us" },
    { 0x35CC36B9u, (uint16_t)offsetof(vesc_mcconf_t, foc_encoder_inverted), 1, "foc_encoder_inverted" },
    { 0x8C002E14u, (uint16_t)offsetof(vesc_mcconf_t, foc_encoder_offset), 0, "foc_encoder_offset" },
    { 0x96C5985Du, (uint16_t)offsetof(vesc_mcconf_t, foc_encoder_ratio), 0, "foc_encoder_ratio" },
    { 0x4121BD2Cu, (uint16_t)offsetof(vesc_mcconf_t, foc_sensor_mode), 1, "foc_sensor_mode" },
    { 0xA0AC19D9u, (uint16_t)offsetof(vesc_mcconf_t, foc_pll_kp), 0, "foc_pll_kp" },
    { 0xC4C7B119u, (uint16_t)offsetof(vesc_mcconf_t, foc_pll_ki), 0, "foc_pll_ki" },
    { 0x48640F47u, (uint16_t)offsetof(vesc_mcconf_t, foc_motor_l), 0, "foc_motor_l" },
    { 0xC6BE81A1u, (uint16_t)offsetof(vesc_mcconf_t, foc_motor_ld_lq_diff), 0, "foc_motor_ld_lq_diff" },
    { 0xB26B3224u, (uint16_t)offsetof(vesc_mcconf_t, foc_motor_r), 0, "foc_motor_r" },
    { 0x0E55C595u, (uint16_t)offsetof(vesc_mcconf_t, foc_motor_flux_linkage), 0, "foc_motor_flux_linkage" },
    { 0x5531BB07u, (uint16_t)offsetof(vesc_mcconf_t, foc_observer_gain), 0, "foc_observer_gain" },
    { 0xF21432FBu, (uint16_t)offsetof(vesc_mcconf_t, foc_observer_gain_slow), 0, "foc_observer_gain_slow" },
    { 0x0F27EFC2u, (uint16_t)offsetof(vesc_mcconf_t, foc_observer_offset), 0, "foc_observer_offset" },
    { 0x493C0FC0u, (uint16_t)offsetof(vesc_mcconf_t, foc_duty_dowmramp_kp), 0, "foc_duty_dowmramp_kp" },
    { 0x2D57A700u, (uint16_t)offsetof(vesc_mcconf_t, foc_duty_dowmramp_ki), 0, "foc_duty_dowmramp_ki" },
    { 0x6FADB5BCu, (uint16_t)offsetof(vesc_mcconf_t, foc_start_curr_dec), 0, "foc_start_curr_dec" },
    { 0xABF66B71u, (uint16_t)offsetof(vesc_mcconf_t, foc_start_curr_dec_rpm), 0, "foc_start_curr_dec_rpm" },
    { 0x6BFD4617u, (uint16_t)offsetof(vesc_mcconf_t, foc_openloop_rpm), 0, "foc_openloop_rpm" },
    { 0x1D48D82Fu, (uint16_t)offsetof(vesc_mcconf_t, foc_openloop_rpm_low), 0, "foc_openloop_rpm_low" },
    { 0xE76C8E2Bu, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_hyst), 0, "foc_sl_openloop_hyst" },
    { 0xF8EBD584u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_time_lock), 0, "foc_sl_openloop_time_lock" },
    { 0x56553C40u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_time_ramp), 0, "foc_sl_openloop_time_ramp" },
    { 0x5F74DA51u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_time), 0, "foc_sl_openloop_time" },
    { 0x2ABCD40Du, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_boost_q), 0, "foc_sl_openloop_boost_q" },
    { 0x7E33D961u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_openloop_max_q), 0, "foc_sl_openloop_max_q" },
    { 0x107E7979u, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_0), 0, "foc_hall_table_0" },
    { 0x677949EFu, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_1), 0, "foc_hall_table_1" },
    { 0xFE701855u, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_2), 0, "foc_hall_table_2" },
    { 0x897728C3u, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_3), 0, "foc_hall_table_3" },
    { 0x1713BD60u, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_4), 0, "foc_hall_table_4" },
    { 0x60148DF6u, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_5), 0, "foc_hall_table_5" },
    { 0xF91DDC4Cu, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_6), 0, "foc_hall_table_6" },
    { 0x8E1AECDAu, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_table_7), 0, "foc_hall_table_7" },
    { 0x9CF4FF7Bu, (uint16_t)offsetof(vesc_mcconf_t, foc_hall_interp_erpm), 0, "foc_hall_interp_erpm" },
    { 0x424033B4u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_erpm_start), 0, "foc_sl_erpm_start" },
    { 0x45A66CF0u, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_erpm), 0, "foc_sl_erpm" },
    { 0xB92E7991u, (uint16_t)offsetof(vesc_mcconf_t, foc_control_sample_mode), 1, "foc_control_sample_mode" },
    { 0xF8B87C5Au, (uint16_t)offsetof(vesc_mcconf_t, foc_current_sample_mode), 1, "foc_current_sample_mode" },
    { 0xBBCF8851u, (uint16_t)offsetof(vesc_mcconf_t, foc_sat_comp_mode), 1, "foc_sat_comp_mode" },
    { 0x270D2A64u, (uint16_t)offsetof(vesc_mcconf_t, foc_sat_comp), 0, "foc_sat_comp" },
    { 0xFE87695Au, (uint16_t)offsetof(vesc_mcconf_t, foc_temp_comp), 1, "foc_temp_comp" },
    { 0xE3E5061Eu, (uint16_t)offsetof(vesc_mcconf_t, foc_temp_comp_base_temp), 0, "foc_temp_comp_base_temp" },
    { 0x81732CAEu, (uint16_t)offsetof(vesc_mcconf_t, foc_current_filter_const), 0, "foc_current_filter_const" },
    { 0xDEFB07B3u, (uint16_t)offsetof(vesc_mcconf_t, foc_cc_decoupling), 1, "foc_cc_decoupling" },
    { 0x097AC12Eu, (uint16_t)offsetof(vesc_mcconf_t, foc_observer_type), 1, "foc_observer_type" },
    { 0xE76C6592u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_amb_mode), 1, "foc_hfi_amb_mode" },
    { 0x6F4EAA99u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_amb_current), 0, "foc_hfi_amb_current" },
    { 0x334CC9B4u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_amb_tres), 0, "foc_hfi_amb_tres" },
    { 0x9E765B43u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_voltage_start), 0, "foc_hfi_voltage_start" },
    { 0x948C07A3u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_voltage_run), 0, "foc_hfi_voltage_run" },
    { 0x598C81EAu, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_voltage_max), 0, "foc_hfi_voltage_max" },
    { 0x67AC6AE8u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_gain), 0, "foc_hfi_gain" },
    { 0x44BB4894u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_max_err), 0, "foc_hfi_max_err" },
    { 0x60B58BD7u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_hyst), 0, "foc_hfi_hyst" },
    { 0xCA67C53Eu, (uint16_t)offsetof(vesc_mcconf_t, foc_sl_erpm_hfi), 0, "foc_sl_erpm_hfi" },
    { 0xCC2486D0u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_reset_erpm), 0, "foc_hfi_reset_erpm" },
    { 0x71D520EFu, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_start_samples), 3, "foc_hfi_start_samples" },
    { 0x1C6860D7u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_obs_ovr_sec), 0, "foc_hfi_obs_ovr_sec" },
    { 0x5E5E8712u, (uint16_t)offsetof(vesc_mcconf_t, foc_hfi_samples), 1, "foc_hfi_samples" },
    { 0xE7B6EA70u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_cal_mode), 1, "foc_offsets_cal_mode" },
    { 0x615887F0u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_current_0), 0, "foc_offsets_current_0" },
    { 0x165FB766u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_current_1), 0, "foc_offsets_current_1" },
    { 0x8F56E6DCu, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_current_2), 0, "foc_offsets_current_2" },
    { 0x1EA089AEu, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_0), 0, "foc_offsets_voltage_0" },
    { 0x69A7B938u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_1), 0, "foc_offsets_voltage_1" },
    { 0xF0AEE882u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_2), 0, "foc_offsets_voltage_2" },
    { 0x372A41E4u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_undriven_0), 0, "foc_offsets_voltage_undriven_0" },
    { 0x402D7172u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_undriven_1), 0, "foc_offsets_voltage_undriven_1" },
    { 0xD92420C8u, (uint16_t)offsetof(vesc_mcconf_t, foc_offsets_voltage_undriven_2), 0, "foc_offsets_voltage_undriven_2" },
    { 0xECF8730Bu, (uint16_t)offsetof(vesc_mcconf_t, foc_phase_filter_enable), 1, "foc_phase_filter_enable" },
    { 0xE8BAD793u, (uint16_t)offsetof(vesc_mcconf_t, foc_phase_filter_disable_fault), 1, "foc_phase_filter_disable_fault" },
    { 0x4BB9DD1Eu, (uint16_t)offsetof(vesc_mcconf_t, foc_phase_filter_max_erpm), 0, "foc_phase_filter_max_erpm" },
    { 0xDCB6A864u, (uint16_t)offsetof(vesc_mcconf_t, foc_mtpa_mode), 1, "foc_mtpa_mode" },
    { 0xF329D6F4u, (uint16_t)offsetof(vesc_mcconf_t, foc_fw_current_max), 0, "foc_fw_current_max" },
    { 0x0A9ADF5Fu, (uint16_t)offsetof(vesc_mcconf_t, foc_fw_duty_start), 0, "foc_fw_duty_start" },
    { 0xC1364672u, (uint16_t)offsetof(vesc_mcconf_t, foc_fw_ramp_time), 0, "foc_fw_ramp_time" },
    { 0x754EF1DBu, (uint16_t)offsetof(vesc_mcconf_t, foc_fw_q_current_factor), 0, "foc_fw_q_current_factor" },
    { 0xC5596695u, (uint16_t)offsetof(vesc_mcconf_t, foc_fw_backoff), 0, "foc_fw_backoff" },
    { 0xA2038A1Cu, (uint16_t)offsetof(vesc_mcconf_t, foc_speed_soure), 1, "foc_speed_soure" },
    { 0xC66DF9D2u, (uint16_t)offsetof(vesc_mcconf_t, foc_short_ls_on_zero_duty), 1, "foc_short_ls_on_zero_duty" },
    { 0x5DB1123Cu, (uint16_t)offsetof(vesc_mcconf_t, foc_overmod_factor), 0, "foc_overmod_factor" },
    { 0x188FD4CCu, (uint16_t)offsetof(vesc_mcconf_t, foc_mag_vd_max), 0, "foc_mag_vd_max" },
    { 0x4B60B24Eu, (uint16_t)offsetof(vesc_mcconf_t, sp_pid_loop_rate), 1, "sp_pid_loop_rate" },
    { 0x08C80230u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_kp), 0, "s_pid_kp" },
    { 0x6CA3AAF0u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_ki), 0, "s_pid_ki" },
    { 0x1212D64Du, (uint16_t)offsetof(vesc_mcconf_t, s_pid_kd), 0, "s_pid_kd" },
    { 0x877C08E3u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_kd_filter), 0, "s_pid_kd_filter" },
    { 0x89B75B89u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_min_erpm), 0, "s_pid_min_erpm" },
    { 0xBB51DF83u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_allow_braking), 1, "s_pid_allow_braking" },
    { 0x943406B9u, (uint16_t)offsetof(vesc_mcconf_t, s_pid_ramp_erpms_s), 0, "s_pid_ramp_erpms_s" },
    { 0x7B79A93Fu, (uint16_t)offsetof(vesc_mcconf_t, s_pid_speed_source), 1, "s_pid_speed_source" },
    { 0x864705D3u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_kp), 0, "p_pid_kp" },
    { 0xE22CAD13u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_ki), 0, "p_pid_ki" },
    { 0x9C9DD1AEu, (uint16_t)offsetof(vesc_mcconf_t, p_pid_kd), 0, "p_pid_kd" },
    { 0x0F35E591u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_kd_proc), 0, "p_pid_kd_proc" },
    { 0x9601629Au, (uint16_t)offsetof(vesc_mcconf_t, p_pid_kd_filter), 0, "p_pid_kd_filter" },
    { 0x40F24939u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_ang_div), 0, "p_pid_ang_div" },
    { 0xEC0A1004u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_gain_dec_angle), 0, "p_pid_gain_dec_angle" },
    { 0xDBA48072u, (uint16_t)offsetof(vesc_mcconf_t, p_pid_offset), 0, "p_pid_offset" },
    { 0x7E7915C9u, (uint16_t)offsetof(vesc_mcconf_t, cc_startup_boost_duty), 0, "cc_startup_boost_duty" },
    { 0x335B0474u, (uint16_t)offsetof(vesc_mcconf_t, cc_min_current), 0, "cc_min_current" },
    { 0xE2C05EF4u, (uint16_t)offsetof(vesc_mcconf_t, cc_gain), 0, "cc_gain" },
    { 0x1D19E37Bu, (uint16_t)offsetof(vesc_mcconf_t, cc_ramp_step_max), 0, "cc_ramp_step_max" },
    { 0x8C33D5EAu, (uint16_t)offsetof(vesc_mcconf_t, m_fault_stop_time_ms), 6, "m_fault_stop_time_ms" },
    { 0x25DE7511u, (uint16_t)offsetof(vesc_mcconf_t, m_duty_ramp_step), 0, "m_duty_ramp_step" },
    { 0xA5D474C2u, (uint16_t)offsetof(vesc_mcconf_t, m_current_backoff_gain), 0, "m_current_backoff_gain" },
    { 0x9ACF9042u, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_counts), 5, "m_encoder_counts" },
    { 0x3AC3695Du, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_sin_amp), 0, "m_encoder_sin_amp" },
    { 0x133445BDu, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_cos_amp), 0, "m_encoder_cos_amp" },
    { 0x0C8A83C3u, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_sin_offset), 0, "m_encoder_sin_offset" },
    { 0xBF805F5Bu, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_cos_offset), 0, "m_encoder_cos_offset" },
    { 0xFBA80264u, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_sincos_filter_constant), 0, "m_encoder_sincos_filter_constant" },
    { 0xA62A0594u, (uint16_t)offsetof(vesc_mcconf_t, m_encoder_sincos_phase_correction), 0, "m_encoder_sincos_phase_correction" },
    { 0x92F7CBFEu, (uint16_t)offsetof(vesc_mcconf_t, m_sensor_port_mode), 1, "m_sensor_port_mode" },
    { 0xA33E6E19u, (uint16_t)offsetof(vesc_mcconf_t, m_invert_direction), 1, "m_invert_direction" },
    { 0xB61405BFu, (uint16_t)offsetof(vesc_mcconf_t, m_drv8301_oc_mode), 1, "m_drv8301_oc_mode" },
    { 0x7014FFD4u, (uint16_t)offsetof(vesc_mcconf_t, m_drv8301_oc_adj), 0, "m_drv8301_oc_adj" },
    { 0xE4B47975u, (uint16_t)offsetof(vesc_mcconf_t, m_bldc_f_sw_min), 0, "m_bldc_f_sw_min" },
    { 0xD8B9462Cu, (uint16_t)offsetof(vesc_mcconf_t, m_bldc_f_sw_max), 0, "m_bldc_f_sw_max" },
    { 0x3F27D1D7u, (uint16_t)offsetof(vesc_mcconf_t, m_dc_f_sw), 0, "m_dc_f_sw" },
    { 0x2F7BB5F9u, (uint16_t)offsetof(vesc_mcconf_t, m_ntc_motor_beta), 0, "m_ntc_motor_beta" },
    { 0x0A31D2D4u, (uint16_t)offsetof(vesc_mcconf_t, m_out_aux_mode), 1, "m_out_aux_mode" },
    { 0x5F0813E1u, (uint16_t)offsetof(vesc_mcconf_t, m_motor_temp_sens_type), 1, "m_motor_temp_sens_type" },
    { 0x1BF40B83u, (uint16_t)offsetof(vesc_mcconf_t, m_ptc_motor_coeff), 0, "m_ptc_motor_coeff" },
    { 0xAC4E3308u, (uint16_t)offsetof(vesc_mcconf_t, m_ntcx_ptcx_res), 0, "m_ntcx_ptcx_res" },
    { 0x04D3802Du, (uint16_t)offsetof(vesc_mcconf_t, m_ntcx_ptcx_temp_base), 0, "m_ntcx_ptcx_temp_base" },
    { 0xA9860DF4u, (uint16_t)offsetof(vesc_mcconf_t, m_hall_extra_samples), 0, "m_hall_extra_samples" },
    { 0xBDB5A009u, (uint16_t)offsetof(vesc_mcconf_t, m_batt_filter_const), 0, "m_batt_filter_const" },
    { 0x13E7AF04u, (uint16_t)offsetof(vesc_mcconf_t, si_motor_poles), 0, "si_motor_poles" },
    { 0xD3146636u, (uint16_t)offsetof(vesc_mcconf_t, si_gear_ratio), 0, "si_gear_ratio" },
    { 0xFDFE1B0Au, (uint16_t)offsetof(vesc_mcconf_t, si_wheel_diameter), 0, "si_wheel_diameter" },
    { 0x1355CEB0u, (uint16_t)offsetof(vesc_mcconf_t, si_battery_type), 1, "si_battery_type" },
    { 0xDC8D6BA1u, (uint16_t)offsetof(vesc_mcconf_t, si_battery_cells), 0, "si_battery_cells" },
    { 0x6DB31B1Au, (uint16_t)offsetof(vesc_mcconf_t, si_battery_ah), 0, "si_battery_ah" },
    { 0xF5E7F161u, (uint16_t)offsetof(vesc_mcconf_t, si_motor_nl_current), 0, "si_motor_nl_current" },
    { 0x87E2FACAu, (uint16_t)offsetof(vesc_mcconf_t, bms_type), 1, "bms_type" },
    { 0xBFE012A2u, (uint16_t)offsetof(vesc_mcconf_t, bms_limit_mode), 1, "bms_limit_mode" },
    { 0xA531FFD5u, (uint16_t)offsetof(vesc_mcconf_t, bms_t_limit_start), 0, "bms_t_limit_start" },
    { 0x999128ABu, (uint16_t)offsetof(vesc_mcconf_t, bms_t_limit_end), 0, "bms_t_limit_end" },
    { 0x39638AF9u, (uint16_t)offsetof(vesc_mcconf_t, bms_soc_limit_start), 0, "bms_soc_limit_start" },
    { 0x34BAF114u, (uint16_t)offsetof(vesc_mcconf_t, bms_soc_limit_end), 0, "bms_soc_limit_end" },
    { 0x1A65052Fu, (uint16_t)offsetof(vesc_mcconf_t, bms_vmin_limit_start), 0, "bms_vmin_limit_start" },
    { 0xBC3CC5B8u, (uint16_t)offsetof(vesc_mcconf_t, bms_vmin_limit_end), 0, "bms_vmin_limit_end" },
    { 0x3DDE621Eu, (uint16_t)offsetof(vesc_mcconf_t, bms_vmax_limit_start), 0, "bms_vmax_limit_start" },
    { 0xFC73BDCEu, (uint16_t)offsetof(vesc_mcconf_t, bms_vmax_limit_end), 0, "bms_vmax_limit_end" },
    { 0x7C994A3Fu, (uint16_t)offsetof(vesc_mcconf_t, bms_fwd_can_mode), 1, "bms_fwd_can_mode" },
    { 0x2CA3D0EFu, (uint16_t)offsetof(vesc_mcconf_t, foc_motor_kind), 1, "foc_motor_kind" },
    { 0x7986BEE5u, (uint16_t)offsetof(vesc_mcconf_t, foc_acim_lm), 0, "foc_acim_lm" },
    { 0xF48EB310u, (uint16_t)offsetof(vesc_mcconf_t, foc_acim_lr), 0, "foc_acim_lr" },
    { 0x20CF8CCFu, (uint16_t)offsetof(vesc_mcconf_t, foc_acim_rr), 0, "foc_acim_rr" },
    { 0x04F14AA0u, (uint16_t)offsetof(vesc_mcconf_t, foc_acim_im), 0, "foc_acim_im" },
    { 0x2383936Bu, (uint16_t)offsetof(vesc_mcconf_t, foc_acim_base_erpm), 0, "foc_acim_base_erpm" },
    { 0xB0F71F09u, (uint16_t)offsetof(vesc_mcconf_t, foc_resc_cc_bw), 0, "foc_resc_cc_bw" },
    { 0x0C180EA3u, (uint16_t)offsetof(vesc_mcconf_t, foc_resc_rl_frac), 0, "foc_resc_rl_frac" },
};
const uint16_t vesc_mcconf_nfields = 210;

int32_t vesc700_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c)
{
    int32_t i = 0;
    vb_put_u32(buf, VESC700_APPCONF_SIGNATURE, &i);
    buf[i++] = (uint8_t)c->controller_id;
    vb_put_u32(buf, c->timeout_msec, &i);
    vb_put_f32_auto(buf, c->timeout_brake_current, &i);
    vb_put_u16(buf, c->can_status_rate_1, &i);
    vb_put_u16(buf, c->can_status_rate_2, &i);
    buf[i++] = (uint8_t)c->can_status_msgs_r1;
    buf[i++] = (uint8_t)c->can_status_msgs_r2;
    buf[i++] = (uint8_t)c->can_baud_rate;
    buf[i++] = (uint8_t)c->pairing_done;
    buf[i++] = (uint8_t)c->permanent_uart_enabled;
    buf[i++] = (uint8_t)c->shutdown_mode;
    buf[i++] = (uint8_t)c->can_mode;
    buf[i++] = (uint8_t)c->uavcan_esc_index;
    buf[i++] = (uint8_t)c->uavcan_raw_mode;
    vb_put_f32_auto(buf, c->uavcan_raw_rpm_max, &i);
    buf[i++] = (uint8_t)c->uavcan_status_current_mode;
    buf[i++] = (uint8_t)c->servo_out_enable;
    buf[i++] = (uint8_t)c->kill_sw_mode;
    buf[i++] = (uint8_t)c->app_to_use;
    buf[i++] = (uint8_t)c->app_ppm_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_ppm_conf_pid_max_erpm, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_hyst, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_start, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_end, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_center, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_median_filter;
    buf[i++] = (uint8_t)c->app_ppm_conf_safe_start;
    vb_put_f32_auto(buf, c->app_ppm_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_throttle_exp_mode;
    vb_put_f32_auto(buf, c->app_ppm_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_ramp_time_neg, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_ppm_conf_tc;
    vb_put_f32_auto(buf, c->app_ppm_conf_tc_max_diff, &i);
    vb_put_f16(buf, c->app_ppm_conf_max_erpm_for_dir, 1, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_smart_rev_max_duty, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_smart_rev_ramp_time, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_adc_conf_hyst, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_start, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_end, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_min, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_max, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_center, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage2_start, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage2_end, 1000, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_use_filter;
    buf[i++] = (uint8_t)c->app_adc_conf_safe_start;
    buf[i++] = (uint8_t)c->app_adc_conf_buttons;
    buf[i++] = (uint8_t)c->app_adc_conf_voltage_inverted;
    buf[i++] = (uint8_t)c->app_adc_conf_voltage2_inverted;
    vb_put_f32_auto(buf, c->app_adc_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_adc_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_throttle_exp_mode;
    vb_put_f32_auto(buf, c->app_adc_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_adc_conf_ramp_time_neg, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_adc_conf_tc;
    vb_put_f32_auto(buf, c->app_adc_conf_tc_max_diff, &i);
    vb_put_u16(buf, c->app_adc_conf_update_rate_hz, &i);
    vb_put_u32(buf, c->app_uart_baudrate, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_chuk_conf_hyst, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_ramp_time_neg, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_stick_erpm_per_s_in_cc, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_throttle_exp_mode;
    buf[i++] = (uint8_t)c->app_chuk_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_chuk_conf_tc;
    vb_put_f32_auto(buf, c->app_chuk_conf_tc_max_diff, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_use_smart_rev;
    vb_put_f32_auto(buf, c->app_chuk_conf_smart_rev_max_duty, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_smart_rev_ramp_time, &i);
    vb_put_f16(buf, c->app_chuk_conf_coast_brake_level, 1000, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_coast_brake_ramp_time, &i);
    buf[i++] = (uint8_t)c->app_nrf_conf_speed;
    buf[i++] = (uint8_t)c->app_nrf_conf_power;
    buf[i++] = (uint8_t)c->app_nrf_conf_crc_type;
    buf[i++] = (uint8_t)c->app_nrf_conf_retry_delay;
    buf[i++] = (uint8_t)c->app_nrf_conf_retries;
    buf[i++] = (uint8_t)c->app_nrf_conf_channel;
    buf[i++] = (uint8_t)c->app_nrf_conf_address_0;
    buf[i++] = (uint8_t)c->app_nrf_conf_address_1;
    buf[i++] = (uint8_t)c->app_nrf_conf_address_2;
    buf[i++] = (uint8_t)c->app_nrf_conf_send_crc_ack;
    buf[i++] = (uint8_t)c->app_pas_conf_ctrl_type;
    buf[i++] = (uint8_t)c->app_pas_conf_sensor_type;
    vb_put_f16(buf, c->app_pas_conf_current_scaling, 1000, &i);
    vb_put_f16(buf, c->app_pas_conf_pedal_rpm_start, 10, &i);
    vb_put_f16(buf, c->app_pas_conf_pedal_rpm_end, 10, &i);
    buf[i++] = (uint8_t)c->app_pas_conf_invert_pedal_direction;
    vb_put_u16(buf, c->app_pas_conf_magnets, &i);
    buf[i++] = (uint8_t)c->app_pas_conf_use_filter;
    vb_put_f16(buf, c->app_pas_conf_ramp_time_pos, 100, &i);
    vb_put_f16(buf, c->app_pas_conf_ramp_time_neg, 100, &i);
    vb_put_u16(buf, c->app_pas_conf_update_rate_hz, &i);
    buf[i++] = (uint8_t)c->imu_conf_type;
    buf[i++] = (uint8_t)c->imu_conf_mode;
    buf[i++] = (uint8_t)c->imu_conf_filter;
    vb_put_f16(buf, c->imu_conf_accel_lowpass_filter_x, 1, &i);
    vb_put_f16(buf, c->imu_conf_accel_lowpass_filter_y, 1, &i);
    vb_put_f16(buf, c->imu_conf_accel_lowpass_filter_z, 1, &i);
    vb_put_f16(buf, c->imu_conf_gyro_lowpass_filter, 1, &i);
    vb_put_u16(buf, c->imu_conf_sample_rate_hz, &i);
    buf[i++] = (uint8_t)c->imu_conf_use_magnetometer;
    vb_put_f32_auto(buf, c->imu_conf_accel_confidence_decay, &i);
    vb_put_f32_auto(buf, c->imu_conf_mahony_kp, &i);
    vb_put_f32_auto(buf, c->imu_conf_mahony_ki, &i);
    vb_put_f32_auto(buf, c->imu_conf_madgwick_beta, &i);
    vb_put_f32_auto(buf, c->imu_conf_rot_roll, &i);
    vb_put_f32_auto(buf, c->imu_conf_rot_pitch, &i);
    vb_put_f32_auto(buf, c->imu_conf_rot_yaw, &i);
    vb_put_f32_auto(buf, c->imu_conf_accel_offsets_0, &i);
    vb_put_f32_auto(buf, c->imu_conf_accel_offsets_1, &i);
    vb_put_f32_auto(buf, c->imu_conf_accel_offsets_2, &i);
    vb_put_f32_auto(buf, c->imu_conf_gyro_offsets_0, &i);
    vb_put_f32_auto(buf, c->imu_conf_gyro_offsets_1, &i);
    vb_put_f32_auto(buf, c->imu_conf_gyro_offsets_2, &i);
    return i;
}

bool vesc700_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c)
{
    int32_t i = 0;
    if (len < 284) return false;
    if (vb_get_u32(buf, &i) != VESC700_APPCONF_SIGNATURE) return false;
    c->controller_id = (float)buf[i++];
    c->timeout_msec = vb_get_u32(buf, &i);
    c->timeout_brake_current = vb_get_f32_auto(buf, &i);
    c->can_status_rate_1 = vb_get_u16(buf, &i);
    c->can_status_rate_2 = vb_get_u16(buf, &i);
    c->can_status_msgs_r1 = (uint8_t)buf[i++];
    c->can_status_msgs_r2 = (uint8_t)buf[i++];
    c->can_baud_rate = (uint8_t)buf[i++];
    c->pairing_done = (uint8_t)buf[i++];
    c->permanent_uart_enabled = (uint8_t)buf[i++];
    c->shutdown_mode = (uint8_t)buf[i++];
    c->can_mode = (uint8_t)buf[i++];
    c->uavcan_esc_index = (float)buf[i++];
    c->uavcan_raw_mode = (uint8_t)buf[i++];
    c->uavcan_raw_rpm_max = vb_get_f32_auto(buf, &i);
    c->uavcan_status_current_mode = (uint8_t)buf[i++];
    c->servo_out_enable = (uint8_t)buf[i++];
    c->kill_sw_mode = (uint8_t)buf[i++];
    c->app_to_use = (uint8_t)buf[i++];
    c->app_ppm_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_ppm_conf_pid_max_erpm = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_start = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_end = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_center = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_median_filter = (uint8_t)buf[i++];
    c->app_ppm_conf_safe_start = (uint8_t)buf[i++];
    c->app_ppm_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_ppm_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_multi_esc = (uint8_t)buf[i++];
    c->app_ppm_conf_tc = (uint8_t)buf[i++];
    c->app_ppm_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_max_erpm_for_dir = vb_get_f16(buf, 1, &i);
    c->app_ppm_conf_smart_rev_max_duty = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_smart_rev_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_adc_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_voltage_start = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_end = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_min = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_max = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_center = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage2_start = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage2_end = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_use_filter = (uint8_t)buf[i++];
    c->app_adc_conf_safe_start = (uint8_t)buf[i++];
    c->app_adc_conf_buttons = (uint8_t)buf[i++];
    c->app_adc_conf_voltage_inverted = (uint8_t)buf[i++];
    c->app_adc_conf_voltage2_inverted = (uint8_t)buf[i++];
    c->app_adc_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_adc_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_multi_esc = (uint8_t)buf[i++];
    c->app_adc_conf_tc = (uint8_t)buf[i++];
    c->app_adc_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_update_rate_hz = vb_get_u16(buf, &i);
    c->app_uart_baudrate = vb_get_u32(buf, &i);
    c->app_chuk_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_chuk_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_stick_erpm_per_s_in_cc = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_chuk_conf_multi_esc = (uint8_t)buf[i++];
    c->app_chuk_conf_tc = (uint8_t)buf[i++];
    c->app_chuk_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_use_smart_rev = (uint8_t)buf[i++];
    c->app_chuk_conf_smart_rev_max_duty = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_smart_rev_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_coast_brake_level = vb_get_f16(buf, 1000, &i);
    c->app_chuk_conf_coast_brake_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_nrf_conf_speed = (uint8_t)buf[i++];
    c->app_nrf_conf_power = (uint8_t)buf[i++];
    c->app_nrf_conf_crc_type = (uint8_t)buf[i++];
    c->app_nrf_conf_retry_delay = (uint8_t)buf[i++];
    c->app_nrf_conf_retries = (float)buf[i++];
    c->app_nrf_conf_channel = (float)buf[i++];
    c->app_nrf_conf_address_0 = (float)buf[i++];
    c->app_nrf_conf_address_1 = (float)buf[i++];
    c->app_nrf_conf_address_2 = (float)buf[i++];
    c->app_nrf_conf_send_crc_ack = (uint8_t)buf[i++];
    c->app_pas_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_pas_conf_sensor_type = (uint8_t)buf[i++];
    c->app_pas_conf_current_scaling = vb_get_f16(buf, 1000, &i);
    c->app_pas_conf_pedal_rpm_start = vb_get_f16(buf, 10, &i);
    c->app_pas_conf_pedal_rpm_end = vb_get_f16(buf, 10, &i);
    c->app_pas_conf_invert_pedal_direction = (uint8_t)buf[i++];
    c->app_pas_conf_magnets = vb_get_u16(buf, &i);
    c->app_pas_conf_use_filter = (uint8_t)buf[i++];
    c->app_pas_conf_ramp_time_pos = vb_get_f16(buf, 100, &i);
    c->app_pas_conf_ramp_time_neg = vb_get_f16(buf, 100, &i);
    c->app_pas_conf_update_rate_hz = vb_get_u16(buf, &i);
    c->imu_conf_type = (uint8_t)buf[i++];
    c->imu_conf_mode = (uint8_t)buf[i++];
    c->imu_conf_filter = (uint8_t)buf[i++];
    c->imu_conf_accel_lowpass_filter_x = vb_get_f16(buf, 1, &i);
    c->imu_conf_accel_lowpass_filter_y = vb_get_f16(buf, 1, &i);
    c->imu_conf_accel_lowpass_filter_z = vb_get_f16(buf, 1, &i);
    c->imu_conf_gyro_lowpass_filter = vb_get_f16(buf, 1, &i);
    c->imu_conf_sample_rate_hz = vb_get_u16(buf, &i);
    c->imu_conf_use_magnetometer = (uint8_t)buf[i++];
    c->imu_conf_accel_confidence_decay = vb_get_f32_auto(buf, &i);
    c->imu_conf_mahony_kp = vb_get_f32_auto(buf, &i);
    c->imu_conf_mahony_ki = vb_get_f32_auto(buf, &i);
    c->imu_conf_madgwick_beta = vb_get_f32_auto(buf, &i);
    c->imu_conf_rot_roll = vb_get_f32_auto(buf, &i);
    c->imu_conf_rot_pitch = vb_get_f32_auto(buf, &i);
    c->imu_conf_rot_yaw = vb_get_f32_auto(buf, &i);
    c->imu_conf_accel_offsets_0 = vb_get_f32_auto(buf, &i);
    c->imu_conf_accel_offsets_1 = vb_get_f32_auto(buf, &i);
    c->imu_conf_accel_offsets_2 = vb_get_f32_auto(buf, &i);
    c->imu_conf_gyro_offsets_0 = vb_get_f32_auto(buf, &i);
    c->imu_conf_gyro_offsets_1 = vb_get_f32_auto(buf, &i);
    c->imu_conf_gyro_offsets_2 = vb_get_f32_auto(buf, &i);
    return true;
}

int32_t rescp_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c)
{
    int32_t i = 0;
    vb_put_u32(buf, RESCP_APPCONF_SIGNATURE, &i);
    buf[i++] = (uint8_t)c->controller_id;
    vb_put_u32(buf, c->timeout_msec, &i);
    vb_put_f32_auto(buf, c->timeout_brake_current, &i);
    vb_put_u16(buf, c->can_status_rate_1, &i);
    vb_put_u16(buf, c->can_status_rate_2, &i);
    buf[i++] = map_bits_out((uint8_t)c->can_status_msgs_r1, KEEP_CAN_STATUS_MSGS_R1, 6);
    buf[i++] = map_bits_out((uint8_t)c->can_status_msgs_r2, KEEP_CAN_STATUS_MSGS_R2, 6);
    buf[i++] = (uint8_t)c->can_baud_rate;
    buf[i++] = (uint8_t)c->shutdown_mode;
    buf[i++] = (uint8_t)c->kill_sw_mode;
    buf[i++] = (uint8_t)c->app_to_use;
    buf[i++] = (uint8_t)c->app_ppm_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_ppm_conf_pid_max_erpm, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_hyst, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_start, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_end, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_pulse_center, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_median_filter;
    buf[i++] = (uint8_t)c->app_ppm_conf_safe_start;
    vb_put_f32_auto(buf, c->app_ppm_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_throttle_exp_mode;
    vb_put_f32_auto(buf, c->app_ppm_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_ramp_time_neg, &i);
    buf[i++] = (uint8_t)c->app_ppm_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_ppm_conf_tc;
    vb_put_f32_auto(buf, c->app_ppm_conf_tc_max_diff, &i);
    vb_put_f16(buf, c->app_ppm_conf_max_erpm_for_dir, 1, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_smart_rev_max_duty, &i);
    vb_put_f32_auto(buf, c->app_ppm_conf_smart_rev_ramp_time, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_adc_conf_hyst, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_start, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_end, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_min, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_max, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage_center, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage2_start, 1000, &i);
    vb_put_f16(buf, c->app_adc_conf_voltage2_end, 1000, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_use_filter;
    buf[i++] = (uint8_t)c->app_adc_conf_safe_start;
    buf[i++] = map_bits_out((uint8_t)c->app_adc_conf_buttons, KEEP_APP_ADC_CONF_BUTTONS, 3);
    buf[i++] = (uint8_t)c->app_adc_conf_voltage_inverted;
    buf[i++] = (uint8_t)c->app_adc_conf_voltage2_inverted;
    vb_put_f32_auto(buf, c->app_adc_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_adc_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_throttle_exp_mode;
    vb_put_f32_auto(buf, c->app_adc_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_adc_conf_ramp_time_neg, &i);
    buf[i++] = (uint8_t)c->app_adc_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_adc_conf_tc;
    vb_put_f32_auto(buf, c->app_adc_conf_tc_max_diff, &i);
    vb_put_u16(buf, c->app_adc_conf_update_rate_hz, &i);
    vb_put_u32(buf, c->app_uart_baudrate, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_ctrl_type;
    vb_put_f32_auto(buf, c->app_chuk_conf_hyst, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_ramp_time_pos, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_ramp_time_neg, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_stick_erpm_per_s_in_cc, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_throttle_exp, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_throttle_exp_brake, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_throttle_exp_mode;
    buf[i++] = (uint8_t)c->app_chuk_conf_multi_esc;
    buf[i++] = (uint8_t)c->app_chuk_conf_tc;
    vb_put_f32_auto(buf, c->app_chuk_conf_tc_max_diff, &i);
    buf[i++] = (uint8_t)c->app_chuk_conf_use_smart_rev;
    vb_put_f32_auto(buf, c->app_chuk_conf_smart_rev_max_duty, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_smart_rev_ramp_time, &i);
    vb_put_f16(buf, c->app_chuk_conf_coast_brake_level, 1000, &i);
    vb_put_f32_auto(buf, c->app_chuk_conf_coast_brake_ramp_time, &i);
    buf[i++] = (uint8_t)c->app_pas_conf_ctrl_type;
    vb_put_f16(buf, c->app_pas_conf_current_scaling, 1000, &i);
    vb_put_f16(buf, c->app_pas_conf_pedal_rpm_start, 10, &i);
    vb_put_f16(buf, c->app_pas_conf_pedal_rpm_end, 10, &i);
    buf[i++] = (uint8_t)c->app_pas_conf_invert_pedal_direction;
    vb_put_u16(buf, c->app_pas_conf_magnets, &i);
    buf[i++] = (uint8_t)c->app_pas_conf_use_filter;
    vb_put_f16(buf, c->app_pas_conf_ramp_time_pos, 100, &i);
    vb_put_f16(buf, c->app_pas_conf_ramp_time_neg, 100, &i);
    vb_put_u16(buf, c->app_pas_conf_update_rate_hz, &i);
    return i;
}

bool rescp_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c)
{
    int32_t i = 0;
    if (len < 196) return false;
    if (vb_get_u32(buf, &i) != RESCP_APPCONF_SIGNATURE) return false;
    c->controller_id = (float)buf[i++];
    c->timeout_msec = vb_get_u32(buf, &i);
    c->timeout_brake_current = vb_get_f32_auto(buf, &i);
    c->can_status_rate_1 = vb_get_u16(buf, &i);
    c->can_status_rate_2 = vb_get_u16(buf, &i);
    c->can_status_msgs_r1 = (uint8_t)map_bits_in(buf[i++], KEEP_CAN_STATUS_MSGS_R1, 6);
    c->can_status_msgs_r2 = (uint8_t)map_bits_in(buf[i++], KEEP_CAN_STATUS_MSGS_R2, 6);
    c->can_baud_rate = (uint8_t)buf[i++];
    c->shutdown_mode = (uint8_t)buf[i++];
    c->kill_sw_mode = (uint8_t)buf[i++];
    c->app_to_use = (uint8_t)buf[i++];
    c->app_ppm_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_ppm_conf_pid_max_erpm = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_start = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_end = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_pulse_center = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_median_filter = (uint8_t)buf[i++];
    c->app_ppm_conf_safe_start = (uint8_t)buf[i++];
    c->app_ppm_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_ppm_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_multi_esc = (uint8_t)buf[i++];
    c->app_ppm_conf_tc = (uint8_t)buf[i++];
    c->app_ppm_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_max_erpm_for_dir = vb_get_f16(buf, 1, &i);
    c->app_ppm_conf_smart_rev_max_duty = vb_get_f32_auto(buf, &i);
    c->app_ppm_conf_smart_rev_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_adc_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_voltage_start = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_end = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_min = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_max = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage_center = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage2_start = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_voltage2_end = vb_get_f16(buf, 1000, &i);
    c->app_adc_conf_use_filter = (uint8_t)buf[i++];
    c->app_adc_conf_safe_start = (uint8_t)buf[i++];
    c->app_adc_conf_buttons = (uint8_t)map_bits_in(buf[i++], KEEP_APP_ADC_CONF_BUTTONS, 3);
    c->app_adc_conf_voltage_inverted = (uint8_t)buf[i++];
    c->app_adc_conf_voltage2_inverted = (uint8_t)buf[i++];
    c->app_adc_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_adc_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_multi_esc = (uint8_t)buf[i++];
    c->app_adc_conf_tc = (uint8_t)buf[i++];
    c->app_adc_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_adc_conf_update_rate_hz = vb_get_u16(buf, &i);
    c->app_uart_baudrate = vb_get_u32(buf, &i);
    c->app_chuk_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_chuk_conf_hyst = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_ramp_time_pos = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_ramp_time_neg = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_stick_erpm_per_s_in_cc = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp_brake = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_throttle_exp_mode = (uint8_t)buf[i++];
    c->app_chuk_conf_multi_esc = (uint8_t)buf[i++];
    c->app_chuk_conf_tc = (uint8_t)buf[i++];
    c->app_chuk_conf_tc_max_diff = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_use_smart_rev = (uint8_t)buf[i++];
    c->app_chuk_conf_smart_rev_max_duty = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_smart_rev_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_chuk_conf_coast_brake_level = vb_get_f16(buf, 1000, &i);
    c->app_chuk_conf_coast_brake_ramp_time = vb_get_f32_auto(buf, &i);
    c->app_pas_conf_ctrl_type = (uint8_t)buf[i++];
    c->app_pas_conf_current_scaling = vb_get_f16(buf, 1000, &i);
    c->app_pas_conf_pedal_rpm_start = vb_get_f16(buf, 10, &i);
    c->app_pas_conf_pedal_rpm_end = vb_get_f16(buf, 10, &i);
    c->app_pas_conf_invert_pedal_direction = (uint8_t)buf[i++];
    c->app_pas_conf_magnets = vb_get_u16(buf, &i);
    c->app_pas_conf_use_filter = (uint8_t)buf[i++];
    c->app_pas_conf_ramp_time_pos = vb_get_f16(buf, 100, &i);
    c->app_pas_conf_ramp_time_neg = vb_get_f16(buf, 100, &i);
    c->app_pas_conf_update_rate_hz = vb_get_u16(buf, &i);
    return true;
}

#ifdef RESC_CONF_PROFILE_RESC
int32_t vesc_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c) { return rescp_appconf_serialize(buf, c); }
bool vesc_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c) { return rescp_appconf_deserialize(buf, len, c); }
#else
int32_t vesc_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c) { return vesc700_appconf_serialize(buf, c); }
bool vesc_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c) { return vesc700_appconf_deserialize(buf, len, c); }
#endif

void vesc_appconf_defaults(vesc_appconf_t *c)
{
    c->controller_id = HW_DEFAULT_ID;
    c->timeout_msec = APPCONF_TIMEOUT_MSEC;
    c->timeout_brake_current = APPCONF_TIMEOUT_BRAKE_CURRENT;
    c->can_status_rate_1 = APPCONF_CAN_STATUS_RATE_1;
    c->can_status_rate_2 = APPCONF_CAN_STATUS_RATE_2;
    c->can_status_msgs_r1 = APPCONF_CAN_STATUS_MSGS_R1;
    c->can_status_msgs_r2 = APPCONF_CAN_STATUS_MSGS_R2;
    c->can_baud_rate = APPCONF_CAN_BAUD_RATE;
    c->pairing_done = APPCONF_PAIRING_DONE;
    c->permanent_uart_enabled = APPCONF_PERMANENT_UART_ENABLED;
    c->shutdown_mode = APPCONF_SHUTDOWN_MODE;
    c->can_mode = APPCONF_CAN_MODE;
    c->uavcan_esc_index = APPCONF_UAVCAN_ESC_INDEX;
    c->uavcan_raw_mode = APPCONF_UAVCAN_RAW_MODE;
    c->uavcan_raw_rpm_max = APPCONF_UAVCAN_RAW_RPM_MAX;
    c->uavcan_status_current_mode = APPCONF_UAVCAN_STATUS_CURRENT_MODE;
    c->servo_out_enable = APPCONF_SERVO_OUT_ENABLE;
    c->kill_sw_mode = APPCONF_KILL_SW_MODE;
    c->app_to_use = APPCONF_APP_TO_USE;
    c->app_ppm_conf_ctrl_type = APPCONF_PPM_CTRL_TYPE;
    c->app_ppm_conf_pid_max_erpm = APPCONF_PPM_PID_MAX_ERPM;
    c->app_ppm_conf_hyst = APPCONF_PPM_HYST;
    c->app_ppm_conf_pulse_start = APPCONF_PPM_PULSE_START;
    c->app_ppm_conf_pulse_end = APPCONF_PPM_PULSE_END;
    c->app_ppm_conf_pulse_center = APPCONF_PPM_PULSE_CENTER;
    c->app_ppm_conf_median_filter = APPCONF_PPM_MEDIAN_FILTER;
    c->app_ppm_conf_safe_start = APPCONF_PPM_SAFE_START;
    c->app_ppm_conf_throttle_exp = APPCONF_PPM_THROTTLE_EXP;
    c->app_ppm_conf_throttle_exp_brake = APPCONF_PPM_THROTTLE_EXP_BRAKE;
    c->app_ppm_conf_throttle_exp_mode = APPCONF_PPM_THROTTLE_EXP_MODE;
    c->app_ppm_conf_ramp_time_pos = APPCONF_PPM_RAMP_TIME_POS;
    c->app_ppm_conf_ramp_time_neg = APPCONF_PPM_RAMP_TIME_NEG;
    c->app_ppm_conf_multi_esc = APPCONF_PPM_MULTI_ESC;
    c->app_ppm_conf_tc = APPCONF_PPM_TC;
    c->app_ppm_conf_tc_max_diff = APPCONF_PPM_TC_MAX_DIFF;
    c->app_ppm_conf_max_erpm_for_dir = APPCONF_PPM_MAX_ERPM_FOR_DIR;
    c->app_ppm_conf_smart_rev_max_duty = APPCONF_PPM_SMART_REV_MAX_DUTY;
    c->app_ppm_conf_smart_rev_ramp_time = APPCONF_PPM_SMART_REV_RAMP_TIME;
    c->app_adc_conf_ctrl_type = APPCONF_ADC_CTRL_TYPE;
    c->app_adc_conf_hyst = APPCONF_ADC_HYST;
    c->app_adc_conf_voltage_start = APPCONF_ADC_VOLTAGE_START;
    c->app_adc_conf_voltage_end = APPCONF_ADC_VOLTAGE_END;
    c->app_adc_conf_voltage_min = APPCONF_ADC_VOLTAGE_MIN;
    c->app_adc_conf_voltage_max = APPCONF_ADC_VOLTAGE_MAX;
    c->app_adc_conf_voltage_center = APPCONF_ADC_VOLTAGE_CENTER;
    c->app_adc_conf_voltage2_start = APPCONF_ADC_VOLTAGE2_START;
    c->app_adc_conf_voltage2_end = APPCONF_ADC_VOLTAGE2_END;
    c->app_adc_conf_use_filter = APPCONF_ADC_USE_FILTER;
    c->app_adc_conf_safe_start = APPCONF_ADC_SAFE_START;
    c->app_adc_conf_buttons = APPCONF_ADC_BUTTONS;
    c->app_adc_conf_voltage_inverted = APPCONF_ADC_VOLTAGE_INVERTED;
    c->app_adc_conf_voltage2_inverted = APPCONF_ADC_VOLTAGE2_INVERTED;
    c->app_adc_conf_throttle_exp = APPCONF_ADC_THROTTLE_EXP;
    c->app_adc_conf_throttle_exp_brake = APPCONF_ADC_THROTTLE_EXP_BRAKE;
    c->app_adc_conf_throttle_exp_mode = APPCONF_ADC_THROTTLE_EXP_MODE;
    c->app_adc_conf_ramp_time_pos = APPCONF_ADC_RAMP_TIME_POS;
    c->app_adc_conf_ramp_time_neg = APPCONF_ADC_RAMP_TIME_NEG;
    c->app_adc_conf_multi_esc = APPCONF_ADC_MULTI_ESC;
    c->app_adc_conf_tc = APPCONF_ADC_TC;
    c->app_adc_conf_tc_max_diff = APPCONF_ADC_TC_MAX_DIFF;
    c->app_adc_conf_update_rate_hz = APPCONF_ADC_UPDATE_RATE_HZ;
    c->app_uart_baudrate = APPCONF_UART_BAUDRATE;
    c->app_chuk_conf_ctrl_type = APPCONF_CHUK_CTRL_TYPE;
    c->app_chuk_conf_hyst = APPCONF_CHUK_HYST;
    c->app_chuk_conf_ramp_time_pos = APPCONF_CHUK_RAMP_TIME_POS;
    c->app_chuk_conf_ramp_time_neg = APPCONF_CHUK_RAMP_TIME_NEG;
    c->app_chuk_conf_stick_erpm_per_s_in_cc = APPCONF_STICK_ERPM_PER_S_IN_CC;
    c->app_chuk_conf_throttle_exp = APPCONF_CHUK_THROTTLE_EXP;
    c->app_chuk_conf_throttle_exp_brake = APPCONF_CHUK_THROTTLE_EXP_BRAKE;
    c->app_chuk_conf_throttle_exp_mode = APPCONF_CHUK_THROTTLE_EXP_MODE;
    c->app_chuk_conf_multi_esc = APPCONF_CHUK_MULTI_ESC;
    c->app_chuk_conf_tc = APPCONF_CHUK_TC;
    c->app_chuk_conf_tc_max_diff = APPCONF_CHUK_TC_MAX_DIFF;
    c->app_chuk_conf_use_smart_rev = APPCONF_CHUK_USE_SMART_REV;
    c->app_chuk_conf_smart_rev_max_duty = APPCONF_CHUK_SMART_REV_MAX_DUTY;
    c->app_chuk_conf_smart_rev_ramp_time = APPCONF_CHUK_SMART_REV_RAMP_TIME;
    c->app_chuk_conf_coast_brake_level = APPCONF_CHUK_COAST_BRAKE_LEVEL;
    c->app_chuk_conf_coast_brake_ramp_time = APPCONF_CHUK_COAST_BRAKE_RAMP_TIME;
    c->app_nrf_conf_speed = APPCONF_NRF_SPEED;
    c->app_nrf_conf_power = APPCONF_NRF_POWER;
    c->app_nrf_conf_crc_type = APPCONF_NRF_CRC;
    c->app_nrf_conf_retry_delay = APPCONF_NRF_RETR_DELAY;
    c->app_nrf_conf_retries = APPCONF_NRF_RETRIES;
    c->app_nrf_conf_channel = APPCONF_NRF_CHANNEL;
    c->app_nrf_conf_address_0 = APPCONF_NRF_ADDR_B0;
    c->app_nrf_conf_address_1 = APPCONF_NRF_ADDR_B1;
    c->app_nrf_conf_address_2 = APPCONF_NRF_ADDR_B2;
    c->app_nrf_conf_send_crc_ack = APPCONF_NRF_SEND_CRC_ACK;
    c->app_pas_conf_ctrl_type = APPCONF_PAS_CTRL_TYPE;
    c->app_pas_conf_sensor_type = APPCONF_PAS_SENSOR_TYPE;
    c->app_pas_conf_current_scaling = APPCONF_PAS_CURRENT_SCALING;
    c->app_pas_conf_pedal_rpm_start = APPCONF_PAS_PEDAL_RPM_START;
    c->app_pas_conf_pedal_rpm_end = APPCONF_PAS_PEDAL_RPM_END;
    c->app_pas_conf_invert_pedal_direction = APPCONF_PAS_INVERT_PEDAL_DIRECTION;
    c->app_pas_conf_magnets = APPCONF_PAS_MAGNETS;
    c->app_pas_conf_use_filter = APPCONF_PAS_USE_FILTER;
    c->app_pas_conf_ramp_time_pos = APPCONF_PAS_RAMP_TIME_POS;
    c->app_pas_conf_ramp_time_neg = APPCONF_PAS_RAMP_TIME_NEG;
    c->app_pas_conf_update_rate_hz = APPCONF_PAS_UPDATE_RATE_HZ;
    c->imu_conf_type = APPCONF_IMU_TYPE;
    c->imu_conf_mode = APPCONF_IMU_AHRS_MODE;
    c->imu_conf_filter = APPCONF_IMU_FILTER;
    c->imu_conf_accel_lowpass_filter_x = APPCONF_IMU_ACCEL_LOWPASS_FILTER_X;
    c->imu_conf_accel_lowpass_filter_y = APPCONF_IMU_ACCEL_LOWPASS_FILTER_Y;
    c->imu_conf_accel_lowpass_filter_z = APPCONF_IMU_ACCEL_LOWPASS_FILTER_Z;
    c->imu_conf_gyro_lowpass_filter = APPCONF_IMU_GYRO_LOWPASS_FILTER;
    c->imu_conf_sample_rate_hz = APPCONF_IMU_SAMPLE_RATE_HZ;
    c->imu_conf_use_magnetometer = APPCONF_IMU_USE_MAGNETOMETER;
    c->imu_conf_accel_confidence_decay = APPCONF_IMU_ACCEL_CONFIDENCE_DECAY;
    c->imu_conf_mahony_kp = APPCONF_IMU_MAHONY_KP;
    c->imu_conf_mahony_ki = APPCONF_IMU_MAHONY_KI;
    c->imu_conf_madgwick_beta = APPCONF_IMU_MADGWICK_BETA;
    c->imu_conf_rot_roll = APPCONF_IMU_ROT_ROLL;
    c->imu_conf_rot_pitch = APPCONF_IMU_ROT_PITCH;
    c->imu_conf_rot_yaw = APPCONF_IMU_ROT_YAW;
    c->imu_conf_accel_offsets_0 = APPCONF_IMU_A_OFFSET_0;
    c->imu_conf_accel_offsets_1 = APPCONF_IMU_A_OFFSET_1;
    c->imu_conf_accel_offsets_2 = APPCONF_IMU_A_OFFSET_2;
    c->imu_conf_gyro_offsets_0 = APPCONF_IMU_G_OFFSET_0;
    c->imu_conf_gyro_offsets_1 = APPCONF_IMU_G_OFFSET_1;
    c->imu_conf_gyro_offsets_2 = APPCONF_IMU_G_OFFSET_2;
#ifdef RESC_CONF_PROFILE_RESC
    /* defauts propres au profil RESC (valeurs du XML RESC) */
#endif
}

const conf_field_t vesc_appconf_fields[] = {
    { 0xF6D1A74Bu, (uint16_t)offsetof(vesc_appconf_t, controller_id), 0, "controller_id" },
    { 0x8B4C93F3u, (uint16_t)offsetof(vesc_appconf_t, timeout_msec), 5, "timeout_msec" },
    { 0xDB17C72Du, (uint16_t)offsetof(vesc_appconf_t, timeout_brake_current), 0, "timeout_brake_current" },
    { 0x87930126u, (uint16_t)offsetof(vesc_appconf_t, can_status_rate_1), 3, "can_status_rate_1" },
    { 0x1E9A509Cu, (uint16_t)offsetof(vesc_appconf_t, can_status_rate_2), 3, "can_status_rate_2" },
    { 0x07B84F2Bu, (uint16_t)offsetof(vesc_appconf_t, can_status_msgs_r1), 1, "can_status_msgs_r1" },
    { 0x9EB11E91u, (uint16_t)offsetof(vesc_appconf_t, can_status_msgs_r2), 1, "can_status_msgs_r2" },
    { 0x420DB376u, (uint16_t)offsetof(vesc_appconf_t, can_baud_rate), 1, "can_baud_rate" },
    { 0xB7941EEDu, (uint16_t)offsetof(vesc_appconf_t, pairing_done), 1, "pairing_done" },
    { 0xCC497DDAu, (uint16_t)offsetof(vesc_appconf_t, permanent_uart_enabled), 1, "permanent_uart_enabled" },
    { 0x497BB5F0u, (uint16_t)offsetof(vesc_appconf_t, shutdown_mode), 1, "shutdown_mode" },
    { 0x28B491E7u, (uint16_t)offsetof(vesc_appconf_t, can_mode), 1, "can_mode" },
    { 0xDF639B70u, (uint16_t)offsetof(vesc_appconf_t, uavcan_esc_index), 0, "uavcan_esc_index" },
    { 0x9CC6CBCAu, (uint16_t)offsetof(vesc_appconf_t, uavcan_raw_mode), 1, "uavcan_raw_mode" },
    { 0xAAD77E5Eu, (uint16_t)offsetof(vesc_appconf_t, uavcan_raw_rpm_max), 0, "uavcan_raw_rpm_max" },
    { 0x4F57779Fu, (uint16_t)offsetof(vesc_appconf_t, uavcan_status_current_mode), 1, "uavcan_status_current_mode" },
    { 0xEE010F3Eu, (uint16_t)offsetof(vesc_appconf_t, servo_out_enable), 1, "servo_out_enable" },
    { 0x336CE692u, (uint16_t)offsetof(vesc_appconf_t, kill_sw_mode), 1, "kill_sw_mode" },
    { 0x278E69C5u, (uint16_t)offsetof(vesc_appconf_t, app_to_use), 1, "app_to_use" },
    { 0xF602D149u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_ctrl_type), 1, "app_ppm_conf_ctrl_type" },
    { 0xE4451595u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_pid_max_erpm), 0, "app_ppm_conf_pid_max_erpm" },
    { 0xFD621F14u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_hyst), 0, "app_ppm_conf_hyst" },
    { 0x613B07F1u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_pulse_start), 0, "app_ppm_conf_pulse_start" },
    { 0x405C25FBu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_pulse_end), 0, "app_ppm_conf_pulse_end" },
    { 0xF7B3F54Du, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_pulse_center), 0, "app_ppm_conf_pulse_center" },
    { 0xD3D9FBAFu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_median_filter), 1, "app_ppm_conf_median_filter" },
    { 0x7E211485u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_safe_start), 1, "app_ppm_conf_safe_start" },
    { 0x2AA8F275u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_throttle_exp), 0, "app_ppm_conf_throttle_exp" },
    { 0x4804BC84u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_throttle_exp_brake), 0, "app_ppm_conf_throttle_exp_brake" },
    { 0x6FB246D3u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_throttle_exp_mode), 1, "app_ppm_conf_throttle_exp_mode" },
    { 0xE42DBB1Eu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_ramp_time_pos), 0, "app_ppm_conf_ramp_time_pos" },
    { 0x12A00993u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_ramp_time_neg), 0, "app_ppm_conf_ramp_time_neg" },
    { 0xA42EA4DBu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_multi_esc), 1, "app_ppm_conf_multi_esc" },
    { 0x9898A83Bu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_tc), 1, "app_ppm_conf_tc" },
    { 0xEC152970u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_tc_max_diff), 0, "app_ppm_conf_tc_max_diff" },
    { 0x94DDA826u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_max_erpm_for_dir), 0, "app_ppm_conf_max_erpm_for_dir" },
    { 0x92222513u, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_smart_rev_max_duty), 0, "app_ppm_conf_smart_rev_max_duty" },
    { 0x8F8E990Fu, (uint16_t)offsetof(vesc_appconf_t, app_ppm_conf_smart_rev_ramp_time), 0, "app_ppm_conf_smart_rev_ramp_time" },
    { 0x732873A9u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_ctrl_type), 1, "app_adc_conf_ctrl_type" },
    { 0xC99B9DCEu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_hyst), 0, "app_adc_conf_hyst" },
    { 0x85509522u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_start), 0, "app_adc_conf_voltage_start" },
    { 0xAD5D6FA3u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_end), 0, "app_adc_conf_voltage_end" },
    { 0x0CDA41C2u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_min), 0, "app_adc_conf_voltage_min" },
    { 0x30D77E9Bu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_max), 0, "app_adc_conf_voltage_max" },
    { 0xE88D1DB1u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_center), 0, "app_adc_conf_voltage_center" },
    { 0x575D62A3u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage2_start), 0, "app_adc_conf_voltage2_start" },
    { 0xDF2149A3u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage2_end), 0, "app_adc_conf_voltage2_end" },
    { 0xB99FDD1Eu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_use_filter), 1, "app_adc_conf_use_filter" },
    { 0xDEAEDC5Fu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_safe_start), 1, "app_adc_conf_safe_start" },
    { 0xD729759Du, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_buttons), 1, "app_adc_conf_buttons" },
    { 0x65D55693u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage_inverted), 1, "app_adc_conf_voltage_inverted" },
    { 0xA06E2C31u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_voltage2_inverted), 1, "app_adc_conf_voltage2_inverted" },
    { 0xC4C035EDu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_throttle_exp), 0, "app_adc_conf_throttle_exp" },
    { 0xBFEA3E27u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_throttle_exp_brake), 0, "app_adc_conf_throttle_exp_brake" },
    { 0xE367B847u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_throttle_exp_mode), 1, "app_adc_conf_throttle_exp_mode" },
    { 0x1A17C8AFu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_ramp_time_pos), 0, "app_adc_conf_ramp_time_pos" },
    { 0xEC9A7A22u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_ramp_time_neg), 0, "app_adc_conf_ramp_time_neg" },
    { 0x2104063Bu, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_multi_esc), 1, "app_adc_conf_multi_esc" },
    { 0x11674930u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_tc), 1, "app_adc_conf_tc" },
    { 0x8AB39D72u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_tc_max_diff), 0, "app_adc_conf_tc_max_diff" },
    { 0xC304E777u, (uint16_t)offsetof(vesc_appconf_t, app_adc_conf_update_rate_hz), 3, "app_adc_conf_update_rate_hz" },
    { 0x03D899E4u, (uint16_t)offsetof(vesc_appconf_t, app_uart_baudrate), 5, "app_uart_baudrate" },
    { 0x23A99F3Fu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_ctrl_type), 1, "app_chuk_conf_ctrl_type" },
    { 0x0884ED15u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_hyst), 0, "app_chuk_conf_hyst" },
    { 0x6B37A7EDu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_ramp_time_pos), 0, "app_chuk_conf_ramp_time_pos" },
    { 0x9DBA1560u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_ramp_time_neg), 0, "app_chuk_conf_ramp_time_neg" },
    { 0xC1C1C401u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_stick_erpm_per_s_in_cc), 0, "app_chuk_conf_stick_erpm_per_s_in_cc" },
    { 0x552AFEA9u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_throttle_exp), 0, "app_chuk_conf_throttle_exp" },
    { 0xF0F5DC0Cu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_throttle_exp_brake), 0, "app_chuk_conf_throttle_exp_brake" },
    { 0x2377C1E4u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_throttle_exp_mode), 1, "app_chuk_conf_throttle_exp_mode" },
    { 0x7185EAADu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_multi_esc), 1, "app_chuk_conf_multi_esc" },
    { 0xCD6BCA3Du, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_tc), 1, "app_chuk_conf_tc" },
    { 0x04144E3Cu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_tc_max_diff), 0, "app_chuk_conf_tc_max_diff" },
    { 0xE02DB809u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_use_smart_rev), 1, "app_chuk_conf_use_smart_rev" },
    { 0x2AD3459Bu, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_smart_rev_max_duty), 0, "app_chuk_conf_smart_rev_max_duty" },
    { 0x6C55637Du, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_smart_rev_ramp_time), 0, "app_chuk_conf_smart_rev_ramp_time" },
    { 0x5D3DF28Du, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_coast_brake_level), 0, "app_chuk_conf_coast_brake_level" },
    { 0x89DE3AB9u, (uint16_t)offsetof(vesc_appconf_t, app_chuk_conf_coast_brake_ramp_time), 0, "app_chuk_conf_coast_brake_ramp_time" },
    { 0x1F5791CCu, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_speed), 1, "app_nrf_conf_speed" },
    { 0xBBFB6E9Au, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_power), 1, "app_nrf_conf_power" },
    { 0x3593427Cu, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_crc_type), 1, "app_nrf_conf_crc_type" },
    { 0x388B695Cu, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_retry_delay), 1, "app_nrf_conf_retry_delay" },
    { 0x1DE25387u, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_retries), 0, "app_nrf_conf_retries" },
    { 0x5A5D3C86u, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_channel), 0, "app_nrf_conf_channel" },
    { 0x5238774Eu, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_address_0), 0, "app_nrf_conf_address_0" },
    { 0x253F47D8u, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_address_1), 0, "app_nrf_conf_address_1" },
    { 0xBC361662u, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_address_2), 0, "app_nrf_conf_address_2" },
    { 0x46D2BA1Bu, (uint16_t)offsetof(vesc_appconf_t, app_nrf_conf_send_crc_ack), 1, "app_nrf_conf_send_crc_ack" },
    { 0xA0504B42u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_ctrl_type), 1, "app_pas_conf_ctrl_type" },
    { 0x536F0646u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_sensor_type), 1, "app_pas_conf_sensor_type" },
    { 0xAD10FBE5u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_current_scaling), 0, "app_pas_conf_current_scaling" },
    { 0xF9E77CABu, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_pedal_rpm_start), 0, "app_pas_conf_pedal_rpm_start" },
    { 0xC03255F3u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_pedal_rpm_end), 0, "app_pas_conf_pedal_rpm_end" },
    { 0x92B1CC40u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_invert_pedal_direction), 1, "app_pas_conf_invert_pedal_direction" },
    { 0x01024DB5u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_magnets), 3, "app_pas_conf_magnets" },
    { 0x8E949ED6u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_use_filter), 1, "app_pas_conf_use_filter" },
    { 0xC92DCA2Eu, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_ramp_time_pos), 0, "app_pas_conf_ramp_time_pos" },
    { 0x3FA078A3u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_ramp_time_neg), 0, "app_pas_conf_ramp_time_neg" },
    { 0x59686EC3u, (uint16_t)offsetof(vesc_appconf_t, app_pas_conf_update_rate_hz), 3, "app_pas_conf_update_rate_hz" },
    { 0x895DDC00u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_type), 1, "imu_conf_type" },
    { 0x9249CC82u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_mode), 1, "imu_conf_mode" },
    { 0x41E89FDDu, (uint16_t)offsetof(vesc_appconf_t, imu_conf_filter), 1, "imu_conf_filter" },
    { 0x3185F13Cu, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_lowpass_filter_x), 0, "imu_conf_accel_lowpass_filter_x" },
    { 0x4682C1AAu, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_lowpass_filter_y), 0, "imu_conf_accel_lowpass_filter_y" },
    { 0xDF8B9010u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_lowpass_filter_z), 0, "imu_conf_accel_lowpass_filter_z" },
    { 0x4F084E06u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_gyro_lowpass_filter), 0, "imu_conf_gyro_lowpass_filter" },
    { 0x9703453Du, (uint16_t)offsetof(vesc_appconf_t, imu_conf_sample_rate_hz), 3, "imu_conf_sample_rate_hz" },
    { 0x8F4C4F8Au, (uint16_t)offsetof(vesc_appconf_t, imu_conf_use_magnetometer), 1, "imu_conf_use_magnetometer" },
    { 0xB73ACA98u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_confidence_decay), 0, "imu_conf_accel_confidence_decay" },
    { 0xFFCBCC41u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_mahony_kp), 0, "imu_conf_mahony_kp" },
    { 0x9BA06481u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_mahony_ki), 0, "imu_conf_mahony_ki" },
    { 0x78C7D5D6u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_madgwick_beta), 0, "imu_conf_madgwick_beta" },
    { 0x8336A445u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_rot_roll), 0, "imu_conf_rot_roll" },
    { 0x5D5867E7u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_rot_pitch), 0, "imu_conf_rot_pitch" },
    { 0xF2F8F9DEu, (uint16_t)offsetof(vesc_appconf_t, imu_conf_rot_yaw), 0, "imu_conf_rot_yaw" },
    { 0x31F3BBD7u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_offsets_0), 0, "imu_conf_accel_offsets_0" },
    { 0x46F48B41u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_offsets_1), 0, "imu_conf_accel_offsets_1" },
    { 0xDFFDDAFBu, (uint16_t)offsetof(vesc_appconf_t, imu_conf_accel_offsets_2), 0, "imu_conf_accel_offsets_2" },
    { 0x96001835u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_gyro_offsets_0), 0, "imu_conf_gyro_offsets_0" },
    { 0xE10728A3u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_gyro_offsets_1), 0, "imu_conf_gyro_offsets_1" },
    { 0x780E7919u, (uint16_t)offsetof(vesc_appconf_t, imu_conf_gyro_offsets_2), 0, "imu_conf_gyro_offsets_2" },
};
const uint16_t vesc_appconf_nfields = 121;


/* choix dependant de la carte (tools/resc_conf/board_enum.py) */
__attribute__((weak)) const uint32_t g_rescp_mcconf_sig = 3930393213u;
__attribute__((weak)) const uint8_t g_keepb_foc_control_sample_mode[] = { 0, 2, 1 };
__attribute__((weak)) const uint8_t g_keepb_foc_control_sample_mode_n = 3;
__attribute__((weak)) const uint8_t g_keepb_foc_current_sample_mode[] = { 0, 1, 2 };
__attribute__((weak)) const uint8_t g_keepb_foc_current_sample_mode_n = 3;
