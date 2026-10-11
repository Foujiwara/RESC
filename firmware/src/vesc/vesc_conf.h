/* GENERE par tools/resc_conf/gen_resc_conf.py depuis les XML de firmware/conf - ne pas editer. */
#ifndef VESC_CONF_H
#define VESC_CONF_H

#include <stdint.h>
#include <stdbool.h>

/* Champ de configuration : crc32 du nom, decalage, type (0 float, 1 u8, 2 i8, 3 u16, 4 i16, 5 u32, 6 i32) */
typedef struct {
    uint32_t hash;
    uint16_t offset;
    uint8_t  ctype;
    const char *name;
} conf_field_t;

/* Profils de configuration : VESC700 (defaut, VESC Tool officiel) ou RESC (RESC Tool).
 * Les structures internes contiennent les champs des deux profils ; chaque profil
 * ne serialise que les siens. Les champs hors profil gardent leur valeur interne. */

typedef struct {
    uint8_t   pwm_mode;
    uint8_t   comm_mode;
    uint8_t   motor_type;
    uint8_t   sensor_mode;
    float     l_current_max;
    float     l_current_min;
    float     l_in_current_max;
    float     l_in_current_min;
    float     l_in_current_map_start;
    float     l_in_current_map_filter;
    float     l_abs_current_max;
    float     l_min_erpm;
    float     l_max_erpm;
    float     l_erpm_start;
    float     l_max_erpm_fbrake;
    float     l_max_erpm_fbrake_cc;
    float     l_min_vin;
    float     l_max_vin;
    float     l_battery_cut_start;
    float     l_battery_cut_end;
    float     l_battery_regen_cut_start;
    float     l_battery_regen_cut_end;
    uint8_t   l_slow_abs_current;
    float     l_temp_fet_start;
    float     l_temp_fet_end;
    float     l_temp_motor_start;
    float     l_temp_motor_end;
    float     l_temp_accel_dec;
    float     l_min_duty;
    float     l_max_duty;
    float     l_watt_max;
    float     l_watt_min;
    float     l_current_max_scale;
    float     l_current_min_scale;
    float     l_duty_start;
    uint8_t   l_additional_faults;
    float     sl_min_erpm;
    float     sl_min_erpm_cycle_int_limit;
    float     sl_max_fullbreak_current_dir_change;
    float     sl_cycle_int_limit;
    float     sl_phase_advance_at_br;
    float     sl_cycle_int_rpm_br;
    float     sl_bemf_coupling_k;
    float     hall_table_0;
    float     hall_table_1;
    float     hall_table_2;
    float     hall_table_3;
    float     hall_table_4;
    float     hall_table_5;
    float     hall_table_6;
    float     hall_table_7;
    float     hall_sl_erpm;
    float     foc_current_kp;
    float     foc_current_ki;
    float     foc_f_zv;
    float     foc_dt_us;
    uint8_t   foc_encoder_inverted;
    float     foc_encoder_offset;
    float     foc_encoder_ratio;
    uint8_t   foc_sensor_mode;
    float     foc_pll_kp;
    float     foc_pll_ki;
    float     foc_motor_l;
    float     foc_motor_ld_lq_diff;
    float     foc_motor_r;
    float     foc_motor_flux_linkage;
    float     foc_observer_gain;
    float     foc_observer_gain_slow;
    float     foc_observer_offset;
    float     foc_duty_dowmramp_kp;
    float     foc_duty_dowmramp_ki;
    float     foc_start_curr_dec;
    float     foc_start_curr_dec_rpm;
    float     foc_openloop_rpm;
    float     foc_openloop_rpm_low;
    float     foc_sl_openloop_hyst;
    float     foc_sl_openloop_time_lock;
    float     foc_sl_openloop_time_ramp;
    float     foc_sl_openloop_time;
    float     foc_sl_openloop_boost_q;
    float     foc_sl_openloop_max_q;
    float     foc_hall_table_0;
    float     foc_hall_table_1;
    float     foc_hall_table_2;
    float     foc_hall_table_3;
    float     foc_hall_table_4;
    float     foc_hall_table_5;
    float     foc_hall_table_6;
    float     foc_hall_table_7;
    float     foc_hall_interp_erpm;
    float     foc_sl_erpm_start;
    float     foc_sl_erpm;
    uint8_t   foc_control_sample_mode;
    uint8_t   foc_current_sample_mode;
    uint8_t   foc_sat_comp_mode;
    float     foc_sat_comp;
    uint8_t   foc_temp_comp;
    float     foc_temp_comp_base_temp;
    float     foc_current_filter_const;
    uint8_t   foc_cc_decoupling;
    uint8_t   foc_observer_type;
    uint8_t   foc_hfi_amb_mode;
    float     foc_hfi_amb_current;
    float     foc_hfi_amb_tres;
    float     foc_hfi_voltage_start;
    float     foc_hfi_voltage_run;
    float     foc_hfi_voltage_max;
    float     foc_hfi_gain;
    float     foc_hfi_max_err;
    float     foc_hfi_hyst;
    float     foc_sl_erpm_hfi;
    float     foc_hfi_reset_erpm;
    uint16_t  foc_hfi_start_samples;
    float     foc_hfi_obs_ovr_sec;
    uint8_t   foc_hfi_samples;
    uint8_t   foc_offsets_cal_mode;
    float     foc_offsets_current_0;
    float     foc_offsets_current_1;
    float     foc_offsets_current_2;
    float     foc_offsets_voltage_0;
    float     foc_offsets_voltage_1;
    float     foc_offsets_voltage_2;
    float     foc_offsets_voltage_undriven_0;
    float     foc_offsets_voltage_undriven_1;
    float     foc_offsets_voltage_undriven_2;
    uint8_t   foc_phase_filter_enable;
    uint8_t   foc_phase_filter_disable_fault;
    float     foc_phase_filter_max_erpm;
    uint8_t   foc_mtpa_mode;
    float     foc_fw_current_max;
    float     foc_fw_duty_start;
    float     foc_fw_ramp_time;
    float     foc_fw_q_current_factor;
    float     foc_fw_backoff;
    uint8_t   foc_speed_soure;
    uint8_t   foc_short_ls_on_zero_duty;
    float     foc_overmod_factor;
    float     foc_mag_vd_max;
    uint8_t   sp_pid_loop_rate;
    float     s_pid_kp;
    float     s_pid_ki;
    float     s_pid_kd;
    float     s_pid_kd_filter;
    float     s_pid_min_erpm;
    uint8_t   s_pid_allow_braking;
    float     s_pid_ramp_erpms_s;
    uint8_t   s_pid_speed_source;
    float     p_pid_kp;
    float     p_pid_ki;
    float     p_pid_kd;
    float     p_pid_kd_proc;
    float     p_pid_kd_filter;
    float     p_pid_ang_div;
    float     p_pid_gain_dec_angle;
    float     p_pid_offset;
    float     cc_startup_boost_duty;
    float     cc_min_current;
    float     cc_gain;
    float     cc_ramp_step_max;
    int32_t   m_fault_stop_time_ms;
    float     m_duty_ramp_step;
    float     m_current_backoff_gain;
    uint32_t  m_encoder_counts;
    float     m_encoder_sin_amp;
    float     m_encoder_cos_amp;
    float     m_encoder_sin_offset;
    float     m_encoder_cos_offset;
    float     m_encoder_sincos_filter_constant;
    float     m_encoder_sincos_phase_correction;
    uint8_t   m_sensor_port_mode;
    uint8_t   m_invert_direction;
    uint8_t   m_drv8301_oc_mode;
    float     m_drv8301_oc_adj;
    float     m_bldc_f_sw_min;
    float     m_bldc_f_sw_max;
    float     m_dc_f_sw;
    float     m_ntc_motor_beta;
    uint8_t   m_out_aux_mode;
    uint8_t   m_motor_temp_sens_type;
    float     m_ptc_motor_coeff;
    float     m_ntcx_ptcx_res;
    float     m_ntcx_ptcx_temp_base;
    float     m_hall_extra_samples;
    float     m_batt_filter_const;
    float     si_motor_poles;
    float     si_gear_ratio;
    float     si_wheel_diameter;
    uint8_t   si_battery_type;
    float     si_battery_cells;
    float     si_battery_ah;
    float     si_motor_nl_current;
    uint8_t   bms_type;
    uint8_t   bms_limit_mode;
    float     bms_t_limit_start;
    float     bms_t_limit_end;
    float     bms_soc_limit_start;
    float     bms_soc_limit_end;
    float     bms_vmin_limit_start;
    float     bms_vmin_limit_end;
    float     bms_vmax_limit_start;
    float     bms_vmax_limit_end;
    uint8_t   bms_fwd_can_mode;
    uint8_t   foc_motor_kind;
    float     foc_acim_lm;
    float     foc_acim_lr;
    float     foc_acim_rr;
    float     foc_acim_im;
    float     foc_acim_base_erpm;
    float     foc_resc_cc_bw;
    float     foc_resc_rl_frac;
} vesc_mcconf_t;

#define VESC700_MCCONF_SIGNATURE 1470992211u
#define RESCP_MCCONF_SIGNATURE   480116984u
#define VESC700_MCCONF_SER_SIZE 488
#define RESCP_MCCONF_SER_SIZE   421
#ifdef RESC_CONF_PROFILE_RESC
#define VESC_MCCONF_SIGNATURE RESCP_MCCONF_SIGNATURE
#else
#define VESC_MCCONF_SIGNATURE VESC700_MCCONF_SIGNATURE
#endif
#define VESC_MCCONF_SER_MAX 488
/* signature RESC et choix dependant de la carte (description de la carte compilee) */
extern const uint32_t g_rescp_mcconf_sig;
extern const uint8_t g_keepb_foc_control_sample_mode[], g_keepb_foc_control_sample_mode_n;
extern const uint8_t g_keepb_foc_current_sample_mode[], g_keepb_foc_current_sample_mode_n;
extern const uint8_t g_keepb_m_sensor_port_mode[], g_keepb_m_sensor_port_mode_n;

int32_t vesc700_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c);
bool    vesc700_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c);
int32_t rescp_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c);
bool    rescp_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c);
int32_t vesc_mcconf_serialize(uint8_t *buf, const vesc_mcconf_t *c);
bool    vesc_mcconf_deserialize(const uint8_t *buf, int32_t len, vesc_mcconf_t *c);
void    vesc_mcconf_defaults(vesc_mcconf_t *c);
extern const conf_field_t vesc_mcconf_fields[];
extern const uint16_t vesc_mcconf_nfields;

typedef struct {
    float     controller_id;
    uint32_t  timeout_msec;
    float     timeout_brake_current;
    uint16_t  can_status_rate_1;
    uint16_t  can_status_rate_2;
    uint8_t   can_status_msgs_r1;
    uint8_t   can_status_msgs_r2;
    uint8_t   can_baud_rate;
    uint8_t   pairing_done;
    uint8_t   permanent_uart_enabled;
    uint8_t   shutdown_mode;
    uint8_t   can_mode;
    float     uavcan_esc_index;
    uint8_t   uavcan_raw_mode;
    float     uavcan_raw_rpm_max;
    uint8_t   uavcan_status_current_mode;
    uint8_t   servo_out_enable;
    uint8_t   kill_sw_mode;
    uint8_t   app_to_use;
    uint8_t   app_ppm_conf_ctrl_type;
    float     app_ppm_conf_pid_max_erpm;
    float     app_ppm_conf_hyst;
    float     app_ppm_conf_pulse_start;
    float     app_ppm_conf_pulse_end;
    float     app_ppm_conf_pulse_center;
    uint8_t   app_ppm_conf_median_filter;
    uint8_t   app_ppm_conf_safe_start;
    float     app_ppm_conf_throttle_exp;
    float     app_ppm_conf_throttle_exp_brake;
    uint8_t   app_ppm_conf_throttle_exp_mode;
    float     app_ppm_conf_ramp_time_pos;
    float     app_ppm_conf_ramp_time_neg;
    uint8_t   app_ppm_conf_multi_esc;
    uint8_t   app_ppm_conf_tc;
    float     app_ppm_conf_tc_max_diff;
    float     app_ppm_conf_max_erpm_for_dir;
    float     app_ppm_conf_smart_rev_max_duty;
    float     app_ppm_conf_smart_rev_ramp_time;
    uint8_t   app_adc_conf_ctrl_type;
    float     app_adc_conf_hyst;
    float     app_adc_conf_voltage_start;
    float     app_adc_conf_voltage_end;
    float     app_adc_conf_voltage_min;
    float     app_adc_conf_voltage_max;
    float     app_adc_conf_voltage_center;
    float     app_adc_conf_voltage2_start;
    float     app_adc_conf_voltage2_end;
    uint8_t   app_adc_conf_use_filter;
    uint8_t   app_adc_conf_safe_start;
    uint8_t   app_adc_conf_buttons;
    uint8_t   app_adc_conf_voltage_inverted;
    uint8_t   app_adc_conf_voltage2_inverted;
    float     app_adc_conf_throttle_exp;
    float     app_adc_conf_throttle_exp_brake;
    uint8_t   app_adc_conf_throttle_exp_mode;
    float     app_adc_conf_ramp_time_pos;
    float     app_adc_conf_ramp_time_neg;
    uint8_t   app_adc_conf_multi_esc;
    uint8_t   app_adc_conf_tc;
    float     app_adc_conf_tc_max_diff;
    uint16_t  app_adc_conf_update_rate_hz;
    uint32_t  app_uart_baudrate;
    uint8_t   app_chuk_conf_ctrl_type;
    float     app_chuk_conf_hyst;
    float     app_chuk_conf_ramp_time_pos;
    float     app_chuk_conf_ramp_time_neg;
    float     app_chuk_conf_stick_erpm_per_s_in_cc;
    float     app_chuk_conf_throttle_exp;
    float     app_chuk_conf_throttle_exp_brake;
    uint8_t   app_chuk_conf_throttle_exp_mode;
    uint8_t   app_chuk_conf_multi_esc;
    uint8_t   app_chuk_conf_tc;
    float     app_chuk_conf_tc_max_diff;
    uint8_t   app_chuk_conf_use_smart_rev;
    float     app_chuk_conf_smart_rev_max_duty;
    float     app_chuk_conf_smart_rev_ramp_time;
    float     app_chuk_conf_coast_brake_level;
    float     app_chuk_conf_coast_brake_ramp_time;
    uint8_t   app_nrf_conf_speed;
    uint8_t   app_nrf_conf_power;
    uint8_t   app_nrf_conf_crc_type;
    uint8_t   app_nrf_conf_retry_delay;
    float     app_nrf_conf_retries;
    float     app_nrf_conf_channel;
    float     app_nrf_conf_address_0;
    float     app_nrf_conf_address_1;
    float     app_nrf_conf_address_2;
    uint8_t   app_nrf_conf_send_crc_ack;
    uint8_t   app_pas_conf_ctrl_type;
    uint8_t   app_pas_conf_sensor_type;
    float     app_pas_conf_current_scaling;
    float     app_pas_conf_pedal_rpm_start;
    float     app_pas_conf_pedal_rpm_end;
    uint8_t   app_pas_conf_invert_pedal_direction;
    uint16_t  app_pas_conf_magnets;
    uint8_t   app_pas_conf_use_filter;
    float     app_pas_conf_ramp_time_pos;
    float     app_pas_conf_ramp_time_neg;
    uint16_t  app_pas_conf_update_rate_hz;
    uint8_t   imu_conf_type;
    uint8_t   imu_conf_mode;
    uint8_t   imu_conf_filter;
    float     imu_conf_accel_lowpass_filter_x;
    float     imu_conf_accel_lowpass_filter_y;
    float     imu_conf_accel_lowpass_filter_z;
    float     imu_conf_gyro_lowpass_filter;
    uint16_t  imu_conf_sample_rate_hz;
    uint8_t   imu_conf_use_magnetometer;
    float     imu_conf_accel_confidence_decay;
    float     imu_conf_mahony_kp;
    float     imu_conf_mahony_ki;
    float     imu_conf_madgwick_beta;
    float     imu_conf_rot_roll;
    float     imu_conf_rot_pitch;
    float     imu_conf_rot_yaw;
    float     imu_conf_accel_offsets_0;
    float     imu_conf_accel_offsets_1;
    float     imu_conf_accel_offsets_2;
    float     imu_conf_gyro_offsets_0;
    float     imu_conf_gyro_offsets_1;
    float     imu_conf_gyro_offsets_2;
} vesc_appconf_t;

#define VESC700_APPCONF_SIGNATURE 2384027283u
#define RESCP_APPCONF_SIGNATURE   1373548469u
#define VESC700_APPCONF_SER_SIZE 284
#define RESCP_APPCONF_SER_SIZE   196
#ifdef RESC_CONF_PROFILE_RESC
#define VESC_APPCONF_SIGNATURE RESCP_APPCONF_SIGNATURE
#else
#define VESC_APPCONF_SIGNATURE VESC700_APPCONF_SIGNATURE
#endif
#define VESC_APPCONF_SER_MAX 284

int32_t vesc700_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c);
bool    vesc700_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c);
int32_t rescp_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c);
bool    rescp_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c);
int32_t vesc_appconf_serialize(uint8_t *buf, const vesc_appconf_t *c);
bool    vesc_appconf_deserialize(const uint8_t *buf, int32_t len, vesc_appconf_t *c);
void    vesc_appconf_defaults(vesc_appconf_t *c);
extern const conf_field_t vesc_appconf_fields[];
extern const uint16_t vesc_appconf_nfields;

#endif
