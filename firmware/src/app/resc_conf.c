/*
 * resc_conf.c — Stockage, bornage et traduction de la configuration.
 */
#include <string.h>
#include "resc_conf.h"
#include "vesc_compat_defaults.h"
#include "hw_config.h"
#include "fmath.h"
#include <math.h>

vesc_mcconf_t  g_mcconf;
vesc_appconf_t g_appconf;

#define CONF_MAGIC    0x52455343u     /* 'RESC' : ancien format (serialisation 7.00) */
#define SLOT_MC       0
#define SLOT_APP      1

typedef struct {
    uint32_t magic;
    uint32_t len;
    uint32_t crc;
    uint32_t reserved;
} conf_hdr_t;

static uint32_t crc32(const uint8_t *d, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(c & 1));
    }
    return ~c;
}

/* --------------------------------------------------------- statut */
float resc_current_cap_frac(void)
{
#if BOARD_STATUS == HW_BOARD_STATUS_VALIDATED
    return 1.0f;
#else
    return HW_UNREVIEWED_I_FRACTION;
#endif
}

bool resc_power_allowed(void)
{
#if BOARD_STATUS == HW_BOARD_STATUS_IMPORTED && !defined(RESC_ALLOW_UNREVIEWED_BOARD)
    return false;
#else
    return true;
#endif
}

uint8_t resc_controller_id(void) { return (uint8_t)g_appconf.controller_id; }

/* Parametres moteur egaux aux defauts de reference = jamais identifies */
bool resc_motor_configured(void)
{
    const vesc_mcconf_t *c = &g_mcconf;
    if (c->foc_motor_kind == 1)
        return c->foc_motor_r > 0.0f && c->foc_motor_l > 0.0f && c->foc_acim_lm > 0.0f && c->foc_acim_lr > c->foc_acim_lm &&
               c->foc_acim_rr > 0.0f && c->foc_acim_im > 0.1f;
    bool is_default = c->foc_motor_r == (float)MCCONF_FOC_MOTOR_R &&
                      c->foc_motor_l == (float)MCCONF_FOC_MOTOR_L &&
                      c->foc_motor_flux_linkage == (float)MCCONF_FOC_MOTOR_FLUX_LINKAGE;
    return !is_default && c->foc_motor_r > 0.0f && c->foc_motor_l > 0.0f && c->foc_motor_flux_linkage > 0.0f;
}

/* --------------------------------------------------------- valeurs derivees */
void resc_conf_derive_current_gains(vesc_mcconf_t *c, float bw)
{
    if (!(c->foc_motor_r > 0.0f && c->foc_motor_l > 0.0f)) return;
    if (!(bw > 10.0f)) bw = c->foc_resc_cc_bw > 10.0f ? c->foc_resc_cc_bw : RESC_CURRENT_BW_DEFAULT;
    c->foc_current_kp = c->foc_motor_l * bw;
    c->foc_current_ki = c->foc_motor_r * bw;
}

void resc_conf_derive_observer_gain(vesc_mcconf_t *c)
{
    float lam = c->foc_motor_flux_linkage;
    if (c->foc_motor_kind == 1 && c->foc_acim_lr > 0.0f)
        lam = c->foc_acim_lm * c->foc_acim_lm / c->foc_acim_lr * c->foc_acim_im;   /* flux actif nominal */
    if (lam > 1e-6f) c->foc_observer_gain = 1e3f / (lam * lam);
}

/* --------------------------------------------------------- defauts */
void resc_conf_defaults_mc(vesc_mcconf_t *c)
{
    vesc_mcconf_defaults(c);
    resc_conf_clamp_mc(c);
}

void resc_conf_defaults_app(vesc_appconf_t *c)
{
    vesc_appconf_defaults(c);
    resc_conf_clamp_app(c);
}

/* --------------------------------------------------------- bornage */
/* Remplace toute valeur non finie (NaN, infini) d'un champ utilise par sa valeur par defaut */
static void sanitize_mc(vesc_mcconf_t *c)
{
    static vesc_mcconf_t d;
    static bool have_d;
    if (!have_d) { vesc_mcconf_defaults(&d); have_d = true; }
#define FIN(f) do { if (!isfinite(c->f)) c->f = d.f; } while (0)
    FIN(l_current_max); FIN(l_current_min); FIN(l_in_current_max); FIN(l_in_current_min);
    FIN(l_abs_current_max); FIN(l_min_erpm); FIN(l_max_erpm); FIN(l_erpm_start);
    FIN(l_min_vin); FIN(l_max_vin); FIN(l_battery_cut_start); FIN(l_battery_cut_end);
    FIN(l_battery_regen_cut_start); FIN(l_battery_regen_cut_end);
    FIN(l_temp_fet_start); FIN(l_temp_fet_end); FIN(l_temp_motor_start); FIN(l_temp_motor_end);
    FIN(l_min_duty); FIN(l_max_duty); FIN(l_watt_max); FIN(l_watt_min);
    FIN(l_current_max_scale); FIN(l_current_min_scale);
    FIN(foc_current_kp); FIN(foc_current_ki); FIN(foc_f_zv); FIN(foc_dt_us);
    FIN(foc_pll_kp); FIN(foc_pll_ki); FIN(foc_motor_l); FIN(foc_motor_ld_lq_diff);
    FIN(foc_motor_r); FIN(foc_motor_flux_linkage); FIN(foc_observer_gain);
    FIN(foc_duty_dowmramp_kp); FIN(foc_duty_dowmramp_ki);
    FIN(foc_openloop_rpm); FIN(foc_openloop_rpm_low);
    FIN(foc_sl_openloop_time_lock); FIN(foc_sl_openloop_time_ramp);
    FIN(foc_sl_openloop_boost_q); FIN(foc_sl_openloop_max_q); FIN(foc_sl_erpm);
    FIN(foc_hall_interp_erpm); FIN(foc_encoder_offset); FIN(foc_encoder_ratio);
    FIN(foc_hall_table_0); FIN(foc_hall_table_1); FIN(foc_hall_table_2); FIN(foc_hall_table_3);
    FIN(foc_hall_table_4); FIN(foc_hall_table_5); FIN(foc_hall_table_6); FIN(foc_hall_table_7);
    FIN(foc_offsets_current_0); FIN(foc_offsets_current_1); FIN(foc_offsets_current_2);
    FIN(foc_offsets_voltage_0); FIN(foc_offsets_voltage_1); FIN(foc_offsets_voltage_2);
    FIN(foc_offsets_voltage_undriven_0); FIN(foc_offsets_voltage_undriven_1); FIN(foc_offsets_voltage_undriven_2);
    FIN(s_pid_kp); FIN(s_pid_ki); FIN(s_pid_kd); FIN(s_pid_kd_filter); FIN(s_pid_min_erpm); FIN(s_pid_ramp_erpms_s);
    FIN(cc_min_current); FIN(m_ntc_motor_beta);
    FIN(foc_observer_gain_slow); FIN(foc_observer_offset); FIN(foc_start_curr_dec); FIN(foc_start_curr_dec_rpm);
    FIN(foc_sl_openloop_hyst); FIN(foc_sl_openloop_time); FIN(foc_sl_erpm_start); FIN(foc_sat_comp);
    FIN(foc_temp_comp_base_temp); FIN(foc_hfi_amb_current); FIN(foc_hfi_amb_tres); FIN(foc_hfi_voltage_start);
    FIN(foc_hfi_voltage_run); FIN(foc_hfi_voltage_max); FIN(foc_hfi_gain); FIN(foc_hfi_max_err); FIN(foc_hfi_hyst);
    FIN(foc_sl_erpm_hfi); FIN(foc_phase_filter_max_erpm); FIN(foc_fw_current_max); FIN(foc_fw_duty_start);
    FIN(foc_fw_ramp_time); FIN(foc_fw_q_current_factor); FIN(m_hall_extra_samples); FIN(l_duty_start);
    FIN(l_temp_accel_dec); FIN(m_batt_filter_const); FIN(m_ptc_motor_coeff); FIN(m_ntcx_ptcx_res);
    FIN(m_ntcx_ptcx_temp_base); FIN(p_pid_kp); FIN(p_pid_ki); FIN(p_pid_kd); FIN(p_pid_kd_filter);
    FIN(p_pid_ang_div); FIN(p_pid_gain_dec_angle); FIN(p_pid_offset);
    FIN(foc_acim_lm); FIN(foc_acim_lr); FIN(foc_acim_rr); FIN(foc_acim_im); FIN(foc_acim_base_erpm);
    FIN(foc_resc_cc_bw); FIN(foc_resc_rl_frac);
    FIN(si_motor_poles); FIN(si_gear_ratio); FIN(si_wheel_diameter); FIN(si_battery_cells);
#undef FIN
    /* parametres physiques : strictement positifs ou nuls */
    if (c->foc_motor_r < 0.0f) c->foc_motor_r = d.foc_motor_r;
    if (c->foc_motor_l < 0.0f) c->foc_motor_l = d.foc_motor_l;
    if (c->foc_motor_flux_linkage < 0.0f) c->foc_motor_flux_linkage = d.foc_motor_flux_linkage;
    if (c->foc_current_kp < 0.0f) c->foc_current_kp = d.foc_current_kp;
    if (c->foc_current_ki < 0.0f) c->foc_current_ki = d.foc_current_ki;
    if (c->foc_observer_gain < 0.0f) c->foc_observer_gain = d.foc_observer_gain;
}

void resc_conf_clamp_mc(vesc_mcconf_t *c)
{
    sanitize_mc(c);
    const float imax = (float)HWLIM_I_MOTOR_A, iin = (float)HWLIM_I_INPUT_A, iabs = (float)HWLIM_I_ABS_TRIP_A;
    c->motor_type = MOTOR_TYPE_FOC;
    c->l_current_max = fm_sat(c->l_current_max, 0.0f, imax);
    c->l_current_min = fm_sat(c->l_current_min, -imax, 0.0f);
    c->l_in_current_max = fm_sat(c->l_in_current_max, 0.0f, iin);
    c->l_in_current_min = fm_sat(c->l_in_current_min, -iin, 0.0f);
    c->l_abs_current_max = fm_sat(c->l_abs_current_max, 1.0f, iabs);
    c->l_max_vin = fm_sat(c->l_max_vin, (float)HWLIM_VBUS_MIN_V + 1.0f, (float)HWLIM_VBUS_MAX_V);
    c->l_min_vin = fm_sat(c->l_min_vin, (float)HWLIM_VBUS_MIN_V, c->l_max_vin - 1.0f);
    c->l_max_erpm = fm_sat(c->l_max_erpm, 0.0f, (float)HWLIM_ERPM);
    c->l_min_erpm = fm_sat(c->l_min_erpm, -(float)HWLIM_ERPM, 0.0f);
    c->l_max_duty = fm_sat(c->l_max_duty, 0.1f, (float)HWLIM_DUTY_MAX_PERMILLE / 1000.0f);
    c->l_temp_fet_end = fm_sat(c->l_temp_fet_end, 0.0f, (float)HWLIM_T_FET_C);
    c->l_temp_fet_start = fm_sat(c->l_temp_fet_start, 0.0f, c->l_temp_fet_end);
    c->foc_f_zv = fm_sat(c->foc_f_zv, 2.0f * (float)HWLIM_F_PWM_MIN_HZ, 2.0f * (float)HWLIM_F_PWM_MAX_HZ);
    c->l_current_max_scale = fm_sat(c->l_current_max_scale, 0.0f, 1.0f);
    c->l_current_min_scale = fm_sat(c->l_current_min_scale, 0.0f, 1.0f);
}

void resc_conf_clamp_app(vesc_appconf_t *c)
{
    if (!isfinite(c->controller_id)) c->controller_id = -1.0f;
    if (!isfinite(c->timeout_brake_current) || c->timeout_brake_current < 0.0f) c->timeout_brake_current = 0.0f;
    if (c->controller_id < 0.0f || c->controller_id > 253.0f) c->controller_id = (float)plat_default_controller_id();
    if (c->can_status_rate_1 > 1000) c->can_status_rate_1 = 1000;
}

/* --------------------------------------------------------- traduction */
static float rpm_to_rad(float erpm) { return erpm * (FM_2PI / 60.0f); }

void resc_conf_build(const vesc_mcconf_t *mc, const vesc_appconf_t *app, foc_params_t *fp, ctrl_limits_t *cl)
{
    float cap = resc_current_cap_frac();
    memset(fp, 0, sizeof(*fp));

    fp->f_pwm = 0.5f * mc->foc_f_zv;
    fp->i_scale = (BOARD_VREF_V / 4095.0f) / (BOARD_CS_SHUNT_OHM * BOARD_CS_AMP_GAIN) * (BOARD_CS_INVERTED ? -1.0f : 1.0f);
    fp->v_scale = (BOARD_VREF_V / 4095.0f) * (BOARD_VBUS_R_TOP_OHM + BOARD_VBUS_R_BOT_OHM) / BOARD_VBUS_R_BOT_OHM;
    fp->vph_scale = (BOARD_VREF_V / 4095.0f) * (BOARD_VPH_R_TOP_OHM + BOARD_VPH_R_BOT_OHM) / BOARD_VPH_R_BOT_OHM;
    fp->cs_lowside = BOARD_CS_TOPOLOGY != HW_CS_INLINE_3;
    fp->cs_two_shunts = BOARD_CS_TOPOLOGY == HW_CS_LOWSIDE_2;
    fp->invert_dir = mc->m_invert_direction != 0;

    fp->r = mc->foc_motor_r;
    fp->lq = mc->foc_motor_l + 0.5f * mc->foc_motor_ld_lq_diff;
    fp->ld = mc->foc_motor_l - 0.5f * mc->foc_motor_ld_lq_diff;
    if (fp->ld < 0.1f * mc->foc_motor_l) fp->ld = 0.1f * mc->foc_motor_l;
    fp->lambda = mc->foc_motor_flux_linkage;
    /* moteur a induction : L = inductance transitoire sigma Ls (mesuree par R/L), sans saillance */
    fp->motor_kind = mc->foc_motor_kind == 1 ? 1 : 0;
    if (fp->motor_kind == 1) {
        fp->ld = fp->lq = mc->foc_motor_l;
        fp->acim_lm = fm_max(mc->foc_acim_lm, 0.0f);
        fp->acim_lr = fm_max(mc->foc_acim_lr, 0.0f);
        fp->acim_rr = fm_max(mc->foc_acim_rr, 0.0f);
        fp->acim_base_omega = mc->foc_acim_base_erpm * (FM_2PI / 60.0f);
        if (fp->acim_lr > 0.0f) fp->lambda = fp->acim_lm * fp->acim_lm / fp->acim_lr * fm_max(mc->foc_acim_im, 0.0f);
    }
    fp->kp = mc->foc_current_kp;
    fp->ki = mc->foc_current_ki;
    fp->obs_gamma = mc->foc_observer_gain;
    fp->pll_kp = mc->foc_pll_kp;
    fp->pll_ki = mc->foc_pll_ki;
    fp->dt_comp = mc->foc_dt_us * 1e-6f;

    fp->m_max = mc->l_max_duty;
    fp->t_win = (float)HWLIM_T_WIN_MIN_NS * 1e-9f;
    if (fp->cs_lowside) {
        /* En SVPWM lineaire, la 2e meilleure fenetre low-side vaut (0.5 - sqrt3/4 * m) Ts :
         * on borne m pour garder en permanence au moins deux phases mesurables. */
        float m_win = (0.5f - fp->t_win * fp->f_pwm) * (4.0f / FM_SQRT3);
        if (m_win < fp->m_max) fp->m_max = m_win;
        if (fp->m_max < 0.3f) fp->m_max = 0.3f;
    }
    fp->delay_comp = 1.0f;

    float i_abs_cap = (float)HWLIM_I_ABS_TRIP_A * cap;
    fp->i_abs_trip = fm_min(mc->l_abs_current_max, i_abs_cap);
    fp->i_abs_slow = mc->l_slow_abs_current != 0;
    fp->vbus_trip_max = mc->l_max_vin;
    fp->vbus_trip_min = mc->l_min_vin;

    float i_max_eff = fm_min(mc->l_current_max, (float)HWLIM_I_MOTOR_A * cap);
    fp->st_i = mc->foc_sl_openloop_boost_q > 0.5f ? mc->foc_sl_openloop_boost_q : 0.3f * i_max_eff;
    if (mc->foc_sl_openloop_max_q > mc->cc_min_current) fp->st_i = fm_min(fp->st_i, mc->foc_sl_openloop_max_q);
    fp->st_i = fm_max(fp->st_i, 1.0f);
    if (fp->motor_kind == 1) {
        fp->acim_im = fm_sat(mc->foc_acim_im, 0.0f, 0.9f * i_max_eff);
        /* demarrage : le courant total (magnetisant + couple) reste dans la limite */
        fp->st_i = fm_min(fp->st_i, fm_sqrt(fm_max(i_max_eff * i_max_eff - fp->acim_im * fp->acim_im, 1.0f)));
    }
    fp->st_t_align = fm_max(mc->foc_sl_openloop_time_lock, 0.2f);
    fp->st_t_ramp = fm_max(mc->foc_sl_openloop_time_ramp, 0.4f);
    fp->st_omega = rpm_to_rad(fm_max(mc->foc_openloop_rpm, 300.0f));
    /* foc_openloop_rpm_low : vitesse de relais a courant nul, en fraction (0 = vitesse fixe) */
    fp->st_low = mc->foc_openloop_rpm_low > 0.05f && mc->foc_openloop_rpm_low < 0.95f ? mc->foc_openloop_rpm_low : 0.0f;
    fp->st_i_ref = i_max_eff;
    fp->st_omega_exit = 0.5f * fp->st_omega * (fp->st_low > 0.0f ? fp->st_low : 1.0f);
    fp->st_t_hold = fm_sat(mc->foc_sl_openloop_time, 0.0f, 5.0f);
    fp->st_hyst = fm_sat(mc->foc_sl_openloop_hyst, 0.0f, 2.0f);
    fp->st_dec = fm_sat(mc->foc_start_curr_dec, 0.0f, 1.0f);
    fp->st_dec_omega = rpm_to_rad(fm_max(mc->foc_start_curr_dec_rpm, 0.0f));
    fp->st_timeout = fp->st_t_align + fp->st_t_ramp + fp->st_t_hold + 3.0f;
    fp->i_slew = 5000.0f;

    /* Observateur et compensations */
    fp->obs_type = mc->foc_observer_type;
    fp->obs_gain_slow = fm_sat(mc->foc_observer_gain_slow, 0.0f, 1.0f);
#ifdef RESC_CONF_PROFILE_RESC
    fp->obs_offset = fm_sat(mc->foc_observer_offset, -3.0f, 3.0f);
#else
    /* profil VESC : -1 (valeur par defaut de la reference) = pas de correction supplementaire */
    fp->obs_offset = fm_sat(mc->foc_observer_offset + 1.0f, -3.0f, 3.0f);
#endif
    fp->speed_source = mc->foc_speed_soure;
    fp->sat_comp_mode = mc->foc_sat_comp_mode & 3u;
    fp->sat_comp = fm_sat(mc->foc_sat_comp, 0.0f, 0.7f);
    fp->i_sat_ref = i_max_eff;
    fp->temp_comp = mc->foc_temp_comp != 0 && BOARD_TMOT_CH != HW_ADC_CH_NONE && mc->m_motor_temp_sens_type != TEMP_SENSOR_DISABLED;
    fp->temp_comp_base = mc->foc_temp_comp_base_temp;
#ifdef RESC_CONF_PROFILE_RESC
    fp->decoupling = mc->foc_cc_decoupling & 3u;
#else
    /* profil VESC : la valeur par defaut de la reference est 'desactive' ; RESC garde au minimum
     * l'anticipation de FEM, sans laquelle les reprises (relais, reprise au vol) sont moins douces */
    fp->decoupling = (mc->foc_cc_decoupling & 3u) | 2u;
#endif
    fp->mtpa_mode = mc->foc_mtpa_mode <= 2 ? mc->foc_mtpa_mode : 0;
    fp->fw_i_max = fm_sat(mc->foc_fw_current_max, 0.0f, i_max_eff);
    fp->fw_duty_start = fm_sat(mc->foc_fw_duty_start, 0.3f, 0.99f);
    fp->fw_ramp_time = fm_max(mc->foc_fw_ramp_time, 0.0f);
    fp->fw_q_factor = fm_sat(mc->foc_fw_q_current_factor, 0.0f, 1.0f);
    fp->duty_dr_kp = fm_max(mc->foc_duty_dowmramp_kp, 0.0f);
    fp->duty_dr_ki = fm_max(mc->foc_duty_dowmramp_ki, 0.0f);

    /* Filtre de phase (si la carte en a un) */
    fp->pf_enable = mc->foc_phase_filter_enable != 0 && BOARD_PHASE_FILTER_PIN != HW_PIN_NONE;
    fp->pf_no_fault = mc->foc_phase_filter_disable_fault != 0;
    fp->pf_omega = rpm_to_rad(fm_max(mc->foc_phase_filter_max_erpm, 0.0f));

    /* Injection haute frequence (HFI, VSS) */
    {
        static const uint16_t ns[3] = { 8, 16, 32 };
        hfi_cfg_t *h = &fp->hfi;
        h->v_start = fm_sat(mc->foc_hfi_voltage_start, 0.5f, 60.0f);
        h->v_run = fm_sat(mc->foc_hfi_voltage_run, 0.5f, 60.0f);
        h->v_max = fm_sat(mc->foc_hfi_voltage_max, h->v_run, 60.0f);
        h->gain = fm_sat(mc->foc_hfi_gain, 0.01f, 1.0f);
        h->max_err = fm_sat(mc->foc_hfi_max_err, 0.05f, 1.5f);
        h->samples = ns[mc->foc_hfi_samples < 3 ? mc->foc_hfi_samples : 1];
        h->start_updates = mc->foc_hfi_start_samples;
        h->amb_mode = mc->foc_hfi_amb_mode;
        h->amb_current = fm_sat(mc->foc_hfi_amb_current, 0.5f, i_max_eff);
        h->amb_time = fm_sat(mc->foc_hfi_amb_tres, 2.0f, 200.0f) * 1e-3f;
        h->sl_omega = rpm_to_rad(fm_max(mc->foc_sl_erpm_hfi, 100.0f));
        h->hyst = fm_sat(mc->foc_hfi_hyst, 0.0f, 0.5f);
        h->i_ref = i_max_eff;
    }

    /* Capteurs d'angle */
    {
        const float tab[8] = { mc->foc_hall_table_0, mc->foc_hall_table_1, mc->foc_hall_table_2, mc->foc_hall_table_3,
                               mc->foc_hall_table_4, mc->foc_hall_table_5, mc->foc_hall_table_6, mc->foc_hall_table_7 };
        int nvalid = 0;
        for (int s = 0; s < 8; s++) {
            fp->hall.valid[s] = tab[s] >= 0.0f && tab[s] <= 200.0f;
            fp->hall.angle[s] = fp->hall.valid[s] ? fm_wrap_pi(tab[s] * (FM_2PI / 200.0f)) : 0.0f;
            nvalid += fp->hall.valid[s] ? 1 : 0;
        }
        fp->hall.interp_omega = rpm_to_rad(mc->foc_hall_interp_erpm);
        fp->sl_omega = rpm_to_rad(mc->foc_sl_erpm);
        fp->sl_omega_start = fm_min(rpm_to_rad(mc->foc_sl_erpm_start), fp->sl_omega);
        fp->hall_extra = (uint8_t)fm_sat(mc->m_hall_extra_samples, 0.0f, 10.0f);
        fp->enc.cpr = mc->m_encoder_counts;
        fp->enc.ratio = mc->foc_encoder_ratio;
        fp->enc.offset = fm_wrap_pi(mc->foc_encoder_offset * (FM_PI / 180.0f));
        fp->enc.inverted = mc->foc_encoder_inverted != 0;
        /* moteur a induction : codeur ou sans capteur (pas de sondes Hall ni d'injection) */
        uint8_t smode = mc->foc_sensor_mode;
        if (mc->foc_motor_kind == 1 && smode != FOC_SENSOR_MODE_ENCODER && smode != FOC_SENSOR_MODE_ENCODER_AB) smode = 0;
        switch (smode) {
        case FOC_SENSOR_MODE_HALL:
            /* table incomplete : jamais d'angle faux, on reste en sensorless */
            fp->sensor_mode = nvalid == 6 ? FOC_SENSOR_HALL : FOC_SENSOR_SENSORLESS;
            break;
        case FOC_SENSOR_MODE_ENCODER:
        case FOC_SENSOR_MODE_ENCODER_AB:
            fp->sensor_mode = (fp->enc.cpr >= 4 && fp->enc.ratio > 0.5f) ? FOC_SENSOR_ENCODER : FOC_SENSOR_SENSORLESS;
            break;
        case FOC_SENSOR_MODE_HFI:
        case FOC_SENSOR_MODE_HFI_V2:
        case FOC_SENSOR_MODE_HFI_V3:
        case FOC_SENSOR_MODE_HFI_V4:
        case FOC_SENSOR_MODE_HFI_V5:
            /* une seule methode HFI dans RESC pour toutes les variantes */
            fp->sensor_mode = FOC_SENSOR_HFI;
            break;
        case FOC_SENSOR_MODE_HFI_START:     /* affiche 'VSS' */
            fp->sensor_mode = FOC_SENSOR_VSS;
            break;
        default:
            fp->sensor_mode = FOC_SENSOR_SENSORLESS;
            break;
        }
    }

    /* Offsets : calibration au demarrage (bit 0) ou valeurs de la calibration manuelle */
    {
        const float oc[3] = { mc->foc_offsets_current_0, mc->foc_offsets_current_1, mc->foc_offsets_current_2 };
        bool ok = true;
        for (int x = 0; x < 3; x++) {
            fp->stored_off_cs[x] = oc[x];
            if (oc[x] < 1448.0f || oc[x] > 2648.0f) ok = false;
        }
        fp->use_stored_offsets = ok && !(mc->foc_offsets_cal_mode & 1u);
        const float ov[3] = { mc->foc_offsets_voltage_undriven_0, mc->foc_offsets_voltage_undriven_1,
                              mc->foc_offsets_voltage_undriven_2 };
        for (int x = 0; x < 3; x++) fp->off_vph[x] = (ov[x] > -2.0f && ov[x] < 2.0f) ? ov[x] : 0.0f;
    }

    memset(cl, 0, sizeof(*cl));
    cl->i_max = i_max_eff;
    cl->i_min = fm_max(mc->l_current_min, -(float)HWLIM_I_MOTOR_A * cap);
    cl->i_max_scale = mc->l_current_max_scale;
    cl->i_min_scale = mc->l_current_min_scale;
    cl->i_in_max = fm_min(mc->l_in_current_max, (float)HWLIM_I_INPUT_A * cap);
    cl->i_in_min = fm_max(mc->l_in_current_min, -(float)HWLIM_I_INPUT_A * cap);
    cl->watt_max = mc->l_watt_max;
    cl->watt_min = mc->l_watt_min;
    cl->cc_min_current = mc->cc_min_current;
    cl->erpm_max = mc->l_max_erpm;
    cl->erpm_min = mc->l_min_erpm;
    cl->erpm_start = fm_sat(mc->l_erpm_start, 0.1f, 1.0f);
    cl->batt_cut_start = mc->l_battery_cut_start;
    cl->batt_cut_end = mc->l_battery_cut_end;
    cl->regen_cut_start = mc->l_battery_regen_cut_start;
    cl->regen_cut_end = mc->l_battery_regen_cut_end;
    cl->tfet_start = mc->l_temp_fet_start;
    cl->tfet_end = mc->l_temp_fet_end;
    cl->tmot_start = mc->l_temp_motor_start;
    cl->tmot_end = mc->l_temp_motor_end;
    cl->tmot_enabled = mc->m_motor_temp_sens_type != TEMP_SENSOR_DISABLED && BOARD_TMOT_CH != HW_ADC_CH_NONE;
    cl->duty_max = mc->l_max_duty;
    cl->duty_min = mc->l_min_duty;
    cl->s_kp = mc->s_pid_kp;
    cl->s_ki = mc->s_pid_ki;
    cl->s_kd = mc->s_pid_kd;
    cl->s_kd_filter = mc->s_pid_kd_filter;
    cl->s_min_erpm = mc->s_pid_min_erpm;
    cl->s_ramp_erpms = mc->s_pid_ramp_erpms_s;
    cl->s_allow_braking = mc->s_pid_allow_braking != 0;
    cl->timeout_ms = app->timeout_msec;
    cl->timeout_brake = app->timeout_brake_current;
    cl->start_erpm_exit = fp->st_omega_exit * (60.0f / FM_2PI);
    cl->duty_start = fm_sat(mc->l_duty_start, 0.0f, 1.0f);
    cl->temp_accel_dec = fm_sat(mc->l_temp_accel_dec, 0.0f, 1.0f);
    cl->fault_stop_ms = mc->m_fault_stop_time_ms > 0 ? (uint32_t)mc->m_fault_stop_time_ms : 0u;
    cl->add_faults = mc->l_additional_faults;
    cl->batt_tau = fm_sat(mc->m_batt_filter_const, 0.0f, 1000.0f) * 1e-3f;
    {
        /* sp_pid_loop_rate : 25 .. 10000 Hz ; la couche de controle tourne a 1 kHz au plus */
        static const uint16_t hz[9] = { 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000 };
        uint16_t f = hz[mc->sp_pid_loop_rate < 9 ? mc->sp_pid_loop_rate : 5];
        cl->s_rate_div = (uint8_t)(f >= 1000 ? 1 : 1000 / f);
    }
    cl->s_speed_src = mc->s_pid_speed_source <= 2 ? mc->s_pid_speed_source : 0;
    cl->p_kp = mc->p_pid_kp;
    cl->p_ki = mc->p_pid_ki;
    cl->p_kd = mc->p_pid_kd;
    cl->p_kd_filter = mc->p_pid_kd_filter;
    cl->p_ang_div = mc->p_pid_ang_div;
    cl->p_gain_dec = mc->p_pid_gain_dec_angle;
    cl->p_offset = mc->p_pid_offset;
    cl->pole_pairs = mc->si_motor_poles >= 2 ? 0.5f * (float)mc->si_motor_poles : 7.0f;
}

/* Fourni par la plateforme : changement de frequence PWM (pont coupe) */
void plat_pwm_set_freq(float f_pwm);
void plat_sensor_config(uint8_t mode, uint32_t cpr);
void plat_hall_samples(uint8_t extra);

static volatile bool s_apply_pending;

static bool motor_busy(void)
{
    foc_state_t s = foc_state();
    return s == FOC_ST_RUN || s == FOC_ST_START || s == FOC_ST_DETECT;
}

/* Les limites (bornes de courant, tension, ERPM...) s'appliquent tout de suite.
 * Les parametres qui changent la structure de la commande (frequence PWM, source
 * d'angle, parametres du coeur FOC) ne sont appliques que moteur arrete, en une
 * seule fois : sinon le pas de temps du coeur et la PWM reelle divergeraient. */
void resc_conf_apply(void)
{
    foc_params_t fp;
    ctrl_limits_t cl;
    resc_conf_build(&g_mcconf, &g_appconf, &fp, &cl);
    ctrl_set_limits(&cl);
    if (motor_busy()) {
        s_apply_pending = true;
        return;
    }
    s_apply_pending = false;
    if (fp.f_pwm != foc_params()->f_pwm) plat_pwm_set_freq(fp.f_pwm);
    static uint8_t last_mode = 0xFF;
    static uint32_t last_cpr = 0;
    plat_hall_samples(fp.hall_extra);
    if (fp.sensor_mode != last_mode || fp.enc.cpr != last_cpr) {
        plat_sensor_config(fp.sensor_mode, fp.enc.cpr);
        last_mode = fp.sensor_mode;
        last_cpr = fp.enc.cpr;
    }
    foc_set_params(&fp);
}

bool resc_conf_apply_pending(void) { return s_apply_pending; }

void resc_conf_poll(void)
{
    if (s_apply_pending && !motor_busy()) resc_conf_apply();
}

/* --------------------------------------------------------- flash
 * Format de stockage (independant du profil et de la version) : en-tete, puis un
 * enregistrement par champ : [crc32 du nom u32][type u8][valeur 4 octets], petit-boutiste.
 * A la lecture, chaque champ connu est recopie (converti si son type a change), les
 * champs nouveaux gardent leur defaut, les champs disparus sont ignores : la
 * configuration survit aux changements de profil et de version.
 * Ancien format (RESC 0.x, serialisation VESC 7.00 avec signature) : relu une fois avec
 * le parseur 7.00, puis reecrit dans le format par nom. */
#define CONF_MAGIC_V2  0x32534552u     /* 'RES2' */
#define REC_SIZE       9u

static uint8_t s_buf[16 + REC_SIZE * 320u];
static resc_conf_load_t s_load_mc, s_load_app;

resc_conf_load_t resc_conf_load_status(bool app) { return app ? s_load_app : s_load_mc; }

static float field_get(const void *conf, const conf_field_t *f)
{
    const uint8_t *p = (const uint8_t *)conf + f->offset;
    switch (f->ctype) {
    case 0: { float v; memcpy(&v, p, 4); return v; }
    case 1: return (float)*p;
    case 2: return (float)(int8_t)*p;
    case 3: { uint16_t v; memcpy(&v, p, 2); return (float)v; }
    case 4: { int16_t v; memcpy(&v, p, 2); return (float)v; }
    case 5: { uint32_t v; memcpy(&v, p, 4); return (float)v; }
    default: { int32_t v; memcpy(&v, p, 4); return (float)v; }
    }
}

static void field_set(void *conf, const conf_field_t *f, float v)
{
    uint8_t *p = (uint8_t *)conf + f->offset;
    switch (f->ctype) {
    case 0: memcpy(p, &v, 4); break;
    case 1: *p = (uint8_t)fm_sat(v, 0.0f, 255.0f); break;
    case 2: *p = (uint8_t)(int8_t)fm_sat(v, -128.0f, 127.0f); break;
    case 3: { uint16_t x = (uint16_t)fm_sat(v, 0.0f, 65535.0f); memcpy(p, &x, 2); break; }
    case 4: { int16_t x = (int16_t)fm_sat(v, -32768.0f, 32767.0f); memcpy(p, &x, 2); break; }
    case 5: { uint32_t x = (uint32_t)fm_max(v, 0.0f); memcpy(p, &x, 4); break; }
    default: { int32_t x = (int32_t)v; memcpy(p, &x, 4); break; }
    }
}

/* Recherche d'un champ par nom (terminal) */
const conf_field_t *resc_conf_field(bool app, const char *name)
{
    const conf_field_t *t = app ? vesc_appconf_fields : vesc_mcconf_fields;
    uint16_t n = app ? vesc_appconf_nfields : vesc_mcconf_nfields;
    for (uint16_t k = 0; k < n; k++)
        if (!strcmp(t[k].name, name)) return &t[k];
    return NULL;
}
float resc_conf_field_get(bool app, const conf_field_t *f) { return field_get(app ? (void *)&g_appconf : (void *)&g_mcconf, f); }
void resc_conf_field_set(bool app, const conf_field_t *f, float v) { field_set(app ? (void *)&g_appconf : (void *)&g_mcconf, f, v); }

static bool store(uint32_t slot, const void *conf, const conf_field_t *t, uint16_t n)
{
    if (foc_state() == FOC_ST_RUN || foc_state() == FOC_ST_START || foc_state() == FOC_ST_DETECT) return false;
    uint8_t *d = s_buf + sizeof(conf_hdr_t);
    for (uint16_t k = 0; k < n; k++) {
        const uint8_t *p = (const uint8_t *)conf + t[k].offset;
        uint8_t raw[4] = { 0, 0, 0, 0 };
        uint8_t sz = t[k].ctype == 0 || t[k].ctype >= 5 ? 4 : (t[k].ctype >= 3 ? 2 : 1);
        memcpy(raw, p, sz);
        memcpy(d, &t[k].hash, 4);
        d[4] = t[k].ctype;
        memcpy(d + 5, raw, 4);
        d += REC_SIZE;
    }
    conf_hdr_t h;
    h.magic = CONF_MAGIC_V2;
    h.len = (uint32_t)n * REC_SIZE;
    h.crc = crc32(s_buf + sizeof(h), h.len);
    h.reserved = slot;
    memcpy(s_buf, &h, sizeof(h));
    return plat_conf_write(slot, s_buf, h.len + sizeof(h));
}

bool resc_conf_store_mc(void) { return store(SLOT_MC, &g_mcconf, vesc_mcconf_fields, vesc_mcconf_nfields); }
bool resc_conf_store_app(void) { return store(SLOT_APP, &g_appconf, vesc_appconf_fields, vesc_appconf_nfields); }

/* Lit un emplacement : format par nom ou ancien format. Retourne l'etat de la lecture. */
static resc_conf_load_t load(uint32_t slot, void *conf, const conf_field_t *t, uint16_t n,
                             bool (*old_deser)(const uint8_t *, int32_t, void *))
{
    uint32_t len;
    const uint8_t *p = plat_conf_read(slot, &len);
    if (!p || len < sizeof(conf_hdr_t)) return RESC_LOAD_EMPTY;
    conf_hdr_t h;
    memcpy(&h, p, sizeof(h));
    if (h.magic == 0xFFFFFFFFu) return RESC_LOAD_EMPTY;
    if (h.len > len - sizeof(h) || crc32(p + sizeof(h), h.len) != h.crc) return RESC_LOAD_FAILED;
    const uint8_t *d = p + sizeof(h);
    if (h.magic == CONF_MAGIC_V2) {
        for (uint32_t r = 0; r + REC_SIZE <= h.len; r += REC_SIZE) {
            uint32_t hash;
            memcpy(&hash, d + r, 4);
            uint8_t ct = d[r + 4];
            for (uint16_t k = 0; k < n; k++) {
                if (t[k].hash != hash) continue;
                if (ct == t[k].ctype) {
                    uint8_t sz = ct == 0 || ct >= 5 ? 4 : (ct >= 3 ? 2 : 1);
                    memcpy((uint8_t *)conf + t[k].offset, d + r + 5, sz);
                } else {
                    /* type change : conversion par la valeur */
                    conf_field_t tmp = t[k];
                    tmp.ctype = ct;
                    tmp.offset = 0;
                    uint8_t raw[4];
                    memcpy(raw, d + r + 5, 4);
                    field_set(conf, &t[k], field_get(raw, &tmp));
                }
                break;
            }
        }
        return RESC_LOAD_OK;
    }
    if (h.magic == CONF_MAGIC) {
        /* ancien format : configuration 7.00 serialisee */
        return old_deser(d, (int32_t)h.len, conf) ? RESC_LOAD_MIGRATED : RESC_LOAD_FAILED;
    }
    return RESC_LOAD_FAILED;
}

static bool old_mc(const uint8_t *b, int32_t n, void *c) { return vesc700_mcconf_deserialize(b, n, (vesc_mcconf_t *)c); }
static bool old_app(const uint8_t *b, int32_t n, void *c) { return vesc700_appconf_deserialize(b, n, (vesc_appconf_t *)c); }

void resc_conf_init(void)
{
    resc_conf_defaults_mc(&g_mcconf);
    resc_conf_defaults_app(&g_appconf);
    static vesc_mcconf_t mc;
    static vesc_appconf_t app;
    mc = g_mcconf;
    app = g_appconf;
    s_load_mc = load(SLOT_MC, &mc, vesc_mcconf_fields, vesc_mcconf_nfields, old_mc);
    if (s_load_mc == RESC_LOAD_OK || s_load_mc == RESC_LOAD_MIGRATED) {
        resc_conf_clamp_mc(&mc);
        g_mcconf = mc;
    }
    s_load_app = load(SLOT_APP, &app, vesc_appconf_fields, vesc_appconf_nfields, old_app);
    if (s_load_app == RESC_LOAD_OK || s_load_app == RESC_LOAD_MIGRATED) {
        resc_conf_clamp_app(&app);
        g_appconf = app;
    }
    /* migration : reecriture immediate dans le format par nom */
    if (s_load_mc == RESC_LOAD_MIGRATED) resc_conf_store_mc();
    if (s_load_app == RESC_LOAD_MIGRATED) resc_conf_store_app();
}
