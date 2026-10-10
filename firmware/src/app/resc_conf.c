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

#define CONF_MAGIC    0x52455343u     /* 'RESC' */
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
    bool is_default = c->foc_motor_r == (float)MCCONF_FOC_MOTOR_R &&
                      c->foc_motor_l == (float)MCCONF_FOC_MOTOR_L &&
                      c->foc_motor_flux_linkage == (float)MCCONF_FOC_MOTOR_FLUX_LINKAGE;
    return !is_default && c->foc_motor_r > 0.0f && c->foc_motor_l > 0.0f && c->foc_motor_flux_linkage > 0.0f;
}

/* --------------------------------------------------------- valeurs derivees */
void resc_conf_derive_current_gains(vesc_mcconf_t *c, float bw)
{
    if (!(c->foc_motor_r > 0.0f && c->foc_motor_l > 0.0f)) return;
    if (bw <= 0.0f) bw = RESC_CURRENT_BW_DEFAULT;
    c->foc_current_kp = c->foc_motor_l * bw;
    c->foc_current_ki = c->foc_motor_r * bw;
}

void resc_conf_derive_observer_gain(vesc_mcconf_t *c)
{
    float lam = c->foc_motor_flux_linkage;
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
    fp->vbus_trip_max = mc->l_max_vin;
    fp->vbus_trip_min = mc->l_min_vin;

    float i_max_eff = fm_min(mc->l_current_max, (float)HWLIM_I_MOTOR_A * cap);
    fp->st_i = mc->foc_sl_openloop_boost_q > 0.5f ? mc->foc_sl_openloop_boost_q : 0.3f * i_max_eff;
    if (mc->foc_sl_openloop_max_q > mc->cc_min_current) fp->st_i = fm_min(fp->st_i, mc->foc_sl_openloop_max_q);
    fp->st_i = fm_max(fp->st_i, 1.0f);
    fp->st_t_align = fm_max(mc->foc_sl_openloop_time_lock, 0.2f);
    fp->st_t_ramp = fm_max(mc->foc_sl_openloop_time_ramp, 0.4f);
    fp->st_omega = rpm_to_rad(fm_max(mc->foc_openloop_rpm, 300.0f));
    float low = mc->foc_openloop_rpm_low > 0.05f && mc->foc_openloop_rpm_low < 0.95f ? mc->foc_openloop_rpm_low : 0.5f;
    fp->st_omega_exit = low * fp->st_omega;
    fp->st_timeout = fp->st_t_align + fp->st_t_ramp + 3.0f;
    fp->i_slew = 5000.0f;

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
        fp->enc.cpr = mc->m_encoder_counts;
        fp->enc.ratio = mc->foc_encoder_ratio;
        fp->enc.offset = fm_wrap_pi(mc->foc_encoder_offset * (FM_PI / 180.0f));
        fp->enc.inverted = mc->foc_encoder_inverted != 0;
        switch (mc->foc_sensor_mode) {
        case FOC_SENSOR_MODE_HALL:
            /* table incomplete : jamais d'angle faux, on reste en sensorless */
            fp->sensor_mode = nvalid == 6 ? FOC_SENSOR_HALL : FOC_SENSOR_SENSORLESS;
            break;
        case FOC_SENSOR_MODE_ENCODER:
        case FOC_SENSOR_MODE_ENCODER_AB:
            fp->sensor_mode = (fp->enc.cpr >= 4 && fp->enc.ratio > 0.5f) ? FOC_SENSOR_ENCODER : FOC_SENSOR_SENSORLESS;
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
    cl->tmot_enabled = mc->m_motor_temp_sens_type == TEMP_SENSOR_NTC_10K_25C && BOARD_TMOT_CH != HW_ADC_CH_NONE;
    cl->duty_max = mc->l_max_duty;
    cl->duty_min = mc->l_min_duty;
    cl->duty_kp = mc->foc_duty_dowmramp_kp;
    cl->duty_ki = mc->foc_duty_dowmramp_ki;
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
}

/* Fourni par la plateforme : changement de frequence PWM (pont coupe) */
void plat_pwm_set_freq(float f_pwm);
void plat_sensor_config(uint8_t mode, uint32_t cpr);

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

/* --------------------------------------------------------- flash */
static uint8_t s_buf[VESC_MCCONF_SER_MAX > VESC_APPCONF_SER_MAX ? VESC_MCCONF_SER_MAX + 16 : VESC_APPCONF_SER_MAX + 16];

static bool store(uint32_t slot, int32_t (*ser)(uint8_t *, const void *), const void *conf)
{
    if (foc_state() == FOC_ST_RUN || foc_state() == FOC_ST_START || foc_state() == FOC_ST_DETECT) return false;
    conf_hdr_t h;
    int32_t n = ser(s_buf + sizeof(h), conf);
    h.magic = CONF_MAGIC;
    h.len = (uint32_t)n;
    h.crc = crc32(s_buf + sizeof(h), (uint32_t)n);
    h.reserved = slot;
    memcpy(s_buf, &h, sizeof(h));
    return plat_conf_write(slot, s_buf, (uint32_t)n + sizeof(h));
}

static int32_t ser_mc(uint8_t *b, const void *c) { return vesc_mcconf_serialize(b, (const vesc_mcconf_t *)c); }
static int32_t ser_app(uint8_t *b, const void *c) { return vesc_appconf_serialize(b, (const vesc_appconf_t *)c); }

bool resc_conf_store_mc(void) { return store(SLOT_MC, ser_mc, &g_mcconf); }
bool resc_conf_store_app(void) { return store(SLOT_APP, ser_app, &g_appconf); }

static const uint8_t *load(uint32_t slot, uint32_t *len)
{
    uint32_t n;
    const uint8_t *p = plat_conf_read(slot, &n);
    if (!p || n < sizeof(conf_hdr_t)) return NULL;
    conf_hdr_t h;
    memcpy(&h, p, sizeof(h));
    if (h.magic != CONF_MAGIC || h.len > n - sizeof(h) || crc32(p + sizeof(h), h.len) != h.crc) return NULL;
    *len = h.len;
    return p + sizeof(h);
}

void resc_conf_init(void)
{
    resc_conf_defaults_mc(&g_mcconf);
    resc_conf_defaults_app(&g_appconf);
    uint32_t n;
    const uint8_t *p = load(SLOT_MC, &n);
    vesc_mcconf_t mc;
    if (p && vesc_mcconf_deserialize(p, (int32_t)n, &mc)) {
        resc_conf_clamp_mc(&mc);
        g_mcconf = mc;
    }
    p = load(SLOT_APP, &n);
    vesc_appconf_t app;
    if (p && vesc_appconf_deserialize(p, (int32_t)n, &app)) {
        resc_conf_clamp_app(&app);
        g_appconf = app;
    }
}
