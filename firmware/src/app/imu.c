/*
 * imu.c — Centrale inertielle BMI160 (registres du constructeur), attitude et calibration.
 *
 * BMI160 : identifiant 0xD1 (registre 0x00), adresse I2C 0x68 (0x69 si SDO a 1). Accelerometre
 * +-16 g, gyrometre +-2000 deg/s ; cadence interne (ODR) au-dessus de la cadence demandee et
 * filtre interne selon imu_conf.filter (bas : moyenne de 4, moyen : OSR2 et ODR x2, haut : OSR4
 * et ODR x4), comme le firmware VESC.
 */
#include <math.h>
#include <string.h>
#include "imu.h"
#include "ahrs.h"
#include "i2c_bb.h"
#include "resc_conf.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "hw_config.h"
#include "stm32.h"
#include "systime.h"

#define BMI_ADDR0       0x68
#define REG_CHIP_ID     0x00
#define REG_DATA_GYR    0x0C            /* gx, gy, gz, ax, ay, az : 12 octets */
#define REG_ACC_CONF    0x40
#define REG_ACC_RANGE   0x41
#define REG_GYR_CONF    0x42
#define REG_GYR_RANGE   0x43
#define REG_CMD         0x7E

static struct {
    bool on, ok;
    i2c_bb_t bus;
    uint8_t addr;
    uint32_t period_us, next_us, last_cyc;
    uint32_t t_start_ms;
    bool ready;
    uint8_t mode;
    float rot[3];                      /* roulis, tangage, lacet (rad) */
    float acc_off[3], gyr_off[3];
    float decay, kp, ki, beta;
    biquad_t fa[3], fg;
    biquad_t fg3[3];
    bool fa_on[3], fg_on;
    float acc[3], gyr[3];
    ahrs_t att;
    uint32_t err;
} I;

/* calibration */
static struct {
    int stage;                         /* 0 inactive */
    const comm_iface_t *reply;
    float yaw;
    uint32_t t_ms;
    int n;
    float acc[3];
    float g0[3];
    /* reglages sauvegardes */
    uint8_t mode;
    float rot[3], acc_off[3], gyr_off[3], decay, kp, ki, beta;
} C;

static bool wr(uint8_t reg, uint8_t v)
{
    uint8_t t[2] = { reg, v };
    return i2c_bb_txrx(&I.bus, I.addr, t, 2, 0, 0);
}

static bool rd(uint8_t reg, uint8_t *d, int n) { return i2c_bb_txrx(&I.bus, I.addr, &reg, 1, d, n); }

static void wait_ms(uint32_t ms) { uint32_t t = systime_ms(); while (systime_ms() - t < ms) { } }

static bool bmi_init(uint16_t rate_hz, uint8_t filter)
{
    uint8_t id = 0;
    I.addr = BMI_ADDR0;
    if (!rd(REG_CHIP_ID, &id, 1) || id != 0xD1) {
        I.addr = BMI_ADDR0 + 1;
        if (!rd(REG_CHIP_ID, &id, 1) || id != 0xD1) return false;
    }
    wr(REG_CMD, 0xB6);                 /* remise a zero logicielle */
    wait_ms(20);
    wr(REG_CMD, 0x11);                 /* accelerometre : mode normal */
    wait_ms(5);
    wr(REG_CMD, 0x15);                 /* gyrometre : mode normal */
    wait_ms(80);
    /* ODR : 6 = 25 Hz ... 12 = 1600 Hz (gyro jusqu'a 13 = 3200 Hz) */
    uint8_t odr = rate_hz <= 25 ? 6 : rate_hz <= 50 ? 7 : rate_hz <= 100 ? 8 : rate_hz <= 200 ? 9 :
                  rate_hz <= 400 ? 10 : rate_hz <= 800 ? 11 : 12;
    uint8_t bw = 2, oa = odr, og = odr;          /* bas : normal / moyenne de 4 */
    if (filter == 1) { bw = 1; oa = (uint8_t)(odr + 1 > 12 ? 12 : odr + 1); og = (uint8_t)(odr + 1 > 13 ? 13 : odr + 1); }
    if (filter == 2) { bw = 0; oa = (uint8_t)(odr + 2 > 12 ? 12 : odr + 2); og = (uint8_t)(odr + 2 > 13 ? 13 : odr + 2); }
    bool ok = wr(REG_ACC_CONF, (uint8_t)(oa | (bw << 4)));
    ok &= wr(REG_ACC_RANGE, 0x0C);     /* +-16 g */
    ok &= wr(REG_GYR_CONF, (uint8_t)(og | (bw << 4)));
    ok &= wr(REG_GYR_RANGE, 0x00);     /* +-2000 deg/s */
    return ok;
}

static void apply_params(void)
{
    const vesc_appconf_t *a = &g_appconf;
    I.mode = a->imu_conf_mode <= 2 ? a->imu_conf_mode : 0;
    I.rot[0] = a->imu_conf_rot_roll * (3.14159265f / 180.0f);
    I.rot[1] = a->imu_conf_rot_pitch * (3.14159265f / 180.0f);
    I.rot[2] = a->imu_conf_rot_yaw * (3.14159265f / 180.0f);
    I.acc_off[0] = a->imu_conf_accel_offsets_0; I.acc_off[1] = a->imu_conf_accel_offsets_1; I.acc_off[2] = a->imu_conf_accel_offsets_2;
    I.gyr_off[0] = a->imu_conf_gyro_offsets_0; I.gyr_off[1] = a->imu_conf_gyro_offsets_1; I.gyr_off[2] = a->imu_conf_gyro_offsets_2;
    I.decay = a->imu_conf_accel_confidence_decay;
    I.kp = a->imu_conf_mahony_kp;
    I.ki = a->imu_conf_mahony_ki;
    I.beta = a->imu_conf_madgwick_beta;
    float fs = a->imu_conf_sample_rate_hz > 0 ? (float)a->imu_conf_sample_rate_hz : 200.0f;
    const float fc[3] = { a->imu_conf_accel_lowpass_filter_x, a->imu_conf_accel_lowpass_filter_y,
                          a->imu_conf_accel_lowpass_filter_z };
    for (int k = 0; k < 3; k++) {
        I.fa_on[k] = fc[k] > 0.0f;
        if (I.fa_on[k]) biquad_lowpass(&I.fa[k], fc[k] / fs);
    }
    I.fg_on = a->imu_conf_gyro_lowpass_filter > 0.0f;
    if (I.fg_on) for (int k = 0; k < 3; k++) biquad_lowpass(&I.fg3[k], a->imu_conf_gyro_lowpass_filter / fs);
}

void imu_configure(void)
{
    const vesc_appconf_t *a = &g_appconf;
    I.on = false;
    I.ok = false;
    I.ready = false;
    uint8_t sda = HW_PIN_NONE, scl = HW_PIN_NONE;
    if (a->imu_conf_type == 1) { sda = BOARD_IMU_SDA_PIN; scl = BOARD_IMU_SCL_PIN; }
    else if (a->imu_conf_type == 4 && a->app_to_use != 3 && a->app_to_use != 4 && a->app_to_use != 5) {
        sda = BOARD_I2C_SDA_PIN; scl = BOARD_I2C_SCL_PIN;   /* port UART libre (pas d'application UART) */
    }
    if (sda == HW_PIN_NONE || scl == HW_PIN_NONE) return;
    uint16_t rate = a->imu_conf_sample_rate_hz ? a->imu_conf_sample_rate_hz : 200;
    if (rate > 1000) rate = 1000;
    i2c_bb_init(&I.bus, sda, scl);
    I.ok = bmi_init(rate, a->imu_conf_filter);
    I.on = true;
    I.period_us = 1000000u / rate;
    I.t_start_ms = systime_ms();
    apply_params();
    ahrs_init(&I.att);
    ahrs_params(&I.att, 1.0f, 10.0f, 0.0f, 2.0f);   /* convergence rapide pendant la premiere seconde */
    I.last_cyc = DWT->CYCCNT;
}

bool imu_ready(void) { return I.ready; }
void imu_get_rpy(float rpy[3]) { ahrs_rpy(&I.att, rpy); }
void imu_get_accel(float a[3]) { memcpy(a, I.acc, sizeof(I.acc)); }
void imu_get_gyro(float g[3]) { memcpy(g, I.gyr, sizeof(I.gyr)); }
void imu_get_quat(float q[4]) { q[0] = I.att.q0; q[1] = I.att.q1; q[2] = I.att.q2; q[3] = I.att.q3; }

/* rotation de montage (lacet, tangage, roulis) comme VESC */
static void rotate(const float r[3], const float in[3], float out[3], bool reverse)
{
    float s1 = sinf(r[2]), c1 = cosf(r[2]), s2 = sinf(r[1]), c2 = cosf(r[1]), s3 = sinf(r[0]), c3 = cosf(r[0]);
    float m[3][3] = { { c1 * c2, c1 * s2 * s3 - c3 * s1, s1 * s3 + c1 * c3 * s2 },
                      { c2 * s1, c1 * c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3 },
                      { -s2, c2 * s3, c2 * c3 } };
    for (int i = 0; i < 3; i++)
        out[i] = reverse ? in[0] * m[0][i] + in[1] * m[1][i] + in[2] * m[2][i]
                         : in[0] * m[i][0] + in[1] * m[i][1] + in[2] * m[i][2];
}

static void sample(void)
{
    uint8_t d[12];
    if (!rd(REG_DATA_GYR, d, 12)) { I.err++; return; }
    float g[3], acc[3];
    for (int k = 0; k < 3; k++) {
        int16_t gv = (int16_t)(d[2 * k] | d[2 * k + 1] << 8), av = (int16_t)(d[6 + 2 * k] | d[7 + 2 * k] << 8);
        g[k] = (float)gv * (2000.0f / 32768.0f);
        acc[k] = (float)av * (16.0f / 32768.0f);
    }
    uint32_t cyc = DWT->CYCCNT;
    float dt = (float)(cyc - I.last_cyc) / 168e6f;
    I.last_cyc = cyc;
    if (BOARD_IMU_FLIP) { acc[0] = -acc[0]; acc[2] = -acc[2]; g[0] = -g[0]; g[2] = -g[2]; }
    rotate(I.rot, acc, I.acc, false);
    rotate(I.rot, g, I.gyr, false);
    for (int k = 0; k < 3; k++) {
        I.acc[k] -= I.acc_off[k];
        I.gyr[k] -= I.gyr_off[k];
        if (I.fa_on[k]) I.acc[k] = biquad_run(&I.fa[k], I.acc[k]);
        if (I.fg_on) I.gyr[k] = biquad_run(&I.fg3[k], I.gyr[k]);
    }
    if (!I.ready && systime_ms() - I.t_start_ms > 1000u) {
        ahrs_params(&I.att, I.decay, I.kp, I.ki, I.beta);
        I.ready = true;
    }
    float gr[3] = { I.gyr[0] * (3.14159265f / 180.0f), I.gyr[1] * (3.14159265f / 180.0f), I.gyr[2] * (3.14159265f / 180.0f) };
    ahrs_update(&I.att, (ahrs_mode_t)I.mode, gr, I.acc, dt);
}

/* ------------------------------------------------------------------ calibration */
static void cal_restore(void)
{
    I.mode = C.mode;
    memcpy(I.rot, C.rot, sizeof(I.rot));
    memcpy(I.acc_off, C.acc_off, sizeof(I.acc_off));
    memcpy(I.gyr_off, C.gyr_off, sizeof(I.gyr_off));
    ahrs_params(&I.att, C.decay, C.kp, C.ki, C.beta);
    ahrs_init(&I.att);
}

void imu_calibration_start(float yaw_deg, const comm_iface_t *reply)
{
    if (!I.on || C.stage) return;
    C.reply = reply;
    C.yaw = yaw_deg;
    C.mode = I.mode;
    memcpy(C.rot, I.rot, sizeof(C.rot));
    memcpy(C.acc_off, I.acc_off, sizeof(C.acc_off));
    memcpy(C.gyr_off, I.gyr_off, sizeof(C.gyr_off));
    C.decay = I.att.decay; C.kp = I.att.kp; C.ki = I.att.ki; C.beta = I.att.beta;
    I.mode = AHRS_MADGWICK;
    ahrs_params(&I.att, 1.0f, 10.0f, 0.0f, 2.0f);
    memset(I.rot, 0, sizeof(I.rot));
    memset(I.acc_off, 0, sizeof(I.acc_off));
    memset(I.gyr_off, 0, sizeof(I.gyr_off));
    C.stage = 1;
    C.n = 0;
    C.acc[0] = C.acc[1] = C.acc[2] = 0.0f;
    C.t_ms = systime_ms();
}

static void cal_step(void)
{
    uint32_t now = systime_ms();
    float rpy[3];
    switch (C.stage) {
    case 1:     /* gyrometre au repos : decalages (1 s) */
        for (int k = 0; k < 3; k++) C.acc[k] += I.gyr[k];
        if (++C.n >= (int)(1000000u / I.period_us)) {
            for (int k = 0; k < 3; k++) C.g0[k] = C.acc[k] / (float)C.n;
            memcpy(I.gyr_off, C.g0, sizeof(I.gyr_off));
            ahrs_init(&I.att);
            C.stage = 2; C.t_ms = now; C.n = 0; C.acc[0] = 0.0f;
        }
        break;
    case 2:     /* roulis moyen apres 1.5 s de convergence (0.25 s) */
    case 3:     /* tangage moyen */
        if (now - C.t_ms < 1500u) break;
        ahrs_rpy(&I.att, rpy);
        C.acc[0] += C.stage == 2 ? rpy[0] : rpy[1];
        if (++C.n >= (int)(250000u / I.period_us)) {
            float m = C.acc[0] / (float)C.n;
            /* comme VESC : rot_roll = -roulis moyen, rot_pitch = +tangage moyen */
            if (C.stage == 2) I.rot[0] = -m; else I.rot[1] = m;
            rotate(I.rot, C.g0, I.gyr_off, false);
            ahrs_init(&I.att);
            C.stage++; C.t_ms = now; C.n = 0; C.acc[0] = 0.0f;
        }
        break;
    case 4: {
        I.rot[2] = C.yaw * (3.14159265f / 180.0f);
        rotate(I.rot, C.g0, I.gyr_off, false);
        float cal[9] = { I.rot[0] * 57.29578f, I.rot[1] * 57.29578f, I.rot[2] * 57.29578f,
                         0.0f, 0.0f, 0.0f, I.gyr_off[0], I.gyr_off[1], I.gyr_off[2] };
        uint8_t b[40];
        int32_t o = 0;
        b[o++] = COMM_GET_IMU_CALIBRATION;
        for (int k = 0; k < 9; k++) vb_put_f32(b, cal[k], 1e6f, &o);
        packet_send(C.reply, b, (uint32_t)o);
        cal_restore();
        C.stage = 0;
    } break;
    default:
        C.stage = 0;
        break;
    }
}

void imu_poll(uint32_t now_us)
{
    if (!I.on || !I.ok) return;
    if ((int32_t)(now_us - I.next_us) < 0) return;
    I.next_us = now_us + I.period_us;
    sample();
    if (C.stage) cal_step();
}

int32_t imu_data_reply(uint8_t *b, uint16_t mask)
{
    int32_t o = 0;
    float rpy[3], q[4];
    imu_get_rpy(rpy);
    imu_get_quat(q);
    const float mag[3] = { 0.0f, 0.0f, 0.0f };
    const float *v[16] = { &rpy[0], &rpy[1], &rpy[2], &I.acc[0], &I.acc[1], &I.acc[2], &I.gyr[0], &I.gyr[1],
                           &I.gyr[2], &mag[0], &mag[1], &mag[2], &q[0], &q[1], &q[2], &q[3] };
    b[o++] = COMM_GET_IMU_DATA;
    vb_put_u16(b, mask, &o);
    for (int k = 0; k < 16; k++) if (mask & (1u << k)) vb_put_f32_auto(b, *v[k], &o);
    b[o++] = resc_controller_id();
    return o;
}
