#include <math.h>
#include <stdlib.h>
#include "sim_motor.h"

static double sgn(double x) { return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0); }

double sim_theta_e(const sim_motor_t *m) { return fmod(m->theta * m->pp, 2 * M_PI); }
double sim_omega_e(const sim_motor_t *m) { return m->omega_m * m->pp; }

void sim_phase_currents(const sim_motor_t *m, double i[3])
{
    double th = m->theta * m->pp, s = sin(th), c = cos(th);
    double ia = c * m->id - s * m->iq;
    double ib = s * m->id + c * m->iq;
    i[0] = ia;
    i[1] = -0.5 * ia + sqrt(3) / 2 * ib;
    i[2] = -0.5 * ia - sqrt(3) / 2 * ib;
}

void sim_init(sim_motor_t *m)
{
    m->id = m->iq = m->theta = m->omega_m = m->t = 0;
    for (int k = 0; k < 3; k++) m->duty_cur[k] = m->duty_next[k] = 0.5;
    m->en_cur = m->en_next = false;
}

static double gauss(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static void integrate(sim_motor_t *m, double T, const double d[3], bool en)
{
    const int N = 20;
    double h = T / N;
    for (int n = 0; n < N; n++) {
        double th = m->theta * m->pp, s = sin(th), c = cos(th);
        double we = m->omega_m * m->pp;
        if (en) {
            double i[3];
            sim_phase_currents(m, i);
            double v[3];
            /* L'erreur de temps mort suit le signe du courant instantane. L'ondulation PWM
             * ne vient que des ecarts de rapport cyclique entre phases (PWM centree) :
             * ondulation crete-crete ~ Vbus * (dmax - dmin) * Ts / (2 L). Sous l'ondulation,
             * l'erreur varie lineairement avec le courant. */
            double dmx = d[0], dmn = d[0];
            for (int x = 1; x < 3; x++) { if (d[x] > dmx) dmx = d[x]; if (d[x] < dmn) dmn = d[x]; }
            double ripple = m->vbus * (dmx - dmn) / (2.0 * 0.5 * (m->ld + m->lq) * m->f_pwm) + 0.05;
            for (int x = 0; x < 3; x++) {
                double k = i[x] / (0.5 * ripple);
                if (k > 1) k = 1;
                if (k < -1) k = -1;
                double dd = d[x] - k * m->t_dead * m->f_pwm;
                if (dd < 0) dd = 0;
                if (dd > 1) dd = 1;
                v[x] = dd * m->vbus;
            }
            double va = (2 * v[0] - v[1] - v[2]) / 3.0;
            double vb = (v[1] - v[2]) / sqrt(3);
            double vd = c * va + s * vb;
            double vq = -s * va + c * vb;
            double did = (vd - m->r * m->id + we * m->lq * m->iq) / m->ld;
            double diq = (vq - m->r * m->iq - we * (m->ld * m->id + m->lambda)) / m->lq;
            m->id += did * h;
            m->iq += diq * h;
        } else {
            /* pont coupe : le courant s'eteint par les diodes (FEM < Vbus suppose) */
            m->id *= 0.5;
            m->iq *= 0.5;
        }
        double te = 1.5 * m->pp * (m->lambda * m->iq + (m->ld - m->lq) * m->id * m->iq);
        double tl = m->t_load * sgn(m->omega_m) + m->b * m->omega_m;
        if (fabs(m->omega_m) < 1e-3 && fabs(te) < m->t_load) tl = te;   /* frottement sec a l'arret */
        m->omega_m += (te - tl) / m->j * h;
        m->theta += m->omega_m * h;
        m->t += h;
    }
}

static uint16_t to_adc(double x)
{
    long v = lround(x);
    if (v < 0) v = 0;
    if (v > 4095) v = 4095;
    return (uint16_t)v;
}

void sim_step(sim_motor_t *m, const foc_pwm_t *out, foc_adc_t *adc)
{
    double T = 1.0 / m->f_pwm;
    for (int k = 0; k < 3; k++) m->duty_next[k] = out->duty[k];
    m->en_next = out->enable;
    integrate(m, T / 2, m->duty_cur, m->en_cur);     /* sommet -> creux : ancienne commande */
    for (int k = 0; k < 3; k++) m->duty_cur[k] = m->duty_next[k];
    m->en_cur = m->en_next;
    integrate(m, T / 2, m->duty_cur, m->en_cur);     /* creux -> sommet : nouvelle commande */

    double i[3];
    sim_phase_currents(m, i);
    for (int x = 0; x < 3; x++) adc->cs[x] = to_adc(m->off[x] + i[x] / m->i_lsb + m->noise_lsb * gauss());
    adc->vbus = to_adc(m->vbus / m->v_lsb + m->noise_lsb * gauss());
    /* tensions de phase : FEM + neutre a Vbus/2 quand le pont est coupe, 0 sinon (low-sides passants) */
    double th = m->theta * m->pp, we = m->omega_m * m->pp;
    for (int x = 0; x < 3; x++) {
        double e = -we * m->lambda * sin(th - x * 2 * M_PI / 3);
        double v = m->en_cur ? 0.0 : e + m->vbus / 2;
        adc->vph[x] = to_adc(v / m->vph_lsb + m->noise_lsb * gauss());
    }
    double h = th - m->hall_off;
    adc->hall = (uint8_t)((sin(h) > 0 ? 1 : 0) | (sin(h - 2 * M_PI / 3) > 0 ? 2 : 0) | (sin(h + 2 * M_PI / 3) > 0 ? 4 : 0));
    if (m->enc_cpr) {
        double mech = (m->enc_rev ? -m->theta : m->theta) + m->enc_off;
        double c = fmod(mech / (2 * M_PI) * m->enc_cpr, (double)m->enc_cpr);
        if (c < 0) c += m->enc_cpr;
        adc->enc = (uint32_t)c;
    }
}
