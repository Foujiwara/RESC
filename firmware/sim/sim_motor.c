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
    if (m->acim) { ia = m->ia_s; ib = m->ib_s; }
    i[0] = ia;
    i[1] = -0.5 * ia + sqrt(3) / 2 * ib;
    i[2] = -0.5 * ia - sqrt(3) / 2 * ib;
}

void sim_init(sim_motor_t *m)
{
    m->id = m->iq = m->theta = m->omega_m = m->t = 0;
    m->ia_s = m->ib_s = m->pra = m->prb = m->ea_s = m->eb_s = 0;
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
            /* Les capacites des MOSFET retardent la commutation a faible courant : l'erreur
             * n'est complete qu'au-dessus de i_soft (zone lineaire elargie) */
            double isat = 0.5 * ripple > m->i_soft ? 0.5 * ripple : m->i_soft;
            for (int x = 0; x < 3; x++) {
                double k = i[x] / isat;
                if (k > 1) k = 1;
                if (k < -1) k = -1;
                double dd = d[x] - k * m->t_dead * m->f_pwm;
                if (dd < 0) dd = 0;
                if (dd > 1) dd = 1;
                v[x] = dd * m->vbus;
            }
            if (m->pf_on) {
                /* tensions de phase filtrees (RC) : moyenne glissante de la tension appliquee */
                double k = 1.0 - exp(-h / m->pf_tau);
                for (int x = 0; x < 3; x++) m->vph_f[x] += k * (v[x] - m->vph_f[x]);
            }
            double va = (2 * v[0] - v[1] - v[2]) / 3.0;
            double vb = (v[1] - v[2]) / sqrt(3);
            if (m->acim) {
                /* repere fixe : d(psi_r)/dt = Lm/Tr i - psi_r/Tr + J we psi_r ;
                 * sigma Ls di/dt = v - Rs i - (Lm/Lr) d(psi_r)/dt */
                double tr = m->lr / m->rr, k = m->lm / m->lr;
                double dpa = m->lm / tr * m->ia_s - m->pra / tr - we * m->prb;
                double dpb = m->lm / tr * m->ib_s - m->prb / tr + we * m->pra;
                m->ea_s = k * dpa;
                m->eb_s = k * dpb;
                m->ia_s += (va - m->r * m->ia_s - m->ea_s) / m->ld * h;
                m->ib_s += (vb - m->r * m->ib_s - m->eb_s) / m->ld * h;
                m->pra += dpa * h;
                m->prb += dpb * h;
                goto mech;
            }
            double vd = c * va + s * vb;
            double vq = -s * va + c * vb;
            double ldi = m->ld * (1.0 - m->ld_sat * m->id);
            if (ldi < 0.5 * m->ld) ldi = 0.5 * m->ld;
            if (ldi > 1.5 * m->ld) ldi = 1.5 * m->ld;
            double did = (vd - m->r * m->id + we * m->lq * m->iq) / ldi;
            double diq = (vq - m->r * m->iq - we * (m->ld * m->id + m->lambda)) / m->lq;
            m->id += did * h;
            m->iq += diq * h;
        } else {
            /* pont coupe : le courant s'eteint par les diodes (FEM < Vbus suppose) */
            m->id *= 0.5;
            m->iq *= 0.5;
            if (m->acim) {
                m->ia_s *= 0.5;
                m->ib_s *= 0.5;
                double tr = m->lr / m->rr, k = m->lm / m->lr;
                double dpa = m->lm / tr * m->ia_s - m->pra / tr - we * m->prb;
                double dpb = m->lm / tr * m->ib_s - m->prb / tr + we * m->pra;
                m->ea_s = k * dpa;
                m->eb_s = k * dpb;
                m->pra += dpa * h;
                m->prb += dpb * h;
            }
        }
    mech:;
        double te = m->acim ? 1.5 * m->pp * (m->lm / m->lr) * (m->pra * m->ib_s - m->prb * m->ia_s)
                            : 1.5 * m->pp * (m->lambda * m->iq + (m->ld - m->lq) * m->id * m->iq);
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
    if (m->full_delay) {
        integrate(m, T, m->duty_cur, m->en_cur);      /* periode de boucle entiere : commande precedente */
        for (int k = 0; k < 3; k++) m->duty_cur[k] = m->duty_next[k];
        m->en_cur = m->en_next;
    } else {
        /* sommet -> creux : ancienne commande, ou sa seconde moitie interpolee */
        integrate(m, T / 2, m->h_ok && m->en_cur ? m->duty_h : m->duty_cur, m->en_cur);
        for (int k = 0; k < 3; k++) m->duty_cur[k] = m->duty_next[k];
        m->en_cur = m->en_next;
        integrate(m, T / 2, m->duty_cur, m->en_cur);  /* creux -> sommet : nouvelle commande */
        m->h_ok = m->hn_ok;
        for (int k = 0; k < 3; k++) m->duty_h[k] = m->duty_hn[k];
    }

    double i[3];
    sim_phase_currents(m, i);
    for (int x = 0; x < 3; x++) adc->cs[x] = to_adc(m->off[x] + i[x] / m->i_lsb + m->noise_lsb * gauss());
    adc->vbus = to_adc(m->vbus / m->v_lsb + m->noise_lsb * gauss());
    /* tensions de phase : FEM + neutre a Vbus/2 quand le pont est coupe, 0 sinon (low-sides passants) */
    double th = m->theta * m->pp, we = m->omega_m * m->pp;
    for (int x = 0; x < 3; x++) {
        double e = -we * m->lambda * sin(th - x * 2 * M_PI / 3);
        if (m->acim) e = x == 0 ? m->ea_s : -0.5 * m->ea_s + (x == 1 ? 1 : -1) * sqrt(3) / 2 * m->eb_s;
        double v = m->en_cur ? (m->pf_on ? m->vph_f[x] : 0.0) : e + m->vbus / 2;
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
