/*
 * detect_app.c — Detections manuelles (boucle principale).
 *
 * Toutes les sequences sont celles du coeur RESC (foc.c) :
 *  offsets  : pont coupe (courants + tensions de phase), puis vecteur nul sous PWM
 *             (courants + ecarts de tension entre phases)
 *  R/L      : rampe au courant de mesure, 3 paliers montants (pente dV/dI), creneaux d/q
 *  flux     : rotation I/f jusqu'au rapport cyclique cible, FEM pont actif et pont coupe
 *  Hall     : rotation lente a courant d impose, 3 tours avant / arriere / avant,
 *             centre circulaire de chaque etat
 *  codeur   : meme rotation ; rapport et sens au premier passage, offset ensuite
 * Seul le format des reponses reprend le protocole, pour les boutons de VESC Tool.
 * Pas de sequence enchainee qui ecrase la configuration : chaque mesure est
 * lancee a la main (ordre conseille : offsets, R/L, flux, Hall).
 * Apres chaque mesure reussie, le resultat ET les valeurs qui en dependent sont
 * appliques et sauvegardes, quelle que soit l'origine de la demande :
 *  R/L   -> R, L, Ld-Lq, puis kp = L*bw, ki = R*bw
 *  flux  -> lambda, puis gain d'observateur 1000 / lambda^2
 *  Hall  -> table ; codeur -> offset, rapport, sens ; offsets -> offsets.
 */
#include "detect_app.h"
#include "commands.h"
#include "foc.h"
#include "ctrl.h"
#include "resc_conf.h"
#include "vesc_buffer.h"
#include "vesc_defaults.h"
#include "hw_config.h"
#include "fmath.h"

typedef enum { DA_IDLE = 0, DA_OFFSETS, DA_RL, DA_LAMBDA, DA_HALL, DA_ENCODER, DA_ACIM } da_state_t;

static da_state_t s_st;
static const comm_iface_t *s_reply;

bool detect_app_busy(void) { return s_st != DA_IDLE; }

static float i_cap(void)
{
    return fm_min(g_mcconf.l_current_max, (float)HWLIM_I_MOTOR_A * resc_current_cap_frac());
}

static bool launch(foc_det_t *d)
{
    if (!resc_power_allowed()) {
        commands_printf("Refuse : carte non relue (statut IMPORTED)");
        return false;
    }
    ctrl_release();
    if (!foc_detect_start(d)) {
        commands_printf("Refuse : moteur en marche, defaut actif ou calibration initiale non terminee");
        return false;
    }
    return true;
}

static void put(const uint8_t *b, int32_t n) { if (s_reply) packet_send(s_reply, b, (uint32_t)n); }

/* ---------------------------------------------------------------- offsets */
bool detect_app_start_offsets(const comm_iface_t *reply)
{
    if (s_st != DA_IDLE) return false;
    foc_det_t d = { 0 };
    d.kind = FOC_DET_OFFSETS;
    if (!launch(&d)) return false;
    s_reply = reply;
    s_st = DA_OFFSETS;
    commands_printf("Calibration des offsets (moteur a l'arret)...");
    return true;
}

/* ---------------------------------------------------------------- R/L */
static void send_rl(float r, float l_uh, float diff_uh)
{
    uint8_t b[16];
    int32_t o = 0;
    b[o++] = COMM_DETECT_MOTOR_R_L;
    vb_put_f32(b, r, 1e6f, &o);
    vb_put_f32(b, l_uh, 1e3f, &o);
    vb_put_f32(b, diff_uh, 1e3f, &o);
    put(b, o);
}

bool detect_app_start_rl(const comm_iface_t *reply, float cur)
{
    if (s_st != DA_IDLE) return false;
    s_reply = reply;
    foc_det_t d = { 0 };
    d.kind = FOC_DET_RL;
    /* Courant du palier haut : valeur demandee, sinon une fraction du courant moteur max
     * configure (l_current_max, borne par le statut de la carte). Paliers a 1/2, 3/4, 1. */
    d.i_test = fm_sat(cur > 0.5f ? cur : fm_sat(g_mcconf.foc_resc_rl_frac, 0.1f, 1.0f) * i_cap(), 1.0f, i_cap());
    if (!launch(&d)) { send_rl(0, 0, 0); return false; }
    s_st = DA_RL;
    commands_printf("Mesure R/L a %.1f A...", (double)d.i_test);
    return true;
}

/* ---------------------------------------------------------- induction */
bool detect_app_start_acim(const comm_iface_t *reply, float cur)
{
    if (s_st != DA_IDLE) return false;
    s_reply = reply;
    if (!(g_mcconf.foc_motor_r > 0.0f && g_mcconf.foc_motor_l > 0.0f)) {
        commands_printf("Mesurer R et L d'abord (detect_rl)");
        return false;
    }
    foc_det_t d = { 0 };
    d.kind = FOC_DET_ACIM;
    d.i_test = fm_sat(cur > 0.5f ? cur : fm_sat(g_mcconf.foc_resc_rl_frac, 0.1f, 1.0f) * i_cap(), 1.0f, i_cap());
    d.r_known = g_mcconf.foc_motor_r;
    d.l_known = g_mcconf.foc_motor_l;
    if (!launch(&d)) return false;
    s_st = DA_ACIM;
    commands_printf("Moteur a induction : echelon de courant continu %.1f A -> %.1f A (environ 5 s, le moteur ne tourne pas)...",
                    (double)(0.5f * d.i_test), (double)d.i_test);
    return true;
}

/* ---------------------------------------------------------------- flux */
static void send_lambda(float lam)
{
    uint8_t b[16];
    int32_t o = 0;
    b[o++] = COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
    vb_put_f32(b, lam, 1e7f, &o);
    vb_put_f32(b, -1.0f, 1e6f, &o);
    vb_put_f32(b, -1.0f, 1e6f, &o);
    b[o++] = 0;
    put(b, o);
}

bool detect_app_start_lambda(const comm_iface_t *reply, float cur, float erpm_s, float duty, float r, float l)
{
    if (s_st != DA_IDLE) return false;
    s_reply = reply;
    if (g_mcconf.foc_motor_kind == 1) {
        commands_printf("Moteur a induction : pas de flux d'aimant a mesurer (detect_acim)");
        send_lambda(0);
        return false;
    }
    if (r <= 0.0f) r = g_mcconf.foc_motor_r;
    if (l <= 0.0f) l = g_mcconf.foc_motor_l;
    foc_det_t d = { 0 };
    d.kind = FOC_DET_LAMBDA;
    d.i_test = fm_sat(cur, 1.0f, i_cap());
    d.r_known = r;
    d.l_known = l;
    d.ramp_rate = (erpm_s > 100.0f ? erpm_s : 1800.0f) * (FM_2PI / 60.0f);
    d.duty_target = fm_sat(duty, 0.1f, 0.6f);
    d.omega_test = fm_min(g_mcconf.l_max_erpm, 60000.0f) * (FM_2PI / 60.0f);
    if (!launch(&d)) { send_lambda(0); return false; }
    s_st = DA_LAMBDA;
    commands_printf("Mesure du flux a %.1f A (moteur libre, il va tourner)...", (double)d.i_test);
    return true;
}

/* ---------------------------------------------------------------- Hall */
static void send_hall(const uint8_t tab[8], bool ok)
{
    uint8_t b[12];
    int32_t o = 0;
    b[o++] = COMM_DETECT_HALL_FOC;
    for (int k = 0; k < 8; k++) b[o++] = tab[k];
    b[o++] = ok ? 0 : 1;
    put(b, o);
}

bool detect_app_start_hall(const comm_iface_t *reply, float cur)
{
    static const uint8_t none[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
    if (s_st != DA_IDLE) return false;
    s_reply = reply;
    foc_det_t d = { 0 };
    d.kind = FOC_DET_HALL;
    d.i_test = fm_sat(cur > 0.5f ? cur : 0.1f * i_cap(), 1.0f, i_cap());
    d.r_known = g_mcconf.foc_motor_r;
    d.l_known = g_mcconf.foc_motor_l;
    if (!launch(&d)) { send_hall(none, false); return false; }
    s_st = DA_HALL;
    commands_printf("Detection des sondes Hall a %.1f A (le moteur tourne lentement)...", (double)d.i_test);
    return true;
}

/* ---------------------------------------------------------------- codeur */
static void send_enc(float off_deg, float ratio, bool inv)
{
    uint8_t b[12];
    int32_t o = 0;
    b[o++] = COMM_DETECT_ENCODER;
    vb_put_f32(b, off_deg, 1e6f, &o);
    vb_put_f32(b, ratio, 1e6f, &o);
    b[o++] = inv ? 1 : 0;
    put(b, o);
}

bool detect_app_start_encoder(const comm_iface_t *reply, float cur)
{
    if (s_st != DA_IDLE) return false;
    s_reply = reply;
    if (g_mcconf.m_encoder_counts < 4) {
        commands_printf("Nombre de fronts du codeur non configure");
        send_enc(1001.0f, 0.0f, false);
        return false;
    }
    foc_det_t d = { 0 };
    d.kind = FOC_DET_ENCODER;
    d.i_test = fm_sat(cur > 0.5f ? cur : 0.1f * i_cap(), 1.0f, i_cap());
    d.r_known = g_mcconf.foc_motor_r;
    d.l_known = g_mcconf.foc_motor_l;
    if (!launch(&d)) { send_enc(1001.0f, 0.0f, false); return false; }
    s_st = DA_ENCODER;
    commands_printf("Detection du codeur a %.1f A (le moteur tourne lentement)...", (double)d.i_test);
    return true;
}

/* ---------------------------------------------------------------- suivi */
static const char *err_text(int e)
{
    switch (e) {
    case -2: return "resistance hors plage";
    case -3: return "inductance hors plage";
    case -4: return "flux hors plage";
    case -5: return "le rotor n'a pas suivi la boucle ouverte (charge ? R/L faux ?)";
    case -6: return "offsets instables sous PWM (bruit, masse ?)";
    case -7: return "etats Hall invalides ou manquants (cablage, alimentation des sondes ?)";
    case -8: return "le codeur ne compte pas ou rapport incoherent";
    case -9: return "moteur en rotation";
    case -10: return "reponse de flux rotorique non mesurable (courant trop faible, moteur a aimants ?)";
    default: return e > 0 ? "defaut pendant la mesure" : "echec";
    }
}

void detect_app_poll(uint32_t now_ms)
{
    (void)now_ms;
    if (s_st == DA_IDLE) return;
    const foc_det_t *d = foc_detect_status();
    if (d->busy || !d->done) return;
    if (d->error) commands_printf("Echec : %s (code %d)", err_text(d->error), d->error);

    switch (s_st) {
    case DA_OFFSETS:
        if (!d->error) {
            g_mcconf.foc_offsets_current_0 = d->off_cs[0];
            g_mcconf.foc_offsets_current_1 = d->off_cs[1];
            g_mcconf.foc_offsets_current_2 = d->off_cs[2];
            g_mcconf.foc_offsets_voltage_0 = d->off_vph_drv[0];
            g_mcconf.foc_offsets_voltage_1 = d->off_vph_drv[1];
            g_mcconf.foc_offsets_voltage_2 = d->off_vph_drv[2];
            g_mcconf.foc_offsets_voltage_undriven_0 = d->off_vph[0];
            g_mcconf.foc_offsets_voltage_undriven_1 = d->off_vph[1];
            g_mcconf.foc_offsets_voltage_undriven_2 = d->off_vph[2];
            g_mcconf.foc_offsets_cal_mode &= (uint8_t)~1u;     /* utiliser ces offsets au demarrage */
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("Offsets courant : %.1f %.1f %.1f LSB",
                            (double)d->off_cs[0], (double)d->off_cs[1], (double)d->off_cs[2]);
            commands_printf("Offsets tension sous PWM : %.4f %.4f %.4f V", (double)d->off_vph_drv[0],
                            (double)d->off_vph_drv[1], (double)d->off_vph_drv[2]);
            commands_printf("Offsets tension pont coupe : %.4f %.4f %.4f V (sauvegardes)", (double)d->off_vph[0],
                            (double)d->off_vph[1], (double)d->off_vph[2]);
        }
        break;

    case DA_RL:
        if (d->error) send_rl(0, 0, 0);
        else {
            float l = 0.5f * (d->ld + d->lq);
            g_mcconf.foc_motor_r = d->r;
            g_mcconf.foc_motor_l = l;
            g_mcconf.foc_motor_ld_lq_diff = d->lq - d->ld;
            resc_conf_derive_current_gains(&g_mcconf, g_mcconf.foc_resc_cc_bw);
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("Paliers %.1f / %.1f / %.1f A, chute ohmique %.3f V, tension perdue (temps mort) %.3f V",
                            (double)d->r_i_low, (double)(0.75f * d->r_i_high), (double)d->r_i_high,
                            (double)d->r_v_high, (double)d->r_v_dt);
            if (d->warn == 1)
                commands_printf("ATTENTION : chute ohmique faible (%.2f V) : R peu precise. "
                                "Augmenter l_current_max pour la mesure.", (double)d->r_v_high);
            if (d->warn == 2)
                commands_printf("ATTENTION : temps mort non sature au palier bas : R calculee sur les 2 paliers hauts. "
                                "Augmenter l_current_max pour la mesure.");
            commands_printf("R=%.2f mOhm  Ld=%.1f uH  Lq=%.1f uH  ->  kp=%.4f ki=%.2f (sauvegarde)",
                            (double)(d->r * 1e3f), (double)(d->ld * 1e6f), (double)(d->lq * 1e6f),
                            (double)g_mcconf.foc_current_kp, (double)g_mcconf.foc_current_ki);
            send_rl(d->r, l * 1e6f, (d->lq - d->ld) * 1e6f);
        }
        break;

    case DA_LAMBDA:
        if (d->error) send_lambda(0);
        else {
            g_mcconf.foc_motor_flux_linkage = d->lambda;
            resc_conf_derive_observer_gain(&g_mcconf);
            resc_conf_derive_current_gains(&g_mcconf, g_mcconf.foc_resc_cc_bw);   /* si R/L deja connus */
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("lambda = %.3f mWb  ->  gain observateur %.4g (sauvegarde)%s", (double)(d->lambda * 1e3f),
                            (double)g_mcconf.foc_observer_gain, resc_motor_configured() ? " ; moteur pret" : "");
            send_lambda(d->lambda);
        }
        break;

    case DA_HALL: {
        uint8_t tab[8];
        for (int k = 0; k < 8; k++) {
            float a = d->hall_angle[k];
            if (a < 0.0f) a += FM_2PI;
            tab[k] = (!d->error && d->hall_valid[k]) ? (uint8_t)(a * (200.0f / FM_2PI) + 0.5f) % 200 : 255;
        }
        if (!d->error) {
            g_mcconf.foc_hall_table_0 = tab[0]; g_mcconf.foc_hall_table_1 = tab[1];
            g_mcconf.foc_hall_table_2 = tab[2]; g_mcconf.foc_hall_table_3 = tab[3];
            g_mcconf.foc_hall_table_4 = tab[4]; g_mcconf.foc_hall_table_5 = tab[5];
            g_mcconf.foc_hall_table_6 = tab[6]; g_mcconf.foc_hall_table_7 = tab[7];
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("Table Hall : %u %u %u %u %u %u %u %u (0-200 = 0-360 deg)", tab[0], tab[1], tab[2], tab[3],
                            tab[4], tab[5], tab[6], tab[7]);
        }
        send_hall(tab, d->error == 0);
        break;
    }

    case DA_ACIM:
        if (!d->error) {
            g_mcconf.foc_acim_lm = d->acim_lm;
            g_mcconf.foc_acim_lr = d->acim_lr;
            g_mcconf.foc_acim_rr = d->acim_rr;
            if (!(g_mcconf.foc_acim_im > 0.1f)) g_mcconf.foc_acim_im = 0.3f * i_cap();
            g_mcconf.foc_motor_kind = 1;
            resc_conf_derive_observer_gain(&g_mcconf);
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("Lm=%.3f mH  Lr=%.3f mH  Rr=%.2f mOhm  Tr=%.1f ms  (Lm^2/Lr=%.3f mH)", (double)(d->acim_lm * 1e3f),
                            (double)(d->acim_lr * 1e3f), (double)(d->acim_rr * 1e3f), (double)(d->acim_tr * 1e3f),
                            (double)(d->acim_k * 1e3f));
            commands_printf("Courant magnetisant foc_acim_im = %.1f A (a ajuster : courant a vide du moteur). Sauvegarde.",
                            (double)g_mcconf.foc_acim_im);
        }
        break;

    case DA_ENCODER:
        if (d->error) send_enc(1001.0f, 0.0f, false);
        else {
            float off_deg = d->enc_offset * (180.0f / FM_PI);
            if (off_deg < 0.0f) off_deg += 360.0f;
            g_mcconf.foc_encoder_offset = off_deg;
            g_mcconf.foc_encoder_ratio = d->enc_ratio;
            g_mcconf.foc_encoder_inverted = d->enc_inverted ? 1 : 0;
            resc_conf_apply();
            resc_conf_store_mc();
            commands_printf("Codeur : offset %.1f deg, rapport %.2f, %s", (double)off_deg, (double)d->enc_ratio,
                            d->enc_inverted ? "inverse" : "direct");
            send_enc(off_deg, d->enc_ratio, d->enc_inverted);
        }
        break;

    default:
        break;
    }
    s_st = DA_IDLE;
}
