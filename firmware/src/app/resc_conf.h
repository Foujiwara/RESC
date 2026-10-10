/*
 * resc_conf.h — Configuration RESC.
 *
 * La configuration est conservee dans le format de VESC Tool (MCCONF/APPCONF)
 * pour que l'outil puisse la lire et l'ecrire. resc_conf_build() la traduit
 * vers les parametres du coeur RESC (foc_params_t, ctrl_limits_t) en
 * appliquant les limites absolues de la carte et le plafond lie a son statut.
 */
#ifndef RESC_CONF_H
#define RESC_CONF_H

#include <stdint.h>
#include <stdbool.h>
#include "vesc_conf.h"
#include "foc.h"
#include "ctrl.h"

extern vesc_mcconf_t  g_mcconf;
extern vesc_appconf_t g_appconf;

void resc_conf_init(void);                      /* defauts puis lecture flash */
void resc_conf_defaults_mc(vesc_mcconf_t *c);
void resc_conf_defaults_app(vesc_appconf_t *c);
void resc_conf_clamp_mc(vesc_mcconf_t *c);      /* bornes absolues de la carte */
void resc_conf_clamp_app(vesc_appconf_t *c);

/* Traduit la configuration vers le coeur (f_pwm, echelles, gains, limites) */
void resc_conf_build(const vesc_mcconf_t *mc, const vesc_appconf_t *app, foc_params_t *fp, ctrl_limits_t *cl);
/* Applique g_mcconf / g_appconf au coeur et a la couche de controle */
void resc_conf_apply(void);
void resc_conf_poll(void);                      /* applique ce qui attendait l'arret du moteur */
bool resc_conf_apply_pending(void);

bool resc_conf_store_mc(void);                  /* refuse si le moteur tourne */
bool resc_conf_store_app(void);

/* Valeurs derivees des mesures, calculees automatiquement apres chaque detection :
 *  gains de courant  kp = L * bw, ki = R * bw  (bw : bande passante, rad/s)
 *  gain observateur  gamma = 1000 / lambda^2   (convergence radiale ~1000 rad/s) */
#define RESC_CURRENT_BW_DEFAULT 1000.0f
void resc_conf_derive_current_gains(vesc_mcconf_t *c, float bw);
void resc_conf_derive_observer_gain(vesc_mcconf_t *c);

bool resc_motor_configured(void);               /* parametres moteur identifies ou saisis */
float resc_current_cap_frac(void);              /* plafond lie au statut de la carte */
bool resc_power_allowed(void);                  /* pont autorise par la politique de carte */
uint8_t resc_controller_id(void);

/* Fournis par la plateforme */
bool plat_conf_write(uint32_t slot, const uint8_t *data, uint32_t len);
const uint8_t *plat_conf_read(uint32_t slot, uint32_t *len);
uint8_t plat_default_controller_id(void);

#endif /* RESC_CONF_H */
