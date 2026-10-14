/*
 * blackbox.h — Enregistreur pre-defaut du coeur FOC.
 *
 * Les 256 dernieres periodes de la boucle FOC (~17 ms a 15 kHz) sont gardees en permanence dans un
 * tampon circulaire. Au premier defaut, l'enregistrement continue 32 periodes (ce qui suit la coupure)
 * puis se fige jusqu'a blackbox_release() (terminal : clear ou fault_dump reset). Les rearmements
 * automatiques ne l'effacent pas : le premier defaut reste lisible.
 * Le tampon est en RAM non initialisee : il survit a un redemarrage logiciel ou du watchdog, pas a une
 * coupure d'alimentation.
 */
#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>
#include <stdbool.h>

#define BLACKBOX_N     256
#define BLACKBOX_POST  32

typedef struct {
    int16_t i[3];        /* A x 10, courants de phase */
    int16_t id, iq;      /* A x 10 */
    int16_t vd, vq;      /* V x 10 */
    int16_t theta;       /* rad x 10000, angle de commande */
    int16_t omega;       /* rad/s el. */
    uint16_t vbus;       /* V x 100 */
    uint8_t state;       /* foc_state_t */
    uint8_t duty;        /* modulation x 200 */
} blackbox_rec_t;

#define BLACKBOX_CODE_START  1000u   /* declenchement manuel : pont remis en marche (reprise, demarrage) */

/* ISR FOC */
void blackbox_push(const blackbox_rec_t *r);
void blackbox_freeze(uint32_t code, uint32_t line, uint32_t ticks);
void blackbox_start_event(uint32_t ticks);   /* pont remis en marche : declenche si arme */

/* Hors ISR */
void blackbox_init(void);        /* au demarrage : garde un enregistrement valide d'avant le redemarrage */
void blackbox_release(void);     /* efface et relance l'enregistrement */
void blackbox_arm_start(void);   /* relance, puis fige a la prochaine remise en marche du pont (32 avant, 224 apres) */
bool blackbox_frozen(void);
bool blackbox_from_previous_boot(void);
uint32_t blackbox_fault(uint32_t *line, uint32_t *ticks);
int blackbox_count(void);
bool blackbox_get(int k, blackbox_rec_t *r);   /* k = 0 : le plus ancien */
int blackbox_trigger_index(void);              /* position du defaut dans l'ordre de blackbox_get */

#endif /* BLACKBOX_H */
