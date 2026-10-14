/*
 * blackbox.c — Enregistreur pre-defaut du coeur FOC (voir blackbox.h).
 */
#include <string.h>
#include "blackbox.h"
#include "platform.h"

#define BB_MAGIC 0x42424F58u     /* 'BBOX' */

typedef struct {
    uint32_t magic;
    uint32_t head;               /* prochain emplacement */
    uint32_t n;                  /* enregistrements valides (<= BLACKBOX_N) */
    uint32_t post;               /* periodes restantes apres le defaut ; 0 et fige : arret */
    uint32_t frozen;
    uint32_t code, line, ticks;  /* defaut */
    uint32_t prev_boot;
    uint32_t arm_start;          /* figer a la prochaine remise en marche */
    uint32_t post_len;           /* periodes gardees apres l'evenement */
    uint32_t check;              /* somme de controle de l'en-tete */
    blackbox_rec_t rec[BLACKBOX_N];
} bb_t;

#if defined(RESC_HOST)
static bb_t B;
#else
NOINIT static bb_t B;
#endif

RAMFUNC static uint32_t hdr_sum(void)
{
    return B.magic ^ (B.head * 3u) ^ (B.n * 5u) ^ (B.post * 7u) ^ (B.frozen * 11u) ^ (B.code * 13u) ^
           (B.line * 17u) ^ (B.ticks * 19u) ^ 0x5A5A5A5Au;
}

void blackbox_release(void)
{
    B.frozen = 0;
    B.post = 0;
    B.n = 0;
    B.head = 0;
    B.code = B.line = B.ticks = 0;
    B.prev_boot = 0;
    B.arm_start = 0;
    B.post_len = BLACKBOX_POST;
    B.magic = BB_MAGIC;
    B.check = hdr_sum();
}

void blackbox_init(void)
{
    bool ok = B.magic == BB_MAGIC && B.check == hdr_sum() && B.head < BLACKBOX_N && B.n <= BLACKBOX_N &&
              B.frozen && B.post == 0;
    if (ok) {
        B.prev_boot = 1;         /* defaut fige avant le redemarrage : conserve */
        return;
    }
    blackbox_release();
}

RAMFUNC void blackbox_push(const blackbox_rec_t *r)
{
    if (B.frozen && B.post == 0) return;
    B.rec[B.head] = *r;
    B.head = (B.head + 1u) % BLACKBOX_N;
    if (B.n < BLACKBOX_N) B.n++;
    if (B.frozen) {
        B.post--;
        B.check = hdr_sum();
    }
}

RAMFUNC static void freeze_post(uint32_t code, uint32_t line, uint32_t ticks, uint32_t post)
{
    if (B.frozen) return;        /* le premier evenement est garde */
    B.frozen = 1;
    B.post = post;
    B.post_len = post;
    B.code = code;
    B.line = line;
    B.ticks = ticks;
    B.check = hdr_sum();
}

RAMFUNC void blackbox_freeze(uint32_t code, uint32_t line, uint32_t ticks)
{
    freeze_post(code, line, ticks, BLACKBOX_POST);
}

RAMFUNC void blackbox_start_event(uint32_t ticks)
{
    if (!B.arm_start || B.frozen) return;
    B.arm_start = 0;
    freeze_post(BLACKBOX_CODE_START, 0, ticks, BLACKBOX_N - 32u);
}

void blackbox_arm_start(void)
{
    if (B.frozen || B.magic != BB_MAGIC) blackbox_release();   /* l'historique en cours est garde (32 periodes avant) */
    B.arm_start = 1;
}

bool blackbox_frozen(void) { return B.frozen != 0 && B.post == 0; }
bool blackbox_from_previous_boot(void) { return B.prev_boot != 0; }

uint32_t blackbox_fault(uint32_t *line, uint32_t *ticks)
{
    if (line) *line = B.line;
    if (ticks) *ticks = B.ticks;
    return B.frozen ? B.code : 0u;
}

int blackbox_count(void) { return (int)B.n; }

bool blackbox_get(int k, blackbox_rec_t *r)
{
    if (k < 0 || (uint32_t)k >= B.n) return false;
    uint32_t first = (B.head + BLACKBOX_N - B.n) % BLACKBOX_N;
    *r = B.rec[(first + (uint32_t)k) % BLACKBOX_N];
    return true;
}

/* Position du defaut : avant les enregistrements faits apres lui */
int blackbox_trigger_index(void)
{
    if (!B.frozen) return -1;
    uint32_t after = B.post_len - B.post;
    return B.n >= after ? (int)(B.n - after) : 0;
}
