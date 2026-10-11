/*
 * conf_desc.c — Commande COMM_RESC_GET_CONF_XML (250) : description XML de la configuration.
 *
 *   requete : [250][conf u8][len i32][offset i32]          (gros-boutiste)
 *             conf : 0 = mcconf, 1 = appconf, 2 = info ; bit 7 (0x80 | conf) : variante complete
 *             (reponse avec le meme octet)
 *   reponse : [250][conf u8][total i32][offset i32][donnees offset .. offset+n-1]
 *             n = min(len, 400, total - offset), 0 si offset hors limites ou len <= 0
 *             total = taille du XML compresse (qCompress : u32 taille + zlib) ; 0 si non fourni
 *
 * Le flux est assemble a la demande :
 *   [u32 taille du XML][partie fixe : en-tete zlib + blocs deflate, vidage complet, sans bloc final]
 *   [onglets : blocs deflate non compresses de 65535 octets au plus, le dernier final][adler32]
 * Les onglets (<Grouping>) ne contiennent que les parametres dont la condition est vraie pour la
 * configuration en cours : RESC Tool n'affiche que ce qui sert (type de moteur, capteur, options).
 * La taille et l'adler32 des onglets sont recalcules a chaque demande a l'offset 0 (debut du
 * telechargement) ; les morceaux suivants regenerent seulement la portion de texte demandee.
 *
 * Variante complete (bit 7, RESC Tool qui evalue les conditions) : tous les onglets, chacun avec
 * son attribut visibleIf s'il en a un, puis une section <Pages>. Seules les conditions qui ne
 * dependent que de la carte sont evaluees ici. <Params> et <SerOrder> sont identiques.
 */
#include <string.h>
#include "conf_desc.h"
#include "vesc_buffer.h"

#define CHUNK_MAX 400
#define BLK_MAX   65535u
#define ADLER_MOD 65521u

static const vesc_mcconf_t *s_mc;
static const vesc_appconf_t *s_app;

typedef struct {
    bool valid;
    uint32_t text_len;
    uint32_t adler;
} layout_t;
static layout_t s_lay[2][2];      /* [conf][variante complete] */

void conf_desc_source(const vesc_mcconf_t *mc, const vesc_appconf_t *app)
{
    s_mc = mc;
    s_app = app;
    s_lay[0][0].valid = s_lay[1][0].valid = s_lay[0][1].valid = s_lay[1][1].valid = false;
}

/* ---------------------------------------------------------------- texte des onglets */
typedef struct {
    uint32_t pos;            /* position dans le texte des onglets */
    uint32_t from, to;       /* portion a copier dans dst */
    uint8_t *dst;
    uint32_t a, b;           /* adler32 en cours */
    bool adler;
} sink_t;

static void put(sink_t *s, const char *t)
{
    for (; *t; t++) {
        uint8_t c = (uint8_t)*t;
        if (s->adler) {
            s->a = (s->a + c) % ADLER_MOD;
            s->b = (s->b + s->a) % ADLER_MOD;
        }
        if (s->dst && s->pos >= s->from && s->pos < s->to) s->dst[s->pos - s->from] = c;
        s->pos++;
        if (!s->adler && s->pos >= s->to) return;
    }
}

static bool is_sep(const conf_grp_t *e) { return strncmp(e->text, "::sep::", 7) == 0; }

/* full : variante complete, seules les conditions de la carte sont evaluees */
static bool shown(const conf_desc_t *d, const conf_grp_t *e, bool full)
{
    if (e->kind != 'P' || is_sep(e)) return false;
    if (full) return e->vif != CONF_VIF_BOARD || d->vis(e->cond, s_mc, s_app);
    return e->cond == 0 || !s_mc || !s_app || d->vis(e->cond, s_mc, s_app);
}

/* un parametre visible avant la fin de l'onglet (ou avant le prochain separateur si stop_sep) */
static bool any_shown(const conf_desc_t *d, const conf_grp_t *e, bool stop_sep, bool stop_sub, bool full)
{
    for (; e->kind == 'P' || (!stop_sub && e->kind == 'S'); e++) {
        if (e->kind == 'P' && stop_sep && is_sep(e)) return false;
        if (shown(d, e, full)) return true;
    }
    return false;
}

/* ouverture d'un element avec son attribut visibleIf (variante complete) */
static void open_tag(const conf_desc_t *d, sink_t *s, const char *tag, const conf_grp_t *e, bool full)
{
    put(s, "<");
    put(s, tag);
    if (full && e->vif != 0 && e->vif != CONF_VIF_BOARD) {
        put(s, " visibleIf=\"");
        put(s, d->vif[e->vif]);
        put(s, "\"");
    }
    put(s, ">");
}

static void gen(const conf_desc_t *d, sink_t *s, bool full)
{
    put(s, "<Grouping>");
    const conf_grp_t *e = d->grp;
    while (e->kind == 'G') {
        bool gvis = any_shown(d, e + 1, false, false, full);
        if (gvis) { open_tag(d, s, "group", e, full); put(s, "<groupName>"); put(s, e->text); put(s, "</groupName>"); }
        e++;
        while (e->kind == 'S') {
            bool svis = any_shown(d, e + 1, false, true, full);
            if (svis) {
                open_tag(d, s, "subgroup", e, full);
                put(s, "<subgroupName>"); put(s, e->text); put(s, "</subgroupName><subgroupParams>");
            }
            e++;
            for (; e->kind == 'P'; e++) {
                if (!svis) continue;
                if (is_sep(e) ? any_shown(d, e + 1, true, true, full) : shown(d, e, full)) {
                    open_tag(d, s, "param", e, full); put(s, e->text); put(s, "</param>");
                }
            }
            if (svis) put(s, "</subgroupParams></subgroup>");
        }
        if (gvis) put(s, "</group>");
    }
    put(s, "</Grouping>\n");
    if (full && d->pages[0].name) {
        put(s, "<Pages>");
        for (const conf_page_t *p = d->pages; p->name; p++) {
            put(s, "<page name=\""); put(s, p->name); put(s, "\" visibleIf=\""); put(s, p->vif); put(s, "\"/>");
        }
        put(s, "</Pages>\n");
    }
    put(s, "</ConfigParams>\n");
}

static void layout(const conf_desc_t *d, layout_t *l, bool full)
{
    sink_t s = { 0, 0, 0, 0, d->adler & 0xFFFFu, d->adler >> 16, true };
    gen(d, &s, full);
    l->text_len = s.pos;
    l->adler = (s.b << 16) | s.a;
    l->valid = true;
}

static uint32_t total_len(const conf_desc_t *d, const layout_t *l)
{
    uint32_t nblk = (l->text_len + BLK_MAX - 1u) / BLK_MAX;
    return 4u + d->z_len + l->text_len + 5u * nblk + 4u;
}

/* octets [off, off + k) du flux complet */
static void stream_copy(const conf_desc_t *d, const layout_t *l, bool full, uint32_t off, uint32_t k, uint8_t *out)
{
    uint32_t raw = d->raw_len + l->text_len;
    uint32_t st0 = 4u + d->z_len;                         /* debut des blocs non compresses */
    uint32_t nblk = (l->text_len + BLK_MAX - 1u) / BLK_MAX;
    uint32_t st1 = st0 + l->text_len + 5u * nblk;         /* debut de l'adler32 */

    /* portion de texte couverte par le morceau */
    uint8_t txt[CHUNK_MAX];
    uint32_t t0 = 0, t1 = 0;
    if (off + k > st0 && off < st1) {
        uint32_t a = off > st0 ? off - st0 : 0, b = (off + k < st1 ? off + k : st1) - st0;
        t0 = (a / (BLK_MAX + 5u)) * BLK_MAX + (a % (BLK_MAX + 5u) > 5u ? a % (BLK_MAX + 5u) - 5u : 0u);
        t1 = t0 + (b - a);
        if (t1 > l->text_len) t1 = l->text_len;
        sink_t s = { 0, t0, t1, txt, 0, 0, false };
        gen(d, &s, full);
    }
    for (uint32_t i = 0; i < k; i++) {
        uint32_t p = off + i;
        uint8_t c;
        if (p < 4u) c = (uint8_t)(raw >> (8u * (3u - p)));
        else if (p < st0) c = d->z[p - 4u];
        else if (p < st1) {
            uint32_t q = p - st0, j = q / (BLK_MAX + 5u), r = q % (BLK_MAX + 5u);
            uint32_t n = (j + 1u < nblk) ? BLK_MAX : l->text_len - j * BLK_MAX;
            switch (r) {
            case 0:  c = (j + 1u == nblk) ? 1u : 0u; break;       /* BFINAL, BTYPE = 00 */
            case 1:  c = (uint8_t)n; break;
            case 2:  c = (uint8_t)(n >> 8); break;
            case 3:  c = (uint8_t)~n; break;
            case 4:  c = (uint8_t)(~n >> 8); break;
            default: c = txt[j * BLK_MAX + r - 5u - t0]; break;
            }
        } else c = (uint8_t)(l->adler >> (8u * (3u - (p - st1))));
        out[i] = c;
    }
}

int32_t conf_desc_reply(const uint8_t *d, uint32_t n, uint8_t *out)
{
    int32_t o = 0;
    out[o++] = COMM_RESC_GET_CONF_XML;
    if (n < 9) {
        out[o++] = n >= 1 ? d[0] : 0;
        vb_put_i32(out, 0, &o);
        vb_put_i32(out, 0, &o);
        return o;
    }
    int32_t i = 0;
    uint8_t cb = vb_get_u8(d, &i);
    uint8_t conf = cb & 0x7Fu;
    bool full = (cb & 0x80u) != 0;
    int32_t len = vb_get_i32(d, &i);
    int32_t off = vb_get_i32(d, &i);
    const conf_desc_t *desc = conf == 0 ? &conf_desc_mc : conf == 1 ? &conf_desc_app : 0;
    int32_t total = 0;
    layout_t *l = desc ? &s_lay[conf][full] : 0;
    if (desc) {
        if (off == 0 || !l->valid) layout(desc, l, full);
        total = (int32_t)total_len(desc, l);
    }
    out[o++] = cb;
    vb_put_i32(out, total, &o);
    vb_put_i32(out, off, &o);
    if (desc && len > 0 && off >= 0 && off < total) {
        int32_t k = len < CHUNK_MAX ? len : CHUNK_MAX;
        if (k > total - off) k = total - off;
        stream_copy(desc, l, full, (uint32_t)off, (uint32_t)k, out + o);
        o += k;
    }
    return o;
}
