/*
 * motor_hw.c — Liaison materielle du coeur FOC.
 *
 * TIM1 : PWM centree (CMS=01), CH1..3 + complementaires, dead-time materiel
 *        (BOARD_DEADTIME_NS), OSSR/OSSI=1 : MOE=0 force les sorties a l'etat
 *        de repos (MOSFET bloques). CH4 en PWM2, OC4REF -> TRGO : front
 *        montant juste avant le sommet du compteur, au milieu de la fenetre
 *        ou tous les low-sides conduisent.
 * ADC  : ADC1/2/3 en mode injecte simultane, declenches par TIM1_TRGO.
 *        Rang 1 : courants ; rang 2 : tensions de phase ; rang 3 : Vbus sur
 *        son ADC, voie lente ailleurs ; rang 4 : voies lentes en rotation.
 *        Watchdog analogique sur la voie de courant de chaque ADC, arme a
 *        chaque periode seulement si la fenetre de mesure de la phase est
 *        valide : coupure du pont ~1,5 us apres l'echantillon, sans attendre
 *        la fin des conversions ni le calcul FOC.
 * Mises a jour des rapports cycliques : preload, UDIS pendant l'ecriture,
 * prises en compte au creux suivant (une demi-periode apres l'echantillon).
 */
#include "motor_hw.h"
#include "stm32.h"
#include "gpio.h"
#include "board.h"
#include "hw_config.h"
#include "foc.h"
#include "fault_codes.h"
#include "platform.h"

#define ADC_SMP_15   1u          /* 15 cycles d'echantillonnage */
#define TRIG_ADVANCE 60u         /* ticks avant le sommet : centre l'echantillonnage */

static CCMBSS struct {
    uint32_t arr;
    float f_pwm;
    float win_frac;              /* fenetre minimale en fraction de periode */
    uint8_t slow_ch[3][8];
    uint8_t slow_n[3];
    uint8_t slow_idx[3];
    uint8_t cur_slow[3][2];      /* voies lentes en cours de conversion (rangs 3, 4) */
    bool rank3_slow[3];
    uint8_t cs_adc[3];           /* ADC (0..2) de chaque courant de phase */
    uint32_t cyc, cyc_max, overruns;
    bool awd_ready;
} H;

static uint16_t s_slow_raw[18];
static TIM_TypeDef *s_enc_tim;

/* ------------------------------------------------------------ capteurs */
RAMFUNC static uint8_t hall_read(void)
{
    return (uint8_t)((gpio_read(BOARD_HALL1_PIN) ? 1u : 0u) | (gpio_read(BOARD_HALL2_PIN) ? 2u : 0u) |
                     (gpio_read(BOARD_HALL3_PIN) ? 4u : 0u));
}

void motor_hw_sensor_config(uint8_t mode, uint32_t cpr)
{
    s_enc_tim = 0;
    if (mode != 1 || cpr < 4) {
        gpio_cfg(BOARD_HALL1_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
        gpio_cfg(BOARD_HALL2_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
        gpio_cfg(BOARD_HALL3_PIN, GPIO_MODE_IN, false, 0, GPIO_PUPD_UP, 0);
        return;
    }
    TIM_TypeDef *t;
#if BOARD_ENC_TIM == 4
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    t = TIM4;
#else
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    t = TIM3;
#endif
    __DSB();
    gpio_cfg(BOARD_HALL1_PIN, GPIO_MODE_AF, false, 1, GPIO_PUPD_UP, 2);
    gpio_cfg(BOARD_HALL2_PIN, GPIO_MODE_AF, false, 1, GPIO_PUPD_UP, 2);
    t->CR1 = 0;
    t->SMCR = 3u << TIM_SMCR_SMS_Pos;                                     /* codeur x4 */
    t->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos) |
               (6u << TIM_CCMR1_IC1F_Pos) | (6u << TIM_CCMR1_IC2F_Pos);   /* filtre d'entree */
    t->CCER = 0;
    t->ARR = cpr - 1u;
    t->CNT = 0;
    t->CR1 = TIM_CR1_CEN;
    s_enc_tim = t;
}

static ADC_TypeDef *const ADCS[3] = { ADC1, ADC2, ADC3 };

/* -------------------------------------------------------------- TIM1 */
static uint32_t dtg_from_ns(uint32_t ns)
{
    const float t = 1e9f / (float)TIM1_CLK_HZ;     /* 5,95 ns */
    float n = (float)ns / t;
    if (n <= 127.0f) return (uint32_t)(n + 0.5f);
    if (n <= 2.0f * 127.0f) { uint32_t v = (uint32_t)(n / 2.0f + 0.5f) - 64u; return 0x80u | (v & 0x3Fu); }
    if (n <= 8.0f * 63.0f) { uint32_t v = (uint32_t)(n / 8.0f + 0.5f) - 32u; return 0xC0u | (v & 0x1Fu); }
    uint32_t v = (uint32_t)(n / 16.0f + 0.5f);
    if (v > 63u) v = 63u;
    return 0xE0u | ((v - 32u) & 0x1Fu);
}

static void tim1_init(float f)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __DSB();
    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

    H.f_pwm = f;
    H.arr = (uint32_t)((float)TIM1_CLK_HZ / (2.0f * f));
    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_ARPE;     /* centre mode 1 */
    TIM1->PSC = 0;
    TIM1->ARR = H.arr;
    TIM1->RCR = 0;

    /* CH1..3 PWM1 avec preload ; CH4 PWM2 avec preload */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    uint32_t ccer = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    if (!BOARD_PWM_HS_ACTIVE_HIGH) ccer |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
    if (!BOARD_PWM_LS_ACTIVE_HIGH) ccer |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
    TIM1->CCER = ccer;

    /* Etats de repos = MOSFET bloques ; OC4REF -> TRGO */
    uint32_t cr2 = (7u << TIM_CR2_MMS_Pos);
    if (!BOARD_PWM_HS_ACTIVE_HIGH) cr2 |= TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3;
    if (!BOARD_PWM_LS_ACTIVE_HIGH) cr2 |= TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N;
    TIM1->CR2 = cr2;

    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = H.arr / 2;
    TIM1->CCR4 = H.arr - TRIG_ADVANCE;

    uint32_t bdtr = TIM_BDTR_OSSR | TIM_BDTR_OSSI | dtg_from_ns(BOARD_DEADTIME_NS);
#if BOARD_BKIN_PIN != HW_PIN_NONE
    bdtr |= TIM_BDTR_BKE | (BOARD_BKIN_ACTIVE_HIGH ? TIM_BDTR_BKP : 0);
    gpio_cfg(BOARD_BKIN_PIN, GPIO_MODE_AF, false, 2, GPIO_PUPD_NONE, 1);
#endif
    TIM1->BDTR = bdtr;                              /* MOE = 0, AOE = 0 : rearmement logiciel */
    TIM1->BDTR = bdtr | (1u << TIM_BDTR_LOCK_Pos);  /* LOCK 1 : dead-time et break figes */

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;
#if BOARD_BKIN_PIN != HW_PIN_NONE
    TIM1->DIER = TIM_DIER_BIE;
    NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, PRIO_BRK);
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
#endif
    TIM1->CR1 |= TIM_CR1_CEN;

    /* Les broches ne passent sur TIM1 qu'une fois les etats de repos actifs */
    const uint8_t pins[6] = { BOARD_PWM_AH_PIN, BOARD_PWM_BH_PIN, BOARD_PWM_CH_PIN,
                              BOARD_PWM_AL_PIN, BOARD_PWM_BL_PIN, BOARD_PWM_CL_PIN };
    for (int k = 0; k < 6; k++) gpio_cfg(pins[k], GPIO_MODE_AF, false, 3, GPIO_PUPD_NONE, 1);
}

RAMFUNC static void pwm_write(const float d[3], bool en)
{
    uint32_t a = H.arr;
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CCR1 = (uint32_t)(d[0] * (float)a);
    TIM1->CCR2 = (uint32_t)(d[1] * (float)a);
    TIM1->CCR3 = (uint32_t)(d[2] * (float)a);
    TIM1->CR1 &= ~TIM_CR1_UDIS;
    if (en) {
        if (!(TIM1->BDTR & TIM_BDTR_MOE) && !(TIM1->SR & TIM_SR_BIF)) TIM1->BDTR |= TIM_BDTR_MOE;
    } else {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
}

RAMFUNC void foc_hw_kill(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    if (TIM1->SR & TIM_SR_BIF) {
        TIM1->SR = ~TIM_SR_BIF;
        foc_raise_fault(RESC_FAULT_BRK);
    }
}

/* ---------------------------------------------------------------- ADC */
static void smpr_set(ADC_TypeDef *a, uint8_t ch)
{
    if (ch <= 9) a->SMPR2 = (a->SMPR2 & ~(7u << (3 * ch))) | (ADC_SMP_15 << (3 * ch));
    else if (ch <= 18) a->SMPR1 = (a->SMPR1 & ~(7u << (3 * (ch - 10)))) | (ADC_SMP_15 << (3 * (ch - 10)));
}

static void slow_add(uint8_t ch, bool adc1_only)
{
    if (ch == HW_ADC_CH_NONE) return;
    /* ADC le moins charge compatible avec la voie */
    int best = -1;
    for (int k = 0; k < 3; k++) {
        if (adc1_only && k != 0) continue;
        if (k == 2 && !HW_ADC_CH_ON_ADC3(ch)) continue;
        if (best < 0 || H.slow_n[k] < H.slow_n[best]) best = k;
    }
    if (best >= 0 && H.slow_n[best] < 8) H.slow_ch[best][H.slow_n[best]++] = ch;
}

static uint8_t cs_ch_of_adc[3], vph_ch_of_adc[3];

RAMFUNC static uint8_t slow_next(int k)
{
    if (H.slow_n[k] == 0) return vph_ch_of_adc[k];   /* rien a convertir : voie deja utilisee */
    uint8_t c = H.slow_ch[k][H.slow_idx[k]];
    H.slow_idx[k] = (uint8_t)((H.slow_idx[k] + 1) % H.slow_n[k]);
    return c;
}

RAMFUNC static void jsqr_load(void)
{
    for (int k = 0; k < 3; k++) {
        uint8_t r3;
        if (H.rank3_slow[k]) { r3 = slow_next(k); H.cur_slow[k][0] = r3; }
        else { r3 = BOARD_VBUS_CH; H.cur_slow[k][0] = 0xFF; }
        uint8_t r4 = slow_next(k);
        H.cur_slow[k][1] = r4;
        ADCS[k]->JSQR = (3u << ADC_JSQR_JL_Pos) | ((uint32_t)r4 << 15) | ((uint32_t)r3 << 10) |
                        ((uint32_t)vph_ch_of_adc[k] << 5) | cs_ch_of_adc[k];
    }
}

static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
    __DSB();
    /* ADCCLK = 84/4 = 21 MHz ; triple injecte simultane ; Vrefint/capteur temperature */
    ADC123_COMMON->CCR = (1u << ADC_CCR_ADCPRE_Pos) | (0x15u << ADC_CCR_MULTI_Pos) | ADC_CCR_TSVREFE;

    /* Affectation des voies par ADC */
    const uint8_t cs_ch[3] = { BOARD_CS_A_CH, BOARD_CS_B_CH, BOARD_CS_C_CH };
    const uint8_t cs_adc[3] = { BOARD_CS_A_ADC, BOARD_CS_B_ADC, BOARD_CS_C_ADC };
    const uint8_t vp_ch[3] = { BOARD_VPH_A_CH, BOARD_VPH_B_CH, BOARD_VPH_C_CH };
    const uint8_t vp_adc[3] = { BOARD_VPH_A_ADC, BOARD_VPH_B_ADC, BOARD_VPH_C_ADC };
    for (int k = 0; k < 3; k++) { cs_ch_of_adc[k] = 0xFF; vph_ch_of_adc[k] = vp_ch[0]; }
    for (int x = 0; x < 3; x++) {
        if (cs_ch[x] != HW_ADC_CH_NONE) { cs_ch_of_adc[cs_adc[x] - 1] = cs_ch[x]; H.cs_adc[x] = (uint8_t)(cs_adc[x] - 1); }
        else H.cs_adc[x] = 0xFF;
        vph_ch_of_adc[vp_adc[x] - 1] = vp_ch[x];
    }
    for (int k = 0; k < 3; k++) {
        if (cs_ch_of_adc[k] == 0xFF) cs_ch_of_adc[k] = vph_ch_of_adc[k];   /* 2 shunts : ADC libre */
        H.rank3_slow[k] = (k != BOARD_VBUS_ADC - 1);
    }

    H.slow_n[0] = H.slow_n[1] = H.slow_n[2] = 0;
    slow_add(BOARD_TFET_CH, false);
    slow_add(BOARD_TFET2_CH, false);
    slow_add(BOARD_TFET3_CH, false);
    slow_add(BOARD_TMOT_CH, false);
    slow_add(BOARD_ADC_EXT1_CH, false);
    slow_add(BOARD_ADC_EXT2_CH, false);
    slow_add(BOARD_ADC_EXT3_CH, false);
    slow_add(HW_ADC_CH_VREFINT, true);

    /* Broches analogiques */
    const uint8_t an[] = { BOARD_CS_A_CH, BOARD_CS_B_CH, BOARD_CS_C_CH, BOARD_VPH_A_CH, BOARD_VPH_B_CH, BOARD_VPH_C_CH,
                           BOARD_VBUS_CH, BOARD_TFET_CH, BOARD_TFET2_CH, BOARD_TFET3_CH, BOARD_TMOT_CH,
                           BOARD_ADC_EXT1_CH, BOARD_ADC_EXT2_CH, BOARD_ADC_EXT3_CH };
    for (unsigned i = 0; i < sizeof(an); i++)
        if (an[i] != HW_ADC_CH_NONE) gpio_cfg(adc_ch_pin(an[i]), GPIO_MODE_AN, false, 0, GPIO_PUPD_NONE, 0);

    for (int k = 0; k < 3; k++) {
        ADC_TypeDef *a = ADCS[k];
        a->CR2 = 0;
        a->CR1 = ADC_CR1_SCAN;
        a->SMPR1 = a->SMPR2 = 0;
        for (uint8_t ch = 0; ch <= 17; ch++) smpr_set(a, ch);
        a->HTR = 4095;
        a->LTR = 0;
        a->CR2 = ADC_CR2_ADON;
    }
    jsqr_load();
    /* ADC1 maitre : declenchement TIM1_TRGO (JEXTSEL=0001), front montant */
    ADC1->CR2 |= (1u << ADC_CR2_JEXTSEL_Pos) | (1u << ADC_CR2_JEXTEN_Pos);
    ADC1->CR1 |= ADC_CR1_JEOCIE;
    for (int k = 0; k < 3; k++) {
        if (cs_ch_of_adc[k] != 0xFF)
            ADCS[k]->CR1 |= ADC_CR1_AWDSGL | ADC_CR1_AWDIE | ((uint32_t)cs_ch_of_adc[k] << ADC_CR1_AWDCH_Pos);
    }
    NVIC_SetPriority(ADC_IRQn, PRIO_ADC_FOC);
    NVIC_EnableIRQ(ADC_IRQn);
}

void motor_hw_set_awd(const float off[3], float i_trip, float i_scale)
{
    float s = i_scale < 0.0f ? -i_scale : i_scale;
    float cnt = 1.15f * i_trip / s;
    for (int x = 0; x < 3; x++) {
        if (H.cs_adc[x] == 0xFF) continue;
        ADC_TypeDef *a = ADCS[H.cs_adc[x]];
        float hi = off[x] + cnt, lo = off[x] - cnt;
        a->HTR = hi > 4095.0f ? 4095u : (uint32_t)hi;
        a->LTR = lo < 0.0f ? 0u : (uint32_t)lo;
    }
    H.awd_ready = true;
}

/* ---------------------------------------------------------------- ISR */
RAMFUNC void ADC_IRQHandler(void)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t sr1 = ADC1->SR, sr2 = ADC2->SR, sr3 = ADC3->SR;

    /* Watchdog analogique : coupure immediate, avant tout calcul */
    if ((sr1 | sr2 | sr3) & ADC_SR_AWD) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        ADC1->SR = ~ADC_SR_AWD;
        ADC2->SR = ~ADC_SR_AWD;
        ADC3->SR = ~ADC_SR_AWD;
        foc_raise_fault(RESC_FAULT_ABS_OVER_CURRENT);
    }
    if (!(sr1 & ADC_SR_JEOC)) return;
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);

    uint16_t r[3][4];
    for (int k = 0; k < 3; k++) {
        r[k][0] = (uint16_t)ADCS[k]->JDR1;
        r[k][1] = (uint16_t)ADCS[k]->JDR2;
        r[k][2] = (uint16_t)ADCS[k]->JDR3;
        r[k][3] = (uint16_t)ADCS[k]->JDR4;
    }

    foc_adc_t in;
    in.cs[0] = r[BOARD_CS_A_ADC - 1][0];
    in.cs[1] = r[BOARD_CS_B_ADC - 1][0];
    in.cs[2] = BOARD_CS_TOPOLOGY == HW_CS_LOWSIDE_2 ? 2048 : r[BOARD_CS_C_ADC - 1][0];
    in.vph[0] = r[BOARD_VPH_A_ADC - 1][1];
    in.vph[1] = r[BOARD_VPH_B_ADC - 1][1];
    in.vph[2] = r[BOARD_VPH_C_ADC - 1][1];
    in.vbus = r[BOARD_VBUS_ADC - 1][2];
    in.hall = hall_read();
    in.enc = s_enc_tim ? s_enc_tim->CNT : 0u;

    for (int k = 0; k < 3; k++) {
        if (H.cur_slow[k][0] != 0xFF) s_slow_raw[H.cur_slow[k][0]] = r[k][2];
        s_slow_raw[H.cur_slow[k][1]] = r[k][3];
    }
    jsqr_load();

    foc_pwm_t out;
    foc_isr(&in, &out);
    pwm_write(out.duty, out.enable);

    /* Watchdog analogique arme uniquement sur les phases dont l'echantillon suivant sera valide */
    if (H.awd_ready) {
        for (int x = 0; x < 3; x++) {
            if (H.cs_adc[x] == 0xFF) continue;
            bool ok = !out.enable || BOARD_CS_TOPOLOGY == HW_CS_INLINE_3 || (1.0f - out.duty[x]) >= H.win_frac;
            if (ok) ADCS[H.cs_adc[x]]->CR1 |= ADC_CR1_JAWDEN;
            else ADCS[H.cs_adc[x]]->CR1 &= ~ADC_CR1_JAWDEN;
        }
    }

    uint32_t dt = DWT->CYCCNT - t0;
    H.cyc = dt;
    if (dt > H.cyc_max) H.cyc_max = dt;
    if (ADC1->SR & ADC_SR_JEOC) H.overruns++;   /* nouvel echantillon arrive pendant le calcul */
    g_foc_telem.isr_cycles = dt;
    g_foc_telem.isr_cycles_max = H.cyc_max;
    g_foc_telem.n_overrun = H.overruns;
}

/* -------------------------------------------------- defaut du driver */
#if BOARD_GD_FAULT_PIN != HW_PIN_NONE
#define FAULT_LINE HW_PIN_NUM(BOARD_GD_FAULT_PIN)
static void exti_fault(void)
{
    if (EXTI->PR & (1u << FAULT_LINE)) {
        EXTI->PR = 1u << FAULT_LINE;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        foc_raise_fault(RESC_FAULT_DRV);
    }
}
#if FAULT_LINE == 0
void EXTI0_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI0_IRQn
#elif FAULT_LINE == 1
void EXTI1_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI1_IRQn
#elif FAULT_LINE == 2
void EXTI2_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI2_IRQn
#elif FAULT_LINE == 3
void EXTI3_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI3_IRQn
#elif FAULT_LINE == 4
void EXTI4_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI4_IRQn
#elif FAULT_LINE <= 9
void EXTI9_5_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI9_5_IRQn
#else
void EXTI15_10_IRQHandler(void) { exti_fault(); }
#define FAULT_IRQ EXTI15_10_IRQn
#endif
static void fault_exti_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DSB();
    uint32_t n = FAULT_LINE;
    SYSCFG->EXTICR[n >> 2] = (SYSCFG->EXTICR[n >> 2] & ~(0xFu << ((n & 3) * 4))) |
                             ((uint32_t)HW_PIN_PORT(BOARD_GD_FAULT_PIN) << ((n & 3) * 4));
    if (BOARD_GD_FAULT_ACTIVE_LOW) EXTI->FTSR |= 1u << n; else EXTI->RTSR |= 1u << n;
    EXTI->PR = 1u << n;
    EXTI->IMR |= 1u << n;
    NVIC_SetPriority(FAULT_IRQ, PRIO_FAULT_IO);
    NVIC_EnableIRQ(FAULT_IRQ);
}
#else
static void fault_exti_init(void) { }
#endif

/* ---------------------------------------------------------------- API */
void motor_hw_init(float f)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    H.win_frac = (float)HWLIM_T_WIN_MIN_NS * 1e-9f * f;
    adc_init();
    tim1_init(f);
    fault_exti_init();
}

void motor_hw_set_freq(float f)
{
    if (TIM1->BDTR & TIM_BDTR_MOE) return;
    H.f_pwm = f;
    H.arr = (uint32_t)((float)TIM1_CLK_HZ / (2.0f * f));
    H.win_frac = (float)HWLIM_T_WIN_MIN_NS * 1e-9f * f;
    TIM1->ARR = H.arr;
    TIM1->CCR4 = H.arr - TRIG_ADVANCE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = H.arr / 2;
}

float motor_hw_freq(void) { return H.f_pwm; }
uint16_t motor_hw_slow_raw(uint8_t ch) { return ch < 18 ? s_slow_raw[ch] : 0; }
uint32_t motor_hw_isr_cycles(void) { return H.cyc; }
uint32_t motor_hw_isr_cycles_max(void) { return H.cyc_max; }
uint32_t motor_hw_overruns(void) { return H.overruns; }
void motor_hw_reset_stats(void) { H.cyc_max = 0; H.overruns = 0; }

void plat_pwm_set_freq(float f) { motor_hw_set_freq(f); }
void plat_sensor_config(uint8_t mode, uint32_t cpr) { motor_hw_sensor_config(mode, cpr); }
