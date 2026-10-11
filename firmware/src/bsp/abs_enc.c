/*
 * abs_enc.c — Codeurs de position absolus (protocoles des capteurs, lus dans l'ISR FOC).
 *
 * SPI logiciel : horloge au repos basse, donnee lue pendant le niveau haut de l'horloge (vote
 * sur 5 lectures), comme les capteurs le specifient (front montant : mise a jour, lecture
 * avant le front descendant). SSC (TLE5012) : une seule ligne de donnees, ecrite au front
 * montant, lue au niveau bas. SPI1 materiel : echanges scrutes (quelques microsecondes).
 */
#include <math.h>
#include "abs_enc.h"
#include "stm32.h"
#include "gpio.h"
#include "hw_config.h"
#include "platform.h"
#include "inputs.h"
#include "abs_enc_proto.h"
#include "uart.h"
#include "resc_conf.h"

enum {
    PM_AS5047 = 2, PM_AD2S1205 = 3, PM_SINCOS = 4, PM_MT6816 = 7, PM_AS5X47U = 8,
    PM_TLE5012 = 10, PM_PWM = 13, PM_MA782 = 15, PM_AMT22 = 16,
    PM_TS5700 = 5, PM_TS5700_MT = 6, PM_BISSC = 9, PM_TLE5012_HW = 11, PM_PWM_ABI = 14
};

static uint8_t s_mode;
static uint32_t s_last = 0, s_err;
static float s_err_rate;
static abs_enc_sincos_t s_sc;
static float s_sin_f, s_cos_f;
static uint8_t s_cs, s_sck, s_miso, s_mosi;     /* broches du SPI logiciel */
/* BiSS-C : 8 octets recus par interruption de SPI1 */
static volatile uint8_t s_bx[8];
static volatile uint8_t s_bn;
static volatile bool s_bdone, s_bbusy;
static uint8_t s_crc6[64];
/* TS5700N8501 */
static uart_t *s_ts_u;
static uint8_t s_ts_de;
static uint32_t s_ts_t;
static bool s_ts_wait;
/* PWM + ABI */
static TIM_TypeDef *s_abi;

void abs_enc_set_abi_timer(void *tim) { s_abi = (TIM_TypeDef *)tim; }
bool abs_enc_uart_owned(void) { return s_mode == PM_TS5700 || s_mode == PM_TS5700_MT; }

uint32_t abs_enc_errors(void) { return s_err; }
float abs_enc_error_rate(void) { return s_err_rate; }

/* ---------------------------------------------------------------- SPI logiciel */
RAMFUNC static void dly(int n) { for (volatile int i = 0; i < n; i++) __NOP(); }

RAMFUNC static uint16_t bb_xfer(uint16_t send, int bits)
{
    uint16_t rx = 0;
    for (int b = 0; b < bits; b++) {
        if (s_mosi != HW_PIN_NONE) { gpio_write(s_mosi, (send >> (bits - 1)) & 1u); send <<= 1; }
        gpio_write(s_sck, true);
        dly(2);
        int v = 0;
        for (int k = 0; k < 5; k++) v += gpio_read(s_miso) ? 1 : 0;
        rx = (uint16_t)(rx << 1) | (v > 2 ? 1u : 0u);
        gpio_write(s_sck, false);
        dly(2);
    }
    return rx;
}

RAMFUNC static void bb_begin(void) { dly(6); gpio_write(s_cs, false); dly(6); }
RAMFUNC static void bb_end(void) { dly(6); gpio_write(s_cs, true); dly(6); }

/* SSC (TLE5012) : ligne de donnees bidirectionnelle (s_miso) */
RAMFUNC static uint16_t ssc_xfer(uint16_t send, bool write)
{
    uint16_t rx = 0;
    if (write) gpio_cfg(s_miso, GPIO_MODE_OUT, false, 3, GPIO_PUPD_NONE, 0);
    else gpio_cfg(s_miso, GPIO_MODE_IN, false, 3, GPIO_PUPD_UP, 0);
    for (int b = 0; b < 16; b++) {
        gpio_write(s_sck, true);
        if (write) { gpio_write(s_miso, (send >> 15) & 1u); send <<= 1; }
        dly(2);
        gpio_write(s_sck, false);
        dly(2);
        rx = (uint16_t)(rx << 1) | (gpio_read(s_miso) ? 1u : 0u);
    }
    return rx;
}

/* ---------------------------------------------------------------- SPI1 materiel */
RAMFUNC static uint16_t spi1_xfer(uint16_t v)
{
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    SPI1->DR = v;
    while (!(SPI1->SR & SPI_SR_RXNE)) { }
    return (uint16_t)SPI1->DR;
}

static void spi1_init(uint32_t cr1)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    __DSB();
    gpio_cfg(BOARD_SPI_SCK_PIN, GPIO_MODE_AF, false, 3, GPIO_PUPD_NONE, 5);
    gpio_cfg(BOARD_SPI_MISO_PIN, GPIO_MODE_AF, false, 3, GPIO_PUPD_UP, 5);
    gpio_cfg(BOARD_SPI_MOSI_PIN, GPIO_MODE_AF, false, 3, GPIO_PUPD_NONE, 5);
    gpio_out(BOARD_SPI_NSS_PIN, true);
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = cr1 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* ---------------------------------------------------------------- configuration */
void abs_enc_config(uint8_t m, const abs_enc_sincos_t *sc)
{
    if (abs_enc_uart_owned() && s_ts_u) {
        /* fin du TS5700N8501 : UART rendu au port COMM a la vitesse des applications */
        uart_set_baud(s_ts_u, g_appconf.app_uart_baudrate ? g_appconf.app_uart_baudrate : 115200u);
        uart_attach_pins(BOARD_UART_INST, BOARD_UART_TX_PIN, BOARD_UART_RX_PIN);
        gpio_cfg(s_ts_de, GPIO_MODE_AN, false, 0, GPIO_PUPD_NONE, 0);
    }
    NVIC_DisableIRQ(SPI1_IRQn);
    s_mode = abs_enc_supported(m) ? m : 0;
    if (sc) s_sc = *sc;
    s_err = 0;
    s_err_rate = 0.0f;
    s_mosi = HW_PIN_NONE;
    switch (s_mode) {
    case PM_AS5047: case PM_AMT22: case PM_TLE5012:
        s_cs = BOARD_HALL3_PIN; s_sck = BOARD_HALL1_PIN; s_miso = BOARD_HALL2_PIN;
        gpio_out(s_cs, true);
        gpio_out(s_sck, false);
        gpio_cfg(s_miso, GPIO_MODE_IN, false, 3, GPIO_PUPD_UP, 0);
        break;
    case PM_TLE5012_HW:
        /* SSC sur les broches du port COMM : donnees sur MOSI */
        s_cs = BOARD_SPI_NSS_PIN; s_sck = BOARD_SPI_SCK_PIN; s_miso = BOARD_SPI_MOSI_PIN;
        gpio_out(s_cs, true);
        gpio_out(s_sck, false);
        gpio_cfg(s_miso, GPIO_MODE_IN, false, 3, GPIO_PUPD_UP, 0);
        break;
    case PM_BISSC:      /* mode 3, 8 bits, APB2/32 ; reception par interruption */
        for (int i = 0; i < 64; i++) {
            int c = i;
            for (int j = 0; j < 6; j++) c = (c & 0x20) ? ((c << 1) ^ 0x43) : (c << 1);
            s_crc6[i] = (uint8_t)(c & 0x3F);
        }
        spi1_init(SPI_CR1_CPOL | SPI_CR1_CPHA | (4u << SPI_CR1_BR_Pos));
        SPI1->CR2 = SPI_CR2_RXNEIE;
        s_bdone = s_bbusy = false;
        NVIC_SetPriority(SPI1_IRQn, PRIO_ADC_FOC + 1);
        NVIC_EnableIRQ(SPI1_IRQn);
        break;
    case PM_TS5700: case PM_TS5700_MT:
        /* RS485 a 2.5 Mbit/s : reception par l'UART du port COMM, emission logicielle sur TX,
         * sens (DE) sur la broche de l'entree ADC 1 */
        s_ts_u = uart_get(BOARD_UART_INST);
        s_ts_de = BOARD_ADC_EXT1_CH != HW_ADC_CH_NONE ? adc_ch_pin(BOARD_ADC_EXT1_CH) : HW_PIN_NONE;
        if (!s_ts_u || s_ts_de == HW_PIN_NONE) { s_mode = 0; break; }
        uart_set_baud(s_ts_u, 2500000u);
        uart_attach_pins(BOARD_UART_INST, HW_PIN_NONE, BOARD_UART_RX_PIN);
        gpio_out(BOARD_UART_TX_PIN, true);
        gpio_out(s_ts_de, false);
        s_ts_wait = false;
        break;
    case PM_AD2S1205:
        s_cs = BOARD_SPI_NSS_PIN; s_sck = BOARD_SPI_SCK_PIN; s_miso = BOARD_SPI_MISO_PIN;
        gpio_out(s_cs, true);
        gpio_out(s_sck, true);
        gpio_cfg(s_miso, GPIO_MODE_IN, false, 3, GPIO_PUPD_UP, 0);
        break;
    case PM_MT6816:      /* mode 3, 16 bits, APB2/8 = 10.5 MHz */
        spi1_init(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF | (2u << SPI_CR1_BR_Pos));
        break;
    case PM_AS5X47U:     /* mode 1, 8 bits, APB2/16 */
        spi1_init(SPI_CR1_CPHA | (3u << SPI_CR1_BR_Pos));
        break;
    case PM_MA782:       /* mode 0, 16 bits, APB2/32 */
        spi1_init(SPI_CR1_DFF | (4u << SPI_CR1_BR_Pos));
        break;
    case PM_PWM: case PM_PWM_ABI:
#if BOARD_SERVO_PIN != HW_PIN_NONE && BOARD_ENC_TIM != 4
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        __DSB();
        gpio_cfg(BOARD_SERVO_PIN, GPIO_MODE_AF, false, 1, GPIO_PUPD_DOWN, 2);
        TIM4->CR1 = 0;
        TIM4->PSC = (uint16_t)(2u * APB1_HZ / 1000000u - 1u);       /* 1 MHz */
        TIM4->ARR = 0xFFFF;
        TIM4->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (2u << TIM_CCMR1_CC2S_Pos) |
                      (2u << TIM_CCMR1_IC1F_Pos) | (2u << TIM_CCMR1_IC2F_Pos);
        TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
        TIM4->SMCR = (5u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos);
        TIM4->EGR = TIM_EGR_UG;
        TIM4->SR = 0;
        TIM4->CR1 = TIM_CR1_CEN;
#else
        s_mode = 0;
#endif
        break;
    default:
        break;
    }
}

/* ---------------------------------------------------------------- lecture (ISR) */
RAMFUNC static void bad(void)
{
    s_err++;
    s_err_rate += 0.001f * (1.0f - s_err_rate);
}

RAMFUNC static void good(uint32_t c)
{
    s_last = c & (ABS_ENC_CPR - 1u);
    s_err_rate -= 0.001f * s_err_rate;
}

RAMFUNC bool abs_enc_sample(uint32_t *counts)
{
    switch (s_mode) {
    case PM_AS5047: {
        bb_begin();
        uint16_t p = bb_xfer(0xFFFF, 16);
        bb_end();
        if (p != 0x0000 && p != 0xFFFF && abs_parity_even(p) && !(p & 0x4000u)) good(p & 0x3FFFu); else bad();
    } break;

    case PM_AMT22: {
        /* deux octets, pause entre les octets ; 2 bits de controle de parite impaire */
        bb_begin();
        dly(12);
        uint16_t b0 = bb_xfer(0x00, 8);
        dly(12);
        uint16_t b1 = bb_xfer(0x00, 8);
        bb_end();
        uint16_t p = (uint16_t)(b0 << 8 | b1);
        if (abs_amt22_ok(p)) good(p & 0x3FFFu); else bad();
    } break;

    case PM_TLE5012_HW:
    case PM_TLE5012: {
        /* lecture de AVAL (adresse 0x02) avec mot de securite ; CRC8 (0x1D, graine 0xFF) */
        uint16_t cmd = (uint16_t)((1u << 15) | (0x02u << 4) | 1u);
        bb_begin();
        ssc_xfer(cmd, true);
        uint16_t v = ssc_xfer(0, false);
        uint16_t sw = ssc_xfer(0, false);
        bb_end();
        bool ok = abs_tle5012_ok(cmd, v, sw);
        if (ok) good((uint32_t)(v & 0x7FFFu) >> 1); else bad();
    } break;

    case PM_AD2S1205: {
        gpio_write(s_sck, true);
        dly(6);
        bb_begin();
        uint16_t p = bb_xfer(0xFFFF, 16);
        bb_end();
        bool rdvel = (p & 0x0008u) != 0;
        bool dos = (p & 0x04u) == 0, lot = (p & 0x02u) == 0;
        /* trame de 16 bits a parite impaire */
        if (p != 0 && rdvel && !abs_parity_even(p) && !dos && !lot) good((uint32_t)((p >> 4) & 0x0FFFu) << 2); else bad();
    } break;

    case PM_MT6816: {
        gpio_write(BOARD_SPI_NSS_PIN, false);
        dly(2);
        uint16_t d3 = spi1_xfer(0x8300);
        gpio_write(BOARD_SPI_NSS_PIN, true);
        dly(6);
        gpio_write(BOARD_SPI_NSS_PIN, false);
        dly(2);
        uint16_t d4 = spi1_xfer(0x8400);
        gpio_write(BOARD_SPI_NSS_PIN, true);
        uint16_t p = (uint16_t)((d3 & 0xFFu) << 8 | (d4 & 0xFFu));
        if (abs_parity_even(p) && !(p & 0x0002u)) good(p >> 2); else bad();
    } break;

    case PM_AS5X47U: {
        /* commande de lecture de la position (0x3FFF | lecture) et son CRC ; la reponse arrive
         * avec la commande suivante (position de l'echange precedent) */
        uint8_t rx[3];
        gpio_write(BOARD_SPI_NSS_PIN, false);
        dly(2);
        rx[0] = (uint8_t)spi1_xfer(0x7F);
        rx[1] = (uint8_t)spi1_xfer(0xFF);
        rx[2] = (uint8_t)spi1_xfer(0xBD);
        gpio_write(BOARD_SPI_NSS_PIN, true);
        uint8_t c = (uint8_t)(abs_crc8_1d(rx, 2, 0xC4) ^ 0xFFu);
        uint16_t d = (uint16_t)(rx[0] << 8 | rx[1]);
        if (c == rx[2] && !(d & 0x4000u)) good(d & 0x3FFFu); else bad();
    } break;

    case PM_MA782: {
        gpio_write(BOARD_SPI_NSS_PIN, false);
        dly(2);
        uint16_t a = spi1_xfer(0x0000);
        gpio_write(BOARD_SPI_NSS_PIN, true);
        a &= 0xFF80u;                                 /* 9 bits de resolution */
        good((uint32_t)a >> 2);
    } break;

    case PM_SINCOS: {
        float sn = (inputs_adc_v(1) - s_sc.s_offset) * s_sc.s_gain;
        float cs = (inputs_adc_v(2) - s_sc.c_offset) * s_sc.c_gain;
        float k = s_sc.filter > 0.0f && s_sc.filter <= 1.0f ? s_sc.filter : 1.0f;
        s_sin_f += k * (sn - s_sin_f);
        s_cos_f += k * (cs - s_cos_f);
        sn = s_sin_f;
        cs = (s_cos_f + s_sin_f * s_sc.sph) / (s_sc.cph != 0.0f ? s_sc.cph : 1.0f);
        float m2 = sn * sn + cs * cs;
        if (m2 > 1.3f * 1.3f || m2 < 0.7f * 0.7f) { bad(); break; }
        float a = atan2f(sn, cs) + 3.14159265f;          /* 0..2 pi */
        good((uint32_t)(a * ((float)ABS_ENC_CPR / 6.2831853f)));
    } break;

    case PM_PWM:
        if (TIM4->SR & TIM_SR_CC1IF) {
            uint32_t per = TIM4->CCR1, w = TIM4->CCR2;
            if (per > 10u) good((uint32_t)((float)(w < per ? w : per) / (float)per * (float)ABS_ENC_CPR));
            else bad();
        }
        break;

    case PM_PWM_ABI:
        /* ABI incremental ; chaque periode PWM recale le compteur sur la position absolue */
        if (!s_abi) return false;
        if (TIM4->SR & TIM_SR_CC1IF) {
            uint32_t per = TIM4->CCR1, w = TIM4->CCR2;
            if (per > 10u) s_abi->CNT = (uint32_t)((float)(w < per ? w : per) / (float)per * (float)(s_abi->ARR + 1u)) % (s_abi->ARR + 1u);
        }
        *counts = s_abi->CNT;
        return true;

    case PM_BISSC:
        if (s_bdone) {
            /* trame alignee sur le premier 1 (debut), 2 bits retires, position 22 bits, erreur,
             * avertissement, CRC6 inverse */
            uint64_t v = 0;
            for (int k = 0; k < 8; k++) v = v << 8 | s_bx[k];
            s_bdone = false;
            if (v) {
                v <<= __builtin_clzll(v);
                v &= 0x3FFFFFFFFFFFFFFFull;
                int nb = 64 - (v ? __builtin_clzll(v) : 64);
                if (nb >= 22 + 10) v >>= (unsigned)(nb - (22 + 10));
                uint8_t crc_rx = (uint8_t)(v & 0x3Fu);
                uint32_t d = (uint32_t)(v >> 6) & ((1u << 24) - 1u);
                uint8_t c = (uint8_t)((d >> 30) & 0x03u);
                for (int sh = 18; sh >= 0; sh -= 6) c = s_crc6[((d >> sh) & 0x3Fu) ^ c];
                c = (uint8_t)(0x3Fu & ~c);
                uint32_t pos = (d >> 2) & ((1u << 22) - 1u);
                if (c == crc_rx) good(pos >> 8); else bad();
            } else {
                bad();
            }
        }
        if (!s_bbusy) {
            /* trame suivante : 8 octets, l'interruption enchaine */
            s_bn = 0;
            s_bbusy = true;
            gpio_write(BOARD_SPI_NSS_PIN, false);
            (void)SPI1->DR;
            SPI1->DR = 0xFF;
        }
        break;

    case PM_TS5700: case PM_TS5700_MT:
        break;      /* echanges dans abs_enc_poll() */

    default:
        return false;
    }
    *counts = s_last;
    return true;
}

/* BiSS-C : un octet recu, le suivant est demande ; fin de trame apres 8 octets */
RAMFUNC void SPI1_IRQHandler(void)
{
    if (!(SPI1->SR & SPI_SR_RXNE)) return;
    uint8_t b = (uint8_t)SPI1->DR;
    if (!s_bbusy) return;
    s_bx[s_bn++] = b;
    if (s_bn < 8) {
        SPI1->DR = 0xFF;
    } else {
        gpio_write(BOARD_SPI_NSS_PIN, true);
        s_bbusy = false;
        s_bdone = true;
    }
}

/* TS5700N8501 : un octet a 2.5 Mbit/s, poids fort en tete comme le pilote VESC, emission
 * logicielle cadencee sur le compteur de cycles, sens (DE) pilote autour */
static void ts_send(uint8_t b)
{
    const uint32_t bit = 168000000u / 2500000u;      /* 67 cycles */
    __disable_irq();
    gpio_write(s_ts_de, true);
    uint32_t t = DWT->CYCCNT + bit;
    while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    gpio_write(BOARD_UART_TX_PIN, false);            /* depart */
    t += bit;
    for (int i = 0; i < 8; i++) {
        while ((int32_t)(DWT->CYCCNT - t) < 0) { }
        gpio_write(BOARD_UART_TX_PIN, (b & (0x80u >> i)) != 0);
        t += bit;
    }
    while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    gpio_write(BOARD_UART_TX_PIN, true);             /* arret */
    t += bit;
    while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    gpio_write(s_ts_de, false);                      /* reception : la reponse part apres ~3 us */
    __enable_irq();
}

void abs_enc_poll(uint32_t now_us)
{
    if (!abs_enc_uart_owned() || !s_ts_u) return;
    if (s_ts_wait) {
        if (now_us - s_ts_t < 600u) return;          /* 11 octets en 44 us, marge */
        uint8_t rep[11];
        int n = 0, c;
        while ((c = uart_getc(s_ts_u)) >= 0) if (n < 11) rep[n++] = (uint8_t)c;
        uint8_t x = 0;
        for (int i = 0; i < n - 1; i++) x ^= rep[i];
        if (n == 11 && x == rep[10]) {
            uint32_t pos = (uint32_t)rep[2] | (uint32_t)rep[3] << 8 | (uint32_t)rep[4] << 16;   /* 17 bits */
            good(pos >> 3);
        } else {
            bad();
        }
        s_ts_wait = false;
    }
    if (now_us - s_ts_t >= 1000u) {
        while (uart_getc(s_ts_u) >= 0) { }
        ts_send(0x58);                               /* lecture de la position (0b01011000) */
        s_ts_t = now_us;
        s_ts_wait = true;
    }
}
