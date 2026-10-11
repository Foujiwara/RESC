#include "can.h"
#include "stm32.h"
#include "gpio.h"
#include "hw_config.h"

#define RXQ 64u
#define TXQ 64u

static can_frame_t s_rx[RXQ], s_tx[TXQ];
static volatile uint16_t s_rh, s_rt, s_th, s_tt;
static volatile uint32_t s_err;
static bool s_ok;

static bool tx_mailbox(const can_frame_t *f)
{
    uint32_t tsr = CAN1->TSR;
    int mb = (tsr & CAN_TSR_TME0) ? 0 : (tsr & CAN_TSR_TME1) ? 1 : (tsr & CAN_TSR_TME2) ? 2 : -1;
    if (mb < 0) return false;
    CAN_TxMailBox_TypeDef *m = &CAN1->sTxMailBox[mb];
    m->TDTR = f->len & 0xFu;
    m->TDLR = (uint32_t)f->data[0] | (uint32_t)f->data[1] << 8 | (uint32_t)f->data[2] << 16 | (uint32_t)f->data[3] << 24;
    m->TDHR = (uint32_t)f->data[4] | (uint32_t)f->data[5] << 8 | (uint32_t)f->data[6] << 16 | (uint32_t)f->data[7] << 24;
    m->TIR = f->std ? ((f->id & 0x7FFu) << CAN_TI0R_STID_Pos) | CAN_TI0R_TXRQ
                    : (f->id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;
    return true;
}

void CAN1_TX_IRQHandler(void)
{
    CAN1->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
    while (s_tt != s_th) {
        if (!tx_mailbox(&s_tx[s_tt])) break;
        s_tt = (uint16_t)((s_tt + 1) % TXQ);
    }
}

void CAN1_RX0_IRQHandler(void)
{
    while (CAN1->RF0R & CAN_RF0R_FMP0) {
        CAN_FIFOMailBox_TypeDef *m = &CAN1->sFIFOMailBox[0];
        uint32_t rir = m->RIR;
        {
            uint16_t n = (uint16_t)((s_rh + 1) % RXQ);
            if (n != s_rt) {
                can_frame_t *f = &s_rx[s_rh];
                f->std = (rir & CAN_RI0R_IDE) ? 0 : 1;
                f->id = f->std ? (rir >> CAN_RI0R_STID_Pos) & 0x7FFu : rir >> CAN_RI0R_EXID_Pos;
                f->len = (uint8_t)(m->RDTR & 0xFu);
                if (f->len > 8) f->len = 8;
                uint32_t l = m->RDLR, h = m->RDHR;
                for (int k = 0; k < 4; k++) { f->data[k] = (uint8_t)(l >> (8 * k)); f->data[4 + k] = (uint8_t)(h >> (8 * k)); }
                s_rh = n;
            } else {
                s_err++;
            }
        }
        CAN1->RF0R = CAN_RF0R_RFOM0;
    }
    if (CAN1->RF0R & CAN_RF0R_FOVR0) { CAN1->RF0R = CAN_RF0R_FOVR0; s_err++; }
}

void CAN1_SCE_IRQHandler(void)
{
    CAN1->MSR = CAN_MSR_ERRI;
    s_err++;
}

void can_set_bitrate(uint32_t br)
{
    /* APB1 42 MHz, 14 quanta : BS1 = 11, BS2 = 2, point d'echantillonnage 85,7 % */
    uint32_t brp = APB1_HZ / (14u * br);
    if (brp < 1) brp = 1;
    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK)) { }
    CAN1->BTR = (0u << CAN_BTR_SJW_Pos) | (1u << CAN_BTR_TS2_Pos) | (10u << CAN_BTR_TS1_Pos) | (brp - 1);
    CAN1->MCR &= ~CAN_MCR_INRQ;
    uint32_t t = 0;
    while ((CAN1->MSR & CAN_MSR_INAK) && t < 1000000u) t++;
}

bool can_init(uint32_t br)
{
#if !BOARD_HAS_CAN
    (void)br;
    return false;
#else
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    __DSB();
    gpio_cfg(BOARD_CAN_RX_PIN, GPIO_MODE_AF, false, 2, GPIO_PUPD_UP, 9);
    gpio_cfg(BOARD_CAN_TX_PIN, GPIO_MODE_AF, false, 2, GPIO_PUPD_NONE, 9);

    CAN1->MCR = CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK)) { }
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP;

    /* Filtre 0 : tout accepter, FIFO 0 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~1u;
    CAN1->FS1R |= 1u;
    CAN1->FM1R &= ~1u;
    CAN1->FFA1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    can_set_bitrate(br);
    CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FOVIE0 | CAN_IER_TMEIE;
    NVIC_SetPriority(CAN1_RX0_IRQn, PRIO_CAN);
    NVIC_SetPriority(CAN1_TX_IRQn, PRIO_CAN);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    s_ok = true;
    return true;
#endif
}

static bool send_frame(uint32_t id, bool std, const uint8_t *data, uint8_t len)
{
    if (!s_ok) return false;
    if (len > 8) len = 8;
    can_frame_t f;
    f.id = std ? (id & 0x7FFu) : (id & 0x1FFFFFFFu);
    f.std = std ? 1 : 0;
    f.len = len;
    for (int k = 0; k < 8; k++) f.data[k] = k < len ? data[k] : 0;
    NVIC_DisableIRQ(CAN1_TX_IRQn);
    bool ok = true;
    if (s_tt == s_th && tx_mailbox(&f)) {
        /* envoye directement */
    } else {
        uint16_t n = (uint16_t)((s_th + 1) % TXQ);
        if (n == s_tt) { ok = false; s_err++; }
        else { s_tx[s_th] = f; s_th = n; }
    }
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    return ok;
}

bool can_recv(can_frame_t *f)
{
    if (s_rt == s_rh) return false;
    *f = s_rx[s_rt];
    s_rt = (uint16_t)((s_rt + 1) % RXQ);
    return true;
}

bool can_send(uint32_t eid, const uint8_t *data, uint8_t len) { return send_frame(eid, false, data, len); }
bool can_send_std(uint32_t sid, const uint8_t *data, uint8_t len) { return send_frame(sid, true, data, len); }

uint32_t can_errors(void) { return s_err; }
