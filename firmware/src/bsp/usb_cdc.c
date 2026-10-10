/*
 * usb_cdc.c — Peripherique USB CDC ACM sur l'OTG FS du STM32F405, sans DMA.
 *
 * EP0 controle (64 o), EP1 bulk IN/OUT (64 o, donnees), EP2 interruption IN
 * (notifications CDC). FIFO : RX 128 mots, TX0 32, TX1 128, TX2 16 (304/320).
 */
#include <string.h>
#include "usb_cdc.h"
#include "stm32.h"
#include "gpio.h"
#include "board.h"

#define OTG      USB_OTG_FS
#define DEV      ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define INEP(n)  ((USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (n) * USB_OTG_EP_REG_SIZE))
#define OUTEP(n) ((USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (n) * USB_OTG_EP_REG_SIZE))
#define FIFO(n)  (*(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (n) * USB_OTG_FIFO_SIZE))
#define PCGCCTL  (*(volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

#define EP0_MPS 64u
#define EP1_MPS 64u

/* ------------------------------------------------------------ descripteurs */
static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, EP0_MPS,
    0x83, 0x04, 0x40, 0x57,       /* VID 0x0483, PID 0x5740 */
    0x00, 0x02, 1, 2, 3, 1
};

static const uint8_t cfg_desc[67] = {
    9, 2, 67, 0, 2, 1, 0, 0xC0, 50,
    /* Interface 0 : communication */
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,              /* header */
    5, 0x24, 0x01, 0x00, 0x01,              /* call management */
    4, 0x24, 0x02, 0x02,                    /* ACM */
    5, 0x24, 0x06, 0x00, 0x01,              /* union */
    7, 5, 0x82, 0x03, 16, 0, 0x10,          /* EP2 IN interruption */
    /* Interface 1 : donnees */
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, 0x01, 0x02, EP1_MPS, 0, 0,        /* EP1 OUT bulk */
    7, 5, 0x81, 0x02, EP1_MPS, 0, 0         /* EP1 IN bulk */
};

static const char *const str_ascii[3] = { "RESC", "RESC Motor Controller", 0 };

/* ------------------------------------------------------------ etat */
#define RXN 2048u
#define TXN 4096u
static uint8_t s_rx[RXN], s_tx[TXN];
static volatile uint16_t s_rh, s_rt, s_th, s_tt;
static volatile bool s_configured, s_tx_busy, s_need_zlp;
static uint8_t s_setup[8];
static uint8_t s_ep0_buf[64];
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };   /* 115200 8N1 */
static uint8_t s_ep0_out_req;      /* requete en attente de sa phase de donnees */
static uint8_t s_str_buf[64];

/* ------------------------------------------------------------ FIFO */
static void fifo_write(uint32_t ep, const uint8_t *d, uint32_t n)
{
    uint32_t words = (n + 3) / 4;
    for (uint32_t w = 0; w < words; w++) {
        uint32_t v = 0;
        for (uint32_t b = 0; b < 4; b++) {
            uint32_t i = w * 4 + b;
            if (i < n) v |= (uint32_t)d[i] << (8 * b);
        }
        FIFO(ep) = v;
    }
}

static void fifo_read(uint8_t *d, uint32_t n)
{
    uint32_t words = (n + 3) / 4;
    for (uint32_t w = 0; w < words; w++) {
        uint32_t v = FIFO(0);
        for (uint32_t b = 0; b < 4; b++) {
            uint32_t i = w * 4 + b;
            if (d && i < n) d[i] = (uint8_t)(v >> (8 * b));
        }
    }
}

/* ------------------------------------------------------------ EP0 */
static void ep0_out_arm(void)
{
    OUTEP(0)->DOEPTSIZ = (3u << USB_OTG_DOEPTSIZ_STUPCNT_Pos) | (1u << 19) | EP0_MPS;
    OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

static void ep0_send(const uint8_t *d, uint32_t n)
{
    uint32_t pk = n == 0 ? 1 : (n + EP0_MPS - 1) / EP0_MPS;
    INEP(0)->DIEPTSIZ = (pk << 19) | n;
    INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    if (n) fifo_write(0, d, n);
    ep0_out_arm();      /* phase de statut (ZLP OUT) ou SETUP suivant */
}

static void ep0_stall(void)
{
    INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    ep0_out_arm();
}

static uint32_t string_desc(uint8_t idx, const uint8_t **out)
{
    uint8_t *b = s_str_buf;
    if (idx == 0) {
        b[0] = 4; b[1] = 3; b[2] = 0x09; b[3] = 0x04;
        *out = b;
        return 4;
    }
    char tmp[25];
    const char *s;
    if (idx == 3) {
        const uint8_t *u = board_uid();
        static const char hx[] = "0123456789ABCDEF";
        for (int i = 0; i < 12; i++) { tmp[2 * i] = hx[u[i] >> 4]; tmp[2 * i + 1] = hx[u[i] & 15]; }
        tmp[24] = 0;
        s = tmp;
    } else if (idx <= 2) {
        s = str_ascii[idx - 1];
    } else {
        return 0;
    }
    uint32_t n = 0;
    while (s[n] && n < 30) { b[2 + 2 * n] = (uint8_t)s[n]; b[3 + 2 * n] = 0; n++; }
    b[0] = (uint8_t)(2 + 2 * n);
    b[1] = 3;
    *out = b;
    return b[0];
}

static void configure_eps(void)
{
    /* EP1 IN bulk (FIFO 1), EP2 IN interruption (FIFO 2), EP1 OUT bulk */
    INEP(1)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | (2u << USB_OTG_DIEPCTL_EPTYP_Pos) | EP1_MPS |
                       (1u << USB_OTG_DIEPCTL_TXFNUM_Pos) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK;
    INEP(2)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP | (3u << USB_OTG_DIEPCTL_EPTYP_Pos) | 16u |
                       (2u << USB_OTG_DIEPCTL_TXFNUM_Pos) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK;
    OUTEP(1)->DOEPTSIZ = (1u << 19) | EP1_MPS;
    OUTEP(1)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP | (2u << USB_OTG_DOEPCTL_EPTYP_Pos) | EP1_MPS |
                        USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
    DEV->DAINTMSK |= (1u << 1) | (1u << 2) | (1u << 17);
    s_tx_busy = false;
    s_configured = true;
}

static void handle_setup(void)
{
    uint8_t bm = s_setup[0], req = s_setup[1];
    uint16_t val = (uint16_t)(s_setup[2] | s_setup[3] << 8);
    uint16_t len = (uint16_t)(s_setup[6] | s_setup[7] << 8);

    if ((bm & 0x60) == 0x00) {               /* requetes standard */
        switch (req) {
        case 0x06: {                         /* GET_DESCRIPTOR */
            const uint8_t *d = 0;
            uint32_t n = 0;
            uint8_t type = (uint8_t)(val >> 8);
            if (type == 1) { d = dev_desc; n = sizeof(dev_desc); }
            else if (type == 2) { d = cfg_desc; n = sizeof(cfg_desc); }
            else if (type == 3) { n = string_desc((uint8_t)val, &d); }
            if (!d || !n) { ep0_stall(); return; }
            if (n > len) n = len;
            ep0_send(d, n);
            return;
        }
        case 0x05:                           /* SET_ADDRESS : appliquee avant la phase de statut */
            DEV->DCFG = (DEV->DCFG & ~USB_OTG_DCFG_DAD) | ((uint32_t)(val & 0x7F) << USB_OTG_DCFG_DAD_Pos);
            ep0_send(0, 0);
            return;
        case 0x09:                           /* SET_CONFIGURATION */
            if (val) configure_eps(); else s_configured = false;
            ep0_send(0, 0);
            return;
        case 0x08: {                         /* GET_CONFIGURATION */
            uint8_t c = s_configured ? 1 : 0;
            ep0_send(&c, 1);
            return;
        }
        case 0x00: {                         /* GET_STATUS */
            uint8_t st[2] = { 0, 0 };
            ep0_send(st, len < 2 ? len : 2);
            return;
        }
        case 0x0B:                           /* SET_INTERFACE */
        case 0x01:                           /* CLEAR_FEATURE */
        case 0x03:                           /* SET_FEATURE */
            ep0_send(0, 0);
            return;
        case 0x0A: {                         /* GET_INTERFACE */
            uint8_t a = 0;
            ep0_send(&a, 1);
            return;
        }
        default:
            ep0_stall();
            return;
        }
    }
    if ((bm & 0x60) == 0x20) {               /* requetes de classe CDC */
        switch (req) {
        case 0x20:                           /* SET_LINE_CODING : phase de donnees OUT */
            s_ep0_out_req = 0x20;
            ep0_out_arm();
            return;
        case 0x21:                           /* GET_LINE_CODING */
            ep0_send(s_line_coding, len < 7 ? len : 7);
            return;
        case 0x22:                           /* SET_CONTROL_LINE_STATE */
        case 0x23:                           /* SEND_BREAK */
            ep0_send(0, 0);
            return;
        default:
            ep0_stall();
            return;
        }
    }
    ep0_stall();
}

/* ------------------------------------------------------------ emission */
static void tx_kick(void)
{
    if (!s_configured || s_tx_busy) return;
    uint32_t avail = (uint32_t)((s_th + TXN - s_tt) % TXN);
    if (avail == 0) {
        if (s_need_zlp) {
            s_need_zlp = false;
            s_tx_busy = true;
            INEP(1)->DIEPTSIZ = (1u << 19);
            INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
        }
        return;
    }
    uint32_t space = (INEP(1)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) * 4u;
    uint32_t n = avail < 448u ? avail : 448u;
    if (n > space) n = space - (space % EP1_MPS);
    if (n == 0) return;
    uint8_t tmp[448];
    for (uint32_t k = 0; k < n; k++) tmp[k] = s_tx[(s_tt + k) % TXN];
    s_tt = (uint16_t)((s_tt + n) % TXN);
    uint32_t pk = (n + EP1_MPS - 1) / EP1_MPS;
    s_need_zlp = (n % EP1_MPS) == 0;
    s_tx_busy = true;
    INEP(1)->DIEPTSIZ = (pk << 19) | n;
    INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    fifo_write(1, tmp, n);
}

/* ------------------------------------------------------------ IRQ */
static void bus_reset(void)
{
    for (int i = 0; i < 4; i++) {
        OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
        INEP(i)->DIEPINT = 0xFF;
        OUTEP(i)->DOEPINT = 0xFF;
    }
    DEV->DAINTMSK = (1u << 0) | (1u << 16);
    DEV->DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
    DEV->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
    DEV->DCFG &= ~USB_OTG_DCFG_DAD;
    OTG->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10u << USB_OTG_GRSTCTL_TXFNUM_Pos);
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) { }
    ep0_out_arm();
    s_configured = false;
    s_tx_busy = false;
}

void OTG_FS_IRQHandler(void)
{
    uint32_t st = OTG->GINTSTS & OTG->GINTMSK;

    if (st & USB_OTG_GINTSTS_USBRST) {
        OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;
        bus_reset();
    }
    if (st & USB_OTG_GINTSTS_ENUMDNE) {
        OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        INEP(0)->DIEPCTL &= ~0x3u;           /* MPSIZ = 64 octets */
        DEV->DCTL |= USB_OTG_DCTL_CGINAK;
    }
    if (st & USB_OTG_GINTSTS_RXFLVL) {
        OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
        uint32_t s = OTG->GRXSTSP;
        uint32_t ep = s & USB_OTG_GRXSTSP_EPNUM;
        uint32_t n = (s & USB_OTG_GRXSTSP_BCNT) >> 4;
        uint32_t pkt = (s & USB_OTG_GRXSTSP_PKTSTS) >> 17;
        if (pkt == 6) {                      /* donnees SETUP */
            fifo_read(s_setup, 8);
        } else if (pkt == 2 && n) {          /* donnees OUT */
            if (ep == 1) {
                uint8_t tmp[64];
                fifo_read(tmp, n);
                for (uint32_t k = 0; k < n; k++) {
                    uint16_t nh = (uint16_t)((s_rh + 1) % RXN);
                    if (nh == s_rt) break;
                    s_rx[s_rh] = tmp[k];
                    s_rh = nh;
                }
            } else {
                fifo_read(s_ep0_buf, n > 64 ? 64 : n);
                if (s_ep0_out_req == 0x20 && n >= 7) {
                    memcpy(s_line_coding, s_ep0_buf, 7);
                    s_ep0_out_req = 0;
                    ep0_send(0, 0);          /* statut */
                }
            }
        }
        OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    }
    if (st & USB_OTG_GINTSTS_OEPINT) {
        uint32_t daint = DEV->DAINT;
        if (daint & (1u << 16)) {
            uint32_t f = OUTEP(0)->DOEPINT;
            OUTEP(0)->DOEPINT = f;
            if (f & USB_OTG_DOEPINT_STUP) handle_setup();
        }
        if (daint & (1u << 17)) {
            uint32_t f = OUTEP(1)->DOEPINT;
            OUTEP(1)->DOEPINT = f;
            if (f & USB_OTG_DOEPINT_XFRC) {
                OUTEP(1)->DOEPTSIZ = (1u << 19) | EP1_MPS;
                OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
            }
        }
    }
    if (st & USB_OTG_GINTSTS_IEPINT) {
        uint32_t daint = DEV->DAINT;
        for (int i = 0; i < 3; i++) {
            if (daint & (1u << i)) {
                uint32_t f = INEP(i)->DIEPINT;
                INEP(i)->DIEPINT = f;
                if (i == 1 && (f & USB_OTG_DIEPINT_XFRC)) {
                    s_tx_busy = false;
                    tx_kick();
                }
            }
        }
    }
    if (st & USB_OTG_GINTSTS_USBSUSP) {
        OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    }
}

/* ------------------------------------------------------------ API */
void usb_cdc_init(void)
{
    gpio_cfg(HW_PA11, GPIO_MODE_AF, false, 3, GPIO_PUPD_NONE, 10);
    gpio_cfg(HW_PA12, GPIO_MODE_AF, false, 3, GPIO_PUPD_NONE, 10);
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    __DSB();

    OTG->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
    while (!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL)) { }
    OTG->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST) { }
    for (volatile int i = 0; i < 2000; i++) { }

    OTG->GUSBCFG = (OTG->GUSBCFG & ~(USB_OTG_GUSBCFG_TRDT | USB_OTG_GUSBCFG_FHMOD)) |
                   USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | (6u << USB_OTG_GUSBCFG_TRDT_Pos);
    for (volatile int i = 0; i < 400000; i++) { }   /* ~25 ms : bascule en mode device */

    OTG->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
    PCGCCTL = 0;
    DEV->DCFG = (DEV->DCFG & ~USB_OTG_DCFG_DSPD) | USB_OTG_DCFG_DSPD;   /* full speed, PHY interne */

    OTG->GRXFSIZ = 128;
    OTG->DIEPTXF0_HNPTXFSIZ = (32u << 16) | 128u;
    OTG->DIEPTXF[0] = (128u << 16) | 160u;
    OTG->DIEPTXF[1] = (16u << 16) | 288u;
    OTG->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10u << USB_OTG_GRSTCTL_TXFNUM_Pos);
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) { }
    OTG->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) { }

    OTG->GINTSTS = 0xFFFFFFFFu;
    OTG->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_RXFLVLM |
                   USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_USBSUSPM;
    OTG->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
    NVIC_SetPriority(OTG_FS_IRQn, PRIO_USB);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    DEV->DCTL &= ~USB_OTG_DCTL_SDIS;       /* connexion */
}

bool usb_cdc_configured(void) { return s_configured; }

int usb_cdc_getc(void)
{
    if (s_rh == s_rt) return -1;
    uint8_t c = s_rx[s_rt];
    s_rt = (uint16_t)((s_rt + 1) % RXN);
    return c;
}

uint32_t usb_cdc_write(const uint8_t *d, uint32_t n)
{
    if (!s_configured) return 0;
    uint32_t k = 0;
    NVIC_DisableIRQ(OTG_FS_IRQn);
    while (k < n) {
        uint16_t nh = (uint16_t)((s_th + 1) % TXN);
        if (nh == s_tt) break;
        s_tx[s_th] = d[k++];
        s_th = nh;
    }
    tx_kick();
    NVIC_EnableIRQ(OTG_FS_IRQn);
    return k;
}

void usb_cdc_poll(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    tx_kick();
    NVIC_EnableIRQ(OTG_FS_IRQn);
}
