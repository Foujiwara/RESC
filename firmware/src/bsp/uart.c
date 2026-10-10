#include "uart.h"
#include "stm32.h"
#include "gpio.h"

#define RXN 1024u
#define TXN 2048u

struct uart_s {
    USART_TypeDef *r;
    uint32_t pclk;
    volatile uint16_t rh, rt, th, tt;
    uint8_t rx[RXN];
    uint8_t tx[TXN];
};

static uart_t s_u[2];
static int s_n;
static uart_t *s_by_inst[7];

static void irq(uart_t *u)
{
    if (!u) return;
    uint32_t sr = u->r->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t c = (uint8_t)u->r->DR;          /* lecture DR : efface aussi ORE/FE/NE */
        uint16_t n = (uint16_t)((u->rh + 1) % RXN);
        if (n != u->rt && (sr & USART_SR_RXNE)) { u->rx[u->rh] = c; u->rh = n; }
    }
    if ((sr & USART_SR_TXE) && (u->r->CR1 & USART_CR1_TXEIE)) {
        if (u->tt != u->th) {
            u->r->DR = u->tx[u->tt];
            u->tt = (uint16_t)((u->tt + 1) % TXN);
        } else {
            u->r->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

void USART1_IRQHandler(void) { irq(s_by_inst[1]); }
void USART2_IRQHandler(void) { irq(s_by_inst[2]); }
void USART3_IRQHandler(void) { irq(s_by_inst[3]); }
void UART4_IRQHandler(void)  { irq(s_by_inst[4]); }
void UART5_IRQHandler(void)  { irq(s_by_inst[5]); }
void USART6_IRQHandler(void) { irq(s_by_inst[6]); }

uart_t *uart_open(int inst, uint8_t tx, uint8_t rx, uint32_t baud)
{
    if (inst < 1 || inst > 6 || s_n >= 2) return 0;
    uart_t *u = &s_u[s_n++];
    IRQn_Type irqn;
    uint32_t af = 7;
    switch (inst) {
    case 1: RCC->APB2ENR |= RCC_APB2ENR_USART1EN; u->r = USART1; u->pclk = APB2_HZ; irqn = USART1_IRQn; break;
    case 2: RCC->APB1ENR |= RCC_APB1ENR_USART2EN; u->r = USART2; u->pclk = APB1_HZ; irqn = USART2_IRQn; break;
    case 3: RCC->APB1ENR |= RCC_APB1ENR_USART3EN; u->r = USART3; u->pclk = APB1_HZ; irqn = USART3_IRQn; break;
    case 4: RCC->APB1ENR |= RCC_APB1ENR_UART4EN;  u->r = UART4;  u->pclk = APB1_HZ; irqn = UART4_IRQn; af = 8; break;
    case 5: RCC->APB1ENR |= RCC_APB1ENR_UART5EN;  u->r = UART5;  u->pclk = APB1_HZ; irqn = UART5_IRQn; af = 8; break;
    default: RCC->APB2ENR |= RCC_APB2ENR_USART6EN; u->r = USART6; u->pclk = APB2_HZ; irqn = USART6_IRQn; af = 8; break;
    }
    __DSB();
    s_by_inst[inst] = u;
    gpio_cfg(tx, GPIO_MODE_AF, false, 2, GPIO_PUPD_NONE, af);
    gpio_cfg(rx, GPIO_MODE_AF, false, 2, GPIO_PUPD_UP, af);
    u->r->CR1 = 0;
    uart_set_baud(u, baud);
    u->r->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_SetPriority(irqn, PRIO_UART);
    NVIC_EnableIRQ(irqn);
    return u;
}

void uart_set_baud(uart_t *u, uint32_t baud)
{
    if (!u || baud == 0) return;
    u->r->BRR = (u->pclk + baud / 2) / baud;
}

int uart_getc(uart_t *u)
{
    if (!u || u->rh == u->rt) return -1;
    uint8_t c = u->rx[u->rt];
    u->rt = (uint16_t)((u->rt + 1) % RXN);
    return c;
}

uint32_t uart_tx_free(uart_t *u)
{
    if (!u) return 0;
    return (uint32_t)((u->tt + TXN - u->th - 1) % TXN);
}

uint32_t uart_write(uart_t *u, const uint8_t *d, uint32_t n)
{
    if (!u) return 0;
    uint32_t k = 0;
    while (k < n) {
        uint16_t nh = (uint16_t)((u->th + 1) % TXN);
        if (nh == u->tt) break;                 /* tampon plein : le reste est perdu */
        u->tx[u->th] = d[k++];
        u->th = nh;
    }
    u->r->CR1 |= USART_CR1_TXEIE;
    return k;
}
