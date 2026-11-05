/*
 * STM32G474 port of the controller core: clocks, GPIO, FDCAN, half-duplex USART,
 * USB CDC through TinyUSB, the safety chain and the watchdog.
 *
 * Structure: interrupts only move bytes and frames into rings, and everything else
 * runs in the main loop, so the core needs no locks (see rac_core.h).
 *
 * Status: compiles and links for the real target, but has not run on silicon. The
 * board does not exist yet. docs/bringup.md lists what to verify, in order.
 */
#include <string.h>

#include "board.h"
#include "rac_core.h"
#include "rac_port.h"
#include "tusb.h"

/* ------------------------------------------------------------- GPIO helpers */

enum { GPIO_INPUT = 0, GPIO_OUTPUT = 1, GPIO_AF = 2, GPIO_ANALOG = 3 };

static void gpio_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}

static void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    volatile uint32_t *reg = &port->AFR[pin >> 3];
    uint32_t shift = (pin & 7u) * 4u;
    *reg = (*reg & ~(0xFu << shift)) | (af << shift);
    gpio_mode(port, pin, GPIO_AF);
    port->OSPEEDR |= 2u << (pin * 2u);
}

static inline void gpio_write(GPIO_TypeDef *port, uint32_t pin, bool level)
{
    port->BSRR = level ? (1u << pin) : (1u << (pin + 16u));
}

static inline bool gpio_read(const GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1u;
}

/* ------------------------------------------------------------------- clocks */

static void fatal_blink(void)
{
    /* Nothing can run safely without a clock: the relay stays off and FAULT blinks. */
    for (;;) {
        gpio_write(LED_FAULT_GPIO, LED_FAULT_PIN, true);
        for (volatile uint32_t i = 0; i < 400000u; i++) {
        }
        gpio_write(LED_FAULT_GPIO, LED_FAULT_PIN, false);
        for (volatile uint32_t i = 0; i < 400000u; i++) {
        }
    }
}

static void clocks_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;
    gpio_mode(LED_FAULT_GPIO, LED_FAULT_PIN, GPIO_OUTPUT);

    /* HSE, because a CAN bit timing needs crystal accuracy. */
    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = 2000000u;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        if (--timeout == 0u) fatal_blink();
    }

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR5 &= ~PWR_CR5_R1MODE; /* range 1 boost, required above 150 MHz */

    FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS) {
    }

    /* VCO = 8 / 2 * 85 = 340 MHz. SYSCLK = /2 = 170 MHz. FDCAN kernel = /4 = 85 MHz. */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (1u << RCC_PLLCFGR_PLLM_Pos) | (85u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN | (1u << RCC_PLLCFGR_PLLQ_Pos) |
                   RCC_PLLCFGR_PLLQEN;
    RCC->CR |= RCC_CR_PLLON;
    timeout = 2000000u;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        if (--timeout == 0u) fatal_blink();
    }

    /* Boost mode needs one microsecond at half speed on the way up (RM0440). */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV2;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }
    for (volatile uint32_t i = 0; i < 200u; i++) {
    }
    RCC->CFGR &= ~RCC_CFGR_HPRE;

    /* USB: HSI48 disciplined by start-of-frame packets through the CRS. */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {
    }
    RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN | RCC_APB1ENR1_USBEN;
    (void)RCC->APB1ENR1;
    CRS->CFGR = (CRS->CFGR & ~CRS_CFGR_SYNCSRC) | (2u << CRS_CFGR_SYNCSRC_Pos);
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL; /* 00: HSI48 */

    SystemCoreClock = SYSCLK_HZ;
}

/* ---------------------------------------------------------------- time base */

static uint64_t time_high;
static uint32_t time_last;

static void timebase_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;
    TIM2->PSC = (PCLK1_HZ / 1000000u) - 1u;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
}

uint64_t rac_port_now_us(void)
{
    /* Called from interrupts as well, to stamp received frames. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = TIM2->CNT;
    if (now < time_last) {
        time_high += 1ull << 32;
    }
    time_last = now;
    uint64_t result = time_high | now;
    __set_PRIMASK(primask);
    return result;
}

/* --------------------------------------------------------------------- FDCAN */

/* Message RAM layout is fixed in hardware on the G4: 212 words per instance. */
#define RAM_RXF0_OFFSET 0x0B0u
#define RAM_TX_OFFSET 0x278u
#define RAM_ELEMENT_BYTES 72u

#define CAN_RX_RING 64u
#define CAN_TX_RING 64u

typedef struct {
    rac_can_frame_t frame;
    uint64_t ts;
} can_rx_item_t;

typedef struct {
    FDCAN_GlobalTypeDef *regs;
    volatile uint32_t *ram_rx;
    volatile uint32_t *ram_tx;
    volatile uint16_t rx_head, rx_tail;
    uint16_t tx_head, tx_tail;
    can_rx_item_t rx[CAN_RX_RING];
    rac_can_frame_t tx[CAN_TX_RING];
    uint32_t rx_dropped;
} can_bus_t;

static can_bus_t can_buses[2];

static void can_init_one(can_bus_t *bus, FDCAN_GlobalTypeDef *regs, uint32_t ram_offset)
{
    bus->regs = regs;
    bus->ram_rx = (volatile uint32_t *)(SRAMCAN_BASE + ram_offset + RAM_RXF0_OFFSET);
    bus->ram_tx = (volatile uint32_t *)(SRAMCAN_BASE + ram_offset + RAM_TX_OFFSET);

    regs->CCCR |= FDCAN_CCCR_INIT;
    while (!(regs->CCCR & FDCAN_CCCR_INIT)) {
    }
    regs->CCCR |= FDCAN_CCCR_CCE;

    /* Classic CAN, automatic retransmission on. */
    regs->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR | FDCAN_CCCR_TEST | FDCAN_CCCR_MON);

    /* 85 MHz / 5 = 17 MHz, so 17 time quanta per bit is exactly 1 Mbit/s.
     * Sample point (1 + 13) / 17 = 82%. */
    regs->NBTP = (1u << FDCAN_NBTP_NSJW_Pos) | (4u << FDCAN_NBTP_NBRP_Pos) | (12u << FDCAN_NBTP_NTSEG1_Pos) |
                 (2u << FDCAN_NBTP_NTSEG2_Pos);

    /* Everything goes to FIFO 0: the motors' identifiers are decoded in software. */
    regs->RXGFC = 0u;

    /* Clear the instance's whole RAM so no stale element is ever read. */
    volatile uint32_t *ram = (volatile uint32_t *)(SRAMCAN_BASE + ram_offset);
    for (uint32_t i = 0; i < 212u; i++) {
        ram[i] = 0u;
    }

    regs->IE = FDCAN_IE_RF0NE;
    regs->ILS = 0u;
    regs->ILE = FDCAN_ILE_EINT0;

    regs->CCCR &= ~FDCAN_CCCR_INIT;
}

static void can_init(void)
{
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL) | (1u << RCC_CCIPR_FDCANSEL_Pos); /* PLLQ */
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;

    gpio_af(GPIOB, 12u, 9u);
    gpio_af(GPIOB, 13u, 9u);
    gpio_af(GPIOA, 8u, 11u);
    gpio_af(GPIOA, 15u, 11u);

    /* The kernel clock divider sits in the first instance, shared by all three. */
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {
    }
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    FDCAN_CONFIG->CKDIV = 0u;

    can_init_one(&can_buses[0], CAN1_PERIPH, CAN1_RAM_OFFSET);
    can_init_one(&can_buses[1], CAN2_PERIPH, CAN2_RAM_OFFSET);

    NVIC_SetPriority(CAN1_IRQ, 1u);
    NVIC_SetPriority(CAN2_IRQ, 1u);
    NVIC_EnableIRQ(CAN1_IRQ);
    NVIC_EnableIRQ(CAN2_IRQ);
}

static void can_rx_isr(can_bus_t *bus)
{
    FDCAN_GlobalTypeDef *regs = bus->regs;
    regs->IR = FDCAN_IR_RF0N;

    while ((regs->RXF0S & FDCAN_RXF0S_F0FL) != 0u) {
        uint32_t index = (regs->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
        const volatile uint32_t *element = bus->ram_rx + index * (RAM_ELEMENT_BYTES / 4u);
        uint32_t r0 = element[0];
        uint32_t r1 = element[1];
        uint16_t head = bus->rx_head;
        uint16_t next = (uint16_t)((head + 1u) % CAN_RX_RING);

        if (next != bus->rx_tail && !(r0 & (1u << 29))) {
            can_rx_item_t *item = &bus->rx[head];
            uint32_t a = element[2];
            uint32_t b = element[3];
            item->frame.extended = (r0 >> 30) & 1u;
            item->frame.id = item->frame.extended ? (r0 & 0x1FFFFFFFu) : ((r0 >> 18) & 0x7FFu);
            item->frame.dlc = (uint8_t)((r1 >> 16) & 0xFu);
            memcpy(item->frame.data, &a, 4);
            memcpy(item->frame.data + 4, &b, 4);
            item->ts = rac_port_now_us();
            bus->rx_head = next;
        } else {
            bus->rx_dropped++;
        }
        regs->RXF0A = index;
    }
}

void FDCAN2_IT0_IRQHandler(void)
{
    can_rx_isr(&can_buses[0]);
}

void FDCAN3_IT0_IRQHandler(void)
{
    can_rx_isr(&can_buses[1]);
}

bool rac_port_can_send(uint8_t can_index, const rac_can_frame_t *frame)
{
    can_bus_t *bus = &can_buses[can_index];
    uint16_t next = (uint16_t)((bus->tx_head + 1u) % CAN_TX_RING);
    if (next == bus->tx_tail) {
        return false;
    }
    bus->tx[bus->tx_head] = *frame;
    bus->tx_head = next;
    return true;
}

static void can_pump(uint8_t index)
{
    can_bus_t *bus = &can_buses[index];
    FDCAN_GlobalTypeDef *regs = bus->regs;

    /* Bus-off puts the controller back in init. Leaving init starts its own recovery. */
    if ((regs->PSR & FDCAN_PSR_BO) && (regs->CCCR & FDCAN_CCCR_INIT)) {
        regs->CCCR &= ~FDCAN_CCCR_INIT;
    }

    while (bus->tx_tail != bus->tx_head && !(regs->TXFQS & FDCAN_TXFQS_TFQF)) {
        const rac_can_frame_t *frame = &bus->tx[bus->tx_tail];
        uint32_t put = (regs->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
        volatile uint32_t *element = bus->ram_tx + put * (RAM_ELEMENT_BYTES / 4u);
        uint32_t a, b;
        memcpy(&a, frame->data, 4);
        memcpy(&b, frame->data + 4, 4);
        element[0] = frame->extended ? ((1u << 30) | (frame->id & 0x1FFFFFFFu)) : ((frame->id & 0x7FFu) << 18);
        element[1] = (uint32_t)(frame->dlc & 0xFu) << 16;
        element[2] = a;
        element[3] = b;
        regs->TXBAR = 1u << put;
        bus->tx_tail = (uint16_t)((bus->tx_tail + 1u) % CAN_TX_RING);
    }

    while (bus->rx_tail != bus->rx_head) {
        const can_rx_item_t *item = &bus->rx[bus->rx_tail];
        rac_core_on_can_frame(index, &item->frame, item->ts);
        bus->rx_tail = (uint16_t)((bus->rx_tail + 1u) % CAN_RX_RING);
    }
}

/* --------------------------------------------------------------------- USART */

#define UART_RX_RING 1024u
#define UART_TX_RING 512u

typedef struct {
    USART_TypeDef *regs;
    volatile uint16_t rx_head, rx_tail;
    volatile uint16_t tx_head, tx_tail;
    volatile uint64_t rx_ts;
    uint8_t rx[UART_RX_RING];
    uint8_t tx[UART_TX_RING];
} uart_bus_t;

static uart_bus_t uart_buses[2];

static void uart_set_baud(uart_bus_t *bus, uint32_t baud)
{
    uint32_t cr1 = bus->regs->CR1;
    bus->regs->CR1 = cr1 & ~USART_CR1_UE;
    bus->regs->BRR = (PCLK1_HZ + baud / 2u) / baud;
    bus->regs->CR1 = cr1;
}

static void uart_init_one(uart_bus_t *bus, USART_TypeDef *regs)
{
    bus->regs = regs;
    regs->CR1 = 0u;
    regs->CR2 = 0u;
    regs->CR3 = USART_CR3_OVRDIS; /* a late byte is lost, and never wedges the receiver */
    regs->BRR = PCLK1_HZ / 1000000u;
    regs->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
}

static void uart_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_USART3EN;
    (void)RCC->APB1ENR1;

    gpio_af(GPIOA, 2u, 7u);
    gpio_af(GPIOA, 3u, 7u);
    gpio_af(GPIOB, 10u, 7u);
    gpio_af(GPIOB, 11u, 7u);
    /* The isolator drives RX; keep it defined while its far side is unpowered. */
    GPIOA->PUPDR |= 1u << (3u * 2u);
    GPIOB->PUPDR |= 1u << (11u * 2u);

    uart_init_one(&uart_buses[0], USART2);
    uart_init_one(&uart_buses[1], USART3);

    NVIC_SetPriority(USART2_IRQn, 2u);
    NVIC_SetPriority(USART3_IRQn, 2u);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
}

static void uart_isr(uart_bus_t *bus)
{
    USART_TypeDef *regs = bus->regs;
    uint32_t status = regs->ISR;

    if (status & (USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        regs->ICR = USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    if (status & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)regs->RDR;
        uint16_t head = bus->rx_head;
        uint16_t next = (uint16_t)((head + 1u) % UART_RX_RING);
        if (next != bus->rx_tail) {
            bus->rx[head] = byte;
            bus->rx_head = next;
            bus->rx_ts = TIM2->CNT; /* low word only: extended in the main loop */
        }
    }
    if ((regs->CR1 & USART_CR1_TXEIE_TXFNFIE) && (status & USART_ISR_TXE_TXFNF)) {
        if (bus->tx_tail != bus->tx_head) {
            regs->TDR = bus->tx[bus->tx_tail];
            bus->tx_tail = (uint16_t)((bus->tx_tail + 1u) % UART_TX_RING);
        } else {
            regs->CR1 = (regs->CR1 & ~USART_CR1_TXEIE_TXFNFIE) | USART_CR1_TCIE;
        }
    }
    if ((regs->CR1 & USART_CR1_TCIE) && (status & USART_ISR_TC)) {
        /* The bus wiring echoes everything sent. With the receiver off during
         * transmission the echo never reaches the parser. */
        regs->CR1 = (regs->CR1 & ~USART_CR1_TCIE) | USART_CR1_RE;
    }
}

void USART2_IRQHandler(void)
{
    uart_isr(&uart_buses[0]);
}

void USART3_IRQHandler(void)
{
    uart_isr(&uart_buses[1]);
}

bool rac_port_uart_send(uint8_t uart_index, const uint8_t *data, size_t len)
{
    uart_bus_t *bus = &uart_buses[uart_index];
    uint16_t head = bus->tx_head;
    uint16_t tail = bus->tx_tail;
    size_t used = (size_t)((head + UART_TX_RING - tail) % UART_TX_RING);
    if (used + len >= UART_TX_RING) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        bus->tx[head] = data[i];
        head = (uint16_t)((head + 1u) % UART_TX_RING);
    }
    __disable_irq();
    bus->tx_head = head;
    bus->regs->CR1 = (bus->regs->CR1 & ~(USART_CR1_RE | USART_CR1_TCIE)) | USART_CR1_TXEIE_TXFNFIE;
    __enable_irq();
    return true;
}

void rac_port_uart_set_baud(uint8_t uart_index, uint32_t baud)
{
    if (baud) uart_set_baud(&uart_buses[uart_index], baud);
}

static void uart_pump(uint8_t index)
{
    uart_bus_t *bus = &uart_buses[index];
    uint8_t chunk[128];
    size_t count = 0;
    while (bus->rx_tail != bus->rx_head && count < sizeof(chunk)) {
        chunk[count++] = bus->rx[bus->rx_tail];
        bus->rx_tail = (uint16_t)((bus->rx_tail + 1u) % UART_RX_RING);
    }
    if (count) {
        /* Bytes of one reply arrive within a millisecond of each other, so the time
         * of draining is close enough for the velocity estimate it feeds. */
        rac_core_on_uart_bytes(index, chunk, count, rac_port_now_us());
    }
}

/* ----------------------------------------------------------------------- USB */

void USB_HP_IRQHandler(void)
{
    tud_int_handler(0);
}

void USB_LP_IRQHandler(void)
{
    tud_int_handler(0);
}

void USBWakeUp_IRQHandler(void)
{
    tud_int_handler(0);
}

bool rac_port_host_send(const uint8_t *data, size_t len)
{
    if (!tud_cdc_connected()) {
        return true; /* nobody is listening, and that is not an error here */
    }
    if (tud_cdc_write_available() < len) {
        return false;
    }
    tud_cdc_write(data, (uint32_t)len);
    return true;
}

static void usb_pump(void)
{
    tud_task();
    while (tud_cdc_available()) {
        uint8_t buf[256];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));
        if (!count) break;
        rac_core_on_host_bytes(buf, count, rac_port_now_us());
    }
    tud_cdc_write_flush();
}

/* ------------------------------------------------------------------ safety */

static bool safety_enabled;
static uint8_t estop_filter;
static bool estop_state = true; /* stopped until proven otherwise */
#define ESTOP_FILTER_TICKS 6u

static void safety_init(void)
{
    gpio_write(SAFETY_EN_PORT, SAFETY_EN_PIN, false);
    gpio_mode(SAFETY_EN_PORT, SAFETY_EN_PIN, GPIO_OUTPUT);
    gpio_mode(ESTOP_CH1_PORT, ESTOP_CH1_PIN, GPIO_INPUT);
    gpio_mode(ESTOP_CH2_PORT, ESTOP_CH2_PIN, GPIO_INPUT);
    gpio_mode(STOP_SENSE_PORT, STOP_SENSE_PIN, GPIO_INPUT);
}

bool rac_port_estop_active(void)
{
    bool closed1 = gpio_read(ESTOP_CH1_PORT, ESTOP_CH1_PIN);
    bool closed2 = !gpio_read(ESTOP_CH2_PORT, ESTOP_CH2_PIN);
    /* While the MCU drives the gate high, a low gate means the STOP key is held. */
    bool stop_key = safety_enabled && !gpio_read(STOP_SENSE_PORT, STOP_SENSE_PIN);
    bool raw = !closed1 || !closed2 || stop_key;

    /* Pressing takes effect at once; only leaving the stopped state is filtered,
     * so contact bounce cannot let the system believe the button was released. */
    if (raw) {
        estop_filter = 0;
        estop_state = true;
    } else if (estop_state && ++estop_filter >= ESTOP_FILTER_TICKS) {
        estop_state = false;
    }
    return estop_state;
}

void rac_port_safety_enable(bool enable)
{
    safety_enabled = enable;
    gpio_write(SAFETY_EN_PORT, SAFETY_EN_PIN, enable);
}

void rac_port_sync_pulse(bool level)
{
    gpio_write(SYNC_OUT_PORT, SYNC_OUT_PIN, level);
}

static void watchdog_init(void)
{
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 2u;     /* 32 kHz / 16 = 2 kHz */
    IWDG->RLR = 400u;  /* 200 ms */
    while (IWDG->SR != 0u) {
    }
    IWDG->KR = 0xAAAAu;
}

void rac_port_watchdog_kick(void)
{
    IWDG->KR = 0xAAAAu;
}

/* ------------------------------------------------------------------- misc */

void rac_port_led_status(uint8_t port_index, bool online, bool active)
{
    static GPIO_TypeDef *const gpios[] = {LED_PORT0_GPIO, LED_PORT1_GPIO, LED_PORT2_GPIO, LED_PORT3_GPIO};
    static const uint8_t pins[] = {LED_PORT0_PIN, LED_PORT1_PIN, LED_PORT2_PIN, LED_PORT3_PIN};
    (void)active;
    if (port_index < 4u) gpio_write(gpios[port_index], pins[port_index], online);
}

void rac_port_led_run(bool on)
{
    gpio_write(LED_RUN_GPIO, LED_RUN_PIN, on);
}

void rac_port_led_fault(bool on)
{
    gpio_write(LED_FAULT_GPIO, LED_FAULT_PIN, on);
}

void rac_port_unique_id(uint8_t out[12])
{
    memcpy(out, (const void *)UID_BASE, 12);
}

uint32_t rac_port_capabilities(void)
{
    return RAC_CAP_HW_ESTOP;
}

bool rac_port_handle_extension(uint8_t msg_type, uint8_t arm_id, const uint8_t *body, size_t len)
{
    (void)msg_type;
    (void)arm_id;
    (void)body;
    (void)len;
    return false;
}

static void gpio_init(void)
{
    static GPIO_TypeDef *const led_gpio[] = {LED_PORT0_GPIO, LED_PORT1_GPIO, LED_PORT2_GPIO, LED_PORT3_GPIO, LED_RUN_GPIO, LED_FAULT_GPIO};
    static const uint8_t led_pin[] = {LED_PORT0_PIN, LED_PORT1_PIN, LED_PORT2_PIN, LED_PORT3_PIN, LED_RUN_PIN, LED_FAULT_PIN};
    for (uint32_t i = 0; i < 6u; i++) {
        gpio_write(led_gpio[i], led_pin[i], false);
        gpio_mode(led_gpio[i], led_pin[i], GPIO_OUTPUT);
    }
    gpio_write(SYNC_OUT_PORT, SYNC_OUT_PIN, false);
    gpio_mode(SYNC_OUT_PORT, SYNC_OUT_PIN, GPIO_OUTPUT);
    gpio_mode(SYNC_IN_PORT, SYNC_IN_PIN, GPIO_INPUT);
}

int main(void)
{
    clocks_init();
    safety_init(); /* first, so the relay stays off while everything else comes up */
    gpio_init();
    timebase_init();
    can_init();
    uart_init();
    tusb_init();
    rac_core_init();
    watchdog_init();

    uint64_t next_tick = rac_port_now_us() + RAC_TICK_US;
    for (;;) {
        usb_pump();
        for (uint8_t i = 0; i < 2u; i++) {
            can_pump(i);
            uart_pump(i);
        }
        uint64_t now = rac_port_now_us();
        if (now >= next_tick) {
            rac_core_tick(now);
            next_tick += RAC_TICK_US;
            /* After a stall, resume on the grid rather than running to catch up. */
            if (next_tick <= now) {
                next_tick = now + RAC_TICK_US;
            }
        }
    }
}
