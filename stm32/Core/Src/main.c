#include "main.h"
#include "tracker.h"

#include <string.h>

#define RX_CAPACITY 256U
#define RX_MASK (RX_CAPACITY - 1U)

typedef struct {
    UART_HandleTypeDef uart;
    TIM_HandleTypeDef *timer;
    Tracker tracker;
    uint8_t rx_byte, rx_data[RX_CAPACITY];
    uint32_t rx_time[RX_CAPACITY];
    volatile uint16_t rx_head, rx_tail;
    volatile uint8_t rx_fault;
    uint8_t tx_data[TRACKER_MAX_LEN + 4U];
    uint8_t pwm_started;
} Channel;

static TIM_HandleTypeDef timers[TRACKER_CHANNELS];
static Channel channels[TRACKER_CHANNELS];
static IWDG_HandleTypeDef watchdog;

/* Stop pulses, including when HAL handles are not initialized. Reset/fault
 * handling intentionally differs from ordinary link-loss angle holding. */
static void pulses_off(void)
{
    TIM2->CCER = 0;
    TIM3->CCER = 0;
    TIM4->CCER = 0;
}

void Error_Handler(void)
{
    __disable_irq();
    pulses_off();
    for (;;) {} /* If already started, the independent watchdog resets us. */
}

void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&channels[0].uart); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&channels[1].uart); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&channels[2].uart); }

/* ISR work is limited to a byte/timestamp queue. Parsing, motor operations
 * and ACK generation all run in the main loop. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    unsigned i;
    for (i = 0; i < TRACKER_CHANNELS; ++i) {
        Channel *c = &channels[i];
        if (uart == &c->uart) {
            uint16_t next = (uint16_t)((c->rx_head + 1U) & RX_MASK);
            if (next == c->rx_tail) c->rx_fault = 1;
            else {
                c->rx_data[c->rx_head] = c->rx_byte;
                c->rx_time[c->rx_head] = HAL_GetTick();
                __DMB();
                c->rx_head = next;
            }
            if (HAL_UART_Receive_IT(uart, &c->rx_byte, 1) != HAL_OK) c->rx_fault = 1;
            return;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    unsigned i;
    for (i = 0; i < TRACKER_CHANNELS; ++i)
        if (uart == &channels[i].uart) channels[i].rx_fault = 1;
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    /* Matches the reference F103C8 board: external 8 MHz crystal, 72 MHz CPU. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void gpio_config(GPIO_TypeDef *port, uint32_t pins, uint32_t mode)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pins;
    gpio.Mode = mode;
    gpio.Pull = mode == GPIO_MODE_INPUT ? GPIO_PULLUP : GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

static void peripherals_init(void)
{
    static TIM_TypeDef *const instances[] = {TIM2, TIM3, TIM4};
    static USART_TypeDef *const serials[] = {USART1, USART2, USART3};
    static const IRQn_Type irqs[] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
    TIM_OC_InitTypeDef oc = {0};
    unsigned i;
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    pulses_off();
    gpio_config(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9, GPIO_MODE_AF_PP);
    gpio_config(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10, GPIO_MODE_AF_PP);
    gpio_config(GPIOA, GPIO_PIN_2, GPIO_MODE_AF_PP);
    gpio_config(GPIOA, GPIO_PIN_3 | GPIO_PIN_10, GPIO_MODE_INPUT);
    gpio_config(GPIOB, GPIO_PIN_11, GPIO_MODE_INPUT);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0; /* No pulse on ordinary boot. */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (i = 0; i < TRACKER_CHANNELS; ++i) {
        TIM_HandleTypeDef *timer = &timers[i];
        UART_HandleTypeDef *uart = &channels[i].uart;
        timer->Instance = instances[i];
        timer->Init.Prescaler = 71; /* APB1 timer clock 72 MHz -> 1 us/count. */
        timer->Init.CounterMode = TIM_COUNTERMODE_UP;
        timer->Init.Period = 19999; /* 50 Hz servo pulses. */
        timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_PWM_Init(timer) != HAL_OK ||
            HAL_TIM_PWM_ConfigChannel(timer, &oc, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIM_PWM_ConfigChannel(timer, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
        channels[i].timer = timer;
        uart->Instance = serials[i];
        uart->Init.BaudRate = 115200;
        uart->Init.WordLength = UART_WORDLENGTH_8B;
        uart->Init.StopBits = UART_STOPBITS_1;
        uart->Init.Parity = UART_PARITY_NONE;
        uart->Init.Mode = UART_MODE_TX_RX;
        uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        uart->Init.OverSampling = UART_OVERSAMPLING_16;
        if (HAL_UART_Init(uart) != HAL_OK) Error_Handler();
        HAL_NVIC_SetPriority(irqs[i], 2, 0);
        HAL_NVIC_EnableIRQ(irqs[i]);
    }
}

static void receive_pending(Channel *c)
{
    unsigned budget = RX_CAPACITY;
    while (budget--) {
        uint8_t byte;
        uint32_t received;
        __disable_irq();
        if (c->rx_fault) {
            c->rx_tail = c->rx_head;
            c->rx_fault = 0;
            __enable_irq();
            Tracker_ResetRx(&c->tracker);
            (void)HAL_UART_AbortReceive(&c->uart);
            __HAL_UART_CLEAR_OREFLAG(&c->uart);
            if (HAL_UART_Receive_IT(&c->uart, &c->rx_byte, 1) != HAL_OK) c->rx_fault = 1;
            return;
        }
        if (c->rx_tail == c->rx_head) { __enable_irq(); return; }
        byte = c->rx_data[c->rx_tail];
        received = c->rx_time[c->rx_tail];
        c->rx_tail = (uint16_t)((c->rx_tail + 1U) & RX_MASK);
        __enable_irq();
        Tracker_Receive(&c->tracker, byte, received);
    }
}

static int send_frame(void *user, const uint8_t *frame, uint8_t size)
{
    Channel *c = (Channel *)user;
    if (c->uart.gState != HAL_UART_STATE_READY) return 0;
    memcpy(c->tx_data, frame, size); /* Persistent storage until TX IRQ completes. */
    return HAL_UART_Transmit_IT(&c->uart, c->tx_data, size) == HAL_OK;
}

static void apply_pwm(Channel *c)
{
    if (!c->tracker.output_enabled) return;
    __HAL_TIM_SET_COMPARE(c->timer, TIM_CHANNEL_1, Tracker_Pulse(&c->tracker, 0));
    __HAL_TIM_SET_COMPARE(c->timer, TIM_CHANNEL_2, Tracker_Pulse(&c->tracker, 1));
    if (!c->pwm_started) {
        if (HAL_TIM_PWM_Start(c->timer, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIM_PWM_Start(c->timer, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
        c->pwm_started = 1;
    }
}

int main(void)
{
    unsigned i;
    TrackerConfig config;
    HAL_Init();
    clock_init();
    peripherals_init();
    for (i = 0; i < TRACKER_CHANNELS; ++i) {
        Tracker_DefaultConfig(&config);
        /* Customize each channel/axis here after confirming servo travel,
         * pulse widths and physical manual directions. No joystick ADC input. */
        if (!Tracker_Init(&channels[i].tracker, &config, HAL_GetTick())) Error_Handler();
        if (HAL_UART_Receive_IT(&channels[i].uart, &channels[i].rx_byte, 1) != HAL_OK) Error_Handler();
    }
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_32;
    watchdog.Init.Reload = 1249; /* About one second; LSI tolerance applies. */
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) Error_Handler();
    for (;;) {
        for (i = 0; i < TRACKER_CHANNELS; ++i) {
            Channel *c = &channels[i];
            uint32_t now;
            receive_pending(c);
            now = HAL_GetTick();
            Tracker_Tick(&c->tracker, now);
            apply_pwm(c);
            Tracker_ServiceTx(&c->tracker, now, send_frame, c);
        }
        if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) Error_Handler();
    }
}
