#include <stdio.h>

#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"

#include "drv8833.h"
#include "encoder.h"

#include "uart.h"

/* LED da Black Pill: PC13 (ativo em nivel baixo) */
#define LED_PORT        GPIOC
#define LED_PIN         13U
#define LED_CLK_EN      RCC_AHB1ENR_GPIOCEN

#define PWM_FREQ_HZ         10000UL
#define PWM_DUTY            1.f       /* 60% */

#define ENC_SAMPLE_MS       100U
#define ENC_PPR             11U         /* ciclos por canal, no eixo do motor */
#define ENC_GEAR_RATIO      33.8f

#define LOG_UART        USART1

static drv8833_t motor;
static encoder_t enc;

/* Ultima velocidade medida (leia pelo debugger ou por outra task) */
static volatile float g_output_rpm;
static volatile float g_output_rad_s;

static void led_init(void)
{
    RCC->AHB1ENR |= LED_CLK_EN;
    (void)RCC->AHB1ENR;

    LED_PORT->MODER   &= ~(3U << (LED_PIN * 2U));
    LED_PORT->MODER   |=  (1U << (LED_PIN * 2U));
    LED_PORT->OTYPER  &= ~(1U << LED_PIN);
    LED_PORT->OSPEEDR &= ~(3U << (LED_PIN * 2U));
    LED_PORT->PUPDR   &= ~(3U << (LED_PIN * 2U));
}

static void log_init(void)
{
    const uart_config_t cfg = {
        .usart              = LOG_UART,
        .clock_hz           = SystemCoreClock,  /* HSI 16 MHz, APB2 sem divisao */
        .baudrate           = 115200U,
        .gpio               = GPIOA,
        .tx_pin             = 9,
        .rx_pin             = 10,
        .alternate_function = 7,
    };

    uart_init(&cfg);
}

/* Executa na ISR do EXTI: so solta o motor */
static void on_fault(void *ctx)
{
    drv8833_coast((drv8833_t *)ctx, DRV8833_BRIDGE_A);
}

/* Executa na task do encoder, a cada 100 ms */
static void on_sample(const encoder_sample_t *s, void *ctx)
{
    (void)ctx;
    g_output_rpm   = s->output_rpm;
    g_output_rad_s = s->output_rad_s;
}

static void motor_init(void)
{
    drv8833_config_t cfg = {
        .timer       = TIM3,
        .prescale    = 0,
        .autorreload = (SystemCoreClock / PWM_FREQ_HZ) - 1U,   /* 1599 */
        .mode        = DRV8833_PARALLEL_MODE,
        .ain = {
            { GPIOA, 6, 2, 1 },     /* AIN1: PA6 = TIM3_CH1 (AF2) */
            { GPIOA, 7, 2, 2 },     /* AIN2: PA7 = TIM3_CH2 (AF2) */
        },
        .sleep       = { GPIOB, 0 },
        .fault       = { GPIOB, 1 },
        .fault_cb    = on_fault,
        .fault_ctx   = &motor,
    };

    drv8833_init(&motor, &cfg);
    drv8833_drive(&motor, DRV8833_BRIDGE_A,
                  DRV8833_DIR_FORWARD, DRV8833_DECAY_FAST, PWM_DUTY);
    drv8833_enable(&motor, DRV8833_STATE_ON);
}

/* Le o CCR do canal (1..4) e converte para 0..1000 (0,1%) */
static uint32_t duty_permille(const drv883_gpio_config_t *g)
{
    TIM_TypeDef *t = motor.cfg.timer;
    uint32_t ccr = (&t->CCR1)[g->channel - 1U];

    return (ccr * 1000UL) / (t->ARR + 1UL);
}

static void encoder_setup(void)
{
    encoder_config_t cfg = {
        .timer            = TIM4,
        .ch1              = { GPIOB, 6, 2 },    /* PB6 = TIM4_CH1 (AF2) */
        .ch2              = { GPIOB, 7, 2 },    /* PB7 = TIM4_CH2 (AF2) */
        .input_filter     = 4,
        .mode             = ENCODER_X1,
        .pulses_per_rev   = ENC_PPR,
        .gear_ratio       = ENC_GEAR_RATIO,
        .sample_period_ms = ENC_SAMPLE_MS,
        .sample_cb        = on_sample,
        .sample_ctx       = NULL,
    };

    encoder_init(&enc, &cfg);
}

void EXTI1_IRQHandler(void)
{
    drv8833_fault_irq_handler(1);
}

static void vEncoderTask(void *pvParameters)
{
    (void)pvParameters;

    char buf[96];
    encoder_sample_t s;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ENC_SAMPLE_MS));
        encoder_update(&enc);

        if (encoder_get_sample(&enc, &s) != ENCODER_OK)
            continue;

        /* RPM do motor com 1 casa decimal, tratando o sinal */
        int32_t rpm10 = (int32_t)(s.output_rpm * 10.0f);
        const char *sign = (rpm10 < 0) ? "-" : "";
        if (rpm10 < 0) rpm10 = -rpm10;

        uint32_t d1 = duty_permille(&motor.cfg.ain[0]);
        uint32_t d2 = duty_permille(&motor.cfg.ain[1]);

        snprintf(buf, sizeof(buf),
                 "pulsos:%ld pos:%ld rpm:%s%ld.%ld duty1:%lu.%lu%% duty2:%lu.%lu%%\r\n",
                 (long)s.delta_counts, (long)s.position_counts,
                 sign, (long)(rpm10 / 10), (long)(rpm10 % 10),
                 (unsigned long)(d1 / 10), (unsigned long)(d1 % 10),
                 (unsigned long)(d2 / 10), (unsigned long)(d2 % 10));

        uart_send_string(LOG_UART, buf);
    }
}

static void vBlinkTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        LED_PORT->ODR ^= (1U << LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

int main(void)
{
    SystemCoreClockUpdate();
    led_init();
    log_init();
    motor_init();
    encoder_setup();

    xTaskCreate(vBlinkTask,   "blink", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vEncoderTask, "enc",   384, NULL, 2, NULL);   /* stack maior por causa do snprintf */

    vTaskStartScheduler();

    for (;;) { }
}