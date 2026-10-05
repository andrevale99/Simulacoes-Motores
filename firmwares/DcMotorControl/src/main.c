#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"

#include "drv8833.h"

/* LED da Black Pill: PC13 (ativo em nivel baixo) */
#define LED_PORT        GPIOC
#define LED_PIN         13U
#define LED_CLK_EN      RCC_AHB1ENR_GPIOCEN

#define PWM_FREQ_HZ     10000UL
#define PWM_DUTY        0.60f           /* 60% */

static drv8833_t motor;

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

/* Executa na ISR do EXTI: so solta o motor */
static void on_fault(void *ctx)
{
    drv8833_coast((drv8833_t *)ctx, DRV8833_BRIDGE_A);
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
                  DRV8833_DIR_FORWARD, DRV8833_DECAY_SLOW, PWM_DUTY);
    drv8833_enable(&motor, DRV8833_STATE_ON);
}

void EXTI1_IRQHandler(void)
{
    drv8833_fault_irq_handler(1);
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
    SystemCoreClockUpdate();    /* HSI 16 MHz */
    led_init();
    motor_init();

    xTaskCreate(vBlinkTask, "blink", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    for (;;) { }
}