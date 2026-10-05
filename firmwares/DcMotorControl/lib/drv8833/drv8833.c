#include "drv8833.h"
#include "drivers/gpio/driver_gpio.h"

/** @brief Tabela de handles por linha EXTI, usada para achar o dono do fault na ISR. */
static drv8833_t *s_fault_owner[16];

/**
 * @brief Habilita o clock do timer no barramento APB correspondente.
 *
 * @param t Timer (TIM1 a TIM5). Timers nao suportados sao ignorados.
 */
static void timer_clock_enable(TIM_TypeDef *t)
{
    if (t == TIM1)
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (t == TIM2)
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (t == TIM3)
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (t == TIM4)
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (t == TIM5)
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
}

/**
 * @brief Verifica se o timer e suportado pela biblioteca.
 *
 * @param t Timer a verificar.
 *
 * @retval true  TIM1 a TIM5.
 * @retval false Qualquer outro.
 */
static bool timer_supported(TIM_TypeDef *t)
{
    return t == TIM1 || t == TIM2 || t == TIM3 || t == TIM4 || t == TIM5;
}

/**
 * @brief Configura um canal do timer em PWM mode 1 com preload e habilita a saida (CCER).
 *
 * @param t  Timer.
 * @param ch Canal (1 a 4).
 */
static void channel_init(TIM_TypeDef *t, uint8_t ch)
{
    volatile uint32_t *ccmr = (ch <= 2) ? &t->CCMR1 : &t->CCMR2;
    uint32_t sh = ((ch - 1) & 1U) * 8U;

    *ccmr &= ~(0xFFUL << sh);
    *ccmr |= ((6UL << 4) | (1UL << 3)) << sh; /* PWM mode 1 + preload */
    t->CCER |= 1UL << ((ch - 1) * 4U);        /* habilita saída */
}

/**
 * @brief Escreve o valor de comparacao (CCRx) de um canal.
 *
 * @param t  Timer.
 * @param ch Canal (1 a 4).
 * @param v  Valor de comparacao (0 a ARR+1; ARR+1 = 100%).
 *
 * @note Assume CCR1..CCR4 contiguos na memoria.
 */
static inline void set_ccr(TIM_TypeDef *t, uint8_t ch, uint32_t v)
{
    (&t->CCR1)[ch - 1] = v; /* CCR1..CCR4 são contíguos */
}

/**
 * @brief Configura o pino como alternate function do timer, inicializa o canal e zera o duty.
 *
 * @param g Configuracao do pino/canal.
 * @param t Timer.
 *
 * @retval DRV8833_OK                      Configurado.
 * @retval DRV8833_ERR_INVALID_CHANNEL     Canal fora de 1 a 4.
 * @retval DRV8833_ERR_INVALID_CONFIG_PORT Falha na configuracao do GPIO.
 */
static drv8833_err_t pin_setup(const drv883_gpio_config_t *g, TIM_TypeDef *t)
{
    if (g->channel < 1 || g->channel > 4)
        return DRV8833_ERR_INVALID_CHANNEL;

    if (driver_gpio_enable_clock(g->gpio_port) != DRIVER_OK ||
        driver_gpio_set_alternate_function(g->gpio_port, g->pin,
                                           g->alternate_function) != DRIVER_OK ||
        driver_gpio_set_speed(g->gpio_port, g->pin, DRIVER_GPIO_SPEED_HIGH) != DRIVER_OK)
        return DRV8833_ERR_INVALID_CONFIG_PORT;

    channel_init(t, g->channel);
    set_ccr(t, g->channel, 0);
    return DRV8833_OK;
}

/**
 * @brief Configura o pino nFAULT como entrada com pull-up e interrupcao EXTI por borda de descida.
 *
 * Nao faz nada se fault.port for NULL.
 *
 * @param drv Handle com a configuracao ja copiada.
 *
 * @retval DRV8833_OK                      Configurado (ou fault desabilitado).
 * @retval DRV8833_ERR_INVALID_CONFIG_PORT Falha na configuracao do GPIO.
 */
static drv8833_err_t fault_setup(drv8833_t *drv)
{
    const drv8833_pin_t *f = &drv->cfg.fault;
    if (f->port == NULL)
        return DRV8833_OK;

    if (driver_gpio_enable_clock(f->port) != DRIVER_OK ||
        driver_gpio_set_mode(f->port, f->pin, DRIVER_GPIO_INPUT) != DRIVER_OK ||
        driver_gpio_set_pull(f->port, f->pin, DRIVER_GPIO_PULL_UP) != DRIVER_OK)
        return DRV8833_ERR_INVALID_CONFIG_PORT;

    uint32_t port_idx = ((uint32_t)f->port - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE);
    uint32_t sh = (f->pin & 3U) * 4U;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[f->pin >> 2] &= ~(0xFUL << sh);
    SYSCFG->EXTICR[f->pin >> 2] |= port_idx << sh;

    EXTI->FTSR |= 1UL << f->pin; /* nFAULT cai em falha */
    EXTI->PR = 1UL << f->pin;
    EXTI->IMR |= 1UL << f->pin;

    IRQn_Type irq = (f->pin <= 4)   ? (IRQn_Type)(EXTI0_IRQn + f->pin)
                    : (f->pin <= 9) ? EXTI9_5_IRQn
                                    : EXTI15_10_IRQn;
    NVIC_EnableIRQ(irq);

    s_fault_owner[f->pin] = drv;
    return DRV8833_OK;
}

void drv8833_fault_irq_handler(uint8_t pin)
{
    if (pin > 15 || !(EXTI->PR & (1UL << pin)))
        return;
    EXTI->PR = 1UL << pin;

    drv8833_t *d = s_fault_owner[pin];
    if (d && d->cfg.fault_cb)
        d->cfg.fault_cb(d->cfg.fault_ctx);
}

drv8833_err_t drv8833_init(drv8833_t *drv, const drv8833_config_t *config)
{
    if (drv == NULL || config == NULL || !timer_supported(config->timer))
        return DRV8833_ERR_INVALID_ARG;

    drv->cfg = *config;
    drv->initialized = false;
    TIM_TypeDef *t = drv->cfg.timer;
    drv8833_err_t err;

    timer_clock_enable(t);

    /* No modo paralelo só a ponte A é usada (BIN1/BIN2 ligados em AIN1/AIN2) */
    uint8_t n = (config->mode == DRV8833_PARALLEL_MODE) ? 1 : DRV8833_MAX_CHANNELS;

    for (uint8_t i = 0; i < DRV8833_MAX_CHANNELS; ++i)
        if ((err = pin_setup(&drv->cfg.ain[i], t)) != DRV8833_OK)
            return err;

    if (config->mode != DRV8833_PARALLEL_MODE)
        for (uint8_t i = 0; i < n; ++i)
            if ((err = pin_setup(&drv->cfg.bin[i], t)) != DRV8833_OK)
                return err;

    /* sleep: começa desligado */
    if (driver_gpio_enable_clock(drv->cfg.sleep.port) != DRIVER_OK ||
        driver_gpio_set_mode(drv->cfg.sleep.port, drv->cfg.sleep.pin, DRIVER_GPIO_OUTPUT) != DRIVER_OK ||
        driver_gpio_write_pin(drv->cfg.sleep.port, drv->cfg.sleep.pin, DRIVER_GPIO_PIN_RESET) != DRIVER_OK)
        return DRV8833_ERR_INVALID_CONFIG_PORT;

    if ((err = fault_setup(drv)) != DRV8833_OK)
        return err;

    t->PSC = config->prescale;
    t->ARR = config->autorreload;
    t->CR1 &= ~(TIM_CR1_CMS_Msk | TIM_CR1_DIR_Msk);
    t->CR1 |= TIM_CR1_ARPE;
    t->EGR = TIM_EGR_UG;
    if (t == TIM1)
        t->BDTR |= TIM_BDTR_MOE;
    t->CR1 |= TIM_CR1_CEN;

    drv->initialized = true;
    return DRV8833_OK;
}

drv8833_err_t drv8833_deinit(drv8833_t *drv)
{
    if (drv == NULL || !drv->initialized)
        return DRV8833_ERR_NOT_INIT;

    drv8833_enable(drv, DRV8833_STATE_OFF);
    drv->cfg.timer->CR1 &= ~TIM_CR1_CEN;
    drv->cfg.timer->CCER = 0;
    if (drv->cfg.fault.port)
    {
        EXTI->IMR &= ~(1UL << drv->cfg.fault.pin);
        s_fault_owner[drv->cfg.fault.pin] = NULL;
    }
    drv->initialized = false;
    return DRV8833_OK;
}

drv8833_err_t drv8833_enable(drv8833_t *drv, drv8833_state_t state)
{
    if (drv == NULL || !drv->initialized)
        return DRV8833_ERR_NOT_INIT;
    driver_gpio_write_pin(drv->cfg.sleep.port, drv->cfg.sleep.pin,
                          state == DRV8833_STATE_ON ? DRIVER_GPIO_PIN_SET
                                                    : DRIVER_GPIO_PIN_RESET);
    return DRV8833_OK;
}

static const drv883_gpio_config_t *bridge_pins(drv8833_t *drv, drv8833_bridge_t br)
{
    if (br == DRV8833_BRIDGE_B && drv->cfg.mode != DRV8833_PARALLEL_MODE)
        return drv->cfg.bin;
    return drv->cfg.ain;
}

drv8833_err_t drv8833_drive(drv8833_t *drv, drv8833_bridge_t br,
                            drv8833_dir_t dir, drv8833_decay_t decay, float duty)
{
    if (drv == NULL || !drv->initialized)
        return DRV8833_ERR_NOT_INIT;

    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;

    TIM_TypeDef *t = drv->cfg.timer;
    const drv883_gpio_config_t *p = bridge_pins(drv, br);
    uint32_t top = drv->cfg.autorreload + 1U; /* CCR = ARR+1 -> 100% */
    uint32_t d = (uint32_t)(duty * (float)top);
    uint32_t in1, in2;

    /* Tabela do datasheet (PWM em IN ativo):
     * FWD fast: IN1=PWM(d)     IN2=0
     * FWD slow: IN1=1          IN2=PWM(1-d)
     * REV fast: IN1=0          IN2=PWM(d)
     * REV slow: IN1=PWM(1-d)   IN2=1          */
    if (decay == DRV8833_DECAY_FAST)
    {
        in1 = (dir == DRV8833_DIR_FORWARD) ? d : 0;
        in2 = (dir == DRV8833_DIR_FORWARD) ? 0 : d;
    }
    else
    {
        in1 = (dir == DRV8833_DIR_FORWARD) ? top : top - d;
        in2 = (dir == DRV8833_DIR_FORWARD) ? top - d : top;
    }

    set_ccr(t, p[0].channel, in1);
    set_ccr(t, p[1].channel, in2);
    return DRV8833_OK;
}

drv8833_err_t drv8833_brake(drv8833_t *drv, drv8833_bridge_t br)
{
    if (drv == NULL || !drv->initialized)
        return DRV8833_ERR_NOT_INIT;
    const drv883_gpio_config_t *p = bridge_pins(drv, br);
    uint32_t top = drv->cfg.autorreload + 1U;
    set_ccr(drv->cfg.timer, p[0].channel, top);
    set_ccr(drv->cfg.timer, p[1].channel, top);
    return DRV8833_OK;
}

drv8833_err_t drv8833_coast(drv8833_t *drv, drv8833_bridge_t br)
{
    if (drv == NULL || !drv->initialized)
        return DRV8833_ERR_NOT_INIT;
    const drv883_gpio_config_t *p = bridge_pins(drv, br);
    set_ccr(drv->cfg.timer, p[0].channel, 0);
    set_ccr(drv->cfg.timer, p[1].channel, 0);
    return DRV8833_OK;
}