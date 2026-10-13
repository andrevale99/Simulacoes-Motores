#include "encoder.h"
#include "drivers/gpio/driver_gpio.h"

/** @brief Constante 2*pi, usada na conversao de rotacoes/s para rad/s. */
#define ENC_TWO_PI 6.28318530718f

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
 * @brief Habilita o clock do timer no barramento APB correspondente.
 *
 * @param t Timer (TIM1 a TIM5). Timers nao suportados sao ignorados.
 */
static void timer_clock_enable(TIM_TypeDef *t)
{
    if (t == TIM1)      RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (t == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (t == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (t == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (t == TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
}

/**
 * @brief Configura um pino de canal como alternate function com pull-up.
 *
 * @param p Configuracao do pino.
 *
 * @retval ENCODER_OK                       Configurado.
 * @retval ENCODER_ERR_INVALID_CONFIG_PORT  Falha na configuracao do GPIO.
 */
static encoder_err_t pin_setup(const encoder_pin_t *p)
{
    if (driver_gpio_enable_clock(p->port) != DRIVER_OK ||
        driver_gpio_set_alternate_function(p->port, p->pin,
                                           p->alternate_function) != DRIVER_OK ||
        driver_gpio_set_pull(p->port, p->pin, DRIVER_GPIO_PULL_UP) != DRIVER_OK)
        return ENCODER_ERR_INVALID_CONFIG_PORT;
    return ENCODER_OK;
}

/**
 * @brief Le o valor bruto do contador (CNT) do timer do encoder.
 *
 * @param e Handle do encoder.
 *
 * @return Valor de CNT (16 ou 32 bits, conforme o timer).
 */
static inline uint32_t read_raw(const encoder_t *e)
{
    return e->cfg.timer->CNT;
}

encoder_err_t encoder_init(encoder_t *enc, const encoder_config_t *config)
{
    if (enc == NULL || config == NULL || !timer_supported(config->timer) ||
        config->pulses_per_rev == 0 || config->gear_ratio <= 0.0f ||
        config->sample_period_ms == 0 || config->input_filter > 15 ||
        (config->mode != ENCODER_X1 && config->mode != ENCODER_X2 &&
         config->mode != ENCODER_X4))
        return ENCODER_ERR_INVALID_ARG;

    enc->cfg = *config;
    enc->initialized = false;
    enc->position = 0;
    enc->remainder = 0;
    enc->is_32bit = (config->timer == TIM2 || config->timer == TIM5);

    encoder_err_t err;
    if ((err = pin_setup(&enc->cfg.ch1)) != ENCODER_OK) return err;
    if ((err = pin_setup(&enc->cfg.ch2)) != ENCODER_OK) return err;

    TIM_TypeDef *t = enc->cfg.timer;
    timer_clock_enable(t);

    t->CR1  = 0;
    t->PSC  = 0;
    t->ARR  = enc->is_32bit ? 0xFFFFFFFFUL : 0xFFFFUL;

    /* CC1S = 01 (TI1), CC2S = 01 (TI2), filtros */
    t->CCMR1 = (1UL << 0) | (1UL << 8) |
               ((uint32_t)config->input_filter << 4) |
               ((uint32_t)config->input_filter << 12);

    t->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    t->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    /* X2: SMS=001 (bordas de TI2). X1 e X4: SMS=011 (X1 dividido em software) */
    t->SMCR &= ~TIM_SMCR_SMS;
    t->SMCR |= (config->mode == ENCODER_X2) ? 1UL : 3UL;

    t->CNT = 0;
    t->EGR = TIM_EGR_UG;
    t->CNT = 0;
    t->CR1 |= TIM_CR1_CEN;

    enc->last_raw = 0;
    enc->initialized = true;
    return ENCODER_OK;
}

encoder_err_t encoder_deinit(encoder_t *enc)
{
    if (enc == NULL || !enc->initialized) return ENCODER_ERR_NOT_INIT;
    enc->cfg.timer->CR1 &= ~TIM_CR1_CEN;
    enc->cfg.timer->SMCR &= ~TIM_SMCR_SMS;
    enc->initialized = false;
    return ENCODER_OK;
}

encoder_err_t encoder_update(encoder_t *enc)
{
    if (enc == NULL || !enc->initialized) return ENCODER_ERR_NOT_INIT;

    uint32_t raw = read_raw(enc);
    int32_t delta_raw = enc->is_32bit ? (int32_t)(raw - enc->last_raw)
                                      : (int32_t)(int16_t)(uint16_t)(raw - enc->last_raw);
    enc->last_raw = raw;

    int32_t delta;
    if (enc->cfg.mode == ENCODER_X1) {
        enc->remainder += delta_raw;
        delta = enc->remainder / 4;
        enc->remainder -= delta * 4;
    } else {
        delta = delta_raw;
    }
    enc->position += delta;

    /* contagens por volta do motor = PPR * (1, 2 ou 4) */
    float counts_per_rev = (float)enc->cfg.pulses_per_rev * (float)enc->cfg.mode;
    float dt = (float)enc->cfg.sample_period_ms / 1000.0f;
    float motor_rev_s = ((float)delta / counts_per_rev) / dt;

    encoder_sample_t *s = &enc->last;
    s->delta_counts    = delta;
    s->position_counts = enc->position;
    s->dt_s            = dt;
    s->motor_rad_s     = motor_rev_s * ENC_TWO_PI;
    s->motor_rpm       = motor_rev_s * 60.0f;
    s->output_rad_s    = s->motor_rad_s / enc->cfg.gear_ratio;
    s->output_rpm      = s->motor_rpm   / enc->cfg.gear_ratio;

    if (enc->cfg.sample_cb)
        enc->cfg.sample_cb(s, enc->cfg.sample_ctx);

    return ENCODER_OK;
}

encoder_err_t encoder_get_sample(const encoder_t *enc, encoder_sample_t *out)
{
    if (enc == NULL || out == NULL) return ENCODER_ERR_INVALID_ARG;
    if (!enc->initialized) return ENCODER_ERR_NOT_INIT;
    *out = enc->last;
    return ENCODER_OK;
}

encoder_err_t encoder_reset(encoder_t *enc)
{
    if (enc == NULL || !enc->initialized) return ENCODER_ERR_NOT_INIT;
    enc->position = 0;
    enc->remainder = 0;
    enc->last_raw = read_raw(enc);
    return ENCODER_OK;
}