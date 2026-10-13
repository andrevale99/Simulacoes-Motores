#ifndef ENCODER_H
#define ENCODER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f411xe.h>

/** @brief Codigos de retorno da biblioteca. */
typedef enum
{
    ENCODER_OK = 0,                         /**< Sucesso. */
    ENCODER_ERR_INVALID_ARG = -1,           /**< Argumento nulo ou invalido. */
    ENCODER_ERR_INVALID_CONFIG_PORT = -2,   /**< Falha ao configurar um pino GPIO. */
    ENCODER_ERR_NOT_INIT = -3,              /**< Encoder nao inicializado. */
} encoder_err_t;

/** @brief Modo de contagem de pulsos. */
typedef enum
{
    ENCODER_X1 = 1,     /**< 1 contagem por ciclo (hardware em X4, dividido por 4 em software). */
    ENCODER_X2 = 2,     /**< Conta as bordas de um canal (hardware). */
    ENCODER_X4 = 4,     /**< Conta as bordas dos dois canais (hardware). */
} encoder_mode_t;

/** @brief Pino de um canal do encoder (TIMx_CH1 ou TIMx_CH2). */
typedef struct
{
    GPIO_TypeDef *port;         /**< Porta GPIO (GPIOA, GPIOB, ...). */
    uint16_t pin;               /**< Numero do pino (0 a 15). */
    uint8_t alternate_function; /**< Alternate function do timer (AF0 a AF15). */
} encoder_pin_t;

/** @brief Dados entregues ao callback a cada amostragem. */
typedef struct
{
    int32_t delta_counts;       /**< Contagens no periodo (ja no modo X1/X2/X4). */
    int32_t position_counts;    /**< Posicao acumulada em contagens. */
    float   dt_s;               /**< Periodo de amostragem em segundos. */
    float   motor_rad_s;        /**< Velocidade no eixo do motor em rad/s. */
    float   motor_rpm;          /**< Velocidade no eixo do motor em RPM. */
    float   output_rad_s;       /**< Velocidade no eixo de saida (apos a reducao) em rad/s. */
    float   output_rpm;         /**< Velocidade no eixo de saida (apos a reducao) em RPM. */
} encoder_sample_t;

/**
 * @brief Callback chamado a cada amostragem, dentro de encoder_update().
 *
 * @param sample Amostra calculada (valida apenas durante a chamada).
 * @param ctx    Ponteiro de usuario definido em encoder_config_t::sample_ctx.
 *
 * @note Executa no contexto de quem chama encoder_update().
 */
typedef void (*encoder_sample_cb_t)(const encoder_sample_t *sample, void *ctx);

/** @brief Configuracao do encoder, copiada para encoder_t no init. */
typedef struct
{
    TIM_TypeDef *timer;         /**< Timer usado (TIM1 a TIM5; TIM2 e TIM5 sao 32 bits). */
    encoder_pin_t ch1;          /**< Pino do canal 1 (fase A). */
    encoder_pin_t ch2;          /**< Pino do canal 2 (fase B). */
    uint8_t input_filter;       /**< Filtro digital de entrada ICxF (0 a 15). */

    encoder_mode_t mode;        /**< Modo de contagem: X1, X2 ou X4. */
    uint32_t pulses_per_rev;    /**< Ciclos por volta do eixo do motor, por canal (antes da reducao). */
    float gear_ratio;           /**< Reducao (ex.: 30.0 = 30:1); use 1.0 se nao houver caixa. */
    uint32_t sample_period_ms;  /**< Periodo, em ms, com que encoder_update() sera chamada. */

    encoder_sample_cb_t sample_cb;  /**< Callback de amostragem (pode ser NULL). */
    void *sample_ctx;               /**< Contexto repassado ao callback. */
} encoder_config_t;

/** @brief Handle do encoder. Nao modificar os campos diretamente. */
typedef struct
{
    encoder_config_t cfg;       /**< Copia da configuracao. */
    uint32_t last_raw;          /**< Ultimo valor lido de CNT. */
    int32_t  remainder;         /**< Resto da divisao por 4 no modo X1. */
    int32_t  position;          /**< Posicao acumulada em contagens. */
    bool     is_32bit;          /**< true se o timer tem contador de 32 bits. */
    bool     initialized;       /**< true apos encoder_init() bem-sucedido. */
    encoder_sample_t last;      /**< Ultima amostra calculada. */
} encoder_t;

/**
 * @brief Inicializa o timer em modo encoder, os pinos e zera a contagem.
 *
 * @param enc    Handle a ser inicializado.
 * @param config Configuracao (copiada para @p enc).
 *
 * @retval ENCODER_OK                       Inicializado.
 * @retval ENCODER_ERR_INVALID_ARG          Ponteiro nulo, timer nao suportado, modo,
 *                                          filtro, PPR, reducao ou periodo invalidos.
 * @retval ENCODER_ERR_INVALID_CONFIG_PORT  Falha ao configurar algum pino.
 */
encoder_err_t encoder_init(encoder_t *enc, const encoder_config_t *config);

/**
 * @brief Para o timer e desabilita o modo encoder.
 *
 * @param enc Handle inicializado.
 *
 * @retval ENCODER_OK            Desinicializado.
 * @retval ENCODER_ERR_NOT_INIT  Encoder nao inicializado.
 */
encoder_err_t encoder_deinit(encoder_t *enc);

/**
 * @brief Le o contador, calcula posicao e velocidade e chama o callback.
 *
 * @param enc Handle inicializado.
 *
 * @retval ENCODER_OK            Amostra calculada.
 * @retval ENCODER_ERR_NOT_INIT  Encoder nao inicializado.
 *
 * @note Deve ser chamada a cada encoder_config_t::sample_period_ms. Em timers de
 *       16 bits, o periodo deve ser curto o bastante para menos de 32768 contagens.
 */
encoder_err_t encoder_update(encoder_t *enc);

/**
 * @brief Copia a ultima amostra calculada, sem esperar o callback.
 *
 * @param enc Handle inicializado.
 * @param out Destino da amostra.
 *
 * @retval ENCODER_OK               Copiada.
 * @retval ENCODER_ERR_INVALID_ARG  @p enc ou @p out nulo.
 * @retval ENCODER_ERR_NOT_INIT     Encoder nao inicializado.
 */
encoder_err_t encoder_get_sample(const encoder_t *enc, encoder_sample_t *out);

/**
 * @brief Zera a posicao acumulada e o resto do modo X1.
 *
 * @param enc Handle inicializado.
 *
 * @retval ENCODER_OK            Zerado.
 * @retval ENCODER_ERR_NOT_INIT  Encoder nao inicializado.
 */
encoder_err_t encoder_reset(encoder_t *enc);

#endif