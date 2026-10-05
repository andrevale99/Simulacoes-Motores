#ifndef DRV8833_H
#define DRV8833_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f411xe.h>

/** Numero de canais PWM (IN1/IN2) por ponte H. */
#define DRV8833_MAX_CHANNELS 2

/** @brief Codigos de retorno da biblioteca. */
typedef enum
{
    DRV8833_OK = 0,                     /**< Sucesso. */
    DRV8833_ERR_INVALID_ARG = -1,       /**< Argumento nulo ou invalido. */
    DRV8833_ERR_INVALID_CONFIG_PORT = -2, /**< Falha ao configurar um pino GPIO. */
    DRV8833_ERR_INVALID_CHANNEL = -3,   /**< Canal do timer fora de 1..4. */
    DRV8833_ERR_NOT_INIT = -4,          /**< Driver nao inicializado. */
} drv8833_err_t;

/** @brief Modo de operacao do DRV8833. */
typedef enum
{
    DRV8833_NORMAL_MODE,    /**< Duas pontes independentes (A e B). */
    DRV8833_PARALLEL_MODE,  /**< Pontes em paralelo (BINx ligados em AINx); so a ponte A e controlada. */
} drv8833_mode_t;

/** @brief Estado do driver (pino nSLEEP). */
typedef enum
{
    DRV8833_STATE_OFF,  /**< Driver em sleep (nSLEEP = 0). */
    DRV8833_STATE_ON,   /**< Driver ativo (nSLEEP = 1). */
} drv8833_state_t;

/** @brief Ponte H a ser controlada. */
typedef enum
{
    DRV8833_BRIDGE_A,   /**< Saidas AOUT1/AOUT2. */
    DRV8833_BRIDGE_B,   /**< Saidas BOUT1/BOUT2 (ignorado no modo paralelo). */
} drv8833_bridge_t;

/** @brief Sentido de rotacao. */
typedef enum
{
    DRV8833_DIR_FORWARD,    /**< Sentido horario (IN1 ativo). */
    DRV8833_DIR_REVERSE,    /**< Sentido anti-horario (IN2 ativo). */
} drv8833_dir_t;

/** @brief Tipo de decaimento da corrente durante o tempo off do PWM. */
typedef enum
{
    DRV8833_DECAY_FAST,     /**< Fast decay: motor roda livre (coast). */
    DRV8833_DECAY_SLOW,     /**< Slow decay: motor freia (brake). */
} drv8833_decay_t;

/**
 * @brief Callback chamado quando o pino nFAULT vai para nivel baixo.
 *
 * @param ctx Ponteiro de usuario definido em drv8833_config_t::fault_ctx.
 *
 * @note Executa em contexto de interrupcao (EXTI); deve ser curto.
 */
typedef void (*drv8833_fault_cb_t)(void *ctx);

/** @brief Configuracao de um pino de entrada (INx) ligado a um canal PWM. */
typedef struct
{
    GPIO_TypeDef *gpio_port;    /**< Porta GPIO (GPIOA, GPIOB, ...). */
    uint16_t pin;               /**< Numero do pino (0 a 15). */
    uint8_t alternate_function; /**< Alternate function do timer (AF0 a AF15). */
    uint8_t channel;            /**< Canal do timer associado ao pino (1 a 4). */
} drv883_gpio_config_t;

/** @brief Pino GPIO simples (sleep / fault). */
typedef struct
{
    GPIO_TypeDef *port;         /**< Porta GPIO. */
    uint16_t pin;               /**< Numero do pino (0 a 15). */
} drv8833_pin_t;

/** @brief Configuracao do driver, copiada para dentro de drv8833_t no init. */
typedef struct
{
    TIM_TypeDef *timer;         /**< Timer usado no PWM (TIM1 a TIM5). */
    uint32_t autorreload;       /**< Valor de ARR (define a frequencia e a resolucao do PWM). */
    uint32_t prescale;          /**< Valor de PSC. */
    drv8833_mode_t mode;        /**< Modo de operacao. */

    drv883_gpio_config_t ain[DRV8833_MAX_CHANNELS]; /**< AIN1/AIN2 (ponte A). */
    drv883_gpio_config_t bin[DRV8833_MAX_CHANNELS]; /**< BIN1/BIN2 (ponte B); ignorado no modo paralelo. */

    drv8833_pin_t sleep;        /**< Pino ligado ao nSLEEP. */
    drv8833_pin_t fault;        /**< Pino ligado ao nFAULT (port = NULL desabilita). */
    drv8833_fault_cb_t fault_cb;/**< Callback de falha (pode ser NULL). */
    void *fault_ctx;            /**< Contexto repassado ao callback. */
} drv8833_config_t;

/** @brief Handle do driver. Nao modificar os campos diretamente. */
typedef struct
{
    drv8833_config_t cfg;       /**< Copia da configuracao. */
    bool initialized;           /**< true apos drv8833_init() bem-sucedido. */
} drv8833_t;

/**
 * @brief Inicializa o driver: clocks, GPIOs, timer em PWM, pino sleep e interrupcao de fault.
 *
 * O driver inicia em sleep (nSLEEP = 0) e com duty 0; use drv8833_enable() para ativa-lo.
 *
 * @param drv    Handle a ser inicializado.
 * @param config Configuracao (copiada para @p drv).
 *
 * @retval DRV8833_OK                    Inicializado.
 * @retval DRV8833_ERR_INVALID_ARG       Ponteiro nulo ou timer nao suportado.
 * @retval DRV8833_ERR_INVALID_CHANNEL   Canal fora de 1 a 4.
 * @retval DRV8833_ERR_INVALID_CONFIG_PORT Falha ao configurar algum pino.
 */
drv8833_err_t drv8833_init(drv8833_t *drv, const drv8833_config_t *config);

/**
 * @brief Desliga o driver (sleep), para o timer e desabilita a interrupcao de fault.
 *
 * @param drv Handle inicializado.
 *
 * @retval DRV8833_OK            Desinicializado.
 * @retval DRV8833_ERR_NOT_INIT  Driver nao inicializado.
 */
drv8833_err_t drv8833_deinit(drv8833_t *drv);

/**
 * @brief Liga ou desliga o DRV8833 pelo pino nSLEEP.
 *
 * @param drv   Handle inicializado.
 * @param state DRV8833_STATE_ON ou DRV8833_STATE_OFF.
 *
 * @retval DRV8833_OK            Estado aplicado.
 * @retval DRV8833_ERR_NOT_INIT  Driver nao inicializado.
 */
drv8833_err_t drv8833_enable(drv8833_t *drv, drv8833_state_t state);

/**
 * @brief Define sentido, tipo de decaimento e duty cycle de uma ponte.
 *
 * @param drv   Handle inicializado.
 * @param br    Ponte (no modo paralelo sempre usa a A).
 * @param dir   Sentido de rotacao.
 * @param decay Fast (coast) ou slow (brake) no tempo off do PWM.
 * @param duty  Duty cycle de 0.0 a 1.0 (valores fora sao saturados).
 *
 * @retval DRV8833_OK            Aplicado.
 * @retval DRV8833_ERR_NOT_INIT  Driver nao inicializado.
 *
 * @note No slow decay o PWM sai invertido no pino ativo; a funcao ja compensa.
 */
drv8833_err_t drv8833_drive(drv8833_t *drv, drv8833_bridge_t br,
                            drv8833_dir_t dir, drv8833_decay_t decay,
                            float duty);

/**
 * @brief Freia o motor em curto (IN1 = IN2 = 1).
 *
 * @param drv Handle inicializado.
 * @param br  Ponte.
 *
 * @retval DRV8833_OK            Aplicado.
 * @retval DRV8833_ERR_NOT_INIT  Driver nao inicializado.
 */
drv8833_err_t drv8833_brake(drv8833_t *drv, drv8833_bridge_t br);

/**
 * @brief Deixa o motor girar livre (IN1 = IN2 = 0).
 *
 * @param drv Handle inicializado.
 * @param br  Ponte.
 *
 * @retval DRV8833_OK            Aplicado.
 * @retval DRV8833_ERR_NOT_INIT  Driver nao inicializado.
 */
drv8833_err_t drv8833_coast(drv8833_t *drv, drv8833_bridge_t br);

/**
 * @brief Trata a interrupcao EXTI do pino nFAULT e chama o callback do usuario.
 *
 * @param pin Numero da linha EXTI (0 a 15) tratada no handler.
 *
 * @note Chamar dentro do EXTIx_IRQHandler correspondente. Limpa o flag pendente.
 */
void drv8833_fault_irq_handler(uint8_t pin);

#endif