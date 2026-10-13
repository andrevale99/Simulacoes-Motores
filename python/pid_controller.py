"""
pi_controller.py

Controlador PID (Proporcional-Integral-Derivativo) com anti-windup
por saturacao (clamping). O ganho derivativo (Kd) e opcional: se
Kd = 0.0 (padrao), o controlador se comporta exatamente como um PI
puro, mantendo compatibilidade com o uso anterior da biblioteca.

A derivada e calculada sobre a MEDIDA (feedback), nao sobre o erro:

    D = -Kd * (feedback - feedback_anterior) / Ts

Isso evita o "kick" derivativo classico (pico na saida) que ocorre
quando a referencia muda em degrau -- um degrau de referencia produz
d(erro)/dt enorme, mas nao produz d(medida)/dt enorme, ja que a
planta fisica nao muda de estado instantaneamente. E a abordagem
padrao em controladores industriais (derivative on measurement).

O anti-windup (clamping) segue a mesma logica usada no PI original:
quando a saida satura, o termo integral e corrigido para nao
continuar acumulando erro na direcao da saturacao.

Uso:
    pid = pi_controller_init(Kp, Ki, Ts, has_min, out_min, has_max, out_max, Kd)
    u = pi_controller_update(pid, referencia, realimentacao)
    pi_controller_reset(pid)   # zera integral e memoria da derivada
"""

import math
from dataclasses import dataclass

PI_NO_LIMIT_MIN = -math.inf
PI_NO_LIMIT_MAX = math.inf


@dataclass
class PIDController:
    Kp: float
    Ki: float
    Ts: float
    Kd: float = 0.0

    has_min: bool = False
    has_max: bool = False
    output_min: float = PI_NO_LIMIT_MIN
    output_max: float = PI_NO_LIMIT_MAX

    integral: float = 0.0
    error: float = 0.0

    prev_feedback: float = 0.0
    has_prev_feedback: bool = False   # evita "kick" derivativo no 1o passo


def pid_controller_init(Kp, Ki, Kd, Ts, has_min, output_min, has_max, output_max):
    return PIDController(
        Kp=Kp, Ki=Ki, Kd=Kd, Ts=Ts,
        has_min=has_min, has_max=has_max,
        output_min=output_min if has_min else PI_NO_LIMIT_MIN,
        output_max=output_max if has_max else PI_NO_LIMIT_MAX,
    )


def pid_controller_reset(pid):
    """Zera o termo integral e a memoria da derivada."""
    pid.integral = 0.0
    pid.has_prev_feedback = False


def pid_controller_update(pid, reference, feedback):
    pid.error = reference - feedback
    proportional = pid.Kp * pid.error

    pid.integral += pid.Ki * pid.error * pid.Ts

    if pid.has_prev_feedback:
        derivative = -pid.Kd * (feedback - pid.prev_feedback) / pid.Ts
    else:
        derivative = 0.0

    pid.prev_feedback = feedback
    pid.has_prev_feedback = True

    output = proportional + pid.integral + derivative

    if pid.has_min and output < pid.output_min:
        output = pid.output_min
        if pid.error < 0.0:
            pid.integral -= pid.Ki * pid.error * pid.Ts

    if pid.has_max and output > pid.output_max:
        output = pid.output_max
        if pid.error > 0.0:
            pid.integral -= pid.Ki * pid.error * pid.Ts

    return output

# ======================================================================
#   NPID: PID NAO LINEAR (GANHOS VARIAVEIS COM O MODULO DO ERRO)
# ======================================================================

# Parametro `a` padrao das funcoes de erro: o valor usado no C2 de npid.py.
NPID_DEFAULT_A = 1.0 / 0.6


def npid_shape_rising(a=NPID_DEFAULT_A):
    """
    Funcao de erro "crescente": phi(|e|) = 1 - (1 + a|e|) * exp(-a|e|).

    phi = 0 em e = 0 e tende a 1 quando |e| cresce. Com ela, o ganho
    parte do MINIMO (erro nulo) e sobe ate o MAXIMO (erro grande).
    `a` controla a rapidez da transicao (maior a -> transicao mais rapida).
    """
    return lambda abs_e: 1.0 - (1.0 + a * abs_e) * math.exp(-a * abs_e)


def npid_shape_falling(a=NPID_DEFAULT_A):
    """
    Funcao de erro "decrescente": phi(|e|) = (1 + a|e|) * exp(-a|e|).

    phi = 1 em e = 0 e tende a 0 quando |e| cresce. Com ela, o ganho
    parte do MAXIMO (erro nulo) e cai ate o MINIMO (erro grande).
    """
    return lambda abs_e: (1.0 + a * abs_e) * math.exp(-a * abs_e)


@dataclass
class NPIDController:
    """
    PID nao linear: Kp, Ki e Kd variam com o modulo do erro |e|.

        u = Kp(|e|)*e + Ki(|e|)*Integral(e dt) - Kd(|e|)*d(feedback)/dt

    Cada ganho e calculado a cada passo por:

        ganho_i(|e|) = min_i + (max_i - min_i) * phi_i(|e|)

    onde phi_i e a funcao de erro do ganho i, com saida limitada a
    [0, 1] (ver npid_shape_rising / npid_shape_falling).

    Se `gain_functions` NAO for informado, usa-se o padrao (com
    a = NPID_DEFAULT_A = 1/0.6, o valor do C2 de npid.py):

        Kp: npid_shape_rising   (minimo com erro nulo, maximo com erro grande)
        Ki: npid_shape_falling  (maximo com erro nulo, minimo com erro grande)
        Kd: npid_shape_falling  (maximo com erro nulo, minimo com erro grande)

    Se `gain_functions` for informado, um elemento None da tupla deixa
    aquele ganho fixo no valor nominal de `gains`.

    Tal como no controlador C2 de npid.py, o ganho multiplica a
    integral ACUMULADA do erro (nao o incremento): quando o ganho muda,
    a contribuicao integral muda junto, instantaneamente.

    A derivada e calculada sobre a MEDIDA (como em PIDController), e o
    anti-windup e por saturacao (clamping), desfazendo o ultimo
    incremento da integral quando a saida satura no sentido do erro.

    Parametros
    ----------
    gain_limits : ((kp_min, kp_max), (ki_min, ki_max), (kd_min, kd_max))
        Limites inferior e superior de cada ganho.
    gains : (Kp, Ki, Kd)
        Ganhos nominais: valor inicial e valor fixo dos ganhos cuja
        funcao de erro e None.
    Ts : float
        Periodo de amostragem [s].
    gain_functions : (phi_kp, phi_ki, phi_kd), opcional
        Funcoes de erro de cada ganho: callable(|e|) -> [0, 1] ou None.
        Se omitido, usa as funcoes padrao descritas acima.
    has_min, output_min, has_max, output_max
        Saturacao opcional da saida (mesma convencao de PIDController).

    Exemplo (mesmos Kp e Ki do C2 de npid.py, com as funcoes padrao):
        pid = NPIDController(
            gain_limits=((4.0, 15.0), (3.0, 10.0), (0.0, 0.0)),
            gains=(4.0, 10.0, 0.0),
            Ts=0.001,
        )
        u = pid.update(referencia, realimentacao)
    """
    gain_limits: tuple
    gains: tuple
    Ts: float
    gain_functions: tuple = None

    has_min: bool = False
    has_max: bool = False
    output_min: float = PI_NO_LIMIT_MIN
    output_max: float = PI_NO_LIMIT_MAX

    # ganhos efetivos usados no ultimo update (util para log/grafico)
    Kp: float = 0.0
    Ki: float = 0.0
    Kd: float = 0.0

    integral: float = 0.0          # Integral(e dt) bruta, sem ganho
    error: float = 0.0

    prev_feedback: float = 0.0
    has_prev_feedback: bool = False

    def __post_init__(self):
        if self.gain_functions is None:
            self.gain_functions = (npid_shape_rising(),
                                   npid_shape_falling(),
                                   npid_shape_falling())

        for nome, seq in (("gain_functions", self.gain_functions),
                          ("gain_limits", self.gain_limits),
                          ("gains", self.gains)):
            if len(seq) != 3:
                raise ValueError(f"{nome} deve ter 3 elementos (Kp, Ki, Kd)")

        for gmin, gmax in self.gain_limits:
            if gmin > gmax:
                raise ValueError("em gain_limits, min deve ser <= max")

        self.Kp, self.Ki, self.Kd = self.gains

    def _gain(self, i, abs_e):
        fn = self.gain_functions[i]
        if fn is None:
            return self.gains[i]
        gmin, gmax = self.gain_limits[i]
        phi = min(max(fn(abs_e), 0.0), 1.0)
        return gmin + (gmax - gmin) * phi

    def reset(self):
        """Zera a integral e a memoria da derivada; ganhos voltam aos nominais."""
        self.integral = 0.0
        self.has_prev_feedback = False
        self.Kp, self.Ki, self.Kd = self.gains

    def update(self, reference, feedback):
        """Executa um passo de controle e retorna a saida."""
        self.error = reference - feedback
        abs_e = abs(self.error)

        self.Kp = self._gain(0, abs_e)
        self.Ki = self._gain(1, abs_e)
        self.Kd = self._gain(2, abs_e)

        self.integral += self.error * self.Ts

        if self.has_prev_feedback:
            derivative = -self.Kd * (feedback - self.prev_feedback) / self.Ts
        else:
            derivative = 0.0

        self.prev_feedback = feedback
        self.has_prev_feedback = True

        output = self.Kp * self.error + self.Ki * self.integral + derivative

        if self.has_min and output < self.output_min:
            output = self.output_min
            if self.error < 0.0:
                self.integral -= self.error * self.Ts

        if self.has_max and output > self.output_max:
            output = self.output_max
            if self.error > 0.0:
                self.integral -= self.error * self.Ts

        return output


def npid_controller_init(gain_limits, gains, Ts, gain_functions=None,
                         has_min=False, output_min=PI_NO_LIMIT_MIN,
                         has_max=False, output_max=PI_NO_LIMIT_MAX):
    return NPIDController(
        gain_functions=gain_functions, gain_limits=gain_limits,
        gains=gains, Ts=Ts,
        has_min=has_min, has_max=has_max,
        output_min=output_min if has_min else PI_NO_LIMIT_MIN,
        output_max=output_max if has_max else PI_NO_LIMIT_MAX,
    )


def npid_controller_reset(npid):
    npid.reset()


def npid_controller_update(npid, reference, feedback):
    return npid.update(reference, feedback)