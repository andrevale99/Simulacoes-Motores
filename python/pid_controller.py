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