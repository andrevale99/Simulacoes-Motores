import math
from dataclasses import dataclass, field
from auxs import rads_to_rpm, rpm_to_rads, TWO_PI
from inverter import inverter_phase_voltages, inverter_diode_blocking

PHI_A = 0.0
PHI_B = -2*math.pi/3
PHI_C =  2*math.pi/3

@dataclass
class BLDCMotor:
    R: float; L: float; M: float
    Ke: float; Kt: float
    J: float; B: float
    P: int
    iabc: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    eabc: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    theta_e: float = 0.0
    theta_r: float = 0.0
    omega_e: float = 0.0
    omega_r: float = 0.0
    Te: float = 0.0

@dataclass
class TimeSimulation:
    t0: float; tf: float; dt: float

def trapezoidal_back_emf(theta):
    theta = math.fmod(theta, TWO_PI)
    if theta < 0.0:
        theta += TWO_PI
    if theta < math.pi/6.0:
        return 6.0*theta/math.pi
    elif theta < 5.0*math.pi/6.0:
        return 1.0
    elif theta < 7.0*math.pi/6.0:
        return -6.0*theta/math.pi + 6.0
    elif theta < 11.0*math.pi/6.0:
        return -1.0
    else:
        return 6.0*theta/math.pi - 12.0

def bldc_step(Vabc, motor, time, Tl, trapezoidal_back_emf_flag):
    fabc = [0.0, 0.0, 0.0]

    motor.theta_e = math.fmod(motor.P*motor.theta_r, TWO_PI)
    if motor.theta_e < 0.0:
        motor.theta_e += TWO_PI
    motor.omega_e = motor.P * motor.omega_r

    for i, phi in enumerate((PHI_A, PHI_B, PHI_C)):
        angle = motor.theta_e + phi
        fabc[i] = -trapezoidal_back_emf(angle) if trapezoidal_back_emf_flag else -math.sin(angle)
        motor.eabc[i] = motor.Ke * motor.omega_r * fabc[i]

    # Neutro isolado (estrela sem fio de neutro): vn e a tensao que mantem
    # ia + ib + ic = 0, mesmo com sum(eabc) != 0 (FCEM trapezoidal).
    u = [Vabc[i] - motor.R*motor.iabc[i] - motor.eabc[i] for i in range(3)]
    vn = sum(u) / 3.0
    for i in range(3):
        diabc_i = (u[i] - vn) / (motor.L - motor.M)
        motor.iabc[i] += diabc_i * time.dt

    motor.Te = motor.Kt * sum(motor.iabc[i]*fabc[i] for i in range(3))

    domega_r = (motor.Te - Tl - motor.omega_r*motor.B) / motor.J
    motor.omega_r += domega_r * time.dt
    motor.theta_r += motor.omega_r * time.dt


def bldc_step_switched(gates, inverter, motor, time, Tl, trapezoidal_back_emf_flag=True):
    """
    Um passo do motor BLDC alimentado por um inversor modelado perna a perna
    (chaves + diodos de roda livre), com neutro isolado.

    Diferente de bldc_step(), que recebe as tensoes de fase prontas (as tres
    pernas sempre comandadas por chave), aqui:

      * o inversor (inverter_phase_voltages) resolve as chaves, os diodos e o
        neutro flutuante, e devolve a tensao v_k de cada fase em relacao ao
        neutro (None para uma fase aberta). Uma fase com as duas chaves
        desligadas e corrente zero fica ABERTA; se a FCEM levar o polo dela
        para fora de [0, Vdc], o diodo correspondente passa a conduzir;
      * o motor integra so as correntes das fases que conduzem:

            di_k/dt = (v_k - R*i_k - e_k) / (L - M)    (soma das di_k = 0)

      * o inversor aplica o bloqueio dos diodos sobre as correntes novas
        (inverter_diode_blocking): a roda livre na comutacao e o fim da
        conducao (corrente chegando a zero) acontecem naturalmente.

    Integracao: Euler explicito. Todas as derivadas (correntes, velocidade
    e posicao) sao avaliadas com o estado do inicio do passo (instante k).

    Parametros
    ----------
    gates : (ga, gb, gc), com +1 (chave superior), -1 (inferior) ou 0 (ambas
            desligadas), como em six_step_gates().
    inverter : Inverter (usa Vdc).

    Retorna
    -------
    Vabc : tensoes nos terminais das fases em relacao ao neutro do motor
           (para uma fase aberta, e a propria FCEM).
    """
    dt = time.dt
    R = motor.R
    leff = motor.L - motor.M

    # --- posicao e FCEM (igual ao bldc_step) ---
    motor.theta_e = math.fmod(motor.P*motor.theta_r, TWO_PI)
    if motor.theta_e < 0.0:
        motor.theta_e += TWO_PI
    motor.omega_e = motor.P * motor.omega_r

    fabc = [0.0, 0.0, 0.0]
    for k, phi in enumerate((PHI_A, PHI_B, PHI_C)):
        angle = motor.theta_e + phi
        fabc[k] = -trapezoidal_back_emf(angle) if trapezoidal_back_emf_flag else -math.sin(angle)
        motor.eabc[k] = motor.Ke * motor.omega_r * fabc[k]

    iabc = motor.iabc
    eabc = motor.eabc

    # --- torque e derivada da velocidade com o estado do instante k ---
    Te = motor.Kt * sum(iabc[k]*fabc[k] for k in range(3))
    domega_r = (Te - Tl - motor.omega_r*motor.B) / motor.J

    # --- 1) tensao do inversor em cada fase (None = fase aberta) ---
    vabc, diode = inverter_phase_voltages(inverter, gates, iabc, eabc, R)

    # --- 2) dinamica das correntes das fases que conduzem ---
    cond = [k for k in range(3) if vabc[k] is not None]
    if len(cond) >= 2:
        for k in cond:
            iabc[k] += (vabc[k] - R*iabc[k] - eabc[k]) / leff * dt
    else:
        # com menos de duas fases conectadas nao ha caminho para a corrente
        iabc[0] = iabc[1] = iabc[2] = 0.0

    # --- 3) bloqueio dos diodos sobre as correntes novas ---
    iabc[:] = inverter_diode_blocking(diode, iabc)

    # --- 4) dinamica mecanica (Euler explicito: posicao com a velocidade
    #        do instante k, velocidade com o torque do instante k) ---
    motor.Te = Te
    motor.theta_r += motor.omega_r * dt
    motor.omega_r += domega_r * dt

    # Fase aberta: a tensao no terminal e a propria FCEM
    return [vabc[k] if vabc[k] is not None else eabc[k] for k in range(3)]