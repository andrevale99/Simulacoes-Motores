import math
from dataclasses import dataclass, field
from auxs import rads_to_rpm, rpm_to_rads, TWO_PI
from inverter import inverter_pole_voltages, inverter_open_leg_clamp

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

      * a tensao de polo de cada perna vem das chaves e dos diodos
        (inverter_pole_voltages);
      * o neutro do motor flutua e a lei das correntes (ia + ib + ic = 0) e
        respeitada;
      * uma fase com as duas chaves desligadas e corrente zero fica ABERTA
        (corrente zero, sem tensao imposta). Se a FCEM levar o polo dela para
        fora de [0, Vdc], o diodo correspondente passa a conduzir;
      * a roda livre na comutacao e o fim da conducao (corrente chegando a
        zero) acontecem naturalmente.

    Para o conjunto C de pernas que conduzem, com u_k = polo_k - R*i_k - e_k:

        vn      = media de u_k em C            (tensao do neutro)
        di_k/dt = (u_k - vn) / (L - M)        (soma das di_k = 0)

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

    # --- 1) tensao de polo de cada perna (chaves e diodos) ---
    poles, diode = inverter_pole_voltages(inverter, gates, iabc)

    # --- 2) tensao do neutro e pernas abertas forcadas pelos diodos ---
    cond = [k for k in range(3) if poles[k] is not None]
    vn = None
    if cond:
        vn = sum(poles[k] - R*iabc[k] - eabc[k] for k in cond) / len(cond)

        forcou = False
        for k in range(3):
            if poles[k] is None:
                poles[k], diode[k] = inverter_open_leg_clamp(inverter, vn + eabc[k])
                if poles[k] is not None:
                    forcou = True

        if forcou:
            cond = [k for k in range(3) if poles[k] is not None]
            vn = sum(poles[k] - R*iabc[k] - eabc[k] for k in cond) / len(cond)

    # --- 3) dinamica das correntes das pernas que conduzem ---
    if len(cond) >= 2:
        for k in cond:
            iabc[k] += (poles[k] - R*iabc[k] - eabc[k] - vn) / leff * dt
    else:
        # com menos de duas pernas conectadas nao ha caminho para a corrente
        iabc[0] = iabc[1] = iabc[2] = 0.0

    # --- 4) um diodo so conduz no seu sentido: corrente que cruza zero ---
    zerou = False
    for k in range(3):
        if (diode[k] < 0 and iabc[k] < 0.0) or (diode[k] > 0 and iabc[k] > 0.0):
            iabc[k] = 0.0
            zerou = True

    # --- 5) restabelece ia + ib + ic = 0 depois de uma perna zerar ---
    if zerou:
        vivas = [k for k in range(3) if iabc[k] != 0.0]
        if len(vivas) == 2:
            a, c = vivas
            metade = 0.5 * (iabc[a] - iabc[c])
            iabc[a], iabc[c] = metade, -metade
        elif len(vivas) < 2:
            iabc[0] = iabc[1] = iabc[2] = 0.0

    # --- 6) torque e dinamica mecanica (igual ao bldc_step) ---
    motor.Te = motor.Kt * sum(iabc[k]*fabc[k] for k in range(3))

    domega_r = (motor.Te - Tl - motor.omega_r*motor.B) / motor.J
    motor.omega_r += domega_r * dt
    motor.theta_r += motor.omega_r * dt

    if vn is None:
        return list(eabc)
    return [(poles[k] - vn) if poles[k] is not None else eabc[k] for k in range(3)]