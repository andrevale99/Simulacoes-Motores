"""
inverter.py

Modelo de inversor trifasico de dois niveis (six-switch, topologia
ponte trifasica / VSI - Voltage Source Inverter), equivalente ao
inverter.h/inverter.c em C, com tres modos de acionamento:

  1. Tensao media por duty cycle      (inverter_output_voltage)
  2. Chaveamento real via PWM         (switching_output_voltage)
  3. Comutacao six-step (120 graus)   (six_step_commutation)

Convencao: os tres bracos do inversor compartilham o mesmo barramento
CC (Vdc). Cada braco tem duas chaves complementares; aqui modelamos
apenas o estado da chave superior (1 = ligada -> polo em Vdc,
0 = desligada -> polo em 0V), assumindo chaveamento ideal
(sem tempo morto, sem quedas de tensao).
"""

from dataclasses import dataclass, field
import numpy as np

TWO_PI = 2.0 * np.pi


# ----------------------------------------------------------------------
#   ESTRUTURA DO INVERSOR
# ----------------------------------------------------------------------

@dataclass
class Inverter:
    """Modelo do inversor trifasico de dois niveis."""

    Vdc: float

    def __post_init__(self):
        if self.Vdc <= 0.0:
            raise ValueError("Vdc deve ser positivo.")


def inverter_clamp(value, min_, max_):
    """Limita um valor a um intervalo [min_, max_]."""
    return max(min_, min(value, max_))


def inverter_duty_to_pole_voltage(inverter, duty):
    """
    Converte o duty cycle na tensao media do polo da fase.

        V_pole = duty * Vdc

    duty = 0 -> V_pole = 0 V
    duty = 1 -> V_pole = Vdc
    """
    return duty * inverter.Vdc


# ----------------------------------------------------------------------
#   MODO 1: TENSAO MEDIA (MODELO CONTINUO / AVERAGE MODEL)
# ----------------------------------------------------------------------

def inverter_output_voltage(inverter, duty_a, duty_b, duty_c):
    """
    Calcula as tensoes de fase aplicadas ao motor a partir dos duty
    cycles, usando o valor medio do polo (sem resolver o chaveamento
    instante a instante). Equivalente a inverter_output_voltage() em C.

    O ponto neutro virtual e calculado como a media das tensoes de
    polo, e a tensao de fase e a tensao de polo menos essa media
    (elimina a componente de sequencia zero, valida para carga em
    estrela sem neutro fisico, como o motor BLDC/PMSM):

        Vn = (Va_pole + Vb_pole + Vc_pole) / 3
        Vx = Vx_pole - Vn
    """
    duty_a = inverter_clamp(duty_a, 0.0, 1.0)
    duty_b = inverter_clamp(duty_b, 0.0, 1.0)
    duty_c = inverter_clamp(duty_c, 0.0, 1.0)

    Va_pole = inverter_duty_to_pole_voltage(inverter, duty_a)
    Vb_pole = inverter_duty_to_pole_voltage(inverter, duty_b)
    Vc_pole = inverter_duty_to_pole_voltage(inverter, duty_c)

    Vn = (Va_pole + Vb_pole + Vc_pole) / 3.0

    return [Va_pole - Vn, Vb_pole - Vn, Vc_pole - Vn]


# ----------------------------------------------------------------------
#   MODO 2: CHAVEAMENTO REAL (PORTADORA TRIANGULAR + COMPARACAO)
# ----------------------------------------------------------------------

def inverter_carrier(Ts, t):
    """
    Portadora triangular simetrica, normalizada entre 0 e 1, com
    periodo Ts (equivalente a svpwm_carrier() em C).
    """
    phase = (t % Ts) / Ts
    return 2.0 * phase if phase < 0.5 else 2.0 - 2.0 * phase


def inverter_gate_state(duty, carrier):
    """
    Compara o duty de referencia com a portadora instantanea para
    gerar o estado de chaveamento (0 ou 1) da chave superior de um
    braco (equivalente a svpwm_gate_state() em C).

    duty > carrier -> chave superior ligada (polo em Vdc)
    duty <= carrier -> chave superior desligada (polo em 0V)
    """
    return 1 if duty > carrier else 0


def switching_output_voltage(inverter, duty_a, duty_b, duty_c, Ts, t):
    """
    Modelo de chaveamento REAL: gera os estados de porta (0/1) de
    cada braco comparando o duty de referencia com a portadora
    triangular no instante t, e calcula as tensoes de fase a partir
    do estado instantaneo (0V ou Vdc por polo) -- nao do valor medio.

    Retorna (Vabc, gates), onde:
      Vabc  = [Va, Vb, Vc]         tensoes de fase aplicadas ao motor
      gates = (gate_a, gate_b, gate_c)  estados de chaveamento (0/1)
    """
    carrier = inverter_carrier(Ts, t)

    gate_a = inverter_gate_state(duty_a, carrier)
    gate_b = inverter_gate_state(duty_b, carrier)
    gate_c = inverter_gate_state(duty_c, carrier)

    Vabc = inverter_output_voltage(inverter, gate_a, gate_b, gate_c)

    return Vabc, (gate_a, gate_b, gate_c)


# ----------------------------------------------------------------------
#   MODO 3: COMUTACAO SIX-STEP (120 GRAUS DE CONDUCAO)
# ----------------------------------------------------------------------

# state: 1 = fase alta (Vdc), -1 = fase baixa (0V), 0 = fase flutuante
SIX_STEP_TABLE = {
    1: (1, -1, 0),    # 0deg-60deg:   A alto, B baixo, C flutuante
    2: (1, 0, -1),    # 60deg-120deg: A alto, C baixo, B flutuante
    3: (0, 1, -1),    # 120deg-180deg: B alto, C baixo, A flutuante
    4: (-1, 1, 0),    # 180deg-240deg: B alto, A baixo, C flutuante
    5: (-1, 0, 1),    # 240deg-300deg: C alto, A baixo, B flutuante
    6: (0, -1, 1),    # 300deg-360deg: C alto, B baixo, A flutuante
}


def six_step_sector(theta_e):
    """
    Setor de comutacao (1 a 6), a partir do angulo eletrico [rad],
    alinhado aos cruzamentos de zero da FCEM trapezoidal (fase A = 0deg).
    """
    theta = theta_e % TWO_PI
    sector = int(theta / (np.pi / 3.0)) + 1
    return min(sector, 6)


def six_step_commutation(inverter, theta_e, duty=1.0):
    """
    Aciona o inversor no modo six-step: aplica `duty` na fase que
    deve conduzir em alto, `1 - duty` na que conduz em baixo, e 0.5
    (ponto medio, aproximando alta impedancia) na fase flutuante do
    setor atual. `duty` = 1.0 -> onda quadrada plena (sem PWM);
    valores < 1.0 permitem regular a amplitude efetiva (six-step
    modulado em amplitude / "PWM dentro do setor").

    Retorna (Vabc, sector).
    """
    sector = six_step_sector(theta_e)
    state_a, state_b, state_c = SIX_STEP_TABLE[sector]

    def state_to_duty(state):
        if state == 1:
            return duty
        if state == -1:
            return 1.0 - duty
        return 0.5

    duty_a = state_to_duty(state_a)
    duty_b = state_to_duty(state_b)
    duty_c = state_to_duty(state_c)

    Vabc = inverter_output_voltage(inverter, duty_a, duty_b, duty_c)

    return Vabc, sector


# ----------------------------------------------------------------------
#   GRANDEZAS DERIVADAS
# ----------------------------------------------------------------------

def line_to_line_voltages(Vabc):
    """
    Tensoes de linha a partir das tensoes de fase:

        Vab = Va - Vb
        Vbc = Vb - Vc
        Vca = Vc - Va
    """
    Va, Vb, Vc = Vabc
    return [Va - Vb, Vb - Vc, Vc - Va]


# ----------------------------------------------------------------------
#   AUTOTESTE (roda so quando o arquivo e executado diretamente)
# ----------------------------------------------------------------------

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    inv = Inverter(Vdc=12.0)

    Ts = 1.0 / 20000.0     # 20 kHz de chaveamento
    f_e = 50.0              # 50 Hz eletricos (referencia senoidal)
    dt = 1e-6
    tf = 0.04

    t_vec = np.arange(0.0, tf, dt)

    Va_sw, Vb_sw, Vc_sw = [], [], []
    Va_six, Vb_six, Vc_six = [], [], []

    for t in t_vec:
        theta_e = TWO_PI * f_e * t

        # --- modo PWM senoidal (average duty -> chaveamento real) ---
        m = 0.5  # indice de modulacao
        duty_a = 0.5 + 0.5 * m * np.sin(theta_e)
        duty_b = 0.5 + 0.5 * m * np.sin(theta_e - TWO_PI / 3.0)
        duty_c = 0.5 + 0.5 * m * np.sin(theta_e + TWO_PI / 3.0)

        Vabc_sw, gates = switching_output_voltage(inv, duty_a, duty_b, duty_c, Ts, t)
        Va_sw.append(Vabc_sw[0]); Vb_sw.append(Vabc_sw[1]); Vc_sw.append(Vabc_sw[2])

        # --- modo six-step ---
        Vabc_six, sector = six_step_commutation(inv, theta_e, duty=1.0)
        Va_six.append(Vabc_six[0]); Vb_six.append(Vabc_six[1]); Vc_six.append(Vabc_six[2])

    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    axs[0].plot(t_vec, Va_sw, label="Va (PWM)")
    axs[0].plot(t_vec, Vb_sw, label="Vb (PWM)")
    axs[0].plot(t_vec, Vc_sw, label="Vc (PWM)")
    axs[0].set_ylabel("Tensao [V]")
    axs[0].set_title("Inversor - PWM senoidal (chaveamento real)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(t_vec, Va_six, label="Va (six-step)")
    axs[1].plot(t_vec, Vb_six, label="Vb (six-step)")
    axs[1].plot(t_vec, Vc_six, label="Vc (six-step)")
    axs[1].set_ylabel("Tensao [V]")
    axs[1].set_xlabel("Tempo [s]")
    axs[1].set_title("Inversor - Six-step")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    # plt.savefig("inverter_autoteste.png", dpi=120)
    plt.show()
    print("Autoteste concluido. Grafico salvo em inverter_autoteste.png")