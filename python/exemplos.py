"""
exemplos.py

Exemplos de uso do modelo de inversor trifasico (inverter.py):

  1. Criacao do Inverter                       (Inverter)
  2. Tensao media por duty cycle               (inverter_output_voltage)
  3. Chaveamento real via PWM                  (switching_output_voltage)
  4. Six-step ideal, sem carga                 (six_step_commutation)
  5. Six-step perna a perna acionando um BLDC  (six_step_gates + bldc_step_switched)
  6. BLDC alimentado por fonte ideal, sem chaveamento (bldc_step_switched)

Uso:
    python exemplos.py              # roda todos os exemplos
    python exemplos.py 4 5          # roda apenas os exemplos 4 e 5
    python exemplos.py --salvar img # salva os graficos em img/
    python exemplos.py -h           # ajuda
"""

import argparse
import os

import numpy as np
import matplotlib.pyplot as plt

from inverter import (
    TWO_PI,
    Inverter,
    inverter_output_voltage,
    inverter_carrier,
    switching_output_voltage,
    SIX_STEP_TABLE,
    six_step_sector,
    six_step_commutation,
    six_step_gates,
    line_to_line_voltages,
)
from bldc import BLDCMotor, TimeSimulation, bldc_step_switched
from simulations import SIX_STEP_THETA_OFFSET


# ----------------------------------------------------------------------
#   EXEMPLO 1: CRIACAO DO INVERSOR
# ----------------------------------------------------------------------

def exemplo_1_criacao():
    print("\n=== Exemplo 1: criacao do Inverter ===")

    inv = Inverter(Vdc=24.0)
    print(f"Inversor criado: {inv}")

    # Vdc deve ser positivo: __post_init__ rejeita valores invalidos
    try:
        Inverter(Vdc=-12.0)
    except ValueError as e:
        print(f"Inverter(Vdc=-12.0) -> ValueError: {e}")


# ----------------------------------------------------------------------
#   EXEMPLO 2: TENSAO MEDIA (DUTY -> TENSAO DE FASE)
# ----------------------------------------------------------------------

def exemplo_2_tensao_media():
    print("\n=== Exemplo 2: tensao media (inverter_output_voltage) ===")

    inv = Inverter(Vdc=24.0)

    # (duty_a, duty_b, duty_c)
    casos = [
        (0.5, 0.5, 0.5),     # tudo no meio -> tensao de fase nula
        (1.0, 1.0, 1.0),     # modo comum puro -> tambem nula (neutro acompanha)
        (1.0, 0.0, 0.0),     # A em Vdc, B e C em 0 -> Va = 2Vdc/3
        (1.0, 0.0, 0.5),     # A em Vdc, B em 0, C no meio
        (1.5, -0.2, 0.5),    # fora de [0, 1] -> saturado pelo clamp
    ]

    print(f"{'duty (a, b, c)':>22} | {'Va':>7} {'Vb':>7} {'Vc':>7} | {'soma':>6}")
    for d in casos:
        Vabc = inverter_output_voltage(inv, *d)
        print(f"{str(d):>22} | {Vabc[0]:7.2f} {Vabc[1]:7.2f} {Vabc[2]:7.2f} | "
              f"{sum(Vabc):6.2f}")

    # Referencia senoidal: o modelo medio entrega senoides "limpas"
    f_e, m = 50.0, 0.8
    t = np.linspace(0.0, 2.0 / f_e, 1000)
    Vabc = np.array([
        inverter_output_voltage(inv, *duties_senoidais(TWO_PI * f_e * tk, m))
        for tk in t
    ])

    fig, ax = plt.subplots(figsize=(10, 4))
    for k, nome in enumerate("abc"):
        ax.plot(t * 1e3, Vabc[:, k], label=f"V{nome}")
    ax.set_title(f"Exemplo 2 - Modelo medio, referencia senoidal "
                 f"(m = {m}, Vdc = {inv.Vdc} V)")
    ax.set_xlabel("Tempo [ms]")
    ax.set_ylabel("Tensao de fase [V]")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    return fig


def duties_senoidais(theta_e, m):
    """Duty cycles de um PWM senoidal com indice de modulacao m."""
    return (
        0.5 + 0.5 * m * np.sin(theta_e),
        0.5 + 0.5 * m * np.sin(theta_e - TWO_PI / 3.0),
        0.5 + 0.5 * m * np.sin(theta_e + TWO_PI / 3.0),
    )


# ----------------------------------------------------------------------
#   EXEMPLO 3: CHAVEAMENTO REAL (PORTADORA x DUTY)
# ----------------------------------------------------------------------

def exemplo_3_chaveamento():
    print("\n=== Exemplo 3: chaveamento real (switching_output_voltage) ===")

    inv = Inverter(Vdc=24.0)
    Ts = 1.0 / 10000.0       # 10 kHz
    dt = Ts / 200.0

    # --- 3a) dois periodos de chaveamento com duty constante ---
    duty = (0.8, 0.3, 0.5)
    t = np.arange(0.0, 2.0 * Ts, dt)

    carrier = np.array([inverter_carrier(Ts, tk) for tk in t])
    res = [switching_output_voltage(inv, *duty, Ts, tk) for tk in t]
    Vabc = np.array([r[0] for r in res])
    gates = np.array([r[1] for r in res])

    # a media do sinal chaveado num periodo coincide com o modelo medio
    n_Ts = int(round(Ts / dt))
    media_sw = Vabc[:n_Ts].mean(axis=0)
    media_avg = inverter_output_voltage(inv, *duty)
    print(f"duty = {duty}")
    print(f"media do chaveado em 1 Ts : {np.round(media_sw, 3)}")
    print(f"modelo medio              : {np.round(media_avg, 3)}")

    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(t * 1e6, carrier, "k", label="portadora")
    for k, nome in enumerate("abc"):
        axs[0].axhline(duty[k], ls="--", color=f"C{k}", label=f"duty {nome}")
    axs[0].set_ylabel("[pu]")
    axs[0].set_title("Exemplo 3a - Portadora triangular x duty de referencia")
    axs[0].legend(loc="upper right", fontsize=8)

    for k, nome in enumerate("abc"):
        axs[1].step(t * 1e6, gates[:, k] + 1.2 * (2 - k), where="post",
                    color=f"C{k}", label=f"gate {nome}")
    axs[1].set_yticks([])
    axs[1].set_title("Estado da chave superior de cada braco (1 = ligada)")
    axs[1].legend(loc="upper right", fontsize=8)

    for k, nome in enumerate("abc"):
        axs[2].step(t * 1e6, Vabc[:, k], where="post", color=f"C{k}",
                    label=f"V{nome}")
        axs[2].axhline(media_avg[k], ls=":", color=f"C{k}")
    axs[2].set_ylabel("Tensao de fase [V]")
    axs[2].set_xlabel("Tempo [us]")
    axs[2].set_title("Tensao de fase chaveada (pontilhado = modelo medio)")
    axs[2].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig.tight_layout()

    # --- 3b) PWM senoidal: chaveado x medio ---
    f_e, m = 50.0, 0.8
    dt = 1e-6
    t = np.arange(0.0, 1.0 / f_e, dt)
    Va_sw, Va_avg = [], []
    for tk in t:
        d = duties_senoidais(TWO_PI * f_e * tk, m)
        Va_sw.append(switching_output_voltage(inv, *d, Ts, tk)[0][0])
        Va_avg.append(inverter_output_voltage(inv, *d)[0])

    # filtro de media movel de 1 Ts, para "enxergar" a fundamental
    janela = int(round(Ts / dt))
    Va_filtr = np.convolve(Va_sw, np.ones(janela) / janela, mode="same")

    fig2, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t * 1e3, Va_sw, lw=0.5, alpha=0.5, label="Va chaveada")
    ax.plot(t * 1e3, Va_filtr, lw=2, label="Va chaveada (media movel em Ts)")
    ax.plot(t * 1e3, Va_avg, "k--", label="Va modelo medio")
    ax.set_title(f"Exemplo 3b - PWM senoidal (m = {m}, Fsw = {1/Ts/1e3:.0f} kHz)")
    ax.set_xlabel("Tempo [ms]")
    ax.set_ylabel("Tensao [V]")
    ax.legend(loc="upper right")
    ax.grid(True)
    fig2.tight_layout()

    return fig, fig2


# ----------------------------------------------------------------------
#   EXEMPLO 4: SIX-STEP IDEAL (SEM CARGA)
# ----------------------------------------------------------------------

def exemplo_4_six_step_ideal():
    print("\n=== Exemplo 4: six-step ideal (six_step_commutation) ===")

    inv = Inverter(Vdc=24.0)

    nomes = {1: "alto", -1: "baixo", 0: "flut."}
    print(f"{'setor':>5} | {'angulo':>11} | {'A':>6} {'B':>6} {'C':>6} | "
          f"{'Va':>6} {'Vb':>6} {'Vc':>6}")
    for setor in range(1, 7):
        theta = (setor - 0.5) * np.pi / 3.0      # meio do setor
        Vabc, s = six_step_commutation(inv, theta, duty=1.0)
        estados = [nomes[x] for x in SIX_STEP_TABLE[s]]
        faixa = f"{60*(setor-1):3d}-{60*setor:3d} gr"
        print(f"{s:>5} | {faixa:>11} | {estados[0]:>6} {estados[1]:>6} "
              f"{estados[2]:>6} | {Vabc[0]:6.1f} {Vabc[1]:6.1f} {Vabc[2]:6.1f}")

    # um ciclo eletrico completo, para varios duties
    theta = np.linspace(0.0, TWO_PI, 3601)[:-1]
    sectors = [six_step_sector(th) for th in theta]
    Vabc = np.array([six_step_commutation(inv, th, 1.0)[0] for th in theta])
    Vll = np.array([line_to_line_voltages(v) for v in Vabc])

    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    graus = np.degrees(theta)

    axs[0].step(graus, sectors, where="post", color="k")
    axs[0].set_ylabel("Setor")
    axs[0].set_yticks(range(1, 7))
    axs[0].set_title(f"Exemplo 4 - Six-step ideal (Vdc = {inv.Vdc} V)")

    for k, nome in enumerate("abc"):
        axs[1].plot(graus, Vabc[:, k], label=f"V{nome}")
    axs[1].set_ylabel("Fase [V]")
    axs[1].legend(loc="upper right", fontsize=8)

    for k, nome in enumerate(["ab", "bc", "ca"]):
        axs[2].plot(graus, Vll[:, k], label=f"V{nome}")
    axs[2].set_ylabel("Linha [V]")
    axs[2].legend(loc="upper right", fontsize=8)

    for duty in (1.0, 0.8, 0.6):
        Va = [six_step_commutation(inv, th, duty)[0][0] for th in theta]
        axs[3].plot(graus, Va, label=f"Va, duty = {duty}")
    axs[3].set_ylabel("Va [V]")
    axs[3].set_xlabel("Angulo eletrico [graus]")
    axs[3].legend(loc="upper right", fontsize=8)

    for ax in axs:
        ax.grid(True)
        ax.set_xticks(range(0, 361, 60))
    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------
#   EXEMPLO 5: SIX-STEP ACIONANDO UM MOTOR BLDC (RESPOSTA)
# ----------------------------------------------------------------------

def exemplo_5_six_step_bldc():
    print("\n=== Exemplo 5: six-step perna a perna + motor BLDC ===")

    inv = Inverter(Vdc=24.0)

    # parametros do params.py, com inercia reduzida para a partida ser
    # rapida (com J = 43.3e-3 a constante de tempo mecanica passa de 5 s)
    motor = BLDCMotor(R=0.161, L=0.052e-3, M=0.0, Ke=0.0255, Kt=0.0255,
                      J=1e-4, B=1.2e-6, P=4)

    Ts = 1.0 / 10000.0         # PWM de 10 kHz na fase "alta"
    sim = TimeSimulation(t0=0.0, tf=0.12, dt=1e-6)
    Tl = 0.05                  # torque de carga [N.m]

    # degrau de duty: 50 % ate 60 ms, depois onda quadrada plena
    def duty_ref(t):
        return 0.5 if t < 0.06 else 1.0

    n = int(round((sim.tf - sim.t0) / sim.dt))
    log = {c: np.zeros(n) for c in
           ("t", "Va", "Vb", "Vc", "ia", "ib", "ic", "ea", "Te", "w", "duty", "setor")}

    for k in range(n):
        t = sim.t0 + k * sim.dt
        duty = duty_ref(t)

        # posicao -> setor (alinhado a FCEM, como os sensores Hall)
        motor.theta_e = (motor.P * motor.theta_r) % TWO_PI
        setor = six_step_sector(motor.theta_e + SIX_STEP_THETA_OFFSET)
        gates = six_step_gates(setor, duty, inverter_carrier(Ts, t))

        Vabc = bldc_step_switched(gates, inv, motor, sim, Tl, True)

        log["t"][k] = t
        log["Va"][k], log["Vb"][k], log["Vc"][k] = Vabc
        log["ia"][k], log["ib"][k], log["ic"][k] = motor.iabc
        log["ea"][k] = motor.eabc[0]
        log["Te"][k] = motor.Te
        log["w"][k] = motor.omega_r
        log["duty"][k] = duty
        log["setor"][k] = setor

    rpm = log["w"] * 60.0 / TWO_PI
    i50 = np.searchsorted(log["t"], 0.0599)
    print(f"Velocidade com duty 0.5 (t = 60 ms) : {rpm[i50]:7.1f} rpm")
    print(f"Velocidade com duty 1.0 (t = 120 ms): {rpm[-1]:7.1f} rpm")
    print(f"Velocidade a vazio ideal Vdc/(2 Ke) : "
          f"{inv.Vdc / (2 * motor.Ke) * 60 / TWO_PI:7.1f} rpm")
    print(f"Pico de corrente na partida         : "
          f"{np.max(np.abs(log['ia'])):7.1f} A")

    t_ms = log["t"] * 1e3

    # --- 5a) resposta completa ---
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axs[0].plot(t_ms, log["duty"], "k")
    axs[0].set_ylabel("duty")
    axs[0].set_title(f"Exemplo 5a - Resposta do BLDC ao six-step "
                     f"(Vdc = {inv.Vdc} V, Tl = {Tl} N.m)")
    axs[1].plot(t_ms, rpm)
    axs[1].set_ylabel("Velocidade [rpm]")
    for k, nome in enumerate("abc"):
        axs[2].plot(t_ms, log[f"i{nome}"], lw=0.6, label=f"i{nome}")
    axs[2].set_ylabel("Corrente [A]")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[3].plot(t_ms, log["Te"], lw=0.6, label="Te")
    axs[3].axhline(Tl, color="k", ls="--", label="Tl")
    axs[3].set_ylabel("Torque [N.m]")
    axs[3].set_xlabel("Tempo [ms]")
    axs[3].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig.tight_layout()

    # --- 5b) zoom em um ciclo eletrico, ja em regime com duty = 1 ---
    w_e = motor.P * log["w"][-1]
    T_e = TWO_PI / w_e
    janela = log["t"] >= sim.tf - 1.5 * T_e

    fig2, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    tz = t_ms[janela]
    axs[0].step(tz, log["setor"][janela], where="post", color="k")
    axs[0].set_ylabel("Setor")
    axs[0].set_yticks(range(1, 7))
    axs[0].set_title(f"Exemplo 5b - Zoom em regime, duty = 1 "
                     f"(f_e = {w_e / TWO_PI:.0f} Hz)")
    axs[1].plot(tz, log["Va"][janela], label="Va (terminal - neutro)")
    axs[1].plot(tz, log["ea"][janela], "k--", label="ea (FCEM)")
    axs[1].set_ylabel("[V]")
    axs[1].legend(loc="upper right", fontsize=8)
    for k, nome in enumerate("abc"):
        axs[2].plot(tz, log[f"i{nome}"][janela], label=f"i{nome}")
    axs[2].set_ylabel("Corrente [A]")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[3].plot(tz, log["Te"][janela])
    axs[3].axhline(Tl, color="k", ls="--", label="Tl")
    axs[3].set_ylabel("Torque [N.m]")
    axs[3].set_xlabel("Tempo [ms]")
    axs[3].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig2.tight_layout()

    # --- 5c) zoom com duty = 0.5: PWM dentro do setor (H_PWM-L_ON) ---
    j50 = (log["t"] >= 0.0595) & (log["t"] < 0.0600)
    fig3, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    tz = t_ms[j50]
    for k, nome in enumerate("abc"):
        axs[0].plot(tz, log[f"V{nome}"][j50], lw=0.8, label=f"V{nome}")
    axs[0].set_ylabel("Tensao de fase [V]")
    axs[0].set_title("Exemplo 5c - Duty = 0.5: chave superior da fase alta "
                     "chaveando a 10 kHz")
    axs[0].legend(loc="upper right", fontsize=8)
    for k, nome in enumerate("abc"):
        axs[1].plot(tz, log[f"i{nome}"][j50], label=f"i{nome}")
    axs[1].set_ylabel("Corrente [A]")
    axs[1].set_xlabel("Tempo [ms]")
    axs[1].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig3.tight_layout()

    return fig, fig2, fig3


# ----------------------------------------------------------------------
#   EXEMPLO 6: BLDC ALIMENTADO POR FONTE IDEAL (SEM CHAVEAMENTO)
# ----------------------------------------------------------------------

def exemplo_6_bldc_fonte_ideal():
    """
    Mesmo ensaio do exemplo 5, mas sem PWM: em vez de chavear a fase alta
    a 10 kHz, o barramento e uma fonte CC ideal ajustavel de valor
    duty * Vdc, e as chaves so mudam nas comutacoes (duty = 1 no setor).
    E o equivalente medio do H_PWM-L_ON: a tensao aplicada tem o mesmo
    valor medio, mas sem o ripple de chaveamento nas correntes e no torque.
    """
    print("\n=== Exemplo 6: BLDC alimentado por fonte ideal (sem chaveamento) ===")

    Vdc = 24.0
    motor = BLDCMotor(R=0.161, L=0.052e-3, M=0.0, Ke=0.0255, Kt=0.0255,
                      J=1e-4, B=1.2e-6, P=4)
    sim = TimeSimulation(t0=0.0, tf=0.12, dt=1e-6)
    Tl = 0.05

    def duty_ref(t):
        return 0.5 if t < 0.06 else 1.0

    n = int(round((sim.tf - sim.t0) / sim.dt))
    log = {c: np.zeros(n) for c in
           ("t", "Va", "ia", "ib", "ic", "ea", "Te", "w", "Vfonte", "setor")}

    for k in range(n):
        t = sim.t0 + k * sim.dt

        # fonte CC ideal: a tensao do barramento e o proprio comando
        fonte = Inverter(Vdc=max(duty_ref(t) * Vdc, 1e-9))

        motor.theta_e = (motor.P * motor.theta_r) % TWO_PI
        setor = six_step_sector(motor.theta_e + SIX_STEP_THETA_OFFSET)
        gates = six_step_gates(setor, 1.0, 0.0)   # conducao plena, sem PWM

        Vabc = bldc_step_switched(gates, fonte, motor, sim, Tl, True)

        log["t"][k] = t
        log["Va"][k] = Vabc[0]
        log["ia"][k], log["ib"][k], log["ic"][k] = motor.iabc
        log["ea"][k] = motor.eabc[0]
        log["Te"][k] = motor.Te
        log["w"][k] = motor.omega_r
        log["Vfonte"][k] = fonte.Vdc
        log["setor"][k] = setor

    rpm = log["w"] * 60.0 / TWO_PI
    i50 = np.searchsorted(log["t"], 0.0599)
    print(f"Velocidade com fonte de 12 V (t = 60 ms) : {rpm[i50]:7.1f} rpm")
    print(f"Velocidade com fonte de 24 V (t = 120 ms): {rpm[-1]:7.1f} rpm")
    print(f"Pico de corrente na partida              : "
          f"{np.max(np.abs(log['ia'])):7.1f} A")

    t_ms = log["t"] * 1e3

    # --- 6a) resposta completa ---
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axs[0].plot(t_ms, log["Vfonte"], "k")
    axs[0].set_ylabel("Fonte [V]")
    axs[0].set_title(f"Exemplo 6a - BLDC com fonte ideal, sem chaveamento "
                     f"(Tl = {Tl} N.m)")
    axs[1].plot(t_ms, rpm)
    axs[1].set_ylabel("Velocidade [rpm]")
    for k, nome in enumerate("abc"):
        axs[2].plot(t_ms, log[f"i{nome}"], lw=0.6, label=f"i{nome}")
    axs[2].set_ylabel("Corrente [A]")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[3].plot(t_ms, log["Te"], lw=0.6)
    axs[3].axhline(Tl, color="k", ls="--", label="Tl")
    axs[3].set_ylabel("Torque [N.m]")
    axs[3].set_xlabel("Tempo [ms]")
    axs[3].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig.tight_layout()

    # --- 6b) zoom com fonte de 12 V: correntes sem ripple de PWM ---
    w_e = motor.P * log["w"][i50]
    janela = (log["t"] >= 0.06 - 1.5 * TWO_PI / w_e) & (log["t"] < 0.06)
    tz = t_ms[janela]

    fig2, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axs[0].step(tz, log["setor"][janela], where="post", color="k")
    axs[0].set_ylabel("Setor")
    axs[0].set_yticks(range(1, 7))
    axs[0].set_title("Exemplo 6b - Zoom em regime com fonte de 12 V")
    axs[1].plot(tz, log["Va"][janela], label="Va (terminal - neutro)")
    axs[1].plot(tz, log["ea"][janela], "k--", label="ea (FCEM)")
    axs[1].set_ylabel("[V]")
    axs[1].legend(loc="upper right", fontsize=8)
    for k, nome in enumerate("abc"):
        axs[2].plot(tz, log[f"i{nome}"][janela], label=f"i{nome}")
    axs[2].set_ylabel("Corrente [A]")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[3].plot(tz, log["Te"][janela])
    axs[3].axhline(Tl, color="k", ls="--", label="Tl")
    axs[3].set_ylabel("Torque [N.m]")
    axs[3].set_xlabel("Tempo [ms]")
    axs[3].legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True)
    fig2.tight_layout()

    return fig, fig2


# ----------------------------------------------------------------------
#   MAIN
# ----------------------------------------------------------------------

EXEMPLOS = {
    1: exemplo_1_criacao,
    2: exemplo_2_tensao_media,
    3: exemplo_3_chaveamento,
    4: exemplo_4_six_step_ideal,
    5: exemplo_5_six_step_bldc,
    6: exemplo_6_bldc_fonte_ideal,
}


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Exemplos de uso do modelo de inversor trifasico (inverter.py).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "exemplos disponiveis:\n"
            "  1  criacao do Inverter\n"
            "  2  tensao media por duty cycle (inverter_output_voltage)\n"
            "  3  chaveamento real via PWM (switching_output_voltage)\n"
            "  4  six-step ideal, sem carga (six_step_commutation)\n"
            "  5  six-step perna a perna acionando um BLDC (resposta do motor)\n"
            "  6  BLDC alimentado por fonte ideal, sem ruido de chaveamento\n"
            "\n"
            "uso:\n"
            "  python exemplos.py                 roda todos os exemplos\n"
            "  python exemplos.py 4 5             roda apenas os exemplos 4 e 5\n"
            "  python exemplos.py 5 --salvar img  salva os graficos em img/"
        ),
    )
    parser.add_argument(
        "exemplos", nargs="*", type=int, choices=sorted(EXEMPLOS),
        metavar="N", help="numero(s) dos exemplos a rodar (padrao: todos)",
    )
    parser.add_argument(
        "--salvar", metavar="PASTA", default=None,
        help="salva os graficos em PASTA em vez de abrir as janelas",
    )
    return parser


if __name__ == "__main__":
    args = build_arg_parser().parse_args()
    pasta_saida = args.salvar
    escolhidos = args.exemplos or list(EXEMPLOS)

    figuras = {}
    for n in escolhidos:
        figs = EXEMPLOS[n]()
        if figs is None:
            continue
        if not isinstance(figs, tuple):
            figs = (figs,)
        for j, fig in enumerate(figs):
            figuras[f"exemplo_{n}{'abc'[j] if len(figs) > 1 else ''}"] = fig

    if pasta_saida:
        os.makedirs(pasta_saida, exist_ok=True)
        for nome, fig in figuras.items():
            caminho = os.path.join(pasta_saida, f"inverter_{nome}.png")
            fig.savefig(caminho, dpi=120)
            print(f"Grafico salvo em {caminho}")
    else:
        plt.show()
