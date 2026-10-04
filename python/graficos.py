import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_CSV = "closedloop_simulation.csv"
DEFAULT_PASTA = "img/"
DEFAULT_FIGSIZE = (15, 10)


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["cmr10", "DejaVu Serif", "serif"],
    "axes.formatter.use_mathtext": True,

    # Fontes
    "font.size": 18,
    "axes.titlesize": 18,
    "axes.labelsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 18,
    "figure.titlesize": 22,

    # Espessura dos eixos
    "axes.linewidth": 1.2,

    # Tamanho dos ticks
    "xtick.major.size": 6,
    "ytick.major.size": 6,
    "xtick.major.width": 1.2,
    "ytick.major.width": 1.2,
})


# arq = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
# pasta_saida = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PASTA


# ============================================================
# AUXILIARES
# ============================================================

def no_xlabel():
    plt.tick_params(axis="x", labelbottom=False)


# ============================================================
# Função FFT
# ============================================================

def calcular_fft(sinal, Fs):
    N = len(sinal)

    # Remove componente DC
    sinal = sinal - np.mean(sinal)

    # Janela de Hann
    janela = np.hanning(N)

    sinal = sinal * janela

    # FFT
    fft = np.fft.rfft(sinal)

    # Correção da amplitude
    amplitude = 2 * np.abs(fft) / np.sum(janela)

    frequencia = np.fft.rfftfreq(N, d=1 / Fs)

    return frequencia, amplitude


# ============================================================
# Gráficos da FFT
# ============================================================

def plot_fft(simulation_file_csv=None):

    if simulation_file_csv is None:
        print("Sem arquivo de dados")
        return -1

    arquivo = simulation_file_csv

    df = pd.read_csv(arquivo, sep=";")

    # Nome da coluna de tempo
    tempo = df["time"].values

    # Correntes trifásicas
    ia = df["ia"].values
    ib = df["ib"].values
    ic = df["ic"].values

    # ========================================================
    # Frequência de amostragem
    # ========================================================

    Ts = np.mean(np.diff(tempo))
    Fs = 1 / Ts
    f_max = 50000

    print(f"Fs = {Fs / 1e6:.2f} MHz")
    print(f"Nyquist = {Fs / 2:.1f} kHz")
    print(f"Resolução = {Fs / len(ia):.2f} Hz")

    # ========================================================
    # FFT das três correntes
    # ========================================================

    f_ia, A_ia = calcular_fft(ia, Fs)
    f_ib, A_ib = calcular_fft(ib, Fs)
    f_ic, A_ic = calcular_fft(ic, Fs)

    # ========================================================
    # Limita até f_max
    # ========================================================

    idx = f_ia <= f_max

    # ========================================================
    # Plota
    # ========================================================

    plt.figure(figsize=(12, 7))

    plt.plot(f_ia[idx], A_ia[idx], label="Ia")
    plt.plot(f_ib[idx], A_ib[idx], label="Ib")
    plt.plot(f_ic[idx], A_ic[idx], label="Ic")

    plt.title("FFT das Correntes Trifásicas")
    plt.xlabel("Frequência (Hz)")
    plt.ylabel("Amplitude (A)")
    plt.xlim(0, f_max)
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig(DEFAULT_PASTA + "fft_correntes_tabc.pdf")


# ============================================================
# FUNÇÃO PRINCIPAL DE PLOTAGEM
# ============================================================

def plot_graficos(simulation_file_csv=None, motor_type=None):
    """
    Plota os graficos da simulacao.

    motor_type: "bldc", "dc" ou None.
        Se None, o tipo e inferido automaticamente a partir das
        colunas presentes no CSV (presenca de "ib"/"ic" -> bldc,
        ausencia -> dc). Passar explicitamente e recomendado quando
        a simulacao chama esta funcao, para nao depender de heuristica.
    """

    if simulation_file_csv is None:
        print("Sem arquivo CSV da simulacao")
        return -1

    arq = simulation_file_csv
    pasta_saida = DEFAULT_PASTA

    # ========================================================
    # Leitura dos dados
    # ========================================================

    data = pd.read_csv(arq, sep=";")

    if motor_type is None:
        motor_type = "bldc" if {"ib", "ic"}.issubset(data.columns) else "dc"
    motor_type = motor_type.lower()

    if motor_type == "dc":
        _plot_graficos_dc(data, pasta_saida)
    else:
        _plot_graficos_bldc(data, pasta_saida)
        plot_fft(simulation_file_csv)

    plt.show()


# ============================================================
# Graficos - motor BLDC
# ============================================================

def _plot_graficos_bldc(data, pasta_saida):
    str_time = "time"
    str_va = "Va"
    str_vb = "Vb"
    str_vc = "Vc"
    str_ia = "ia"
    str_ib = "ib"
    str_ic = "ic"
    str_ea = "ea"
    str_eb = "eb"
    str_ec = "ec"
    str_id = "id"
    str_iq = "iq"
    str_te = "Te"
    str_thetar = "theta_r"
    str_omegar = "omega_r"
    str_iqref = "iq_ref"
    str_vdref = "vd_ref"
    str_vqref = "vq_ref"

    time = data[str_time]

    vabc = np.array([
        data[str_va],
        data[str_vb],
        data[str_vc]
    ])

    iabc = np.array([
        data[str_ia],
        data[str_ib],
        data[str_ic]
    ])

    eabc = np.array([
        data[str_ea],
        data[str_eb],
        data[str_ec]
    ])

    iq = data[str_iq]
    _id = data[str_id]
    te = data[str_te]
    omegar = data[str_omegar]

    rpm = omegar * 60.0 / (2 * np.pi)

    try:

        iqref = data[str_iqref]
        vdref = data[str_vdref]
        vqref = data[str_vqref]

        # ====================================================
        # Tempo x iabc, rpm
        # ====================================================

        plt.figure(figsize=DEFAULT_FIGSIZE)

        plt.subplot(211)
        plt.plot(time, iabc.T)
        plt.ylabel("A")
        plt.grid()
        no_xlabel()

        plt.subplot(212)
        plt.plot(time, rpm)
        plt.grid()
        plt.ylabel("RPM")
        plt.xlabel("s")

        plt.tight_layout()
        plt.savefig(pasta_saida + "01_corrente-rpm.pdf")

        # ====================================================
        # Tempo x iq, id, iqref, Te
        # ====================================================

        plt.figure(figsize=DEFAULT_FIGSIZE)

        plt.subplot(211)
        plt.plot(time, _id, label=r"$i_{d}$")
        plt.plot(time, iq, label=r"$i_{q}$")
        plt.plot(time, iqref, label=r"$i_{qref}$", ls="--")
        plt.ylabel("A")
        plt.grid()
        plt.legend()
        no_xlabel()

        plt.subplot(212)
        plt.plot(time, te)
        plt.grid()
        plt.ylabel("Nm")
        plt.xlabel("s")

        plt.tight_layout()
        plt.savefig(pasta_saida + "02_iq-id-Te.pdf")

    except Exception:

        # ====================================================
        # Tempo x iabc, rpm (sem iq/id/iqref -- malha aberta, por ex.)
        # ====================================================

        plt.figure(figsize=DEFAULT_FIGSIZE)

        plt.subplot(211)
        plt.plot(time, iabc.T)
        plt.ylabel("A")
        plt.grid()
        no_xlabel()

        plt.subplot(212)
        plt.plot(time, rpm)
        plt.grid()
        plt.ylabel("RPM")
        plt.xlabel("s")

        plt.tight_layout()
        plt.savefig(pasta_saida + "01_corrente-rpm.pdf")


# ============================================================
# Graficos - motor DC
# ============================================================

def _plot_graficos_dc(data, pasta_saida):
    """
    Graficos para o motor DC: torque, corrente, velocidade e posicao.
    Espera as colunas geradas por dc_motor_log_data() /
    simulation_dc_motor_malha_corrente_velocidade():
        time;V;ia;e;Te;omega_r;theta_r;ia_ref
    """
    str_time = "time"
    str_ia = "ia"
    str_te = "Te"
    str_omegar = "omega_r"
    str_thetar = "theta_r"
    str_iaref = "ia_ref"

    time = data[str_time]
    ia = data[str_ia]
    te = data[str_te]
    omegar = data[str_omegar]
    thetar = data[str_thetar]

    rpm = omegar * 60.0 / (2 * np.pi)

    # ========================================================
    # Tempo x corrente, torque
    # ========================================================

    plt.figure(figsize=DEFAULT_FIGSIZE)

    plt.subplot(211)
    plt.plot(time, ia, label=r"$i_a$")
    if str_iaref in data.columns:
        plt.plot(time, data[str_iaref], label=r"$i_{a,ref}$", ls="--")
        plt.legend()
    plt.ylabel("A")
    plt.grid()
    no_xlabel()

    plt.subplot(212)
    plt.plot(time, te)
    plt.grid()
    plt.ylabel("Nm")
    plt.xlabel("s")

    plt.tight_layout()
    plt.savefig(pasta_saida + "01_dc_corrente-torque.pdf")

    # ========================================================
    # Tempo x velocidade, posicao
    # ========================================================

    plt.figure(figsize=DEFAULT_FIGSIZE)

    plt.subplot(211)
    plt.plot(time, rpm)
    plt.ylabel("RPM")
    plt.grid()
    no_xlabel()

    plt.subplot(212)
    plt.plot(time, thetar)
    plt.grid()
    plt.ylabel("rad")
    plt.xlabel("s")

    plt.tight_layout()
    plt.savefig(pasta_saida + "02_dc_velocidade-posicao.pdf")