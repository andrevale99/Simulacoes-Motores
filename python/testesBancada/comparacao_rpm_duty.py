"""
Compara o ensaio de bancada (log do firmware) com a simulacao:

    linha 1: RPM do ensaio e da simulacao no mesmo grafico
    linha 2: comandos do controlador
             coluna 1: duty cycle de cada fase do ensaio (pwm / periodo)
             coluna 2: duty cycle de cada fase da simulacao (da, db, dc);
                       em csv antigos, sem essas colunas, usa duty_ref

Uso:
    python comparacao_rpm_duty.py <teste.csv> <simulacao.csv> <saida.pdf>

    ex.: python comparacao_rpm_duty.py teste04/log-1150.csv simulacao-1150.csv teste04/comparacao-1150.pdf
"""
import argparse
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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

FIGSIZE = (15, 10)

# Periodo do PWM do firmware em contagens (CCR = 625 -> 50% de duty)
PWM_PERIODO = 1250

# Colunas do log do firmware (o cabecalho pode faltar se o cat comecou
# depois do reset da placa)
COLUNAS_TESTE = ["t_us", "pwm_a", "pwm_b", "pwm_c", "setor", "pos",
                 "pos_unwrap", "vel_rpm_x10"]

# A simulacao grava a cada passo (~10 us); para o grafico basta 1 ponto a
# cada PASSO_SIM linhas
PASSO_SIM = 10


def _rpm_alvo(arquivo):
    """
    Referencia de velocidade tirada do nome do arquivo: o que vem depois do
    ultimo '-' e antes da extensao (ex.: log-900.csv -> 900). Retorna None
    se nao houver um numero nessa posicao.
    """
    nome = os.path.splitext(os.path.basename(arquivo))[0]
    _, sep, valor = nome.rpartition("-")
    if not sep:
        return None
    try:
        rpm = float(valor)
    except ValueError:
        return None
    return int(rpm) if rpm.is_integer() else rpm


def carregar_teste(arquivo):
    # Linhas com '#' sao mensagens do firmware
    df = pd.read_csv(arquivo, comment="#", header=None, names=COLUNAS_TESTE,
                     on_bad_lines="skip", dtype=str)
    # Com cabecalho no arquivo, usa so o que veio depois do ultimo
    cabecalhos = df.index[df["t_us"] == "t_us"]
    if len(cabecalhos) > 0:
        df = df.loc[cabecalhos[-1] + 1:]

    # Linhas cortadas no inicio/fim viram NaN e sao descartadas
    df = df.apply(pd.to_numeric, errors="coerce").dropna().reset_index(drop=True)

    df["t"] = (df["t_us"] - df["t_us"].iloc[0]) * 1e-6
    df["rpm"] = df["vel_rpm_x10"] / 10.0
    for fase in "abc":
        df[f"duty_{fase}"] = df[f"pwm_{fase}"] / PWM_PERIODO
    return df


def carregar_simulacao(arquivo):
    colunas = pd.read_csv(arquivo, sep=";", nrows=0).columns
    usar = ["time", "omega_r", "duty_ref"] + [c for c in ("da", "db", "dc") if c in colunas]
    df = pd.read_csv(arquivo, sep=";", usecols=usar)
    df = df.iloc[::PASSO_SIM].reset_index(drop=True)
    df["rpm"] = df["omega_r"] * 60.0 / (2 * np.pi)
    return df


def plot_comparacao(teste_csv, sim_csv, saida):
    teste = carregar_teste(teste_csv)
    sim = carregar_simulacao(sim_csv)
    rpm_alvo = _rpm_alvo(teste_csv)

    fig = plt.figure(figsize=FIGSIZE)
    gs = fig.add_gridspec(2, 2)
    ax_rpm = fig.add_subplot(gs[0, :])
    ax_teste = fig.add_subplot(gs[1, 0])
    ax_sim = fig.add_subplot(gs[1, 1], sharey=ax_teste)

    # Linha 1: RPM
    ax_rpm.plot(teste["t"], teste["rpm"], lw=0.8, color="red",
                label="Ensaio" if rpm_alvo is None else f"Ensaio {rpm_alvo} RPM")
    ax_rpm.plot(sim["time"], sim["rpm"], "k", lw=1, ls="--",
                label="Simula\\c{c}\\~ao")
    if rpm_alvo is not None:
        ax_rpm.axhline(rpm_alvo, color="C0", lw=1.2, ls=":",
                       label=f"Refer\\^encia {rpm_alvo} RPM")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_xlabel("Tempo [s]")
    ax_rpm.grid()
    ax_rpm.legend()

    # Linha 2, coluna 1: duty de cada fase do ensaio
    for fase, cor in zip("abc", ("C0", "C1", "C2")):
        ax_teste.plot(teste["t"], teste[f"duty_{fase}"], lw=0.8, color=cor,
                      label=f"$d_{fase}$")
    ax_teste.set_title("Ensaio")
    ax_teste.set_ylabel("Duty cycle")
    ax_teste.set_xlabel("Tempo [s]")
    ax_teste.grid()
    ax_teste.legend(loc="upper right")

    # Linha 2, coluna 2: duty de cada fase da simulacao (ou duty_ref)
    if {"da", "db", "dc"} <= set(sim.columns):
        for fase, cor in zip("abc", ("C0", "C1", "C2")):
            ax_sim.plot(sim["time"], sim[f"d{fase}"], lw=0.8, color=cor,
                        label=f"$d_{fase}$")
    else:
        ax_sim.plot(sim["time"], sim["duty_ref"], "k", lw=0.8,
                    label="$d_{ref}$")
    ax_sim.set_title("Simula\\c{c}\\~ao")
    ax_sim.set_xlabel("Tempo [s]")
    ax_sim.grid()
    ax_sim.legend(loc="upper right")

    fig.tight_layout()

    pasta = os.path.dirname(saida)
    if pasta:
        os.makedirs(pasta, exist_ok=True)
    fig.savefig(saida)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("teste", help="csv do log do ensaio (ex.: teste04/log-1150.csv)")
    parser.add_argument("simulacao", help="csv da simulacao (ex.: simulacao-1150.csv)")
    parser.add_argument("saida", help="arquivo pdf a ser salvo (inclui o diretorio)")
    args = parser.parse_args()

    for arq in (args.teste, args.simulacao):
        if not os.path.isfile(arq):
            parser.error(f'arquivo "{arq}" nao encontrado')

    plot_comparacao(args.teste, args.simulacao, args.saida)
