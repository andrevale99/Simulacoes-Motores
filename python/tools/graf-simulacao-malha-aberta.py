"""
Grafico da simulacao do BLDC em uma unica figura (4 linhas, 1 coluna):

    linha 1: FCEM de cada fase (ea, eb, ec)
    linha 2: corrente de cada fase (ia, ib, ic)
    linha 3: torque eletromagnetico (Te)
    linha 4: RPM da simulacao e a referencia de velocidade

A referencia de velocidade e o numero contido no nome do arquivo csv (ex.:
simulacao-1150.csv -> 1150 RPM). Se o nome nao tiver numero, a linha de
referencia nao e desenhada.

Uso:
    python graf-simulacao-malha-aberta.py <simulacao.csv> <saida.pdf>

    ex.: python graf-simulacao-malha-aberta.py simulacao-1150.csv img/malha-aberta-1150.pdf
"""
import argparse
import os
import re

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

FIGSIZE = (15, 13)


def _rpm_alvo(arquivo):
    """
    Referencia de velocidade tirada do nome do arquivo: o ultimo numero que
    aparece no nome, sem a extensao (ex.: simulacao-1150.csv -> 1150).
    Retorna None se o nome nao tiver numero.
    """
    nome = os.path.splitext(os.path.basename(arquivo))[0]
    numeros = re.findall(r"\d+(?:\.\d+)?", nome)
    if not numeros:
        return None
    rpm = float(numeros[-1])
    return int(rpm) if rpm.is_integer() else rpm


def plot_simulacao(sim_csv, saida):
    data = pd.read_csv(sim_csv, sep=";")
    time = data["time"]
    rpm = data["omega_r"] * 60.0 / (2 * np.pi)
    rpm_alvo = _rpm_alvo(sim_csv)

    fig, (ax_e, ax_i, ax_te, ax_rpm) = plt.subplots(4, 1, figsize=FIGSIZE,
                                                    sharex=True)

    # Linha 1: FCEM de cada fase
    for f in "abc":
        ax_e.plot(time, data[f"e{f}"], lw=0.8, label=f"$e_{f}$")
    ax_e.set_ylabel("FCEM")
    ax_e.grid()
    ax_e.legend(loc="upper right")

    # Linha 2: corrente de cada fase
    for f in "abc":
        ax_i.plot(time, data[f"i{f}"], lw=0.8, label=f"$i_{f}$")
    ax_i.set_ylabel("A")
    ax_i.grid()
    ax_i.legend(loc="upper right")

    # Linha 3: torque eletromagnetico
    ax_te.plot(time, data["Te"], "k", lw=0.8, label="$T_e$")
    ax_te.set_ylabel("N.m")
    ax_te.grid()
    ax_te.legend(loc="upper right")

    # Linha 4: RPM da simulacao e referencia tirada do nome do arquivo
    ax_rpm.plot(time, rpm, "k", lw=1, 
                label="Simula\\c{c}\\~ao")
    if rpm_alvo is not None:
        ax_rpm.axhline(rpm_alvo, color="C0", lw=2, ls=":",
                       label=f"Refer\\^encia {rpm_alvo} RPM")
    else:
        print("aviso: nome do arquivo sem numero; referencia de RPM nao desenhada")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_xlabel("s")
    ax_rpm.grid()
    ax_rpm.legend(loc="lower right")

    fig.tight_layout()

    pasta = os.path.dirname(saida)
    if pasta:
        os.makedirs(pasta, exist_ok=True)
    fig.savefig(saida)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("simulacao", help="csv da simulacao (ex.: simulacao-1150.csv)")
    parser.add_argument("saida", help="arquivo pdf a ser salvo (inclui o diretorio)")
    args = parser.parse_args()

    if not os.path.isfile(args.simulacao):
        parser.error(f'arquivo "{args.simulacao}" nao encontrado')

    plot_simulacao(args.simulacao, args.saida)
