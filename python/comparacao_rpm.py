"""
Compara, em um unico grafico, a velocidade (RPM) medida em um ensaio do
firmware (testes/log-*.csv) com a velocidade da simulacao.

Uso:
    python comparacao_rpm.py <teste.csv> <simulacao.csv> <saida.pdf>

    ex.: python comparacao_rpm.py testes/log-500.csv teste-six-step.csv figuras/comparacao.pdf
"""
import argparse
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from graficos import DEFAULT_FIGSIZE  # aplica tambem o rcParams


def _rpm_alvo(arquivo):
    m = re.search(r"log-(\d+)\.csv$", arquivo)
    return int(m.group(1)) if m else 0


def plot_comparacao_rpm(teste_csv, sim_csv, saida):
    plt.figure(figsize=DEFAULT_FIGSIZE)

    # Ensaio: linhas com '#' sao mensagens do firmware
    log = pd.read_csv(teste_csv, comment="#")
    t = log["t_us"] * 1e-6
    rpm = log["vel_rpm_x10"] / 10.0
    rpm_alvo = _rpm_alvo(teste_csv)
    plt.plot(t, rpm, lw=0.8, color='red',label=f"Ensaio {rpm_alvo} RPM")

    # Simulacao
    sim = pd.read_csv(sim_csv, sep=";")
    rpm_sim = sim["omega_r"] * 60.0 / (2 * np.pi)
    plt.plot(sim["time"], rpm_sim, "k", lw=1, ls='--', label="Simula\\c{c}\\~ao")

    plt.ylabel("RPM")
    plt.xlabel("s")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.savefig(saida)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("teste", help="csv do log do ensaio (ex.: testes/log-500.csv)")
    parser.add_argument("simulacao", help="csv da simulacao (ex.: teste-six-step.csv)")
    parser.add_argument("saida", help="arquivo da figura a ser salva (inclui o diretorio)")
    args = parser.parse_args()

    for arq in (args.teste, args.simulacao):
        if not os.path.isfile(arq):
            parser.error(f'arquivo "{arq}" nao encontrado')

    plot_comparacao_rpm(args.teste, args.simulacao, args.saida)
