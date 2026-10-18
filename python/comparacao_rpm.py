"""
Compara, em um unico grafico, a velocidade (RPM) medida em um ensaio do
firmware (testes/log-*.csv) com a velocidade da simulacao.

Uso:
    python comparacao_rpm.py <teste> [simulacao.csv]

    <teste> pode ser o caminho do log ou so o RPM alvo (ex.: 500 ->
    testes/log-500.csv).
"""
import argparse
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from graficos import DEFAULT_PASTA, DEFAULT_FIGSIZE  # aplica tambem o rcParams


DEFAULT_SIM_CSV = "teste-six-step.csv"
DEFAULT_TESTES = "testes/"


def _rpm_alvo(arquivo):
    m = re.search(r"log-(\d+)\.csv$", arquivo)
    return int(m.group(1)) if m else 0


def _arquivo_teste(teste):
    if os.path.isfile(teste):
        return teste
    return os.path.join(DEFAULT_TESTES, f"log-{teste}.csv")


def plot_comparacao_rpm(teste_csv, sim_csv=DEFAULT_SIM_CSV):
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
    plt.savefig(DEFAULT_PASTA + f"05_comparacao_rpm-{rpm_alvo}.pdf")
    plt.show()


if __name__ == "__main__":
    testes = sorted((f for f in os.listdir(DEFAULT_TESTES) if f.startswith("log-")),
                    key=_rpm_alvo)

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("teste", help="log do ensaio ou RPM alvo "
                        f"(disponiveis: {', '.join(str(_rpm_alvo(f)) for f in testes)})")
    parser.add_argument("simulacao", nargs="?", default=DEFAULT_SIM_CSV,
                        help=f"csv da simulacao (padrao: {DEFAULT_SIM_CSV})")
    args = parser.parse_args()

    teste_csv = _arquivo_teste(args.teste)
    if not os.path.isfile(teste_csv):
        parser.error(f'arquivo de teste "{teste_csv}" nao encontrado')

    plot_comparacao_rpm(teste_csv, args.simulacao)
