"""
FFT das correntes de fase (ia, ib, ic) da simulacao do BLDC, em um unico
grafico. O eixo x vai de 0 ate a metade da frequencia de Nyquist (Fs/4).

Antes da FFT, cada corrente tem a media (componente DC) removida e passa
por uma janela de Hann; a amplitude e corrigida pelo ganho da janela, de
modo que um seno de amplitude A aparece com pico ~A.

Uso:
    python graf-ffts-iabc.py <simulacao.csv> <saida.pdf>

    ex.: python graf-ffts-iabc.py simulacao-1150.csv img/fft-iabc-1150.pdf
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


def calcular_fft(sinal, Fs):
    """
    Espectro de amplitude de um lado do sinal amostrado a Fs [Hz].
    Retorna (frequencia [Hz], amplitude).
    """
    sinal = sinal - np.mean(sinal)
    janela = np.hanning(len(sinal))

    fft = np.fft.rfft(sinal * janela)
    amplitude = 2 * np.abs(fft) / np.sum(janela)
    frequencia = np.fft.rfftfreq(len(sinal), d=1 / Fs)

    return frequencia, amplitude


def plot_fft_iabc(sim_csv, saida):
    data = pd.read_csv(sim_csv, sep=";")
    tempo = data["time"].to_numpy()

    # A simulacao usa passo fixo: Fs = 1/dt
    Fs = 1 / np.mean(np.diff(tempo))
    f_nyquist = Fs / 2
    f_max = f_nyquist / 2

    print(f"Fs          = {Fs:.1f} Hz")
    print(f"Nyquist     = {f_nyquist:.1f} Hz")
    print(f"Limite em x = {f_max:.1f} Hz")
    print(f"Resolucao   = {Fs / len(tempo):.3f} Hz")

    fig, ax = plt.subplots(figsize=FIGSIZE)

    for f in "abc":
        freq, amp = calcular_fft(data[f"i{f}"].to_numpy(), Fs)
        idx = freq <= f_max
        ax.plot(freq[idx], amp[idx], lw=0.8, label=f"$i_{f}$")

    ax.set_xlim(0, f_max)
    ax.set_ylabel("A")
    ax.set_xlabel("Hz")
    ax.grid()
    ax.legend(loc="upper right")

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

    plot_fft_iabc(args.simulacao, args.saida)
