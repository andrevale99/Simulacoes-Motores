"""
FFT das correntes de fase (ia, ib, ic) da simulacao do BLDC, em um unico
grafico. O eixo x vai de 0 ate a metade da frequencia de Nyquist (Fs/4).

Antes da FFT, cada corrente tem a media (componente DC) removida e passa
por uma janela de Hann; a amplitude e corrigida pelo ganho da janela, de
modo que um seno de amplitude A aparece com pico ~A.

Com -i/-f, corta os dados nos instantes mais proximos dos tempos dados.

Uso:
    python graf-ffts-iabc.py <simulacao.csv> <saida.pdf> [-i T] [-f T]

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
    "font.serif": ["cmr10"],
    "axes.formatter.use_mathtext": True,
    'pdf.compression': 9,
    'path.simplify': True,
    'path.simplify_threshold' : 0.6,

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


def _cortar(df, col_t, t_inicial=None, t_final=None):
    """
    Mantem as linhas entre os instantes de `col_t` mais proximos de t_inicial
    e de t_final (fora do intervalo dos dados, fica o primeiro/ultimo instante
    disponivel). Limite None nao corta aquele lado.
    """
    ini = df.index[0] if t_inicial is None else (df[col_t] - t_inicial).abs().idxmin()
    fim = df.index[-1] if t_final is None else (df[col_t] - t_final).abs().idxmin()
    return df.loc[ini:fim]


def plot_fft_iabc(sim_csv, saida, t_inicial=None, t_final=None):
    data = pd.read_csv(sim_csv, sep=";")
    data = _cortar(data, "time", t_inicial, t_final)
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
    parser.add_argument("-i", "--t-inicial", type=float, metavar="T",
                        help="tempo inicial dos graficos [s]; usa o instante mais proximo existente")
    parser.add_argument("-f", "--t-final", type=float, metavar="T",
                        help="tempo final dos graficos [s]; usa o instante mais proximo existente")
    args = parser.parse_args()

    if (args.t_inicial is not None and args.t_final is not None
            and args.t_inicial >= args.t_final):
        parser.error("-i deve ser menor que -f")

    if not os.path.isfile(args.simulacao):
        parser.error(f'arquivo "{args.simulacao}" nao encontrado')

    plot_fft_iabc(args.simulacao, args.saida, args.t_inicial, args.t_final)
