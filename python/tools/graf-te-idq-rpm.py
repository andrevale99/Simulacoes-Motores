"""
Grafico da simulacao FOC do BLDC em uma unica figura (3 linhas, 1 coluna):

    linha 1: RPM da simulacao e a referencia de velocidade
    linha 2: correntes id e iq e suas referencias (id_ref = 0, iq_ref)
    linha 3: torque eletromagnetico (Te)

Espera o csv de simulation_bldc_malha_corrente_velocidade (colunas id, iq e
iq_ref). A simulacao usa id_ref = 0 fixo, que nao vai para o csv.

A referencia de velocidade e o numero contido no nome do arquivo csv (ex.:
simulacao-1150.csv -> 1150 RPM). Se o nome nao tiver numero, a linha de
referencia nao e desenhada.

Com -i/-f, corta os dados nos instantes mais proximos dos tempos dados.

Com --fundo, pinta o fundo de vermelho claro na malha aberta e de verde claro
na malha fechada. A troca e o tempo dado (--fundo 2.47) ou, sem valor, o fim
da rampa V/f lido da coluna vf do csv.

Com --sem-ref, nao desenha as referencias id_ref e iq_ref na linha 2.

Uso:
    python graf-te-idq-rpm.py <simulacao.csv> <saida.pdf> [-i T] [-f T] [--fundo [T]] [--sem-ref]

    ex.: python graf-te-idq-rpm.py simulacao-1150.csv img/te-idq-rpm-1150.pdf
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
    "font.serif": ["cmr10"],
    "axes.formatter.use_mathtext": True,
    'pdf.compression': 9,
    'path.simplify': True,
    'path.simplify_threshold' : 0.8,

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

# Referencia de id da simulacao FOC (fixa, nao vai para o csv)
ID_REF = 0.0


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


def _fim_rampa(sim):
    """
    Instante em que a simulacao sai da rampa V/f (coluna vf passa de 1 para
    0). Retorna None se o csv nao tiver a coluna ou se a rampa nao terminar.
    """
    if "vf" not in sim.columns:
        return None
    fim = sim.index[sim["vf"] == 0]
    return sim["time"].iloc[fim[0]] if len(fim) > 0 else None


def _fundo(ax, t_ini, t_lim, t_fim):
    """Vermelho claro de t_ini ate t_lim e verde claro de t_lim ate t_fim."""
    ax.axvspan(t_ini, t_lim, color="red", alpha=0.12, lw=0, zorder=0)
    ax.axvspan(t_lim, t_fim, color="green", alpha=0.12, lw=0, zorder=0)
    ax.set_xlim(t_ini, t_fim)


def _cortar(df, col_t, t_inicial=None, t_final=None):
    """
    Mantem as linhas entre os instantes de `col_t` mais proximos de t_inicial
    e de t_final (fora do intervalo dos dados, fica o primeiro/ultimo instante
    disponivel). Limite None nao corta aquele lado.
    """
    ini = df.index[0] if t_inicial is None else (df[col_t] - t_inicial).abs().idxmin()
    fim = df.index[-1] if t_final is None else (df[col_t] - t_final).abs().idxmin()
    return df.loc[ini:fim]


def plot_te_idq_rpm(sim_csv, saida, t_inicial=None, t_final=None, fundo=None,
                    sem_ref=False):
    data = pd.read_csv(sim_csv, sep=";")
    # Instante da troca malha aberta -> fechada, antes do corte (pode ficar
    # fora de [t_inicial, t_final]); fundo None nao pinta
    t_lim = _fim_rampa(data) if fundo == "auto" else fundo
    data = _cortar(data, "time", t_inicial, t_final)
    time = data["time"]
    rpm = data["omega_r"] * 60.0 / (2 * np.pi)
    rpm_alvo = _rpm_alvo(sim_csv)

    fig, (ax_rpm, ax_i, ax_te) = plt.subplots(3, 1, figsize=FIGSIZE, sharex=True)

    # Linha 1: RPM da simulacao e referencia tirada do nome do arquivo
    ax_rpm.plot(time, rpm, "k", lw=1, label="Simula\\c{c}\\~ao")
    if rpm_alvo is not None:
        ax_rpm.axhline(rpm_alvo, color="C0", lw=2, ls=":",
                       label=f"Refer\\^encia {rpm_alvo} RPM")
    else:
        print("aviso: nome do arquivo sem numero; referencia de RPM nao desenhada")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.grid()
    ax_rpm.legend(loc="lower right")

    # Linha 2: correntes id e iq e suas referencias (cada par com a mesma cor)
    ax_i.plot(time, data["id"], color="C0", lw=0.8, label="$i_d$")
    ax_i.plot(time, data["iq"], color="C1", lw=0.8, label="$i_q$")
    if not sem_ref:
        ax_i.axhline(ID_REF, color="red", lw=1.5, ls="--", label="$i_{d,ref}$")
        ax_i.plot(time, data["iq_ref"], color="black", lw=1.5, ls="--",
                  label="$i_{q,ref}$")
    ax_i.set_ylabel("A")
    ax_i.grid()
    ax_i.legend(loc="upper right", ncol=2)

    # Linha 3: torque eletromagnetico
    ax_te.plot(time, data["Te"], "k", lw=0.8, label="$T_e$")
    ax_te.set_ylabel("N.m")
    ax_te.set_xlabel("s")
    ax_te.grid()
    ax_te.legend(loc="upper right")

    # Fundo: vermelho na malha aberta (ate t_lim), verde na malha fechada
    if t_lim is not None:
        t_ini, t_fim = time.min(), time.max()
        t_lim = max(t_ini, min(t_lim, t_fim))
        for ax in (ax_rpm, ax_i, ax_te):
            _fundo(ax, t_ini, t_lim, t_fim)
    elif fundo == "auto":
        print("aviso: fim da rampa V/f nao encontrado no csv; fundo nao desenhado")

    fig.tight_layout()

    pasta = os.path.dirname(saida)
    if pasta:
        os.makedirs(pasta, exist_ok=True)
    fig.savefig(saida)
    plt.show()


def _fundo_arg(valor):
    """Tipo do argumento --fundo: "auto" (sem valor) ou um tempo em segundos."""
    if valor == "auto":
        return valor
    try:
        return float(valor)
    except ValueError:
        raise argparse.ArgumentTypeError(f"tempo invalido: '{valor}'")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("simulacao", help="csv da simulacao (ex.: simulacao-1150.csv)")
    parser.add_argument("saida", help="arquivo pdf a ser salvo (inclui o diretorio)")
    parser.add_argument("-i", "--t-inicial", type=float, metavar="T",
                        help="tempo inicial dos graficos [s]; usa o instante mais proximo existente")
    parser.add_argument("-f", "--t-final", type=float, metavar="T",
                        help="tempo final dos graficos [s]; usa o instante mais proximo existente")
    parser.add_argument("--fundo", nargs="?", const="auto", type=_fundo_arg, metavar="T",
                        help="pinta o fundo de vermelho claro (malha aberta) ate T [s] e de "
                             "verde claro (malha fechada) depois; sem T, usa o fim da rampa "
                             "V/f (coluna vf do csv)")
    parser.add_argument("--sem-ref", action="store_true",
                        help="nao desenha as referencias id_ref e iq_ref")
    args = parser.parse_args()

    if (args.t_inicial is not None and args.t_final is not None
            and args.t_inicial >= args.t_final):
        parser.error("-i deve ser menor que -f")

    if not os.path.isfile(args.simulacao):
        parser.error(f'arquivo "{args.simulacao}" nao encontrado')

    plot_te_idq_rpm(args.simulacao, args.saida, args.t_inicial, args.t_final,
                    args.fundo, args.sem_ref)
