"""
Compara 2 ou mais simulacoes do BLDC (ate 8; ex.: 3) em uma unica figura
(2 linhas, 1 coluna):

    linha 1: RPM de cada simulacao e a referencia: a rampa depois da troca
             malha aberta -> fechada (coluna omega_cmd) ate o maior RPM final
             e, em seguida, esse valor; se as simulacoes tiverem RPM finais
             diferentes, cada um dos outros vira uma linha horizontal (na cor
             da simulacao) a partir de onde a rampa passa por ele
    linha 2: corrente de uma das fases (--fase, padrao a) de cada simulacao

O RPM final de cada simulacao e o numero contido no nome do csv (ex.:
simulacao-1150.csv -> 1150 RPM). A rampa vem da simulacao com o maior RPM
final. Sem as colunas vf e omega_cmd, desenha so as linhas horizontais dos
RPM finais. Com --sem-ref, nao desenha nenhuma referencia (nem a rampa nem
as linhas dos RPM finais).

O fundo e vermelho claro na malha aberta e verde claro na malha fechada. A
troca e o fim da rampa V/f lido da coluna vf da primeira simulacao, ou o
tempo dado com --fundo T.

Com -i/-f, corta os dados nos instantes mais proximos dos tempos dados.
Com --nomes, troca os nomes das simulacoes na legenda (padrao: nome do csv);
um nome por simulacao, com --nomes depois do pdf de saida.

As labels (nomes das simulacoes, legendas e rotulos dos eixos) ficam em
variaveis no inicio do arquivo (NOMES_SIMULACOES, LABEL_*), para modificar.

Uso:
    python graf-comparacao-entre-simu.py <sim1.csv> <sim2.csv> [<sim3.csv> ...] <saida.pdf> [--fase {a,b,c}] [-i T] [-f T] [--fundo T] [--sem-ref] [--nomes N1 N2 ...]

    ex.: python graf-comparacao-entre-simu.py simulacao-mf-1-900.csv simulacao-mf-2-900.csv simulacao-mf-3-900.csv img/comp-mf-900.pdf --nomes "Sim 1" "Sim 2" "Sim 3"
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

# Cor de cada simulacao, na ordem (a mesma no RPM e na corrente); sem o
# vermelho (C3), usado na referencia
CORES = ("k", "C0", "C1", "C2", "C4", "C5", "C6", "C9")

# ------------------------------------------------------------------------
# Labels dos graficos (modifique aqui). Os textos vao para o LaTeX: escreva
# "_" como "\\_" fora de formulas e "^" como "\\^" (ex.: Refer\\^encia).
# Os campos entre chaves ({nome}, {fase}, {rpm}) sao preenchidos pelo
# script; as demais chaves ficam como estao (ex.: "$i_{a,ref}$").
# ------------------------------------------------------------------------

# Nome de cada simulacao na legenda, na ordem dos csv. Vazio ([]) usa o
# --nomes ou, sem ele, o nome do csv. O --nomes tem prioridade sobre esta lista.
# Exemplo, para 3 simulacoes:
#   NOMES_SIMULACOES = ["PI", "PID", "$K_p = 20$"]
NOMES_SIMULACOES = ["900",
                    "1150", 
                    "1700"]

# Legenda de cada curva de RPM; {nome} e o nome da simulacao.
# Exemplos (simulacao "PI"):
#   "{nome}"                  -> PI   (padrao)
#   "$\\omega_r$ -- {nome}"   -> omega_r -- PI
#   "RPM ({nome})"            -> RPM (PI)
LABEL_RPM = "{nome}"

# Legenda da linha de referencia; {rpm} e o RPM final.
# Exemplos (RPM final 900):
#   "Refer\\^encia {rpm} RPM"   -> Referencia 900 RPM   (padrao)
#   "$\\omega_{ref}$"           -> omega_ref
LABEL_REFERENCIA = "Refer\\^encia {rpm} RPM"

# Legenda de cada curva de corrente; {fase} e a fase (a, b ou c) e {nome}
# o nome da simulacao.
# Exemplos (--fase a, simulacao "PI"):
#   "$i_{fase}$ -- {nome}"                 -> i_a -- PI   (padrao)
#   "{nome}"                               -> PI
#   "Corrente $i_{fase}$ ({nome})"         -> Corrente i_a (PI)
#   "$i_{{fase},\\mathrm{sim}}$ -- {nome}"  -> i_{a,sim} -- PI
LABEL_CORRENTE = "$i_{fase}$ -- {nome}"

# Rotulos dos eixos. Exemplos: "Velocidade [RPM]", "Corrente [A]", "Tempo [s]"
LABEL_Y_RPM = "RPM"
LABEL_Y_CORRENTE = "A"
LABEL_X = "s"


def _preencher(texto, **campos):
    """Troca {campo} pelo valor, sem mexer nas outras chaves do texto (LaTeX)."""
    for chave, valor in campos.items():
        texto = texto.replace("{" + chave + "}", str(valor))
    return texto


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


def _rampa_ref(sim, rpm_alvo, tol=0.5):
    """
    Trecho (t_ini, t_fim) da rampa de referencia de velocidade depois da troca
    malha aberta -> fechada: comeca no primeiro instante com vf = 0 e termina
    quando omega_cmd chega a rpm_alvo (a menos de tol rpm); sem rpm_alvo, ou
    se nao chegar, vai ate o fim dos dados. Retorna None se o csv nao tiver as
    colunas vf e omega_cmd ou se nao houver troca.
    """
    if not {"vf", "omega_cmd"}.issubset(sim.columns):
        return None
    pos = sim[sim["vf"] == 0]
    if pos.empty:
        return None
    t_ini = pos["time"].iloc[0]
    t_fim = pos["time"].iloc[-1]
    if rpm_alvo is not None:
        cmd = pos["omega_cmd"] * 60.0 / (2 * np.pi)
        chegou = pos["time"][(cmd - rpm_alvo).abs() <= tol]
        if len(chegou) > 0:
            t_fim = chegou.iloc[0]
    return t_ini, t_fim


def _linha_referencia(sim, rpm_alvo):
    """
    Pontos (tempos, rpm) da linha unica de referencia: rampa da troca ate o
    RPM final e, dai, o RPM final ate o fim dos dados. Retorna None se nao
    houver rampa (sem as colunas vf/omega_cmd ou sem troca).
    """
    rampa = _rampa_ref(sim, rpm_alvo)
    if rampa is None:
        return None
    def cmd_rpm(t):
        return sim.loc[sim["time"] == t, "omega_cmd"].iloc[0] * 60.0 / (2 * np.pi)

    t_ini, t_fim = rampa
    t_ult = sim["time"].iloc[-1]
    rpm_final = rpm_alvo if rpm_alvo is not None else cmd_rpm(t_fim)
    return np.array([t_ini, t_fim, t_ult]), np.array([cmd_rpm(t_ini), rpm_final, rpm_final])


def _recortar_linha(tempos, valores, t_min, t_max):
    """Recorta a linha poligonal ao intervalo [t_min, t_max] (interpolando as pontas)."""
    t_a, t_b = max(t_min, tempos[0]), min(t_max, tempos[-1])
    if t_a >= t_b:
        return None
    meio = tempos[(tempos > t_a) & (tempos < t_b)]
    t = np.concatenate(([t_a], meio, [t_b]))
    return t, np.interp(t, tempos, valores)


def _cortar(df, col_t, t_inicial=None, t_final=None):
    """
    Mantem as linhas entre os instantes de `col_t` mais proximos de t_inicial
    e de t_final (fora do intervalo dos dados, fica o primeiro/ultimo instante
    disponivel). Limite None nao corta aquele lado.
    """
    ini = df.index[0] if t_inicial is None else (df[col_t] - t_inicial).abs().idxmin()
    fim = df.index[-1] if t_final is None else (df[col_t] - t_final).abs().idxmin()
    return df.loc[ini:fim]


def _latex(texto):
    """Escapa os caracteres especiais do LaTeX (o texto vai para a legenda)."""
    return re.sub(r"([_%&#$])", r"\\\1", texto)


def carregar_simulacao(arquivo, fase):
    """Le so as colunas usadas (os csv da simulacao sao grandes)."""
    colunas = pd.read_csv(arquivo, sep=";", nrows=0).columns
    usar = ["time", "omega_r", f"i{fase}"] + [c for c in ("vf", "omega_cmd") if c in colunas]
    df = pd.read_csv(arquivo, sep=";", usecols=usar)
    df["rpm"] = df["omega_r"] * 60.0 / (2 * np.pi)
    return df


def _inicio_na_rampa(rpm, referencia, t_padrao):
    """
    Instante em que a rampa de referencia passa por `rpm` (onde comeca a
    linha horizontal desse RPM final). Sem rampa, ou com `rpm` abaixo do
    inicio dela, retorna o inicio da rampa ou t_padrao.
    """
    if referencia is None:
        return t_padrao
    tempos, valores = referencia
    if valores[1] <= valores[0]:
        return tempos[0]
    return float(np.interp(rpm, valores[:2], tempos[:2]))


def plot_comparacao_simulacoes(sim_csvs, saida, fase="a", t_inicial=None,
                               t_final=None, fundo=None, nomes=None,
                               referencias=True):
    # Nomes na legenda: --nomes, senao NOMES_SIMULACOES, senao o nome do csv
    if nomes is None:
        nomes = NOMES_SIMULACOES or [_latex(os.path.splitext(os.path.basename(a))[0])
                                     for a in sim_csvs]
    sims = [carregar_simulacao(a, fase) for a in sim_csvs]

    # RPM final de cada simulacao; a rampa vai ate o maior e vem da primeira
    # simulacao que tem esse RPM final
    rpm_alvos = [_rpm_alvo(a) for a in sim_csvs]
    conhecidos = [r for r in rpm_alvos if r is not None]
    rpm_alvo = max(conhecidos) if conhecidos else None
    k_ref = rpm_alvos.index(rpm_alvo) if rpm_alvo is not None else 0

    # Outros RPM finais (diferentes do maior), na cor da primeira simulacao
    # que usa cada um
    outras = []
    for r, cor in zip(rpm_alvos, CORES):
        if r is not None and r != rpm_alvo and r not in [o[0] for o in outras]:
            outras.append((r, cor))

    # Troca malha aberta -> fechada e linha de referencia, antes do corte
    # (podem ficar fora de [t_inicial, t_final])
    t_lim = _fim_rampa(sims[0]) if fundo is None else fundo
    if fundo is None and t_lim is not None:
        for arq, sim in zip(sim_csvs[1:], sims[1:]):
            t_lim_k = _fim_rampa(sim)
            if t_lim_k is not None and abs(t_lim - t_lim_k) > 1e-3:
                print(f"aviso: a troca de {os.path.basename(arq)} acontece em {t_lim_k:.4f} s "
                      f"e a da primeira em {t_lim:.4f} s; o fundo usa a da primeira")
    referencia = _linha_referencia(sims[k_ref], rpm_alvo)

    sims = [_cortar(s, "time", t_inicial, t_final) for s in sims]
    t_ini = min(s["time"].min() for s in sims)
    t_fim = max(s["time"].max() for s in sims)

    fig, (ax_rpm, ax_i) = plt.subplots(2, 1, figsize=FIGSIZE, sharex=True)

    # Linha 1: RPM de cada simulacao, a rampa ate o maior RPM final e as
    # referencias dos outros RPM finais (nenhuma referencia com --sem-ref)
    for sim, nome, cor in zip(sims, nomes, CORES):
        ax_rpm.plot(sim["time"], sim["rpm"], color=cor, lw=1,
                    label=_preencher(LABEL_RPM, nome=nome))
    if referencias:
        label_ref = _preencher(LABEL_REFERENCIA, rpm=rpm_alvo)
        if referencia is not None:
            trecho = _recortar_linha(*referencia, t_ini, t_fim)
            if trecho is not None:
                ax_rpm.plot(*trecho, color="C3", lw=1.5, ls="--", label=label_ref)
        elif rpm_alvo is not None:
            print("aviso: csv sem as colunas vf/omega_cmd ou sem troca; "
                  "desenhadas so as linhas dos RPM finais")
            ax_rpm.axhline(rpm_alvo, color="C3", lw=1.5, ls="--", label=label_ref)
        else:
            print("aviso: nome do arquivo sem numero e sem rampa; referencia de RPM nao desenhada")
        for rpm, cor in outras:
            t_a = max(_inicio_na_rampa(rpm, referencia, t_ini), t_ini)
            if t_a < t_fim:
                ax_rpm.plot([t_a, t_fim], [rpm, rpm], color=cor, lw=1.5, ls=":",
                            label=_preencher(LABEL_REFERENCIA, rpm=rpm))
    ax_rpm.set_ylabel(LABEL_Y_RPM)
    ax_rpm.grid()
    ax_rpm.legend(loc="lower right")

    # Linha 2: corrente da fase escolhida em cada simulacao
    for sim, nome, cor in zip(sims, nomes, CORES):
        ax_i.plot(sim["time"], sim[f"i{fase}"], color=cor, lw=0.8,
                  label=_preencher(LABEL_CORRENTE, fase=fase, nome=nome))
    ax_i.set_ylabel(LABEL_Y_CORRENTE)
    ax_i.set_xlabel(LABEL_X)
    ax_i.grid()
    ax_i.legend(loc="upper right")

    # Fundo: vermelho na malha aberta (ate t_lim), verde na malha fechada
    if t_lim is not None:
        t_lim = max(t_ini, min(t_lim, t_fim))
        for ax in (ax_rpm, ax_i):
            _fundo(ax, t_ini, t_lim, t_fim)
    else:
        print("aviso: fim da rampa V/f nao encontrado no csv; fundo nao desenhado "
              "(use --fundo T)")

    fig.tight_layout()

    pasta = os.path.dirname(saida)
    if pasta:
        os.makedirs(pasta, exist_ok=True)
    fig.savefig(saida)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("simulacoes", nargs="+", metavar="simulacao",
                        help="csv de cada simulacao, 2 ou mais (a primeira define o "
                             "fundo; a de maior RPM final, a rampa)")
    parser.add_argument("saida", help="arquivo pdf a ser salvo (inclui o diretorio)")
    parser.add_argument("--fase", choices="abc", default="a",
                        help="fase da corrente desenhada (padrao: a)")
    parser.add_argument("-i", "--t-inicial", type=float, metavar="T",
                        help="tempo inicial dos graficos [s]; usa o instante mais proximo existente")
    parser.add_argument("-f", "--t-final", type=float, metavar="T",
                        help="tempo final dos graficos [s]; usa o instante mais proximo existente")
    parser.add_argument("--fundo", type=float, metavar="T",
                        help="instante [s] da troca malha aberta -> fechada no fundo "
                             "(padrao: fim da rampa V/f da primeira simulacao)")
    parser.add_argument("--sem-ref", action="store_true",
                        help="nao desenha nenhuma referencia de RPM (nem a rampa nem "
                             "as linhas dos RPM finais)")
    parser.add_argument("--nomes", nargs="+", metavar="N",
                        help="nomes das simulacoes na legenda, um por csv e na mesma "
                             "ordem (padrao: nome do csv)")
    args = parser.parse_args()

    if (args.t_inicial is not None and args.t_final is not None
            and args.t_inicial >= args.t_final):
        parser.error("-i deve ser menor que -f")

    if not 2 <= len(args.simulacoes) <= len(CORES):
        parser.error(f"informe de 2 a {len(CORES)} csv de simulacao "
                     f"(recebidos {len(args.simulacoes)})")
    if args.nomes is not None and len(args.nomes) != len(args.simulacoes):
        parser.error(f"--nomes tem {len(args.nomes)} nomes para "
                     f"{len(args.simulacoes)} simulacoes")
    if (args.nomes is None and NOMES_SIMULACOES
            and len(NOMES_SIMULACOES) != len(args.simulacoes)):
        parser.error(f"NOMES_SIMULACOES (no inicio do arquivo) tem {len(NOMES_SIMULACOES)} "
                     f"nomes para {len(args.simulacoes)} simulacoes")

    for arq in args.simulacoes:
        if not os.path.isfile(arq):
            parser.error(f'arquivo "{arq}" nao encontrado')

    plot_comparacao_simulacoes(args.simulacoes, args.saida, args.fase,
                               args.t_inicial, args.t_final, args.fundo, args.nomes,
                               not args.sem_ref)
