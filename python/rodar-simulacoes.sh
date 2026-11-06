#!/usr/bin/env bash
#
# Roda 3 simulacoes seguidas e gera um grafico de cada uma com
# tools/graf-simulacao-eabc-iabc-te-rpm.py. A simulacao N usa o N-esimo rpm
# lido e o N-esimo item de cada lista abaixo: ganhos (GANHOS, flags do
# main.py), tempo final (TFS) e nomes do csv e do grafico (NOMES_CSV e
# NOMES_IMG).
#
# A simulacao executada e a escolhida em main.py. Os demais parametros vem do
# arquivo de configuracao (CONFIG); o rpm, os ganhos e o tempo final
# sobrescrevem os dele.
#
# Saida, em SAIDA/:
#     <NOMES_CSV[N]>-<RPM>.csv   log da simulacao N
#     <NOMES_IMG[N]>-<RPM>.pdf   grafico da simulacao N
# O script acrescenta o RPM no fim dos nomes porque os graficos tiram a
# referencia de velocidade do ultimo numero do nome do csv.
#
# Uso (a partir de qualquer diretorio):
#     ./rodar-ganhos.sh 900 1150 1700     # um rpm por simulacao
#     ./rodar-ganhos.sh                   # sem argumentos, pergunta os rpm
#
# As variaveis abaixo podem ser trocadas pelo ambiente, por exemplo:
#     SAIDA=img/teste MOSTRAR=1 ./rodar-ganhos.sh 900 1150 1700
#     EXTRA="--Vdc 12" ./rodar-ganhos.sh 900 1150 1700   # flags extras para o main.py

set -euo pipefail

# Roda a partir de python/, onde ficam o main.py e o tools/
cd "$(dirname "$0")"

CONFIG="${CONFIG:-../motors/bldc/a2212.txt}"
SAIDA="${SAIDA:-test/simulacoes}"
GRAF_FLAGS="${GRAF_FLAGS:---fundo}"   # flags do grafico (ex.: "--fundo -i 2 -f 5")
EXTRA="${EXTRA:-}"
MOSTRAR="${MOSTRAR:-0}"               # 1 abre as janelas dos graficos

# Um conjunto de ganhos por simulacao (flags do main.py)
GANHOS=(
    "--KpOmega 30 --KiOmega 30 --KdOmega 2"
    "--KpOmega 10 --KiOmega 20 --KdOmega 1"
    "--KpOmega 5 --KiOmega 10  --KdOmega 0"
)

# Tempo final [s] de cada simulacao (--Tf do main.py), na mesma ordem de GANHOS
TFS=(
    9.5
    9.5
    9.5
)

# Nome do csv e do grafico (pdf) de cada simulacao, sem extensao e sem o rpm
# (o script acrescenta "-<rpm>.csv" e "-<rpm>.pdf"), na mesma ordem de GANHOS
NOMES_CSV=(
    simulacao-mf-1
    simulacao-mf-2
    simulacao-mf-3
)
NOMES_IMG=(
    grafico-mf-1
    grafico-mf-2
    grafico-mf-3
)

# Cada lista precisa de um item por simulacao (o mesmo tamanho de GANHOS)
conferir_tamanho() {
    local nome="$1" tamanho="$2"
    if [ "$tamanho" -ne "${#GANHOS[@]}" ]; then
        echo "Erro: $nome tem $tamanho itens e GANHOS tem ${#GANHOS[@]};" \
             "informe um por simulacao." >&2
        exit 1
    fi
}
conferir_tamanho TFS "${#TFS[@]}"
conferir_tamanho NOMES_CSV "${#NOMES_CSV[@]}"
conferir_tamanho NOMES_IMG "${#NOMES_IMG[@]}"

# rpm de cada simulacao: dos argumentos ou, sem argumentos, do terminal
if [ "$#" -gt 0 ]; then
    RPMS=("$@")
else
    read -r -p "rpm das ${#GANHOS[@]} simulacoes (separados por espaco): " -a RPMS
fi

if [ "${#RPMS[@]}" -ne "${#GANHOS[@]}" ]; then
    echo "Erro: informe ${#GANHOS[@]} valores de rpm (um por conjunto de ganhos)," \
         "recebidos ${#RPMS[@]}." >&2
    exit 1
fi
for rpm in "${RPMS[@]}"; do
    if ! [[ "$rpm" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "$rpm" =~ ^0+([.]0+)?$ ]]; then
        echo "Erro: rpm invalido '$rpm' (use um numero > 0, ex.: 900 ou 1150.5)." >&2
        exit 1
    fi
done

# Sem MOSTRAR, os graficos so sao salvos (nao abrem janela nem travam o script)
if [ "$MOSTRAR" != "1" ]; then
    export MPLBACKEND=Agg
    export PYTHONWARNINGS="ignore::UserWarning"
fi

mkdir -p "$SAIDA"

for i in "${!GANHOS[@]}"; do
    n=$((i + 1))
    rpm="${RPMS[$i]}"
    tf="${TFS[$i]}"
    csv="$SAIDA/${NOMES_CSV[$i]}-$rpm.csv"
    img="$SAIDA/${NOMES_IMG[$i]}-$rpm.pdf"

    echo "############################################################"
    echo "  Simulacao $n de ${#GANHOS[@]}: rpm = $rpm, Tf = $tf s, ${GANHOS[$i]}"
    echo "############################################################"

    # shellcheck disable=SC2086  # GANHOS, EXTRA e GRAF_FLAGS sao listas de flags
    python3 main.py -c "$CONFIG" --rpm "$rpm" --Tf "$tf" ${GANHOS[$i]} $EXTRA -o "$csv"
    # shellcheck disable=SC2086
    python3 tools/graf-simulacao-eabc-iabc-te-rpm.py "$csv" "$img" $GRAF_FLAGS

    echo "Grafico salvo em $img"
    echo
done

echo "Concluido: ${#GANHOS[@]} simulacoes em $SAIDA/"
