#!/usr/bin/env bash
#
# Compara ensaios de bancada (logs do firmware, como
# test/stm32/teste01/log-1700.csv) com simulacoes, gerando um grafico de
# tools/graf-comparacao-simu-exp.py para cada par: RPM do ensaio e da
# simulacao e os duty cycles de cada um.
#
# Os arquivos ficam nas 9 variaveis abaixo (ENSAIO1..3, SIMULACAO1..3 e
# IMAGEM1..3). As comparacoes rodam na ordem 1, 2, 3; a comparacao N usa
# ENSAION, SIMULACAON e salva o grafico em IMAGEMN. Deixe as tres variaveis
# de uma comparacao vazias para pula-la. Os caminhos relativos partem da
# pasta python/.
#
# A referencia de RPM sai do nome do log do ensaio (log-1700.csv -> 1700).
#
# Uso (a partir de qualquer diretorio):
#     ./comparar-ensaio.sh
#
# As variaveis de GRAF_FLAGS e MOSTRAR podem ser trocadas pelo ambiente:
#     MOSTRAR=1 ./comparar-ensaio.sh                 # abre a janela de cada grafico
#     GRAF_FLAGS="--fase a -i 2 -f 6" ./comparar-ensaio.sh
# As flags aceitas sao as do graf-comparacao-simu-exp.py (--open-loop, --fase,
# -i, -f, --sem-referencia, --fundo); veja todas com:
#     python3 tools/graf-comparacao-simu-exp.py -h

set -euo pipefail

# Caminhos relativos a partir de python/, onde fica o tools/
cd "$(dirname "$0")"

GRAF_FLAGS="${GRAF_FLAGS:--i 0 -f 8.5}"   # flags dos graficos (ex.: "--fase a")
MOSTRAR="${MOSTRAR:-1}"        # 1 abre a janela de cada grafico

# ---------------- Comparacao 1 ----------------
ENSAIO1="test/stm32/backup/log-900.csv"
SIMULACAO1="test/simulacoes/simulacao-mf-900.csv"
IMAGEM1="img/comparacao-900.pdf"

# ---------------- Comparacao 2 ----------------
ENSAIO2="test/stm32/backup/log-1150.csv"
SIMULACAO2="test/simulacoes/simulacao-mf-1150.csv"
IMAGEM2="img/comparacao-1150.pdf"

# ---------------- Comparacao 3 ----------------
ENSAIO3="test/stm32/backup/log-1700.csv"
SIMULACAO3="test/simulacoes/simulacao-mf-1700.csv"
IMAGEM3="img/comparacao-1700.pdf"

ENSAIOS=("$ENSAIO1" "$ENSAIO2" "$ENSAIO3")
SIMULACOES=("$SIMULACAO1" "$SIMULACAO2" "$SIMULACAO3")
IMAGENS=("$IMAGEM1" "$IMAGEM2" "$IMAGEM3")

# Confere tudo antes de comecar: cada comparacao tem as 3 variaveis
# preenchidas (ou as 3 vazias) e os csv existem
for i in "${!ENSAIOS[@]}"; do
    n=$((i + 1))
    preenchidas=0
    for v in "${ENSAIOS[$i]}" "${SIMULACOES[$i]}" "${IMAGENS[$i]}"; do
        [ -n "$v" ] && preenchidas=$((preenchidas + 1))
    done
    if [ "$preenchidas" -ne 0 ] && [ "$preenchidas" -ne 3 ]; then
        echo "Erro: preencha ENSAIO$n, SIMULACAO$n e IMAGEM$n (ou deixe as 3 vazias)." >&2
        exit 1
    fi
    for arq in "${ENSAIOS[$i]}" "${SIMULACOES[$i]}"; do
        if [ -n "$arq" ] && [ ! -f "$arq" ]; then
            echo "Erro: arquivo '$arq' da comparacao $n nao encontrado." >&2
            exit 1
        fi
    done
done

# Sem MOSTRAR, os graficos so sao salvos (nao abrem janela nem travam o script)
if [ "$MOSTRAR" != "1" ]; then
    export MPLBACKEND=Agg
    export PYTHONWARNINGS="ignore::UserWarning"
fi

feitas=0
for i in "${!ENSAIOS[@]}"; do
    n=$((i + 1))
    if [ -z "${ENSAIOS[$i]}" ]; then
        echo "Comparacao $n: vazia, pulada."
        echo
        continue
    fi

    echo "############################################################"
    echo "  Comparacao $n de ${#ENSAIOS[@]}"
    echo "    Ensaio:    ${ENSAIOS[$i]}"
    echo "    Simulacao: ${SIMULACOES[$i]}"
    echo "    Imagem:    ${IMAGENS[$i]}"
    echo "############################################################"

    # shellcheck disable=SC2086  # GRAF_FLAGS e uma lista de flags
    python3 tools/graf-comparacao-simu-exp.py "${ENSAIOS[$i]}" "${SIMULACOES[$i]}" \
        "${IMAGENS[$i]}" $GRAF_FLAGS

    echo "Grafico salvo em ${IMAGENS[$i]}"
    echo
    feitas=$((feitas + 1))
done

echo "Concluido: $feitas comparacoes."
