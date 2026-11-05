from simulations import (
    simulation_bldc_malha_corrente_velocidade,
    simulation_bldc_velocidade_vf,
    simulation_bldc_velocidade
)
from params import get_args

# Parametros exibidos antes da simulacao, na ordem e nos grupos de params.py
GRUPOS_PARAMETROS = [
    ("Planta", ["motor"]),
    ("Motor", ["R", "L", "M", "Ke", "Kt", "J", "B", "P", "Tl"]),
    ("Chaveamento", ["Fsw", "PwmSamples"]),
    ("Tempo", ["Tf", "Dt"]),
    ("Injecao de carga", ["Ttl", "Tlnew"]),
    ("Barramento CC", ["Vdc"]),
    ("Controladores PID", ["KpOmega", "KiOmega", "KdOmega",
                           "KpId", "KiId", "KdId",
                           "KpIq", "KiIq", "KdIq"]),
    ("Referencia", ["rpm", "rpmRef"]),
    ("Partida", ["MalhaAberta"]),
    ("Saida", ["filename"]),
]


def passo_integracao(args):
    """
    Passo de integracao usado pelas simulacoes: Dt se for > 0, senao
    1/Fsw / PwmSamples. Retorna (dt, origem).
    """
    if args.Dt > 0.0:
        return args.Dt, "informado (Dt)"
    return 1.0 / args.Fsw / args.PwmSamples, "calculado (1/Fsw / PwmSamples)"


def exibir_parametros(args, simulacao):
    print("=" * 60)
    print(f"  Simulacao: {simulacao.__name__}")
    print("=" * 60)

    for grupo, chaves in GRUPOS_PARAMETROS:
        print(f"\n[{grupo}]")
        for chave in chaves:
            valor = getattr(args, chave)
            if chave == "rpmRef":
                valor = ", ".join(f"{t:g}:{rpm:g}" for t, rpm in valor) or "(vazio: usa rpm)"
            print(f"  {chave:<11} = {valor}")

    Ts = 1.0 / args.Fsw
    dt, origem = passo_integracao(args)
    print("\n[Passo de integracao]")
    print(f"  Ts          = {Ts:.6e} s")
    print(f"  dt          = {dt:.6e} s  {origem}")
    print(f"  Ts/dt       = {Ts/dt:.2f} passos por periodo de chaveamento")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    args = get_args()

    # args.motor seleciona qual planta simular: "bldc" ou "dc".
    # Pode vir da CLI (-m/--motor), do arquivo de config (motor=dc)
    # ou do default em params.py ("bldc"). A CLI valida com
    # choices=["bldc", "dc"]; o arquivo de config nao tem essa
    # validacao, entao o else abaixo cobre um valor invalido vindo
    # so do .txt.
    motor_type = args.motor.lower()

    if motor_type == "dc":
        raise SystemExit("Erro: nao ha simulacao disponivel para o motor DC.")
    elif motor_type == "bldc":
        simulacao = simulation_bldc_malha_corrente_velocidade
        # simulacao = simulation_bldc_velocidade
        # simulacao = simulation_bldc_velocidade_vf
    else:
        raise SystemExit(
            f'Erro: tipo de motor desconhecido "{motor_type}" '
            f"(use --motor bldc ou --motor dc)."
        )

    exibir_parametros(args, simulacao)
    raise SystemExit(simulacao(args))
