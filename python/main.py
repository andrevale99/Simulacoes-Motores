from simulations import (
    simulation_bldc_malha_corrente_velocidade,
    simulation_dc_motor_malha_velocidade,
    simulation_dc_motor_malha_corrente_velocidade,
)
from params import get_args

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
        raise SystemExit(simulation_dc_motor_malha_corrente_velocidade(args))
    elif motor_type == "bldc":
        # raise SystemExit(simulation_bldc_malha_corrente_velocidade(args))
        raise SystemExit(simulation_dc_motor_malha_velocidade(args))
    else:
        raise SystemExit(
            f'Erro: tipo de motor desconhecido "{motor_type}" '
            f"(use --motor bldc ou --motor dc)."
        )