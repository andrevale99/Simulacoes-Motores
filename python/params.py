import argparse
import re
import sys


# ==========================================================
#   Defaults (usados se nao vierem nem do arquivo nem da CLI)
# ==========================================================
DEFAULTS = {
    # Selecao da planta simulada
    "motor": "bldc",

    # Parametros do motor
    "R": 0.161,
    "L": 0.052e-3,
    "M": 0.0,
    "Ke": 0.0255,
    "J": 43.3e-3,
    "B": 1.2e-6,
    "Tl": 0.0,
    "P": 4,
    "Kt": 0.0255,

    # SVPWM
    "Fsw": 10000.0,
    "PwmSamples": 175,

    # Tempo de simulacao
    "Ti": 0.0,
    "Tf": 0.5,
    "Dt": 1e-6,

    # Injecao de nova carga
    "Ttl": 0.3,
    "Tlnew": 0.2,

    # Barramento CC
    "Vdc": 24.0,

    # Controladores PID
    "KpOmega": 1066.91,
    "KiOmega": 134071.803,
    "KdOmega": 0.0,
    "KpId": 0.163,
    "KiId": 505.796,
    "KdId": 0.0,
    "KpIq": 0.163,
    "KiIq": 505.796,
    "KdIq": 0.0,

    # Referencia de velocidade
    "rpm": 20.0,

    # Partida em malha aberta
    "MalhaAberta": False,

    # Saida
    "filename": "closedloop_simulation.csv",
}

# Tipo esperado de cada chave (int, float, bool ou str)
TYPES = {
    "motor": str,
    "R": float, "L": float, "M": float, "Ke": float, "J": float, "B": float,
    "Tl": float, "P": int, "Kt": float,
    "Fsw": float, "PwmSamples": int,
    "Ti": float, "Tf": float, "Dt": float,
    "Ttl": float, "Tlnew": float,
    "Vdc": float,
    "KpOmega": float, "KiOmega": float, "KdOmega": float,
    "KpId": float, "KiId": float, "KdId": float,
    "KpIq": float, "KiIq": float, "KdIq": float,
    "rpm": float,
    "MalhaAberta": bool,
    "filename": str,
}

# Alias das chaves do arquivo de config -> nome interno usado no codigo
KEY_ALIASES = {
    "file": "filename",
}

TRUE_VALUES = {"true", "yes", "on", "1"}
FALSE_VALUES = {"false", "no", "off", "0"}


def parse_bool(value):
    v = str(value).strip().lower()
    if v in TRUE_VALUES:
        return True
    if v in FALSE_VALUES:
        return False
    raise ValueError(f"Valor booleano invalido: '{value}' "
                      f"(use true/false, yes/no, on/off, 1/0)")


def cast_value(key, raw_value):
    typ = TYPES.get(key, str)
    if typ is bool:
        return parse_bool(raw_value)
    try:
        return typ(raw_value)
    except ValueError:
        raise ValueError(f"Valor invalido para '{key}': '{raw_value}' "
                          f"(esperado {typ.__name__})")


def parse_config_file(path):
    """
    Le um arquivo CHAVE=VALOR (tambem aceita 'CHAVE VALOR').
    Linhas em branco ou iniciadas com '#' sao ignoradas.
    Comentarios apos o valor (precedidos por '#') tambem sao removidos.
    Retorna um dict {chave_interna: valor_convertido}.
    """
    result = {}

    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError as e:
        print(f"Erro ao abrir arquivo de configuracao '{path}': {e}",
              file=sys.stderr)
        sys.exit(1)

    for lineno, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        # remove comentario inline ("VALOR # comentario")
        line = re.split(r"(?<!\\)#", line, maxsplit=1)[0].strip()
        if not line:
            continue

        if "=" in line:
            key, _, value = line.partition("=")
        else:
            parts = line.split(None, 1)
            if len(parts) != 2:
                print(f"Aviso: linha {lineno} ignorada (formato invalido): "
                      f"'{raw_line.rstrip()}'", file=sys.stderr)
                continue
            key, value = parts

        key = key.strip()
        value = value.strip()

        key = KEY_ALIASES.get(key, key)

        if key not in DEFAULTS:
            print(f"Aviso: chave desconhecida '{key}' na linha {lineno} "
                  f"(ignorada)", file=sys.stderr)
            continue

        try:
            result[key] = cast_value(key, value)
        except ValueError as e:
            print(f"Erro na linha {lineno} do arquivo de configuracao: {e}",
                  file=sys.stderr)
            sys.exit(1)

    return result


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Simulacao closedloop BLDC/PMSM (FOC + SVPWM).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("-c", "--config", type=str, default=None,
                         help="Arquivo de parametros (CHAVE=VALOR)")

    parser.add_argument("-m", "--motor", type=str, default=None,
                         choices=["bldc", "dc"],
                         help="planta a ser simulada: bldc ou dc")

    # Motor
    g_motor = parser.add_argument_group("Parametros do motor")
    g_motor.add_argument("-R", type=float, default=None, help="Ohm - resistencia de armadura")
    g_motor.add_argument("-L", type=float, default=None, help="H - indutancia de magnetizacao")
    g_motor.add_argument("-M", type=float, default=None, help="H - indutancia mutua")
    g_motor.add_argument("--Ke", type=float, default=None, help="V/(rad/s) - constante eletrica")
    g_motor.add_argument("-J", type=float, default=None, help="kg.m^2 - momento de inercia")
    g_motor.add_argument("-B", type=float, default=None, help="coeficiente de amortecimento")
    g_motor.add_argument("--Tl", type=float, default=None, help="N.m - torque de carga inicial")
    g_motor.add_argument("-P", type=int, default=None, help="numero de pares de polos")
    g_motor.add_argument("--Kt", type=float, default=None, help="N.m/A - constante de torque")

    # SVPWM
    g_pwm = parser.add_argument_group("Parametros do SVPWM")
    g_pwm.add_argument("--Fsw", type=float, default=None, help="Hz - frequencia de chaveamento")
    g_pwm.add_argument("--pwm-samples", dest="PwmSamples", type=int, default=None,
                        help="passos finos de simulacao por periodo Ts")

    # Tempo
    g_time = parser.add_argument_group("Parametros de tempo")
    g_time.add_argument("--Ti", type=float, default=None, help="s - tempo inicial")
    g_time.add_argument("--Tf", type=float, default=None, help="s - tempo final")
    g_time.add_argument("-d", "--dt", dest="Dt", type=float, default=None,
                         help="s - passo de integracao explicito (0 = automatico)")

    # Injecao de carga
    g_load = parser.add_argument_group("Injecao de nova carga")
    g_load.add_argument("--Ttl", type=float, default=None, help="s - tempo de injecao da nova carga")
    g_load.add_argument("--Tlnew", type=float, default=None, help="N.m - torque da nova carga")

    # Barramento CC
    g_bus = parser.add_argument_group("Barramento CC")
    g_bus.add_argument("--Vdc", type=float, default=None, help="V - tensao do barramento CC")

    # PI
    g_pi = parser.add_argument_group("Controladores PID")
    g_pi.add_argument("--KpOmega", type=float, default=None, help="ganho proporcional - malha de velocidade")
    g_pi.add_argument("--KiOmega", type=float, default=None, help="ganho integral - malha de velocidade")
    g_pi.add_argument("--KdOmega", type=float, default=None, help="ganho derivativo - malha de velocidade")
    g_pi.add_argument("--KpId", type=float, default=None, help="ganho proporcional - malha de corrente id")
    g_pi.add_argument("--KiId", type=float, default=None, help="ganho integral - malha de corrente id")
    g_pi.add_argument("--KdId", type=float, default=None, help="ganho derivativo - malha de corrente id")
    g_pi.add_argument("--KpIq", type=float, default=None, help="ganho proporcional - malha de corrente iq")
    g_pi.add_argument("--KiIq", type=float, default=None, help="ganho integral - malha de corrente iq")
    g_pi.add_argument("--KdIq", type=float, default=None, help="ganho derivativo - malha de corrente iq")

    # Referencia
    g_ref = parser.add_argument_group("Referencia de velocidade")
    g_ref.add_argument("--rpm", type=float, default=None, help="rpm - velocidade de referencia")

    # Malha aberta
    g_open = parser.add_argument_group("Partida em malha aberta")
    g_open.add_argument("--malha-aberta", dest="MalhaAberta", type=str, default=None,
                         help="true/false, yes/no, on/off, 1/0 - "
                              "partida com fonte de alimentacao senoidal")

    # Saida
    g_out = parser.add_argument_group("Saida")
    g_out.add_argument("-o", "--file", dest="filename", type=str, default=None,
                        help="arquivo de log (.csv) de saida")

    return parser


def get_args(argv=None):
    """
    Resolve os parametros com prioridade:
    argumentos de linha de comando > arquivo de config (-c) > defaults.
    Retorna um argparse.Namespace pronto para uso na simulacao.
    """
    parser = build_arg_parser()
    cli_args = parser.parse_args(argv)

    values = dict(DEFAULTS)

    if cli_args.config:
        values.update(parse_config_file(cli_args.config))

    for key in DEFAULTS:
        cli_value = getattr(cli_args, key, None)
        if cli_value is not None:
            if key == "MalhaAberta" and isinstance(cli_value, str):
                cli_value = parse_bool(cli_value)
            values[key] = cli_value

    return argparse.Namespace(**values)


if __name__ == "__main__":
    args = get_args()
    for k, v in vars(args).items():
        print(f"{k} = {v}")