import sys
import numpy as np

def gains_controller(caminho):
    """Le arquivo CHAVE=VALOR (ou 'CHAVE VALOR'); ignora '#' e linhas em branco."""
    params = {}
    with open(caminho, "r", encoding="utf-8") as f:
        for linha in f:
            linha = linha.split("#", 1)[0].strip()
            if not linha:
                continue
            if "=" in linha:
                chave, _, valor = linha.partition("=")
            else:
                partes = linha.split(None, 1)
                if len(partes) != 2:
                    continue
                chave, valor = partes
            params[chave.strip()] = valor.strip()
    return params



if __name__ == "__main__":
    
    if len(sys.argv) != 2:
        sys.exit(f"Uso: python {sys.argv[0]} <arquivo_de_parametros.txt>")

    try:
        cfg = gains_controller(sys.argv[1])
    except OSError as e:
        sys.exit(f"Erro ao abrir '{sys.argv[1]}': {e}")

    try:
        R   = float(cfg["R"])
        L   = float(cfg["L"])
        M   = float(cfg.get("M", 0.0))
        Ke  = float(cfg["Ke"])
        J   = float(cfg["J"])
        B   = float(cfg.get("B", 0.0))
        Tl  = float(cfg.get("Tl", 0.0))
        P   = int(cfg.get("P", 1))
        Kt  = float(cfg["Kt"])
        Fsw = float(cfg["Fsw"])
    except KeyError as e:
        sys.exit(f"Parametro obrigatorio ausente no arquivo: {e}")
    except ValueError as e:
        sys.exit(f"Valor invalido no arquivo: {e}")


    omegacc = Fsw / 20 * 2 * np.pi
    Kpq = L * omegacc
    Kiq = R * omegacc

    Kpq = round(Kpq,3)
    Kiq = round(Kiq,3)

    omegacs = omegacc / 5 
    Kpomega = J * omegacs / Kt
    Kiomega = J * omegacs**2 / (5*Kt)

    Kpomega = round(Kpomega,3)
    Kiomega = round(Kiomega,3)

    print(f'KpOmega = {Kpomega}\t\t\t# ganho proporcional - malha de velocidade')
    print(f'KiOmega = {Kiomega}\t\t\t# ganho integral - malha de velocidade')
    print()
    print(f'KpId    = {Kpq}\t\t\t# ganho proporcional - malha de corrente id')
    print(f'KiId    = {Kiq}\t\t\t# ganho integral - malha de corrente id')
    print()
    print(f'KpIq    = {Kpq}\t\t\t# ganho proporcional - malha de corrente iq')
    print(f'KiIq    = {Kiq}\t\t\t# ganho integral - malha de corrente iq')
