"""
dc_motor.py

Modelo de espaco de estados de um motor DC com escovas (motor de
corrente continua classico, com comutador mecanico), equivalente em
estrutura ao bldc.h/bldc.c, porem mais simples: uma unica malha
eletrica (nao ha comutacao trifasica nem FCEM multifasica).

Equacao eletrica (armadura):

    L * di/dt = V - R*i - Ke*omega_r

Equacao mecanica:

    J * domega_r/dt = Kt*i - Tl - B*omega_r

Equacao de posicao:

    dtheta_r/dt = omega_r

Para motor DC ideal com unidades consistentes (SI), Ke == Kt
(constante eletrica = constante de torque), mas a biblioteca mantem
os dois parametros separados para permitir modelar motores reais
onde pequenas diferencas aparecem por perdas/efeitos nao modelados.

Integracao: Euler explicito (mesmo metodo usado em bldc_step()).
"""

from dataclasses import dataclass, field
from auxs import rads_to_rpm, rpm_to_rads


# ----------------------------------------------------------------------
#   ESTRUTURA DO MOTOR
# ----------------------------------------------------------------------

@dataclass
class DCMotor:
    """
    Modelo de simulacao do motor DC.

    Armazena os parametros eletricos e mecanicos do motor, bem como
    as variaveis de estado utilizadas durante a simulacao.
    """

    R: float                  # Ohm       - resistencia de armadura
    L: float                  # H         - indutancia de armadura
    Ke: float                 # V/(rad/s) - constante eletrica (FCEM)
    Kt: float                 # N.m/A     - constante de torque
    J: float                  # kg.m^2    - momento de inercia do rotor
    B: float                  # N.m/(rad/s) - atrito viscoso

    ia: float = 0.0            # A   - corrente de armadura
    Te: float = 0.0            # N.m - torque eletromagnetico
    e: float = 0.0             # V   - forca contraeletromotriz

    theta_r: float = 0.0        # rad   - posicao angular do rotor
    omega_r: float = 0.0        # rad/s - velocidade angular do rotor

    log_file: object = field(default=None, repr=False)





# ----------------------------------------------------------------------
#   LOG
# ----------------------------------------------------------------------

def dc_motor_log_header(filename):
    """Cria o arquivo de log e escreve o cabecalho dos dados."""
    f = open(filename, "w")
    f.write("time;V;ia;e;Te;omega_r;theta_r\n")
    return f


def dc_motor_log_data(motor, log_file, t, V):
    """Registra o estado atual do motor no arquivo de log."""
    log_file.write(
        f"{t:.6f};{V:.4f};{motor.ia:.6f};{motor.e:.6f};"
        f"{motor.Te:.6f};{motor.omega_r:.6f};{motor.theta_r:.6f}\n"
    )


# ----------------------------------------------------------------------
#   PASSO DE SIMULACAO
# ----------------------------------------------------------------------

def dc_motor_step(V, motor, dt, Tl):
    """
    Executa um passo de integracao do modelo do motor DC.

    A funcao realiza, nesta ordem:

      1. Calculo da FCEM:                 e = Ke * omega_r
      2. Calculo da derivada da corrente:  di/dt = (V - R*i - e) / L
      3. Integracao da corrente:           i += di/dt * dt
      4. Calculo do torque eletromagnetico: Te = Kt * i
      5. Calculo da aceleracao angular:    domega/dt = (Te - Tl - B*omega) / J
      6. Atualizacao da velocidade mecanica
      7. Atualizacao da posicao mecanica

    Parametros
    ----------
    V : float
        Tensao aplicada a armadura [V].
    motor : DCMotor
        Modelo do motor DC (estado atualizado in-place).
    dt : float
        Passo de integracao [s].
    Tl : float
        Torque de carga aplicado ao eixo [N.m].
    """
    motor.e = motor.Ke * motor.omega_r

    dia = (V - motor.R * motor.ia - motor.e) / motor.L
    motor.ia += dia * dt

    motor.Te = motor.Kt * motor.ia

    domega_r = (motor.Te - Tl - motor.B * motor.omega_r) / motor.J
    motor.omega_r += domega_r * dt
    motor.theta_r += motor.omega_r * dt


# ----------------------------------------------------------------------
#   AUTOTESTE (roda so quando o arquivo e executado diretamente)
# ----------------------------------------------------------------------

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Parametros de um motor DC pequeno, tipicos (valores ilustrativos)
    motor = DCMotor(
        R=1.0,         # Ohm
        L=0.5e-3,      # H
        Ke=0.02,       # V/(rad/s)
        Kt=0.02,       # N.m/A
        J=1.0e-5,      # kg.m^2
        B=1.0e-6,      # N.m/(rad/s)
    )

    Vdc = 12.0       # degrau de tensao aplicado em t = 0
    Tl = 0.0          # sem carga
    t0, tf, dt = 0.0, 0.05, 1e-6

    n = int((tf - t0) / dt)
    time = [0.0] * n
    ia_hist = [0.0] * n
    omega_hist = [0.0] * n
    Te_hist = [0.0] * n

    t = t0
    for k in range(n):
        dc_motor_step(Vdc, motor, dt, Tl)

        time[k] = t
        ia_hist[k] = motor.ia
        omega_hist[k] = rads_to_rpm(motor.omega_r)
        Te_hist[k] = motor.Te

        t += dt

    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    axs[0].plot(time, ia_hist)
    axs[0].set_ylabel("Corrente [A]")
    axs[0].set_title("Resposta ao degrau de tensao (motor DC)")
    axs[0].grid(True)

    axs[1].plot(time, omega_hist)
    axs[1].set_ylabel("Velocidade [rpm]")
    axs[1].grid(True)

    axs[2].plot(time, Te_hist)
    axs[2].set_ylabel("Torque [N.m]")
    axs[2].set_xlabel("Tempo [s]")
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig("dc_motor_autoteste.png", dpi=120)

    print(f"Autoteste concluido. ia final = {motor.ia:.4f} A, "
          f"omega final = {rads_to_rpm(motor.omega_r):.2f} rpm")
    print("Grafico salvo em dc_motor_autoteste.png")