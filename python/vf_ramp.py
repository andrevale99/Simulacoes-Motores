"""
vf_ramp.py

Partida em malha aberta por rampa V/f (tensao/frequencia), em tres etapas:

  1. Alinhamento, passo 1: vetor de tensao fixo em align_angles[0]
     (padrao 90 graus eletricos), com amplitude u_align, durante t_align.
  2. Alinhamento, passo 2: vetor fixo em align_angles[1] (padrao 0 grau),
     mesma amplitude e duracao. Dois passos evitam que o rotor fique parado
     no ponto morto (180 graus do vetor, onde o torque e nulo). A rampa
     parte do angulo do passo 2.
  3. Rampa: a velocidade comandada parte de rpm_start e sobe a
     rpm_per_s ate rpm_end, e entao fica constante. O vetor gira na
     frequencia eletrica correspondente, com amplitude (em pu):

         u = min(u_offset + u_per_rpm * rpm_cmd, u_max)
         |v| = u * v_base

Nao ha realimentacao de posicao: o rotor e "arrastado" pelo campo girante.
Quando a velocidade MEDIDA do rotor atinge rpm_end, `finished` vai para True
e quem chamou deve passar para a malha fechada.
"""

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi


@dataclass
class VfRamp:
    """
    Parametros
    ----------
    P          : pares de polos
    v_base     : tensao correspondente a u = 1 [V] (ex.: Vdc/sqrt(3) no SVPWM)
    u_align    : amplitude do alinhamento [pu]
    t_align    : duracao de CADA um dos 2 passos do alinhamento [s]
    rpm_start  : velocidade no inicio da rampa [rpm mecanico]
    rpm_end    : velocidade final da rampa [rpm mecanico]
    rpm_per_s  : taxa da rampa [rpm/s]
    u_offset   : amplitude em 0 rpm [pu]
    u_per_rpm  : inclinacao V/f [pu/rpm]
    u_max      : limite da amplitude [pu]
    align_angles : angulos eletricos [rad] dos 2 passos do alinhamento
    """
    P: int
    v_base: float
    u_align: float
    t_align: float
    rpm_start: float
    rpm_end: float
    rpm_per_s: float
    u_offset: float
    u_per_rpm: float
    u_max: float
    align_angles: tuple = (math.pi / 2.0, 0.0)

    # estado
    theta_v: float = 0.0      # angulo eletrico do vetor de tensao [rad]
    rpm_cmd: float = 0.0      # velocidade comandada pela rampa [rpm]
    u: float = 0.0            # amplitude atual [pu]
    elapsed: float = 0.0      # tempo desde o inicio da partida [s]
    finished: bool = False    # True quando o rotor atingiu rpm_end

    def __post_init__(self):
        if self.rpm_end <= 0.0 or self.rpm_per_s <= 0.0:
            raise ValueError("rpm_end e rpm_per_s devem ser positivos.")

    @property
    def omega_r_final(self):
        """Velocidade mecanica final da rampa [rad/s]."""
        return self.rpm_end * TWO_PI / 60.0

    @property
    def f_final(self):
        """Frequencia eletrica final da rampa [Hz]."""
        return self.rpm_end / 60.0 * self.P

    def update(self, dt, omega_r):
        """
        Avanca a partida de dt segundos e retorna (v_alpha, v_beta).

        omega_r : velocidade mecanica medida [rad/s], usada so para decidir
                  o fim da partida (finished).
        """
        self.elapsed += dt
        t_ramp = self.elapsed - 2.0 * self.t_align

        if t_ramp < 0.0:
            # alinhamento: align_angles[0] no 1o passo, align_angles[1] no 2o
            self.rpm_cmd = 0.0
            self.u = self.u_align
            self.theta_v = self.align_angles[0 if self.elapsed < self.t_align else 1]
        else:
            self.rpm_cmd = min(self.rpm_start + self.rpm_per_s * t_ramp,
                               self.rpm_end)
            self.u = min(self.u_offset + self.u_per_rpm * self.rpm_cmd,
                         self.u_max)
            freq = self.rpm_cmd / 60.0 * self.P
            self.theta_v = (self.theta_v + TWO_PI * freq * dt) % TWO_PI

            if omega_r >= self.omega_r_final:
                self.finished = True

        v_mag = self.u * self.v_base
        return (v_mag * math.cos(self.theta_v),
                v_mag * math.sin(self.theta_v))

    def reset(self):
        self.theta_v = self.rpm_cmd = self.u = self.elapsed = 0.0
        self.finished = False
