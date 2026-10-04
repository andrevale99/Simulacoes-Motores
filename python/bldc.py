import math
from dataclasses import dataclass, field
from auxs import rads_to_rpm, rpm_to_rads, TWO_PI

PHI_A = 0.0
PHI_B = -2*math.pi/3
PHI_C =  2*math.pi/3

@dataclass
class BLDCMotor:
    R: float; L: float; M: float
    Ke: float; Kt: float
    J: float; B: float
    P: int
    iabc: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    eabc: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    theta_e: float = 0.0
    theta_r: float = 0.0
    omega_e: float = 0.0
    omega_r: float = 0.0
    Te: float = 0.0

@dataclass
class TimeSimulation:
    t0: float; tf: float; dt: float

def trapezoidal_back_emf(theta):
    theta = math.fmod(theta, TWO_PI)
    if theta < 0.0:
        theta += TWO_PI
    if theta < math.pi/6.0:
        return 6.0*theta/math.pi
    elif theta < 5.0*math.pi/6.0:
        return 1.0
    elif theta < 7.0*math.pi/6.0:
        return -6.0*theta/math.pi + 6.0
    elif theta < 11.0*math.pi/6.0:
        return -1.0
    else:
        return 6.0*theta/math.pi - 12.0

def bldc_step(Vabc, motor, time, Tl, trapezoidal_back_emf_flag):
    fabc = [0.0, 0.0, 0.0]

    motor.theta_e = math.fmod(motor.P*motor.theta_r, TWO_PI)
    if motor.theta_e < 0.0:
        motor.theta_e += TWO_PI
    motor.omega_e = motor.P * motor.omega_r

    for i, phi in enumerate((PHI_A, PHI_B, PHI_C)):
        angle = motor.theta_e + phi
        fabc[i] = -trapezoidal_back_emf(angle) if trapezoidal_back_emf_flag else -math.sin(angle)
        motor.eabc[i] = motor.Ke * motor.omega_r * fabc[i]

    for i in range(3):
        diabc_i = (Vabc[i] - motor.R*motor.iabc[i] - motor.eabc[i]) / (motor.L + motor.M)
        motor.iabc[i] += diabc_i * time.dt

    motor.Te = motor.Kt * sum(motor.iabc[i]*fabc[i] for i in range(3))

    domega_r = (motor.Te - Tl - motor.omega_r*motor.B) / motor.J
    motor.omega_r += domega_r * time.dt
    motor.theta_r += motor.omega_r * time.dt