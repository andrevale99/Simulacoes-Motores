import math
from dataclasses import dataclass

from params import get_args
from graficos import plot_graficos
from pid_controller import pid_controller_init, pid_controller_update
from svpwm import svpwm_init, svpwm_modulate, svpwm_carrier, svpwm_gate_state
from inverter import Inverter, inverter_output_voltage
from transforms import clarke_transform, park_transform, park_inverse_transform
from bldc import BLDCMotor, TimeSimulation, bldc_step
from dc_motor import DCMotor, dc_motor_step
from auxs import rpm_to_rads
from progressbar import progress_bar_init, progress_bar_update, progress_bar_finish

# ajustar conforme os limites reais do projeto original (nao enviados)
PI_IQ_MIN, PI_IQ_MAX = -50.0, 50.0

def simulation_bldc_malha_corrente_velocidade(args):
    motor = BLDCMotor(
        R=args.R, L=args.L, M=args.M, Ke=args.Ke,
        J=args.J, B=args.B, P=args.P, Kt=args.Kt,
    )

    pwm = svpwm_init(args.Fsw, 0.0, args.Vdc)
    if pwm is None:
        print("Erro: falha ao inicializar o SVPWM (verifique Fsw e Vdc).")
        return 1

    if args.Dt > 0.0:
        dt = args.Dt
        if dt > pwm.Ts/2.0:
            print(f"Aviso: Dt={dt:.6e} s e maior que metade do periodo de "
                  f"chaveamento (Ts={pwm.Ts:.6e} s).")
    else:
        dt = pwm.Ts / args.PwmSamples

    time_sim = TimeSimulation(t0=args.Ti, tf=args.Tf, dt=dt)
    total_steps = int((time_sim.tf - time_sim.t0)/dt + 0.5) + 1

    print(f"Periodo de chaveamento (Ts) = {pwm.Ts:.9e} s")
    print(f"Passo de integracao (dt)    = {dt:.9e} s")
    print(f"Total de passos da simulacao ~ {total_steps}\n")

    if total_steps > 5_000_000:
        print(f"Aviso: {total_steps} passos -- a simulacao pode demorar bastante.")

    inverter = Inverter(Vdc=args.Vdc)

    vdc_max = args.Vdc / math.sqrt(3)
    vdc_min = -vdc_max

    dtOmega, dtId, dtIq = 1e-3, pwm.Ts, pwm.Ts
    id_ref = 0.0

    omega_ref = rpm_to_rads(args.rpm)
    print(f"rpm_ref = {args.rpm:.2f} RPM")
    print(f"omega_ref = {omega_ref:.6f} rad/s")

    pi_omega = pid_controller_init(args.KpOmega, args.KiOmega, 0.0, dtOmega, True, PI_IQ_MIN, True, PI_IQ_MAX)
    pi_d = pid_controller_init(args.KpId, args.KiId, 0.0, dtId, True, vdc_min, True, vdc_max)
    pi_q = pid_controller_init(args.KpIq, args.KiIq, 0.0, dtIq, True, vdc_min, True, vdc_max)

    t_next_omega = t_next_id = t_next_iq = time_sim.t0
    iq_ref_hold = vd_ref_hold = vq_ref_hold = 0.0

    try:
        log_file = open(args.filename, "w")
    except OSError as e:
        print(f"Erro ao criar o arquivo de log: {e}")
        return 1

    log_file.write("time;Va;Vb;Vc;ia;ib;ic;ea;eb;ec;id;iq;Te;theta_r;"
                    "omega_r;iq_ref;vd_ref;vq_ref\n")

    pb = progress_bar_init(time_sim.t0, time_sim.tf, time_sim.dt)

    for k in range(total_steps):
        t = time_sim.t0 + k*dt
        if t > time_sim.tf:
            break

        if t >= args.Ttl:
            args.Tl = args.Tlnew

        # FOC em malha fechada
        motor.theta_e = motor.theta_r * motor.P

        if t >= t_next_omega:
            iq_ref_hold = pid_controller_update(pi_omega, omega_ref, motor.omega_r)
            t_next_omega += dtOmega
        iq_ref = iq_ref_hold

        i_alpha, i_beta = clarke_transform(*motor.iabc)
        i_d, i_q = park_transform(i_alpha, i_beta, motor.theta_e)

        if t >= t_next_id:
            vd_ref_hold = pid_controller_update(pi_d, id_ref, i_d)
            t_next_id += dtId
        vd_ref = vd_ref_hold

        if t >= t_next_iq:
            vq_ref_hold = pid_controller_update(pi_q, iq_ref, i_q)
            t_next_iq += dtIq
        vq_ref = vq_ref_hold

        v_alpha, v_beta = park_inverse_transform(vd_ref, vq_ref, motor.theta_e)

        duty_a, duty_b, duty_c = svpwm_modulate(pwm, v_alpha, v_beta)

        carrier = svpwm_carrier(pwm, t)
        gate_a = svpwm_gate_state(duty_a, carrier)
        gate_b = svpwm_gate_state(duty_b, carrier)
        gate_c = svpwm_gate_state(duty_c, carrier)

        Vabc = inverter_output_voltage(inverter, gate_a, gate_b, gate_c)

        bldc_step(Vabc, motor, time_sim, args.Tl, True)

        progress_bar_update(pb, t)

        log_file.write(
            f"{t:.6f};{Vabc[0]:.3f};{Vabc[1]:.3f};{Vabc[2]:.3f};"
            f"{motor.iabc[0]:.4f};{motor.iabc[1]:.4f};{motor.iabc[2]:.4f};"
            f"{motor.eabc[0]:.4f};{motor.eabc[1]:.4f};{motor.eabc[2]:.4f};"
            f"{i_d:.4f};{i_q:.4f};{motor.Te:.4f};{motor.theta_r:.3f};"
            f"{motor.omega_r:.4f};{iq_ref:.4f};{vd_ref:.4f};{vq_ref:.4f}\n"
        )

    progress_bar_finish(pb)
    log_file.close()
    print(f"\n\nSimulacao concluida. Resultados em \"{args.filename}\".\n")

    print(f'\nPlot dos graficos de iabc, Te, idq, RPM e fft das correntes\n\n')
    plot_graficos(args.filename, motor_type="bldc")

    return 0


def simulation_dc_motor_malha_corrente_velocidade(args):
    """
    Simulacao de um motor DC com controle em cascata:

        malha externa de VELOCIDADE (PID) -> referencia de corrente (ia_ref)
        malha interna de CORRENTE  (PID) -> referencia de tensao de armadura (V)

    Diferente da simulacao BLDC, nao ha SVPWM/inversor trifasico: a
    tensao de armadura e aplicada diretamente (fonte de tensao ideal
    saturada em +-Vdc), como em um acionamento por ponte H de um
    unico braco. Campos opcionais (KdOmega, KdId, dtOmega, dtCurrent,
    ia_max) sao lidos com getattr() para nao quebrar caso ainda nao
    existam no params.py.
    """
    motor = DCMotor(
        R=args.R, L=args.L, Ke=args.Ke, Kt=args.Kt,
        J=args.J, B=args.B,
    )

    if args.Dt > 0.0:
        dt = args.Dt
    else:
        dt = 1e-6

    time_sim = TimeSimulation(t0=args.Ti, tf=args.Tf, dt=dt)
    total_steps = int((time_sim.tf - time_sim.t0)/dt + 0.5) + 1

    print(f"Passo de integracao (dt)    = {dt:.9e} s")
    print(f"Total de passos da simulacao ~ {total_steps}\n")

    if total_steps > 5_000_000:
        print(f"Aviso: {total_steps} passos -- a simulacao pode demorar bastante.")

    Vdc = args.Vdc
    ia_max = getattr(args, "ia_max", 50.0)

    dtOmega = getattr(args, "dtOmega", 1e-3)
    dtCurrent = getattr(args, "dtCurrent", 50e-6)

    KdOmega = getattr(args, "KdOmega", 0.0)
    KdId = getattr(args, "KdId", 0.0)

    omega_ref = rpm_to_rads(args.rpm)
    print(f"rpm_ref = {args.rpm:.2f} RPM")
    print(f"omega_ref = {omega_ref:.6f} rad/s")

    pi_omega = pid_controller_init(args.KpOmega, args.KiOmega, KdOmega, dtOmega,
                                   True, -ia_max, True, ia_max)
    pi_current = pid_controller_init(args.KpId, args.KiId, KdId, dtCurrent,
                                     True, -Vdc, True, Vdc)

    t_next_omega = t_next_current = time_sim.t0
    ia_ref_hold = v_ref_hold = 0.0

    try:
        log_file = open(args.filename, "w")
    except OSError as e:
        print(f"Erro ao criar o arquivo de log: {e}")
        return 1

    log_file.write("time;V;ia;e;Te;omega_r;theta_r;ia_ref\n")

    pb = progress_bar_init(time_sim.t0, time_sim.tf, time_sim.dt)

    for k in range(total_steps):
        t = time_sim.t0 + k*dt
        if t > time_sim.tf:
            break

        if t >= args.Ttl:
            args.Tl = args.Tlnew

        # A. malha externa de velocidade (retencao de ultima ordem)
        if t >= t_next_omega:
            ia_ref_hold = pid_controller_update(pi_omega, omega_ref, motor.omega_r)
            t_next_omega += dtOmega
        ia_ref = ia_ref_hold

        # B. malha interna de corrente (retencao de ultima ordem)
        if t >= t_next_current:
            v_ref_hold = pid_controller_update(pi_current, ia_ref, motor.ia)
            t_next_current += dtCurrent
        V = v_ref_hold

        # C. planta
        dc_motor_step(V, motor, dt, args.Tl)

        progress_bar_update(pb, t)

        log_file.write(
            f"{t:.6f};{V:.4f};{motor.ia:.6f};{motor.e:.6f};"
            f"{motor.Te:.6f};{motor.omega_r:.6f};{motor.theta_r:.6f};"
            f"{ia_ref:.6f}\n"
        )

    progress_bar_finish(pb)
    log_file.close()
    print(f"\n\nSimulacao concluida. Resultados em \"{args.filename}\".\n")

    print(f'\nPlot dos graficos de ia, Te, RPM, theta_r\n\n')
    plot_graficos(args.filename, motor_type="dc")

    return 0

def simulation_dc_motor_malha_velocidade(args):
    """
    Simulacao de um motor DC com controle somente de velocidade:

        referencia de velocidade -> PID de velocidade -> tensao de armadura

    A saida do PID de velocidade e aplicada diretamente ao motor,
    limitada entre -Vdc e +Vdc.
    """

    motor = DCMotor(
        R=args.R,
        L=args.L,
        Ke=args.Ke,
        Kt=args.Kt,
        J=args.J,
        B=args.B,
    )

    if args.Dt > 0.0:
        dt = args.Dt
    else:
        dt = 1e-6

    time_sim = TimeSimulation(
        t0=args.Ti,
        tf=args.Tf,
        dt=dt
    )

    total_steps = int(
        (time_sim.tf - time_sim.t0) / dt + 0.5
    ) + 1

    print(f"Passo de integracao (dt)    = {dt:.9e} s")
    print(f"Total de passos da simulacao ~ {total_steps}\n")

    if total_steps > 5_000_000:
        print(
            f"Aviso: {total_steps} passos -- "
            "a simulacao pode demorar bastante."
        )

    Vdc = args.Vdc

    # Periodo de atualizacao do controlador de velocidade
    dtOmega = getattr(args, "dtOmega", 1e-3)

    # Ganho derivativo opcional
    KdOmega = getattr(args, "KdOmega", 0.0)

    # Referencia de velocidade
    omega_ref = rpm_to_rads(args.rpm)

    print(f"rpm_ref = {args.rpm:.2f} RPM")
    print(f"omega_ref = {omega_ref:.6f} rad/s")

    # PID de velocidade
    # A saida agora e diretamente a tensao V
    pi_omega = pid_controller_init(
        args.KpOmega,
        args.KiOmega,
        KdOmega,
        dtOmega,
        True,
        0,
        True,
        Vdc
    )

    t_next_omega = time_sim.t0

    V_hold = 0.0

    try:
        log_file = open(args.filename, "w")
    except OSError as e:
        print(f"Erro ao criar o arquivo de log: {e}")
        return 1

    log_file.write(
        "time;V;ia;e;Te;omega_r;theta_r\n"
    )

    pb = progress_bar_init(
        time_sim.t0,
        time_sim.tf,
        time_sim.dt
    )

    for k in range(total_steps):

        t = time_sim.t0 + k * dt

        if t > time_sim.tf:
            break

        if t >= args.Ttl:
            args.Tl = args.Tlnew

        # Malha de velocidade
        if t >= t_next_omega:

            V_hold = pid_controller_update(
                pi_omega,
                omega_ref,
                motor.omega_r
            )

            t_next_omega += dtOmega

        # Tensao aplicada diretamente ao motor
        V = V_hold

        # Planta
        dc_motor_step(
            V,
            motor,
            dt,
            args.Tl
        )

        progress_bar_update(pb, t)

        log_file.write(
            f"{t:.6f};"
            f"{V:.4f};"
            f"{motor.ia:.6f};"
            f"{motor.e:.6f};"
            f"{motor.Te:.6f};"
            f"{motor.omega_r:.6f};"
            f"{motor.theta_r:.6f}\n"
        )

    progress_bar_finish(pb)
    log_file.close()

    print(
        f'\n\nSimulacao concluida. '
        f'Resultados em "{args.filename}".\n'
    )

    print(
        '\nPlot dos graficos de ia, Te, RPM, theta_r\n\n'
    )

    plot_graficos(
        args.filename,
        motor_type="dc"
    )

    return 0