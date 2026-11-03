# `simulation_bldc_velocidade`: funções de `bldc.py` e `inverter.py`

Este documento descreve as funções dos módulos [bldc.py](bldc.py) e
[inverter.py](inverter.py) usadas pela simulação de controle de velocidade
do BLDC com comutação six-step,
[`simulation_bldc_velocidade`](simulations.py#L442).

## Visão geral: um passo da simulação

A cada passo `dt`, o laço principal ([simulations.py:533-588](simulations.py#L533-L588)) faz:

```
theta_e = (P * theta_r) mod 2pi                       # posição elétrica do rotor
        |
        v
setor   = six_step_sector(theta_e + SIX_STEP_THETA_OFFSET)   # malha fechada
          six_step_sector(vf.theta_v)                         # partida V/f
        |
        v
carrier = inverter_carrier(Ts, t)                     # portadora triangular do PWM
gates   = six_step_gates(setor, duty_ref, carrier)    # estado das chaves (+1, -1, 0)
        |
        v
Vabc    = bldc_step_switched(gates, inverter, motor, time_sim, Tl, True)
          (inversor com diodos + motor: correntes, torque, velocidade, posição)
```

O `duty_ref` vem do PI de velocidade (malha fechada) ou da rampa V/f
(partida em malha aberta). Ele é a única grandeza de "controle". O resto é
a planta: inversor e motor.

---

## `bldc.py`

### `BLDCMotor` (dataclass)

Guarda os parâmetros e o estado do motor.

| Campo | Significado |
|---|---|
| `R`, `L`, `M` | resistência, indutância própria e mútua de fase |
| `Ke`, `Kt` | constante de FCEM [V/(rad/s)] e de torque [N.m/A] |
| `J`, `B` | inércia e atrito viscoso |
| `P` | pares de polos |
| `iabc`, `eabc` | correntes e FCEMs das três fases (estado) |
| `theta_e`, `omega_e` | ângulo e velocidade elétricos |
| `theta_r`, `omega_r` | ângulo e velocidade mecânicos |
| `Te` | torque eletromagnético |

Na simulação, o motor é criado com os valores de `params.py` e atualizado
in-place por `bldc_step_switched()`.

### `TimeSimulation` (dataclass)

Agrupa `t0`, `tf` e `dt`. `bldc_step_switched()` usa só o `dt`.

### `trapezoidal_back_emf(theta)`

Forma de onda **normalizada** (amplitude 1) da FCEM trapezoidal, com período
2π:

| Faixa de `theta` | Valor |
|---|---|
| 0° – 30° | sobe de 0 a 1 |
| 30° – 150° | 1 (patamar) |
| 150° – 210° | desce de 1 a −1 |
| 210° – 330° | −1 (patamar) |
| 330° – 360° | sobe de −1 a 0 |

Cada patamar dura 120°, que é o intervalo de condução de uma fase no
six-step. Em `bldc_step_switched()`, a função de forma de cada fase é
`f_k = -trapezoidal_back_emf(theta_e + phi_k)`, com `phi_a = 0`,
`phi_b = -120°` e `phi_c = +120°`. O sinal negativo e essa defasagem
explicam o deslocamento de 150° em
[`SIX_STEP_THETA_OFFSET`](simulations.py#L426): é o ângulo que faz a
comutação coincidir com os patamares da FCEM, como fariam os sensores Hall.

### `bldc_step_switched(gates, inverter, motor, time, Tl, trapezoidal_back_emf_flag)`

[bldc.py:71](bldc.py#L71). **É a função central da simulação:** integra
um passo de `dt` do conjunto inversor + motor, com neutro isolado (estrela
sem fio de neutro). Ela não recebe tensões prontas: as tensões dependem do
estado das chaves **e** do sentido das correntes, por causa dos diodos de
roda livre.

Etapas:

1. **Posição e FCEM.** Calcula `theta_e = P*theta_r`, `omega_e = P*omega_r`,
   as funções de forma `f_k` e as FCEMs `e_k = Ke * omega_r * f_k`.

2. **Tensão de polo de cada perna.** Chama
   [`inverter_pole_voltages()`](#inverter_pole_voltagesinverter-gates-currents):
   cada perna fica em `Vdc`, em `0` ou aberta (`None`).

3. **Tensão do neutro.** Com o conjunto `C` das pernas que conduzem e
   `u_k = polo_k - R*i_k - e_k`, faz `vn = média de u_k em C`. Essa escolha
   garante `ia + ib + ic = 0`. Para cada perna aberta, verifica com
   [`inverter_open_leg_clamp()`](#inverter_open_leg_clampinverter-v_open)
   se a tensão que a FCEM imporia no polo (`vn + e_k`) saiu de `[0, Vdc]`.
   Se saiu, o diodo correspondente passa a conduzir e `vn` é recalculado.

4. **Dinâmica elétrica (Euler).** Para cada perna que conduz:

   ```
   di_k/dt = (polo_k - R*i_k - e_k - vn) / (L - M)
   ```

   Com menos de duas pernas conectadas não há caminho para a corrente, e
   todas são zeradas.

5. **Bloqueio dos diodos.** Um diodo só conduz num sentido. Se a corrente de
   uma perna que conduzia por diodo cruzou zero, ela é zerada (a fase abre).

6. **Restabelece `ia + ib + ic = 0`.** Se uma corrente foi zerada e restaram
   duas, elas são ajustadas para valores iguais e opostos.

7. **Dinâmica mecânica (Euler explícito).** O torque e a derivada da
   velocidade são calculados no início do passo, com as correntes e a
   velocidade do instante `k`, antes da atualização das correntes:

   ```
   Te        = Kt * sum(i_k[k] * f_k)
   domega_r  = (Te - Tl - B*omega_r[k]) / J
   theta_r  += omega_r[k] * dt
   omega_r  += domega_r * dt
   ```

**Retorno:** `Vabc`, as tensões de terminal de cada fase em relação ao
neutro do motor (`polo_k - vn`). Para uma fase aberta, o valor retornado é
a própria FCEM. É o que vai para as colunas `Va`, `Vb` e `Vc` do CSV.

O modelo reproduz naturalmente:

- a roda livre durante o PWM (fase alta com a chave desligada conduz pelo
  diodo inferior);
- a corrente de comutação, que continua pelo diodo na fase que acabou de
  sair de condução até se extinguir;
- a condução por diodo de uma fase "flutuante" quando a FCEM a leva para
  fora do barramento.

---

## `inverter.py`

### `Inverter` (dataclass)

Guarda só a tensão do barramento CC, `Vdc`. `__post_init__` rejeita
`Vdc <= 0`. A simulação cria `Inverter(Vdc=args.Vdc)` e o repassa a
`bldc_step_switched()`.

### `inverter_carrier(Ts, t)`

Portadora triangular simétrica de período `Ts = 1/Fsw`, normalizada entre
0 e 1:

```
fase = (t mod Ts) / Ts
carrier = 2*fase          se fase < 0.5   (sobe 0 -> 1)
          2 - 2*fase      caso contrário  (desce 1 -> 0)
```

É comparada com o `duty_ref` para gerar o PWM da fase alta.

### `six_step_sector(theta_e)`

Converte um ângulo elétrico [rad] no setor de comutação, de 1 a 6, com
60° por setor:

```
setor = int((theta_e mod 2pi) / (pi/3)) + 1     (limitado a 6)
```

Na simulação, o ângulo de entrada depende da fase:

- **Malha fechada:** `motor.theta_e + SIX_STEP_THETA_OFFSET`, a posição real
  do rotor alinhada à FCEM (equivalente aos sensores Hall).
- **Partida V/f:** `vf.theta_v`, o ângulo comandado pela rampa, sem
  realimentação de posição.

### `SIX_STEP_TABLE`

Tabela que diz, para cada setor, o papel de cada fase:
`1` = alta (ligada a `Vdc`), `-1` = baixa (ligada a 0 V), `0` = flutuante.

| Setor | Faixa | A | B | C |
|---|---|---|---|---|
| 1 | 0° – 60° | alta | baixa | flutuante |
| 2 | 60° – 120° | alta | flutuante | baixa |
| 3 | 120° – 180° | flutuante | alta | baixa |
| 4 | 180° – 240° | baixa | alta | flutuante |
| 5 | 240° – 300° | baixa | flutuante | alta |
| 6 | 300° – 360° | flutuante | baixa | alta |

### `six_step_gates(sector, duty, carrier)`

[inverter.py:223](inverter.py#L223). Traduz o setor e o duty no comando das
chaves de cada perna, no esquema **H_PWM-L_ON**:

| Papel da fase | Comando da perna |
|---|---|
| alta | `+1` (chave superior ligada) se `duty > carrier`, senão `0` (as duas desligadas) |
| baixa | `-1` (chave inferior sempre ligada) |
| flutuante | `0` (as duas desligadas) |

Retorna `(ga, gb, gc)`. Com `duty = 1` a fase alta fica sempre ligada (onda
quadrada plena). Com `duty < 1`, ela é chaveada na frequência `Fsw`, e a
tensão média aplicada fica proporcional ao duty. Quando a chave superior
desliga, a corrente da fase alta continua pelo diodo inferior da mesma
perna (roda livre).

Por dentro, usa `inverter_gate_state(duty, carrier)`, que só faz a
comparação `1 if duty > carrier else 0`.

### `inverter_pole_voltages(inverter, gates, currents)`

[inverter.py:248](inverter.py#L248). Chamada dentro de
`bldc_step_switched()`. Determina a tensão de polo de cada perna (em relação
ao negativo do barramento) a partir do comando e do sentido da corrente:

| Comando | Corrente | Polo | Diodo |
|---|---|---|---|
| `+1` | qualquer | `Vdc` | `0` |
| `-1` | qualquer | `0` | `0` |
| `0` | `> 0` (saindo para o motor) | `0` (diodo inferior conduz) | `-1` |
| `0` | `< 0` (entrando no inversor) | `Vdc` (diodo superior conduz) | `+1` |
| `0` | `= 0` | `None` (perna aberta) | `0` |

Retorna `(poles, diode)`. O vetor `diode` é usado depois para detectar o
bloqueio do diodo quando a corrente cruza zero (etapa 5 de
`bldc_step_switched()`).

### `inverter_open_leg_clamp(inverter, v_open)`

[inverter.py:277](inverter.py#L277). Também é chamada dentro de
`bldc_step_switched()`. Uma perna aberta só continua aberta enquanto a
tensão que a FCEM impõe no seu polo (`v_open = vn + e_k`) estiver dentro do
barramento:

| `v_open` | Resultado |
|---|---|
| `> Vdc` | diodo superior conduz → `(Vdc, +1)` |
| `< 0` | diodo inferior conduz → `(0.0, -1)` |
| em `[0, Vdc]` | continua aberta → `(None, 0)` |

---

## Funções desses módulos que esta simulação **não** usa

- `bldc_step()`: recebe as tensões de fase prontas (as três pernas sempre
  comandadas). Não modela diodos nem fase aberta. É usada na simulação FOC
  (`simulation_bldc_malha_corrente_velocidade`).
- `inverter_output_voltage()` e `switching_output_voltage()`: modelo médio e
  PWM das três pernas (SVPWM/senoidal).
- `six_step_commutation()`: six-step simplificado, que aproxima a fase
  flutuante por duty 0.5 (meio barramento). Aparece só em `exemplos.py` e
  no autoteste do `inverter.py`.
- `line_to_line_voltages()`: tensões de linha a partir das de fase.
