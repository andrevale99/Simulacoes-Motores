# NoBreakMotors

Simulador de motores BLDC (e, em parte, DC) em Python, com modelos de
inversor trifásico, modulação SVPWM e six-step, controladores PID e
partida em malha aberta por rampa V/f. O repositório também traz arquivos
de parâmetros de motores retirados da literatura, um script para comparar
a simulação com ensaios de bancada e um firmware de controle de velocidade
de motor DC para STM32F411.

## Estrutura

```
.
├── python/                  simulador
│   ├── main.py              ponto de entrada
│   ├── params.py            parâmetros: defaults, arquivo de config e CLI
│   ├── simulations.py       laços de simulação (FOC, six-step, V/f)
│   ├── bldc.py              modelo do motor BLDC
│   ├── dc_motor.py          modelo do motor DC com escovas
│   ├── inverter.py          inversor de dois níveis e comutação six-step
│   ├── svpwm.py             modulação vetorial (SVPWM)
│   ├── transforms.py        transformadas de Clarke e Park
│   ├── pid_controller.py    PID com anti-windup e PID não linear (NPID)
│   ├── vf_ramp.py           partida em malha aberta por rampa V/f
│   ├── graficos.py          gráficos e FFT das correntes a partir do CSV
│   ├── ganhos.py            cálculo dos ganhos dos PIs a partir do motor
│   ├── exemplos.py          exemplos de uso do inversor e do BLDC
│   ├── progressbar.py       barra de progresso da simulação
│   ├── auxs.py              conversões rpm <-> rad/s
│   ├── simulation_bldc_velocidade.md   documentação da simulação six-step
│   └── testesBancada/       comparação com ensaios de bancada
├── motors/
│   ├── bldc/                arquivos de parâmetros de motores BLDC
│   └── dc/                  arquivo de parâmetros de motor DC
└── firmwares/
    └── DcMotorControl/      firmware STM32F411 + FreeRTOS (motor DC)
```

## Requisitos

- Python 3 com `numpy`, `pandas` e `matplotlib`;
- uma instalação de LaTeX, porque os gráficos usam `text.usetex = True`
  (em `graficos.py` e em `testesBancada/comparacao_rpm_duty.py`).

```bash
pip install numpy pandas matplotlib
```

## Como usar

Os módulos se importam pelo nome, sem pacote, então rode a partir da
pasta `python/`. Os gráficos são salvos em `python/img/`, que precisa
existir:

```bash
cd python
mkdir -p img
python main.py -c ../motors/bldc/bldc_barros.txt
```

A simulação grava um CSV (separado por `;`) com uma linha por passo de
integração, salva os gráficos em PDF em `img/` e abre as janelas do
matplotlib.

### Parâmetros

Cada parâmetro é resolvido nesta ordem de prioridade:

1. argumento de linha de comando;
2. arquivo de configuração passado com `-c`;
3. valor padrão em `DEFAULTS` ([params.py](python/params.py)).

O arquivo de configuração usa o formato `CHAVE = VALOR` (ou
`CHAVE VALOR`), com comentários iniciados por `#`. Exemplo de sobrescrita
pela linha de comando:

```bash
python main.py -c ../motors/bldc/a2212.txt --rpm 800 --Tf 1.0 -o saida.csv
```

| Grupo | Chaves |
|---|---|
| Planta | `motor` (`bldc` ou `dc`) |
| Motor | `R`, `L`, `M`, `Ke`, `Kt`, `J`, `B`, `P` (pares de polos), `Tl` |
| Chaveamento | `Fsw`, `PwmSamples` |
| Tempo | `Ti`, `Tf`, `Dt` (`Dt = 0` usa `1/Fsw / PwmSamples`) |
| Degrau de carga | `Ttl` (instante), `Tlnew` (novo torque) |
| Barramento | `Vdc` |
| Controladores | `KpOmega`, `KiOmega`, `KdOmega`, `KpId`, `KiId`, `KdId`, `KpIq`, `KiIq`, `KdIq` |
| Referência | `rpm`, `rpmRef` (degraus `"t0:rpm0, t1:rpm1, ..."`) |
| Partida | `MalhaAberta` (`true`/`false`, `1`/`0`, ...) |
| Saída | `file` (ou `-o`/`--file`) |

`python main.py -h` lista todas as opções. `python params.py -c arquivo.txt`
mostra os valores finais depois de combinar as três fontes.

### Escolha da simulação

A simulação BLDC executada é escolhida em [main.py](python/main.py),
comentando e descomentando as chamadas. Por padrão roda
`simulation_bldc_velocidade`.

| Função | Controle | Modelo da planta |
|---|---|---|
| `simulation_bldc_malha_corrente_velocidade` | FOC: PI de velocidade (1 ms) → `iq_ref`; PIs de `id` e `iq` (a cada `Ts`) → SVPWM | `bldc_step`: tensões de fase vindas das chaves das três pernas |
| `simulation_bldc_velocidade` | Six-step H_PWM-L_ON; PID de velocidade em pu com saída (duty) em `[0, 1]`; partida V/f opcional (`MalhaAberta`) e referência em degraus (`rpmRef`) | `bldc_step_switched`: inversor perna a perna com diodos de roda livre e fase flutuante aberta |
| `simulation_bldc_velocidade_vf` | Só a partida V/f em malha aberta: alinhamento, rampa até `rpm` e permanência nessa velocidade, sem PI | `bldc_step_switched` |

As constantes da partida V/f (`VF_*`) e o deslocamento de comutação
`SIX_STEP_THETA_OFFSET` ficam em [simulations.py](python/simulations.py).
A simulação six-step é descrita em detalhes em
[simulation_bldc_velocidade.md](python/simulation_bldc_velocidade.md).

**Motor DC:** o modelo existe em [dc_motor.py](python/dc_motor.py), mas
no momento não há simulação de motor DC em `simulations.py`. Com
`motor = dc`, o `main.py` termina com uma mensagem de erro.

### Colunas do CSV

- FOC: `time;Va;Vb;Vc;ia;ib;ic;ea;eb;ec;id;iq;Te;theta_r;omega_r;iq_ref;vd_ref;vq_ref`
- Six-step e V/f: `time;Va;Vb;Vc;ia;ib;ic;ea;eb;ec;Te;theta_r;omega_r;duty_ref;sector;vf;omega_cmd;da;db;dc`

`vf` vale 1 enquanto a partida V/f está ativa. `da`, `db` e `dc` são os
duties de cada fase na convenção do firmware (fase centrada em 50%), para
comparação com o ensaio.

## Modelos

### Motor BLDC ([bldc.py](python/bldc.py))

Motor trifásico em estrela com neutro isolado e FCEM trapezoidal (ou
senoidal, pela flag `trapezoidal_back_emf_flag`). Para cada fase:

```
e_k      = Ke * omega_r * f_k(theta_e)
di_k/dt  = (v_k - R*i_k - e_k - vn) / (L - M)
Te       = Kt * sum(i_k * f_k)
J*domega_r/dt = Te - Tl - B*omega_r
```

`vn` é a tensão do neutro que mantém `ia + ib + ic = 0`. A integração é
por Euler explícito. `bldc_step_switched` também resolve o estado de cada
perna do inversor: chave superior, chave inferior, diodo conduzindo ou
perna aberta. Com isso, a roda livre no PWM e a corrente de comutação
aparecem naturalmente.

### Inversor ([inverter.py](python/inverter.py)) e SVPWM ([svpwm.py](python/svpwm.py))

- modelo médio (duty → tensão de fase) e chaveamento com portadora
  triangular;
- comutação six-step (120°) pela tabela `SIX_STEP_TABLE`;
- modelo perna a perna com diodos (`six_step_gates`,
  `inverter_pole_voltages`, `inverter_open_leg_clamp`);
- SVPWM por injeção de sequência zero (`svpwm_modulate`). Na simulação
  FOC, o vetor de tensão é limitado à região linear `|v| <= Vdc/sqrt(3)`.

### Controladores ([pid_controller.py](python/pid_controller.py))

- `PIDController`: PID com derivada sobre a medida (sem *derivative
  kick*) e anti-windup por saturação. Com `Kd = 0` é um PI.
- `NPIDController`: PID não linear, com ganhos que variam com `|e|` entre
  um mínimo e um máximo, por funções de forma crescentes ou decrescentes.

### Partida V/f ([vf_ramp.py](python/vf_ramp.py))

Alinhamento em dois passos (para não parar no ponto morto), seguido de uma
rampa de velocidade com amplitude `u = min(u_offset + u_per_rpm*rpm, u_max)`.
Não usa realimentação de posição. Termina quando a velocidade medida
atinge `rpm_end`.

### Motor DC ([dc_motor.py](python/dc_motor.py))

Modelo de armadura com uma malha elétrica e uma mecânica, integrado por
Euler explícito. Rodar o arquivo diretamente executa um autoteste em
malha aberta.

## Ferramentas auxiliares

- **Ganhos dos PIs** ([ganhos.py](python/ganhos.py)): a partir de um
  arquivo de parâmetros, calcula os ganhos das malhas de corrente
  (banda em `Fsw/20`) e de velocidade (5 vezes mais lenta) e imprime as
  linhas prontas para colar no arquivo de configuração.

  ```bash
  python ganhos.py ../motors/bldc/bldc_barros.txt
  ```

- **Exemplos** ([exemplos.py](python/exemplos.py)): seis exemplos de uso
  do inversor e do BLDC, do modelo médio ao six-step acionando o motor.

  ```bash
  python exemplos.py              # todos
  python exemplos.py 4 5          # só os exemplos 4 e 5
  python exemplos.py --salvar img # salva os gráficos em img/
  ```

- **Autoteste do inversor**: `python inverter.py` plota o PWM senoidal e o
  six-step ideal.

- **Comparação com a bancada**
  ([testesBancada/comparacao_rpm_duty.py](python/testesBancada/comparacao_rpm_duty.py)):
  sobrepõe o RPM do log do firmware ao da simulação e mostra os duties de
  cada fase dos dois. A referência de RPM sai do nome do arquivo
  (`log-1150.csv` → 1150 rpm).

  ```bash
  python testesBancada/comparacao_rpm_duty.py teste.csv simulacao.csv saida.pdf \
      [--open-loop] [--fase {a,b,c}] [--t-inicial T] [--t-final T] [--sem-referencia]
  ```

## Motores disponíveis

Arquivos em [motors/bldc/](motors/bldc/). A maioria cita, no cabeçalho, o
artigo de onde vieram os parâmetros.

| Arquivo | Origem | P | Vdc [V] | rpm |
|---|---|---|---|---|
| `a2212.txt` | motor A2212, com partida V/f | 7 | 12 | 1500 |
| `bldc_1.txt` | parâmetros padrão do simulador | 4 | 24 | 20 |
| `bldc_2.txt` | — | 7 | 36 | 955 |
| `bldc_abeykoon.txt` | Shanthar, Abeykoon e Maithripala (CDSR 2022) | 7 | 22 | 2000 |
| `bldc_ahmed.txt` | Majid et al. (IMPACT 2026) | 4 | 350 | 1000 |
| `bldc_barros.txt` | Barros e Maximo (SBR/WRE 2025) | 8 | 18 | 800 |
| `bldc_gael.txt` | mesmos parâmetros de `bldc_1.txt` | 4 | 24 | 20 |
| `bldc_hani.txt` | Prasetyo, Rohman e Santabudi (ICSITech 2017) | 4 | 24 | 500 |
| `bldc_kumar.txt` | Kumar, Gupta e Gupta | 4 | 220 | 900 |

[motors/dc/motor_teste.txt](motors/dc/motor_teste.txt) traz os parâmetros
de um motor DC, que hoje não tem simulação associada.

## Firmware `DcMotorControl`

Controle de velocidade de um motor DC com redução e encoder, numa Black
Pill (STM32F411) com FreeRTOS, sem HAL (registradores e CMSIS):

- driver **DRV8833** em modo paralelo, PWM de 10 kHz no TIM3 (PA6/PA7),
  nSLEEP em PB0 e nFAULT em PB1 (interrupção que solta o motor);
- **encoder** em quadratura no TIM4 (PB6/PB7), 11 pulsos por volta e
  redução de 33,8:1, amostrado a cada 10 ms;
- **PID** de velocidade no eixo de saída (setpoint de 100 rpm, limite de
  ±165 rpm);
- log pela **USART1** (PA9/PA10, 115200 baud): setpoint, rpm, saída do
  PID e duty de cada canal;
- LED em PC13 piscando como sinal de vida.

O código próprio fica em `src/` (main, drivers de GPIO, RCC, UART e LCD
16x2) e `lib/` (drv8833, encoder, pid).

Para compilar e gravar é preciso o `arm-none-eabi-gcc` e um gravador:

```bash
cd firmwares/DcMotorControl
make compile                 # gera build/firmware.elf e build/firmware.bin
make flash                   # grava com st-flash (padrão)
make flash FLASHER=openocd   # ou openocd / dfu-util
```

## Licença

[MIT](LICENSE).
