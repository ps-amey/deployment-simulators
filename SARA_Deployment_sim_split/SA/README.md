# SA Deployment Simulator

`sa_deployment_sim.py` handles SA1 and SA2 command detection, voltage/current reproduction, and the two SA deployment-status outputs. It uses the shared `deployment_sim_core.py`.

## Pin ownership

| Function | SA1 main | SA1 redundant | SA2 main | SA2 redundant |
|---|---:|---:|---:|---:|
| `obc-do` command input | GP0 | GP2 | GP1 | GP3 |
| `obc-ext-adc` voltage output | GP0 | GP2 | GP1 | GP3 |
| `obc-ext-adc` current output | GP8 | GP10 | GP9 | GP11 |

Status feedback on `obc-di` is GP26 for SA1 and GP20 for SA2. The SA simulator never controls the RA pins.

## Running it

Run from the repository root. A hardware run requires one mode and one deployment permission for each SA unit:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

To use explicit device paths:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --command-pico /dev/piersight-hil/inspace-obc-pico-hil-1/obc-do \
  --ext-adc-pico /dev/piersight-hil/inspace-obc-pico-hil-1/obc-ext-adc \
  --feedback-pico /dev/piersight-hil/inspace-obc-pico-hil-1/obc-di \
  --sa1-main --sa2-main \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Use `Ctrl+C` to stop. Owned V/I and status outputs are returned LOW and verified unless `--leave-feedback` is selected for status outputs. The simulator also cleans up after its 500-second runtime limit.

## Mode combinations

Each unit independently requires exactly one mode:

| Mode | Command path | Expected width |
|---|---|---:|
| `--saN-main` | main only | 50 ms |
| `--saN-red` | redundant only | 50 ms |
| `--saN-main-red` | main and redundant HIGH together | 50 ms each |
| `--saN-ex-main` | extended main | 80 ms |
| `--saN-ex-red` | extended redundant | 80 ms |

For `main-red`, both inputs must overlap. V/I outputs remain LOW when only one input is HIGH, both selected V/I pairs rise during overlap, and both pulse widths must belong to the same attempt for status completion.

## V/I input combinations

| `--v-ch-feedback` | `--i-ch-feedback` | Result |
|---|---|---|
| `yes` | `yes` | Voltage and current outputs follow qualified command inputs |
| `yes` | `no` | Only voltage outputs follow commands |
| `no` | `yes` | Only current outputs follow commands |
| `no` | `no` | Commands are monitored, but the external-ADC Pico is not opened |

## Hardware-free test cases

These commands do not open any Pico.

Main/redundant with both statuses permitted:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes --strict-width \
  --dry-run-pulses GP0:50,GP3:50 --log /tmp/sa-main-red.jsonl
```

Expected final state: `{'SA1': True, 'SA2': True}`.

One deployment permission disabled:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-main \
  --sa1-deployment yes --sa2-deployment no \
  --v-ch-feedback yes --i-ch-feedback no --strict-width \
  --dry-run-pulses GP0:50,GP1:50 --log /tmp/sa-permission.jsonl
```

Expected final state: `{'SA1': True, 'SA2': False}`.

Extended modes:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-ex-main --sa2-ex-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback no --i-ch-feedback yes --strict-width \
  --dry-run-pulses GP0:80,GP3:80 --log /tmp/sa-extended.jsonl
```

Invalid width with the default ±15 ms tolerance:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-main \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes --strict-width \
  --dry-run-pulses GP0:80,GP1:50 --log /tmp/sa-invalid-width.jsonl
```

Expected final state: SA1 remains LOW; SA2 becomes HIGH.

Dry-run input has no rising/falling timestamps, so it cannot prove electrical overlap for `main-red`; test that case on hardware.
