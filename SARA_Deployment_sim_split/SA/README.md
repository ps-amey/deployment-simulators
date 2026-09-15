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
| `--saN-ex-main-red` | extended main and redundant HIGH together | 100 ms each |
| `--saN-all` | main and redundant independently | 50 ms each |
| `--saN-ex-main` | extended main | 100 ms |
| `--saN-ex-red` | extended redundant | 100 ms |

For `main-red`, both inputs must overlap. V/I outputs remain LOW when only one input is HIGH, both selected V/I pairs rise during overlap, and both pulse widths must belong to the same attempt for status completion.

For `ex-main-red`, the same overlap and same-attempt rules apply, but both completed pulses must validate against the 100 ms extended width. With deployment permission `yes`, status is asserted only after both valid pulse widths have been received.

For `all`, main and redundant inputs are monitored independently and each V/I output follows its corresponding input regardless of pulse length. Deployment-status feedback still uses the configured pulse-width and deployment-permission rules.

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
  --dry-run-pulses GP0:100,GP3:100 --log /tmp/sa-extended.jsonl
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

## Numbered hardware test cases

Run these commands from the repository root. `Pulse accepted` means that the command pulse was detected and its V/I channels were reproduced. A deployment passes only when the corresponding deployment-status output is also asserted.

Each run automatically creates a new JSON-Lines event log named like `logs/sa_deployment_events_20260915_143025.log`. The filename contains the local date and time. `--log PATH` remains available only when an explicit filename is needed.

### SA01 - deployment through main paths

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-main \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid main pulses reproduce V/I feedback and assert the SA1 and SA2 deployment-status outputs.

### SA02 - deployment through redundant paths

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-red --sa2-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid redundant pulses reproduce V/I feedback and assert the SA1 and SA2 deployment-status outputs.

### SA03 - deployment through extended main paths

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-ex-main --sa2-ex-main \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid extended-main pulses reproduce V/I feedback and assert the SA1 and SA2 deployment-status outputs.

### SA04 - deployment through extended redundant paths

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-ex-red --sa2-ex-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid extended-redundant pulses reproduce V/I feedback and assert the SA1 and SA2 deployment-status outputs.

### SA05 - accepted pulse with deployment disabled

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-main \
  --sa1-deployment no --sa2-deployment no \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid pulses and their V/I feedback are accepted, but both deployment-status outputs remain LOW. The deployment result is **FAIL by test scenario** because deployment permission is `no`.

### SA06 - observe every main and redundant I/V pulse

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-all --sa2-all \
  --sa1-deployment no --sa2-deployment no \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: GP0, GP1, GP2, and GP3 are monitored independently. Every pulse reproduces its corresponding V/I feedback and its measured width is printed, regardless of pulse validity. Deployment-status outputs remain LOW, so this is an I/V-observation case and the deployment result is **FAIL by test scenario**.

### SA07 - SA1 deployment disabled and SA2 deployment enabled

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-main \
  --sa1-deployment no --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: valid pulses from both units reproduce V/I feedback. SA1 deployment status remains LOW and SA1 is **FAIL by test scenario**. SA2 deployment status becomes HIGH after its first valid pulse and SA2 is **PASS**.

### SA08 - deployment through 100 ms main and redundant pulses

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-ex-main-red --sa2-ex-main-red \
  --sa1-deployment yes --sa2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: each SA unit requires its main and redundant inputs to be HIGH together. Both completed pulses must be valid at 100 ms and belong to the same overlap attempt. Once both widths are accepted, that unit's deployment-status output becomes HIGH.
