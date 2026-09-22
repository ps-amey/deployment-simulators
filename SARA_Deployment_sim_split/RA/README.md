# RA Voltage/Current Feedback Simulator

`ra_deployment_sim.py` monitors RA1/RA2 deployment commands and reproduces only their voltage and current feedback on `obc-ext-adc`. It does not open `obc-di`, drive GP21, update RA deployment status, track RA1/RA2 milestones, or require deployment-permission inputs.

## Pin ownership

| Function | RA1 main | RA1 redundant | RA2 main | RA2 redundant |
|---|---:|---:|---:|---:|
| `obc-do` command input | GP4 | GP6 | GP5 | GP7 |
| `obc-ext-adc` voltage output | GP4 | GP6 | GP5 | GP7 |
| `obc-ext-adc` current output | GP12 | GP14 | GP13 | GP15 |

No `obc-di` status pin is owned or touched by this program.

## Running it

Run from the repository root:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-main \
  --v-ch-feedback yes --i-ch-feedback yes
```

To keep the simulator running until Ctrl+C, add:

```bash
--simulator-timeout none
```

Only the command Pico and external-ADC Pico are opened. When both V and I feedback are `no`, only the command Pico is opened.

Use `Ctrl+C` to stop. All owned external-ADC pins are returned LOW and verified. By default, the simulator also cleans up after its 200-second runtime limit; use `--simulator-timeout none` to disable that limit.

## RA input-path combinations

RA1 and RA2 independently select one mode, so these common pairs can be tested:

| RA1 mode | RA2 mode | Monitored command pins |
|---|---|---|
| main | main | GP4 and GP5 |
| main | redundant | GP4 and GP7 |
| redundant | main | GP6 and GP5 |
| redundant | redundant | GP6 and GP7 |
| main-red | main-red | GP4+GP6 and GP5+GP7 |
| ex-main-red | ex-main-red | GP4+GP6 and GP5+GP7 |
| all | all | GP4 and GP6 independently; GP5 and GP7 independently |

`--raN-ex-main` selects the same physical main input and `--raN-ex-red` selects the same physical redundant input. Because RA status and milestones are disabled, pulse duration is not used to decide any deployment-status output.

For `main-red`, the existing electrical qualification remains: both inputs for that RA unit must be HIGH together before its V/I outputs rise; either input falling returns both corresponding outputs LOW.

For `ex-main-red`, the same overlap qualification is used with a 100 ms extended pulse width on both lines. RA still produces V/I feedback only and does not assert a deployment-status output.

For `all`, both physical paths are monitored independently. Each selected V/I output follows its corresponding command input directly, regardless of pulse length or whether the other path is HIGH.

## V/I output combinations

| `--v-ch-feedback` | `--i-ch-feedback` | Result |
|---|---|---|
| `yes` | `yes` | GP4–GP7 voltage and GP12–GP15 current feedback are enabled |
| `yes` | `no` | Only GP4–GP7 voltage feedback is enabled |
| `no` | `yes` | Only GP12–GP15 current feedback is enabled |
| `no` | `no` | Inputs are monitored; no external-ADC device is opened or driven |

## Hardware-free test cases

These commands verify selection and V/I routing without opening Picos.

Both main paths, V and I enabled:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-main \
  --v-ch-feedback yes --i-ch-feedback yes \
  --dry-run-pulses GP4:100,GP5:100 --log /tmp/ra-main-main.jsonl
```

Expected routing: GP4 → V GP4/I GP12; GP5 → V GP5/I GP13.

RA1 main and RA2 redundant, voltage only:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-red \
  --v-ch-feedback yes --i-ch-feedback no \
  --dry-run-pulses GP4:100,GP7:100 --log /tmp/ra-main-red.jsonl
```

Expected routing: GP4 → V GP4; GP7 → V GP7. No current or status outputs.

RA1 redundant and RA2 main, current only:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-red --ra2-main \
  --v-ch-feedback no --i-ch-feedback yes \
  --dry-run-pulses GP6:100,GP5:100 --log /tmp/ra-red-main.jsonl
```

Expected routing: GP6 → I GP14; GP5 → I GP13.

Both redundant paths with V/I disabled:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-red --ra2-red \
  --v-ch-feedback no --i-ch-feedback no \
  --dry-run-pulses GP6:100,GP7:100 --log /tmp/ra-disabled.jsonl
```

Expected result: both commands are detected, but no V/I or status outputs are driven.

An unselected input is ignored in dry-run:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-main \
  --v-ch-feedback yes --i-ch-feedback yes \
  --dry-run-pulses GP6:100,GP5:100 --log /tmp/ra-unselected.jsonl
```

Expected result: GP6 is ignored because RA1 main selected GP4; GP5 routes to V GP5/I GP13.

Dry-run cannot represent simultaneous HIGH overlap. Validate `main-red` overlap behavior using the hardware command inputs.

## Numbered hardware test cases

Run these commands from the repository root. The RA simulator verifies command-pulse detection and V/I reproduction only. It has no RA deployment-status output and no `--raN-deployment yes/no` option. Therefore, an accepted RA pulse does not by itself prove deployment; any test that requires a deployment-status assertion must record the RA deployment result as **FAIL/status unavailable**.

Each run automatically creates a new JSON-Lines event log named like `logs/ra_deployment_events_20260915_143025.log`. The filename contains the local date and time. `--log PATH` remains available only when an explicit filename is needed.

### RA01 - main-path pulse and I/V feedback

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-main \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: main pulses on GP4 and GP5 reproduce their corresponding V/I feedback and print their measured widths.

### RA02 - redundant-path pulse and I/V feedback

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-red --ra2-red \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: redundant pulses on GP6 and GP7 reproduce their corresponding V/I feedback and print their measured widths.

### RA03 - extended-main pulse and I/V feedback

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-ex-main --ra2-ex-main \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: extended-main pulses use GP4 and GP5, reproduce their corresponding V/I feedback, and print their measured widths.

### RA04 - extended-redundant pulse and I/V feedback

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-ex-red --ra2-ex-red \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: extended-redundant pulses use GP6 and GP7, reproduce their corresponding V/I feedback, and print their measured widths.

### RA05 - observe every main and redundant I/V pulse

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-all --ra2-all \
  --v-ch-feedback yes --i-ch-feedback yes
```

Expected: GP4, GP5, GP6, and GP7 are monitored independently. Every pulse reproduces its corresponding V/I feedback and its measured width is printed, regardless of pulse validity. No RA deployment status is generated; if status assertion is required, record the deployment result as **FAIL/status unavailable**.

## Fixed-width test mode

Use `--test-mode --test-pulse-ms WIDTH` with the normal path and channel options when the V/I feedback duration must be controlled by the laptop. Rising-edge qualification is unchanged; after a qualified rise, the laptop drives the selected V/I outputs HIGH and returns them LOW after the requested width, without waiting for the OBC falling edge.

Example:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-all --ra2-all \
  --v-ch-feedback yes --i-ch-feedback yes \
  --test-mode --test-pulse-ms 30
```

RA still has no deployment-status output. OBC falling edges continue to be measured and logged independently.
