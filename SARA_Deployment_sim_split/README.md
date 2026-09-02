# Separate SA and RA deployment simulators

This directory contains separate SA and RA executables backed by one shared core. The original combined implementation in `../SARA_Deployment_sim/` remains unchanged and is the reference/fallback.

## Architecture and ownership

Both programs use the existing three-Pico design: `obc-do` captures OBC command edges, `obc-ext-adc` reproduces voltage/current pulses, and `obc-di` drives deployment status. Each program initializes, updates, stows, and cleans up only its profile-owned pins.

| Program | `obc-do` inputs | `obc-ext-adc` outputs | `obc-di` status outputs |
|---|---|---|---|
| SA | GP0–GP3 | GP0–GP3 (V), GP8–GP11 (I) | GP26 SA1, GP20 SA2 |
| RA | GP4–GP7 | GP4–GP7 (V), GP12–GP15 (I) | GP21 shared RA |

Run only one separated executable at a time. They share the same three serial devices, so simultaneous raw-REPL sessions would interrupt or corrupt each other. Use the original combined simulator when SA and RA must be active simultaneously.

## Usage

SA hardware-free example:

```bash
python3 SARA_Deployment_sim_split/sa_deployment_sim.py \
  --sa1-main --sa2-red \
  --sa1-deployment yes --sa2-deployment no \
  --v-ch-feedback yes --i-ch-feedback yes --strict-width \
  --dry-run-pulses GP0:50,GP3:50
```

RA hardware-free example:

```bash
python3 SARA_Deployment_sim_split/ra_deployment_sim.py \
  --ra1-main --ra2-main \
  --ra1-deployment yes --ra2-deployment yes \
  --v-ch-feedback yes --i-ch-feedback yes --strict-width \
  --dry-run-pulses GP4:100,GP5:100
```

Use `--help` for Pico paths, logging, stow, feedback, timing, and mode options. Events default to `sa_deployment_events.jsonl` or `ra_deployment_events.jsonl`.

## Preserved behavior

SA normal and extended widths are 50 ms and 80 ms; RA widths are 100 ms and 200 ms. The default width tolerance remains 15 ms. `main-red` requires both physical inputs HIGH together; V/I remains LOW for an unpaired input, falls when either input falls, and status completion requires valid widths from the same overlap attempt.

SA1 and SA2 status are independent and honor their respective deployment permissions. The shared RA status rises only after both RA1 and RA2 milestones complete and both deployment permissions are `yes`.

SA milestone timeout is 300 seconds, RA milestone timeout is 500 seconds, and both simulator runs time out after 500 seconds. Startup, manual/automatic stow, signal termination, normal exit, and timeout cleanup drive only owned outputs LOW and verify the RP2040 output latch. `--leave-feedback` preserves the existing status-hold behavior.

The electrical assumptions are unchanged: active-HIGH command and feedback signals, command GPn mapped to V GPn and I GP(n+8), and the corrected status harness mapping listed above.
