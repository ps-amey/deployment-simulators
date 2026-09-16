# SARA Deployment Simulator Test Results

## SA simulator

| ID | Test performed | Observed result | Result |
|---|---|---|---|
| SA-T01 | SA1 deployment disabled; all SA1 pulses observed sequentially; SA2 not attempted | Sequential SA1 pulses and I/V feedback observed; no deployment status asserted | PASS |
| SA-T02 | SA1 deployment through main pulse (`SA1=yes`) | SA1 deployed after valid main pulse | PASS |
| SA-T03 | SA2 deployment through main pulse (`SA2=yes`) | SA2 deployed after valid main pulse | PASS |
| SA-T04 | SA1 deployment through redundant pulse (`SA1=yes`) | SA1 deployed after valid redundant pulse | PASS |
| SA-T05 | SA2 deployment through redundant pulse (`SA2=yes`) | SA2 deployed after valid redundant pulse | PASS |
| SA-T06 | SA1 deployment through extended main pulse (`SA1=yes`, 100 ms) | SA1 deployed after valid extended-main pulse | PASS |
| SA-T07 | SA2 deployment through extended main pulse (`SA2=yes`, 100 ms) | SA2 deployed after valid extended-main pulse | PASS |
| SA-T08 | SA1 deployment through extended redundant pulse (`SA1=yes`, 100 ms) | SA1 deployed after valid extended-redundant pulse | PASS |
| SA-T09 | SA2 deployment through extended redundant pulse (`SA2=yes`, 100 ms) | SA2 deployed after valid extended-redundant pulse | PASS |
| SA-T10 | SA1 deployment with TC sending 50 ms pulses on main and redundant lines (`SA1=yes`) | SA1 deployed after both valid 50 ms pulses | PASS |
| SA-T11 | SA2 deployment with TC sending 50 ms pulses on main and redundant lines (`SA2=yes`) | SA2 deployed after both valid 50 ms pulses | PASS |
| SA-T12 | SA1 deployment with TC sending 100 ms pulses on main and redundant lines (`SA1=yes`) | SA1 deployed after both valid 100 ms pulses | PASS |
| SA-T13 | SA2 deployment with TC sending 100 ms pulses on main and redundant lines (`SA2=yes`) | SA2 deployed after both valid 100 ms pulses | PASS |
| SA-T14 | SA1 status enabled and SA2 status disabled (`SA1=yes`, `SA2=no`) | SA1 deployment status asserted; SA2 shown not deployed as expected | PASS |

For SA tests, `PASS` means the observed deployment-status behavior matched the configured permission and expected pulse qualification. V/I feedback may still be visible when deployment permission is disabled, but the deployment-status output must remain LOW.

## RA simulator

The RA simulator has no deployment-status output. These tests therefore verify command-pulse detection, overlap qualification where applicable, pulse-width reporting, and V/I feedback only.

| ID | Test performed | Pulse condition | Observed result | Result |
|---|---|---|---|---|
| RA-T01 | Individual pulse on each RA line | 50 ms pulses on GP4, GP5, GP6, and GP7 | Each line detected independently; corresponding V/I feedback observed | PASS |
| RA-T02 | Individual pulse on each RA line | 100 ms pulses on GP4, GP5, GP6, and GP7 | Each line detected independently; corresponding V/I feedback and widths observed | PASS |
| RA-T03 | Simultaneous main and redundant pulses | 50 ms pulses on each main-red pair | Main-red overlap detected; paired V/I feedback observed | PASS |
| RA-T04 | Simultaneous main and redundant pulses | 100 ms pulses on each main-red pair | Extended main-red overlap detected; paired V/I feedback and widths observed | PASS |

RA results confirm I/V behavior only; they do not constitute confirmation of a deployment-status assertion.
