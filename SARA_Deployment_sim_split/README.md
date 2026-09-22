# Separated SA and RA Simulators

The split simulators share `deployment_sim_core.py` but have independent entry points and documentation:

- [SA simulator instructions and tests](SA/README.md)
- [RA V/I-only simulator instructions and tests](RA/README.md)

Run only one simulator at a time because both use the same physical serial aliases. The original combined simulator remains available in `../SARA_Deployment_sim/` when SA and RA must operate together.

## Runtime timeout

The split simulators use a 200-second whole-simulator runtime by default. Override it with `--simulator-timeout`:

```bash
# Run until Ctrl+C
python3 SARA_Deployment_sim_split/ra_deployment_sim.py ... --simulator-timeout none
```

If the option is omitted, the timeout remains the default configured in the simulator profile.
