# Separated SA and RA Simulators

The split simulators share `deployment_sim_core.py` but have independent entry points and documentation:

- [SA simulator instructions and tests](SA/README.md)
- [RA V/I-only simulator instructions and tests](RA/README.md)

Run only one simulator at a time because both use the same physical serial aliases. The original combined simulator remains available in `../SARA_Deployment_sim/` when SA and RA must operate together.
