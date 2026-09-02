#!/usr/bin/env python3
from pathlib import Path

from deployment_sim_core import CommandSignal, FeedbackSignal, SimulatorProfile, main

RA_PROFILE = SimulatorProfile(
    name="RA deployment simulator",
    units=("RA1", "RA2"),
    command_signals={
        4: CommandSignal("RA1", "RA", "main", "RA_HDRM_MAIN_1", 4, 12),
        5: CommandSignal("RA2", "RA", "main", "RA_HDRM_MAIN_2", 5, 13),
        6: CommandSignal("RA1", "RA", "redundant", "RA_HDRM_RED_1", 6, 14),
        7: CommandSignal("RA2", "RA", "redundant", "RA_HDRM_RED_2", 7, 15),
    },
    feedback_signals={
        "RA": FeedbackSignal(21, "OB_DTM_19", "RA_HDRM_STATUS"),
    },
    deployment_path_pins={
        "RA1": {"main": 4, "red": 6},
        "RA2": {"main": 5, "red": 7},
    },
    status_requirements={"RA": ("RA1", "RA2")},
    normal_width_ms=100.0,
    extended_width_ms=200.0,
    milestone_timeout_s=500.0,
    simulator_timeout_s=500.0,
    default_log=Path("ra_deployment_events.jsonl"),
)

if __name__ == "__main__":
    raise SystemExit(main(RA_PROFILE))
