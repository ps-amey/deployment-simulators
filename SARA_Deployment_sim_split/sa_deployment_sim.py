#!/usr/bin/env python3
from pathlib import Path

from deployment_sim_core import CommandSignal, FeedbackSignal, SimulatorProfile, main

SA_PROFILE = SimulatorProfile(
    name="SA deployment simulator",
    units=("SA1", "SA2"),
    command_signals={
        0: CommandSignal("SA1", "SA1", "main", "SA1_HDRM_MAIN", 0, 8),
        1: CommandSignal("SA2", "SA2", "main", "SA2_HDRM_MAIN", 1, 9),
        2: CommandSignal("SA1", "SA1", "redundant", "SA1_HDRM_REDN", 2, 10),
        3: CommandSignal("SA2", "SA2", "redundant", "SA2_HDRM_REDN", 3, 11),
    },
    feedback_signals={
        "SA1": FeedbackSignal(26, "OB_DTM_17", "SA1_HDRM_STATUS"),
        "SA2": FeedbackSignal(20, "OB_DTM_18", "SA2_HDRM_STATUS"),
    },
    deployment_path_pins={
        "SA1": {"main": 0, "red": 2},
        "SA2": {"main": 1, "red": 3},
    },
    status_requirements={"SA1": ("SA1",), "SA2": ("SA2",)},
    normal_width_ms=50.0,
    extended_width_ms=80.0,
    milestone_timeout_s=300.0,
    simulator_timeout_s=500.0,
    default_log=Path("sa_deployment_events.jsonl"),
)

if __name__ == "__main__":
    raise SystemExit(main(SA_PROFILE))
