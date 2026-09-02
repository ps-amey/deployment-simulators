#!/usr/bin/env python3
"""Laptop-side SA/RA deployment emulator for MicroPython HIL Picos.

The command Pico observes OBC deployment outputs on GP0..GP7 using GPIO edge
interrupts. The external-ADC Pico reproduces selected voltage/current pulses,
and the feedback Pico optionally drives the corresponding deployment status.
"""

from __future__ import annotations

import argparse
import ast
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
from typing import Callable, Iterable, Mapping, Sequence


DEFAULT_COMMAND_PICO = "/dev/piersight-hil/inspace-obc-pico-hil-1/obc-do"
DEFAULT_EXT_ADC_PICO = "/dev/piersight-hil/inspace-obc-pico-hil-1/obc-ext-adc"
DEFAULT_FEEDBACK_PICO = "/dev/piersight-hil/inspace-obc-pico-hil-1/obc-di"

WIDTH_TOLERANCE_MS = 15.0

PULSE_MODE_PATHS = {
    "main": ("main",),
    "red": ("red",),
    "main-red": ("main", "red"),
    "ex-main": ("main",),
    "ex-red": ("red",),
}
EXTENDED_PULSE_MODES = {"ex-main", "ex-red"}

EDGE_MARKER = "SARA_EDGE="
EDGE_OVERFLOW_MARKER = "SARA_EDGE_OVERFLOW="
LOW_VERIFY_MARKER = "SARA_LOW_VERIFY="


@dataclass(frozen=True)
class CommandSignal:
    unit: str
    status_group: str
    path: str
    signal: str
    voltage_pin: int
    current_pin: int


@dataclass(frozen=True)
class FeedbackSignal:
    pin: int
    obc_label: str
    signal: str


@dataclass(frozen=True)
class SimulatorProfile:
    name: str
    units: tuple[str, ...]
    command_signals: Mapping[int, CommandSignal]
    feedback_signals: Mapping[str, FeedbackSignal]
    deployment_path_pins: Mapping[str, Mapping[str, int]]
    status_requirements: Mapping[str, tuple[str, ...]]
    normal_width_ms: float
    extended_width_ms: float
    milestone_timeout_s: float
    simulator_timeout_s: float
    default_log: Path

    @property
    def command_pins(self) -> tuple[int, ...]:
        return tuple(sorted(self.command_signals))

    @property
    def ext_adc_pins(self) -> tuple[int, ...]:
        pins: set[int] = set()
        for command in self.command_signals.values():
            pins.add(command.voltage_pin)
            pins.add(command.current_pin)
        return tuple(sorted(pins))


@dataclass(frozen=True)
class Pulse:
    pin: int
    width_us: int

    @property
    def width_ms(self) -> float:
        return self.width_us / 1000.0


@dataclass(frozen=True)
class EdgeSnapshot:
    state_mask: int
    changed_mask: int
    timestamp_us: int
    falling_pulses: tuple[Pulse, ...]


class RawRepl:
    """Small, dependency-free MicroPython raw-REPL transport."""

    def __init__(self, port: str, timeout_s: float = 3.0) -> None:
        self.port = port
        self.timeout_s = float(timeout_s)
        self._fd: int | None = None
        self._raw_mode = False
        self._program_running = False
        self._pending = b""

    def __enter__(self) -> "RawRepl":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def open(self) -> None:
        if self._fd is not None:
            return
        self._fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attributes = termios.tcgetattr(self._fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] |= termios.CLOCAL | termios.CREAD
        attributes[3] = 0
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self._fd, termios.TCSANOW, attributes)
        termios.tcflush(self._fd, termios.TCIOFLUSH)
        self.interrupt()

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._raw_mode = False
        self._program_running = False
        self._pending = b""

    def interrupt(self) -> None:
        self._write(b"\x03\x03")
        self._read_for(0.2)

    def run(self, code: str, *, timeout_s: float | None = None) -> bytes:
        if self._fd is None:
            raise RuntimeError("raw REPL port is not open")
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        self.interrupt()
        self._write(b"\x01")
        entered = self._read_until(
            lambda data: b"raw REPL" in data or data.rstrip().endswith(b">"),
            self.timeout_s,
        )
        if b"raw REPL" not in entered and not entered.rstrip().endswith(b">"):
            raise TimeoutError(f"timed out entering raw REPL on {self.port}")
        try:
            self._write(code.encode("ascii") + b"\x04")
            response = self._read_until(
                lambda data: data.endswith(b">") and b"\x04" in data,
                timeout,
            )
            if not (response.endswith(b">") and b"\x04" in response):
                raise TimeoutError(f"timed out waiting for raw REPL on {self.port}")
            if b"Traceback" in response or b"SyntaxError" in response:
                raise RuntimeError(response.decode("utf-8", errors="replace"))
            return response
        except BaseException:
            # Stop a still-running capture before releasing the serial port.
            self._write(b"\x03")
            self._read_for(0.2)
            raise
        finally:
            # Return to friendly REPL. Nothing is saved on the Pico filesystem.
            self._write(b"\x02")
            self._read_for(0.2)

    def enter_raw(self) -> None:
        """Enter raw REPL once for a sequence of low-latency operations."""
        if self._fd is None:
            raise RuntimeError("raw REPL port is not open")
        if self._raw_mode:
            return
        self.interrupt()
        self._write(b"\x01")
        entered = self._read_until(
            lambda data: b"raw REPL" in data or data.rstrip().endswith(b">"),
            self.timeout_s,
        )
        if b"raw REPL" not in entered and not entered.rstrip().endswith(b">"):
            raise TimeoutError(f"timed out entering raw REPL on {self.port}")
        self._raw_mode = True

    def execute_raw(self, code: str, *, timeout_s: float | None = None) -> bytes:
        """Execute code while staying at the raw prompt between operations."""
        if not self._raw_mode:
            raise RuntimeError("raw REPL mode is not active")
        if self._program_running:
            raise RuntimeError("a raw REPL program is already running")
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        self._write(code.encode("ascii") + b"\x04")
        response = self._read_until(
            lambda data: data.endswith(b">") and data.count(b"\x04") >= 2,
            timeout,
        )
        if not (response.endswith(b">") and response.count(b"\x04") >= 2):
            raise TimeoutError(f"timed out waiting for raw REPL on {self.port}")
        if b"Traceback" in response or b"SyntaxError" in response:
            raise RuntimeError(response.decode("utf-8", errors="replace"))
        return response

    def start_raw_program(self, code: str) -> None:
        """Start a long-running raw-REPL program and retain its streamed output."""
        if not self._raw_mode:
            raise RuntimeError("raw REPL mode is not active")
        if self._program_running:
            raise RuntimeError("a raw REPL program is already running")
        self._write(code.encode("ascii") + b"\x04")
        response = self._read_until(lambda data: b"OK" in data, self.timeout_s)
        if b"OK" not in response:
            raise TimeoutError(f"Pico did not acknowledge raw program on {self.port}")
        before, _, after = response.partition(b"OK")
        if b"Traceback" in before or b"SyntaxError" in before:
            raise RuntimeError(response.decode("utf-8", errors="replace"))
        self._pending += after
        self._program_running = True

    def read_stream(self, timeout_s: float = 1.0) -> bytes:
        """Read currently available output from a long-running raw program."""
        if not self._program_running:
            raise RuntimeError("no raw REPL program is running")
        response = self._pending
        self._pending = b""
        if self._fd is None:
            raise RuntimeError("raw REPL port is not open")
        ready, _, _ = select.select([self._fd], [], [], max(0.0, timeout_s))
        if ready:
            while True:
                try:
                    chunk = os.read(self._fd, 4096)
                except BlockingIOError:
                    break
                if not chunk:
                    break
                response += chunk
                ready, _, _ = select.select([self._fd], [], [], 0)
                if not ready:
                    break
        if b"Traceback" in response or b"SyntaxError" in response:
            raise RuntimeError(response.decode("utf-8", errors="replace"))
        return response

    def stop_raw_program(self) -> None:
        if not self._program_running:
            return
        self._write(b"\x03")
        self._read_until(lambda data: data.rstrip().endswith(b">"), self.timeout_s)
        self._program_running = False

    def exit_raw(self) -> None:
        if not self._raw_mode:
            return
        self.stop_raw_program()
        self._write(b"\x02")
        self._read_for(0.2)
        self._raw_mode = False

    def _write(self, payload: bytes) -> None:
        if self._fd is None:
            raise RuntimeError("raw REPL port is not open")
        os.write(self._fd, payload)

    def _read_for(self, seconds: float) -> bytes:
        if self._fd is None:
            raise RuntimeError("raw REPL port is not open")
        deadline = time.monotonic() + seconds
        response = b""
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self._fd], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                response += chunk
        return response

    def _read_until(self, done: Callable[[bytes], bool], timeout_s: float) -> bytes:
        deadline = time.monotonic() + timeout_s
        response = b""
        while time.monotonic() < deadline:
            if self._fd is None:
                raise RuntimeError("raw REPL port is not open")
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([self._fd], [], [], min(0.05, remaining))
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                response += chunk
                if done(response):
                    break
        return response


def build_edge_stream_code(pins: Sequence[int], *, queue_size: int = 64) -> str:
    """Build a temporary interrupt-driven Pico program that streams edges."""
    pin_tuple = tuple(int(pin) for pin in pins)
    pin_mask = sum(1 << pin for pin in pin_tuple)
    queue_size = max(4, int(queue_size))
    return "\n".join(
        [
            "from machine import Pin, mem32, idle",
            "from time import ticks_us, ticks_diff",
            "import micropython",
            "micropython.alloc_emergency_exception_buf(100)",
            f"ids = {pin_tuple!r}",
            f"input_mask = {pin_mask}",
            "inputs = [Pin(pin_id, Pin.IN) for pin_id in ids]",
            f"queue_states = [0] * {queue_size}",
            f"queue_changes = [0] * {queue_size}",
            f"queue_times = [0] * {queue_size}",
            "head = 0",
            "tail = 0",
            "dropped = 0",
            "last_state = mem32[0xD0000004] & input_mask",
            "now = ticks_us()",
            "rise_times = [now if last_state & (1 << pin_id) else None for pin_id in ids]",
            "if last_state:",
            "    queue_states[head] = last_state",
            "    queue_changes[head] = last_state",
            "    queue_times[head] = now",
            "    head = (head + 1) % len(queue_states)",
            "def edge_handler(_pin):",
            "    global head, last_state, dropped",
            "    state = mem32[0xD0000004] & input_mask",
            "    changed = state ^ last_state",
            "    if not changed:",
            "        return",
            "    last_state = state",
            "    next_head = (head + 1) % len(queue_states)",
            "    if next_head == tail:",
            "        dropped += 1",
            "        return",
            "    queue_states[head] = state",
            "    queue_changes[head] = changed",
            "    queue_times[head] = ticks_us()",
            "    head = next_head",
            "for input_pin in inputs:",
            "    input_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=edge_handler)",
            "print('SARA_EDGE_READY=' + repr(last_state))",
            "while True:",
            "    if tail == head:",
            "        idle()",
            "        continue",
            "    state = queue_states[tail]",
            "    changed = queue_changes[tail]",
            "    timestamp = queue_times[tail]",
            "    tail = (tail + 1) % len(queue_states)",
            "    falling = []",
            "    for index in range(len(ids)):",
            "        bit = 1 << ids[index]",
            "        if not changed & bit:",
            "            continue",
            "        if state & bit:",
            "            rise_times[index] = timestamp",
            "        elif rise_times[index] is not None:",
            "            falling.append((ids[index], ticks_diff(timestamp, rise_times[index])))",
            "            rise_times[index] = None",
            f"    print({EDGE_MARKER!r} + repr((state, changed, timestamp, falling)))",
            "    if dropped:",
            f"        print({EDGE_OVERFLOW_MARKER!r} + repr(dropped))",
            "        dropped = 0",
        ]
    )


def build_ext_adc_init_code(pins: Sequence[int]) -> str:
    pin_tuple = tuple(int(pin) for pin in pins)
    return "\n".join(
        [
            "from machine import Pin, mem32",
            f"pins = {pin_tuple!r}",
            "for pin_id in pins:",
            "    Pin(pin_id, Pin.OUT).value(0)",
            "sio = mem32",
            "gpio_set = 0xD0000014",
            "gpio_clear = 0xD0000018",
        ]
    )


def build_ext_adc_update_code(previous_mask: int, new_mask: int) -> str:
    changed = int(previous_mask) ^ int(new_mask)
    set_mask = changed & int(new_mask)
    clear_mask = changed & int(previous_mask)
    commands = []
    if clear_mask:
        commands.append(f"sio[gpio_clear] = {clear_mask}")
    if set_mask:
        commands.append(f"sio[gpio_set] = {set_mask}")
    return "\n".join(commands)


def build_feedback_code(
    profile: SimulatorProfile, states: Mapping[str, bool]
) -> str:
    values = tuple(
        (feedback.pin, 1 if states.get(name, False) else 0)
        for name, feedback in profile.feedback_signals.items()
    )
    return "\n".join(
        [
            "from machine import Pin",
            f"values = {values!r}",
            "for pin_id, value in values:",
            "    Pin(pin_id, Pin.OUT).value(value)",
            "print(\x27SARA_FEEDBACK=\x27 + repr(values))",
        ]
    )


def build_low_verify_code(pins: Sequence[int]) -> str:
    """Drive output pins LOW and report the RP2040 output-latch mask."""
    pin_tuple = tuple(int(pin) for pin in pins)
    pin_mask = sum(1 << pin for pin in pin_tuple)
    return "\n".join(
        [
            "from machine import Pin, mem32",
            f"pins = {pin_tuple!r}",
            "for pin_id in pins:",
            "    Pin(pin_id, Pin.OUT).value(0)",
            f"observed = mem32[0xD0000010] & {pin_mask}",
            f"print({LOW_VERIFY_MARKER!r} + repr(observed))",
        ]
    )


def drive_low_and_verify(
    role: str,
    pico: RawRepl,
    pins: Sequence[int],
    *,
    attempts: int = 2,
) -> None:
    """Drive LOW, verify the Pico output latch, and retry when necessary."""
    failures: list[str] = []
    for attempt in range(1, max(1, int(attempts)) + 1):
        try:
            pico.enter_raw()
            response = pico.execute_raw(build_low_verify_code(pins))
            marker = LOW_VERIFY_MARKER.encode("ascii")
            if marker not in response:
                raise RuntimeError("LOW verification marker missing")
            value_text = response.rsplit(marker, 1)[1].splitlines()[0]
            value_text = value_text.strip(b"\r\x04> ")
            observed_mask = ast.literal_eval(value_text.decode("ascii"))
            if type(observed_mask) is not int:
                raise RuntimeError(f"invalid LOW verification value {observed_mask!r}")
            if observed_mask == 0:
                return
            raise RuntimeError(f"output latch remained HIGH: mask=0x{observed_mask:x}")
        except BaseException as error:
            failures.append(f"attempt {attempt}: {error}")
    raise RuntimeError(
        f"{role} failed LOW verification after {len(failures)} attempts "
        f"({'; '.join(failures)})"
    )


def parse_edge_payload(profile: SimulatorProfile, payload: str) -> EdgeSnapshot:
    raw = ast.literal_eval(payload.strip())
    if not isinstance(raw, tuple) or len(raw) != 4:
        raise RuntimeError(f"invalid edge snapshot: {raw!r}")
    state_mask, changed_mask, timestamp_us, falling_raw = raw
    if (
        type(state_mask) is not int
        or type(changed_mask) is not int
        or type(timestamp_us) is not int
        or not 0 <= state_mask <= 0xFF
        or not 0 <= changed_mask <= 0xFF
        or timestamp_us < 0
        or not isinstance(falling_raw, list)
    ):
        raise RuntimeError(f"invalid edge snapshot: {raw!r}")
    falling: list[Pulse] = []
    for entry in falling_raw:
        if (
            not isinstance(entry, tuple)
            or len(entry) != 2
            or type(entry[0]) is not int
            or type(entry[1]) is not int
            or entry[0] not in profile.command_signals
            or entry[1] < 0
        ):
            raise RuntimeError(f"invalid falling-edge pulse: {entry!r}")
        falling.append(Pulse(entry[0], entry[1]))
    return EdgeSnapshot(state_mask, changed_mask, timestamp_us, tuple(falling))


def ext_adc_mask_for_state(
    profile: SimulatorProfile,
    command_state: int,
    *,
    voltage_enabled: bool,
    current_enabled: bool,
) -> int:
    mask = 0
    for pin, command in profile.command_signals.items():
        if command_state & (1 << pin):
            if voltage_enabled:
                mask |= 1 << command.voltage_pin
            if current_enabled:
                mask |= 1 << command.current_pin
    return mask


def expected_width_for_command(
    profile: SimulatorProfile,
    pin: int,
    deployment_modes: dict[str, str] | None = None,
) -> float:
    command = profile.command_signals[pin]
    mode = deployment_modes[command.unit] if deployment_modes is not None else "main"
    return (
        profile.extended_width_ms
        if mode in EXTENDED_PULSE_MODES
        else profile.normal_width_ms
    )


def apply_pulses(
    profile: SimulatorProfile,
    states: dict[str, bool],
    pulses: Iterable[Pulse],
    *,
    strict_width: bool,
    width_tolerance_ms: float,
    deployment_status_enabled: bool = True,
    deployment_enabled_by_unit: dict[str, bool] | None = None,
    deployment_modes: dict[str, str] | None = None,
    pulse_attempt_ids: dict[int, int] | None = None,
    deployment_gate: (Callable[[str, str, int | None], tuple[bool, dict[str, bool]]] | None) = None,
) -> tuple[dict[str, bool], list[dict[str, object]]]:
    new_states = dict(states)
    events: list[dict[str, object]] = []
    for pulse in pulses:
        command = profile.command_signals[pulse.pin]
        configured_mode = deployment_modes[command.unit] if deployment_modes is not None else "main"
        expected_ms = expected_width_for_command(profile, pulse.pin, deployment_modes)
        error_ms = pulse.width_ms - expected_ms
        width_valid = abs(error_ms) <= width_tolerance_ms
        accepted = width_valid or not strict_width
        milestones: dict[str, bool] = {}
        if accepted and deployment_gate is not None:
            condition_met, milestones = deployment_gate(
                command.unit,
                command.path,
                pulse_attempt_ids.get(pulse.pin) if pulse_attempt_ids else None,
            )
        else:
            condition_met = accepted
        unit_enabled = (
            deployment_enabled_by_unit[command.unit]
            if deployment_enabled_by_unit is not None
            else deployment_status_enabled
        )
        required_units = profile.status_requirements[command.status_group]
        status_enabled = (
            all(deployment_enabled_by_unit[unit] for unit in required_units)
            if deployment_enabled_by_unit is not None
            else deployment_status_enabled
        )
        feedback_accepted = accepted and status_enabled and condition_met
        if feedback_accepted:
            new_states[command.status_group] = True
        events.append({
            "deployable": command.status_group,
            "deployment_unit": command.unit,
            "path": command.path,
            "configured_mode": configured_mode,
            "signal": command.signal,
            "command_pin": pulse.pin,
            "width_us": pulse.width_us,
            "width_ms": round(pulse.width_ms, 3),
            "nearest_expected_ms": expected_ms,
            "width_error_ms": round(error_ms, 3),
            "width_valid": width_valid,
            "pulse_accepted": accepted,
            "deployment_enabled": unit_enabled,
            "deployment_status_enabled": status_enabled,
            "deployment_enable_states": dict(deployment_enabled_by_unit) if deployment_enabled_by_unit is not None else {command.unit: unit_enabled},
            "deployment_condition_met": condition_met,
            "deployment_milestones": milestones,
            "feedback_accepted": feedback_accepted,
            "feedback_pin": profile.feedback_signals[command.status_group].pin,
        })
    return new_states, events


def configured_deployment_modes(
    profile: SimulatorProfile, args: argparse.Namespace
) -> dict[str, str]:
    return {unit: getattr(args, f"{unit.lower()}_mode") for unit in profile.units}


def configured_deployment_enables(
    profile: SimulatorProfile, args: argparse.Namespace
) -> dict[str, bool]:
    return {
        unit: getattr(args, f"{unit.lower()}_deployment") == "yes"
        for unit in profile.units
    }


def deployment_unit_for_command(profile: SimulatorProfile, pin: int) -> str:
    return profile.command_signals[pin].unit


def selected_command_pins(
    profile: SimulatorProfile, deployment_modes: dict[str, str]
) -> tuple[int, ...]:
    pins = [
        profile.deployment_path_pins[unit][path]
        for unit, mode in deployment_modes.items()
        for path in PULSE_MODE_PATHS[mode]
    ]
    return tuple(sorted(pins))


def configured_main_red_pairs(
    profile: SimulatorProfile, deployment_modes: dict[str, str]
) -> dict[str, tuple[int, int]]:
    return {
        unit: (
            profile.deployment_path_pins[unit]["main"],
            profile.deployment_path_pins[unit]["red"],
        )
        for unit, mode in deployment_modes.items()
        if mode == "main-red"
    }


def qualified_command_state(
    profile: SimulatorProfile,
    command_state: int,
    deployment_modes: dict[str, str],
) -> int:
    qualified = 0
    for unit, mode in deployment_modes.items():
        selected_bits = tuple(
            1 << profile.deployment_path_pins[unit][path]
            for path in PULSE_MODE_PATHS[mode]
        )
        if mode == "main-red":
            pair_mask = selected_bits[0] | selected_bits[1]
            if command_state & pair_mask == pair_mask:
                qualified |= pair_mask
        elif command_state & selected_bits[0]:
            qualified |= selected_bits[0]
    return qualified


def build_deployment_gate(
    profile: SimulatorProfile,
    deployment_modes: dict[str, str],
    *,
    monotonic_clock: Callable[[], float] = time.monotonic,
) -> Callable[[str, str, int | None], tuple[bool, dict[str, bool]]]:
    accepted_paths = {unit: set() for unit in profile.units}
    milestone_started_at: dict[str, float | None] = {
        group: None for group in profile.status_requirements
    }
    latest_attempt_by_unit: dict[str, int] = {}

    def required_paths(unit: str) -> set[str]:
        return {
            profile.command_signals[profile.deployment_path_pins[unit][path]].path
            for path in PULSE_MODE_PATHS[deployment_modes[unit]]
        }

    def milestones_met(
        unit: str, path: str, attempt_id: int | None
    ) -> tuple[bool, dict[str, bool]]:
        command = next(
            command
            for command in profile.command_signals.values()
            if command.unit == unit and command.path == path
        )
        group = command.status_group
        now = monotonic_clock()
        started_at = milestone_started_at[group]
        if started_at is not None and now - started_at >= profile.milestone_timeout_s:
            for required_unit in profile.status_requirements[group]:
                accepted_paths[required_unit].clear()
            milestone_started_at[group] = None
        if attempt_id is not None and latest_attempt_by_unit.get(unit) != attempt_id:
            accepted_paths[unit].clear()
            latest_attempt_by_unit[unit] = attempt_id
        if milestone_started_at[group] is None:
            milestone_started_at[group] = now
        accepted_paths[unit].add(path)
        milestones = {
            required_unit: required_paths(required_unit) <= accepted_paths[required_unit]
            for required_unit in profile.status_requirements[group]
        }
        return all(milestones.values()), milestones

    return milestones_met


def validate_required_pico_paths(required_devices: dict[str, str]) -> None:
    """Fail before opening serial ports when a required alias is unavailable."""
    missing = [
        f"{role}: {device}"
        for role, device in required_devices.items()
        if not os.path.exists(device)
    ]
    if missing:
        details = "\n  ".join(missing)
        raise RuntimeError(
            "Pico preflight failed; required device alias(es) missing:\n  "
            f"{details}\nNo GPIO outputs were changed and pulse monitoring was not started."
        )


class EventLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields: object) -> None:
        payload = {
            "time": datetime.now(timezone.utc).isoformat(),
            "event": event,
            **fields,
        }
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, sort_keys=True) + "\n")


def print_mapping(profile: SimulatorProfile) -> None:
    print("Command Pico (OBC outputs):")
    for pin, command in profile.command_signals.items():
        print(f"  GP{pin:<2} {command.signal:<20} {command.path}")
    print("Feedback Pico (to OBC inputs):")
    for name, feedback in profile.feedback_signals.items():
        print(f"  GP{feedback.pin:<2} {feedback.obc_label:<10} {feedback.signal} ({name})")
    print("External ADC Pico (V/I pulse feedback):")
    for pin, command in profile.command_signals.items():
        mapping = command
        print(
            f"  {command.signal:<20} {command.path:<11} V=GP{mapping.voltage_pin:<2} "
            f"I=GP{mapping.current_pin}"
        )


def parse_dry_run_pulses(profile: SimulatorProfile, value: str) -> list[Pulse]:
    pulses: list[Pulse] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        pin_text, separator, width_text = item.partition(":")
        if not separator:
            raise argparse.ArgumentTypeError("dry-run pulses must use PIN:WIDTH_MS")
        pin_text = pin_text.upper().removeprefix("GP")
        try:
            pin = int(pin_text)
            width_us = round(float(width_text) * 1000)
        except ValueError as error:
            raise argparse.ArgumentTypeError(f"invalid dry-run pulse {item!r}") from error
        if pin not in profile.command_signals or width_us < 0:
            raise argparse.ArgumentTypeError(f"invalid dry-run pulse {item!r}")
        pulses.append(Pulse(pin, width_us))
    return pulses


def add_deployment_mode_group(
    parser: argparse.ArgumentParser, deployment: str
) -> None:
    prefix = deployment.lower()
    destination = f"{prefix}_mode"
    group = parser.add_mutually_exclusive_group(required=True)
    for suffix, mode, help_text in (
        ("main", "main", "accept the main deployment path"),
        ("red", "red", "accept the redundant deployment path"),
        ("main-red", "main-red", "require both main and redundant paths"),
        ("ex-main", "ex-main", "accept the extended main path (reserved)"),
        ("ex-red", "ex-red", "accept the extended redundant path (reserved)"),
    ):
        group.add_argument(
            f"--{prefix}-{suffix}",
            dest=destination,
            action="store_const",
            const=mode,
            help=f"{deployment}: {help_text}",
        )


def build_parser(profile: SimulatorProfile) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=profile.name)
    parser.add_argument(
        "--command-pico",
        default=DEFAULT_COMMAND_PICO,
        help=f"obc_do Pico serial device (default: {DEFAULT_COMMAND_PICO})",
    )
    parser.add_argument(
        "--feedback-pico",
        default=DEFAULT_FEEDBACK_PICO,
        help=f"obc_di status Pico serial device (default: {DEFAULT_FEEDBACK_PICO})",
    )
    parser.add_argument(
        "--ext-adc-pico",
        default=DEFAULT_EXT_ADC_PICO,
        help=f"obc_ext_adc Pico serial device (default: {DEFAULT_EXT_ADC_PICO})",
    )
    parser.add_argument("--min-pulse-ms", type=float, default=1.0, help="ignore shorter completed pulses for deployment status (default: 1)")
    parser.add_argument("--strict-width", action="store_true", help="only assert feedback for pulses matching an expected width")
    parser.add_argument("--width-tolerance-ms", type=float, default=WIDTH_TOLERANCE_MS, help=f"deployment width tolerance (default: {WIDTH_TOLERANCE_MS:g})")
    parser.add_argument("--log", type=Path, default=profile.default_log)
    parser.add_argument("--leave-feedback", action="store_true", help="do not force all feedback low when the program exits")
    parser.add_argument("--stow-feedback", action="store_true", help="drive all feedback LOW using only the feedback Pico, then exit")
    parser.add_argument("--stow-after-seconds", type=float, default=0.0, help="automatically drive all feedback LOW this many seconds after an accepted deployment (default: disabled)")
    parser.add_argument("--once", action="store_true", help="exit after the first completed non-glitch pulse")
    parser.add_argument("--show-mapping", action="store_true")
    for deployment in profile.units:
        add_deployment_mode_group(parser, deployment)
    for deployment in profile.units:
        parser.add_argument(
            f"--{deployment.lower()}-deployment",
            required=True,
            choices=("yes", "no"),
            help=f"allow {deployment} to contribute to deployment-status output",
        )
    parser.add_argument(
        "--v-ch-feedback",
        required=True,
        choices=("yes", "no"),
        help=f"reproduce selected command pulses on obc_ext_adc voltage pins {tuple(command.voltage_pin for command in profile.command_signals.values())}",
    )
    parser.add_argument(
        "--i-ch-feedback",
        required=True,
        choices=("yes", "no"),
        help=f"reproduce selected command pulses on obc_ext_adc current pins {tuple(command.current_pin for command in profile.command_signals.values())}",
    )
    parser.add_argument("--dry-run-pulses", type=lambda value: parse_dry_run_pulses(profile, value), metavar=f"GP{profile.command_pins[0]}:{profile.normal_width_ms:g},GP{profile.command_pins[1]}:{profile.normal_width_ms:g}", help="process synthetic pulses without opening Picos")
    return parser


def validate_args(profile: SimulatorProfile, parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    for name in ("min_pulse_ms", "width_tolerance_ms"):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.stow_after_seconds < 0:
        parser.error("--stow-after-seconds must be non-negative")
    selected_modes = sum(
        (
            args.dry_run_pulses is not None,
            args.stow_feedback,
        )
    )
    if selected_modes > 1:
        parser.error(
            "--dry-run-pulses and --stow-feedback are mutually exclusive"
        )
    if args.stow_feedback and not args.feedback_pico:
        parser.error("--feedback-pico is required with --stow-feedback")
    special_mode = args.dry_run_pulses is not None or args.stow_feedback
    if not special_mode and not args.command_pico:
        parser.error("--command-pico is required for pulse capture")
    if not special_mode and not args.feedback_pico:
        parser.error(
            "--feedback-pico is required to initialize all deployment statuses LOW"
        )
    if (
        not special_mode
        and (args.v_ch_feedback == "yes" or args.i_ch_feedback == "yes")
        and not args.ext_adc_pico
    ):
        parser.error(
            "--ext-adc-pico is required when V or I channel feedback is yes"
        )
    devices = [
        device
        for device in (args.command_pico, args.feedback_pico, args.ext_adc_pico)
        if device
    ]
    real_devices = [os.path.realpath(device) for device in devices]
    if len(real_devices) != len(set(real_devices)):
        parser.error("command, feedback and external-ADC Pico devices must be different")


def report_events(events: Sequence[dict[str, object]]) -> None:
    for event in events:
        unit = event["deployment_unit"]
        path = str(event["path"])
        configured_mode = str(event["configured_mode"])
        if configured_mode == "ex-main":
            line_name = "EXTENDED MAIN"
        elif configured_mode == "ex-red":
            line_name = "EXTENDED REDUNDANT"
        else:
            line_name = "REDUNDANT" if path in ("red", "redundant") else "MAIN"
        if not event["width_valid"]:
            print(
                f"PULSE RECEIVED ON {unit} {line_name} LINE, WIDTH INVALID: "
                f"{event['width_ms']:.3f} ms; expected near "
                f"{event['nearest_expected_ms']:.1f} ms"
            )
            continue
        if not event["deployment_enabled"]:
            print(
                f"VALID PULSE RECEIVED ON {unit} {line_name} LINE, "
                "DEPLOYMENT NOT COMMANDED DUE TO TEST SCENARIO"
            )
            continue

        print(f"VALID PULSE RECEIVED ON {unit} {line_name} LINE")
        if not event["deployment_status_enabled"]:
            disabled_ra = [
                name
                for name in ("RA1", "RA2")
                if not event["deployment_enable_states"].get(name, False)
            ]
            print(
                "RA DEPLOYMENT NOT COMMANDED DUE TO TEST SCENARIO "
                f"({', '.join(disabled_ra)} DEPLOYMENT=NO)"
            )
        elif not event["deployment_condition_met"]:
            milestones = event["deployment_milestones"]
            if event["deployable"] == "RA" and milestones:
                milestone_text = ", ".join(
                    f"{name}={'COMPLETE' if complete else 'PENDING'}"
                    for name, complete in milestones.items()
                )
                print(f"RA MILESTONES: {milestone_text}; RA STATUS REMAINS LOW")
            else:
                print(f"{unit} MILESTONE INCOMPLETE; WAITING FOR OTHER PATH")
        else:
            if event["deployable"] == "RA":
                print("RA1 AND RA2 COMPLETE; RA DEPLOYMENT STATUS COMMANDED")
            else:
                print(f"{unit} DEPLOYMENT STATUS COMMANDED")


def run_dry(profile: SimulatorProfile, args: argparse.Namespace, event_log: EventLog) -> int:
    states = {name: False for name in profile.feedback_signals}
    deployment_modes = configured_deployment_modes(profile, args)
    deployment_enables = configured_deployment_enables(profile, args)
    deployment_gate = build_deployment_gate(profile, deployment_modes)
    selected_pins = set(selected_command_pins(profile, deployment_modes))
    selected_pulses = [
        pulse for pulse in args.dry_run_pulses if pulse.pin in selected_pins
    ]
    ignored_pulses = [
        pulse for pulse in args.dry_run_pulses if pulse.pin not in selected_pins
    ]
    event_log.write(
        "configuration",
        deployment_modes=deployment_modes,
        deployment_enables=deployment_enables,
        v_ch_feedback=args.v_ch_feedback,
        i_ch_feedback=args.i_ch_feedback,
        dry_run=True,
    )
    for pulse in ignored_pulses:
        print(
            f"Ignored GP{pulse.pin} ({profile.command_signals[pulse.pin].signal}): not in "
            f"configured deployment paths {deployment_modes}"
        )
        event_log.write(
            "pulse_ignored",
            command_pin=pulse.pin,
            signal=profile.command_signals[pulse.pin].signal,
            reason="unselected_command_path",
        )
    states, events = apply_pulses(
        profile, states,
        selected_pulses,
        strict_width=args.strict_width,
        width_tolerance_ms=args.width_tolerance_ms,
        deployment_enabled_by_unit=deployment_enables,
        deployment_modes=deployment_modes,
        deployment_gate=deployment_gate,
    )
    report_events(events)
    for event in events:
        event_log.write("pulse", **event)
    event_log.write("feedback", states=states, dry_run=True)
    print(f"Dry-run feedback states: {states}")
    return 0


def run_stow_feedback(profile: SimulatorProfile, args: argparse.Namespace, event_log: EventLog) -> int:
    with RawRepl(args.feedback_pico) as feedback_pico:
        feedback_pico.enter_raw()
        try:
            drive_low_and_verify(
                "status Pico",
                feedback_pico,
                tuple(feedback.pin for feedback in profile.feedback_signals.values()),
            )
        finally:
            feedback_pico.exit_raw()
    event_log.write(
        "manual_stow_verified",
        feedback_pico=args.feedback_pico,
        states={name: False for name in profile.feedback_signals},
    )
    print(
        f"{', '.join(profile.feedback_signals)} feedback is STOWED/LOW and Pico latch verified. "
        "No OBC reset was performed."
    )
    return 0


def cleanup_hardware_picos(
    profile: SimulatorProfile,
    args: argparse.Namespace,
    event_log: EventLog,
    command_pico: RawRepl,
    feedback_pico: RawRepl,
    ext_adc_pico: RawRepl | None,
) -> None:
    """Best-effort independent cleanup with verified LOW output latches."""
    failures: list[str] = []

    def log_cleanup(event: str, **fields: object) -> None:
        try:
            event_log.write(event, **fields)
        except BaseException as error:
            print(f"WARNING: cleanup event log failed: {error}", file=sys.stderr)

    if not args.leave_feedback:
        try:
            drive_low_and_verify(
                "status Pico",
                feedback_pico,
                tuple(feedback.pin for feedback in profile.feedback_signals.values()),
            )
            log_cleanup(
                "stopped_safe_verified",
                states={name: False for name in profile.feedback_signals},
            )
            print("Feedback returned to STOWED/LOW and Pico latch verified.")
        except BaseException as error:
            failures.append(f"status Pico ({feedback_pico.port}): {error}")
    else:
        log_cleanup("stopped_feedback_held")
    try:
        feedback_pico.exit_raw()
    except BaseException as error:
        failures.append(f"status Pico raw-REPL exit ({feedback_pico.port}): {error}")

    if ext_adc_pico is not None:
        try:
            drive_low_and_verify(
                "external-ADC Pico",
                ext_adc_pico,
                profile.ext_adc_pins,
            )
            log_cleanup("ext_adc_stopped_safe_verified", output_mask=0)
            print("External ADC V/I feedback returned LOW and Pico latch verified.")
        except BaseException as error:
            failures.append(f"external-ADC Pico ({ext_adc_pico.port}): {error}")
        try:
            ext_adc_pico.exit_raw()
        except BaseException as error:
            failures.append(
                f"external-ADC Pico raw-REPL exit ({ext_adc_pico.port}): {error}"
            )

    try:
        command_pico.stop_raw_program()
    except BaseException as error:
        failures.append(f"command Pico capture stop ({command_pico.port}): {error}")
    try:
        command_pico.exit_raw()
    except BaseException as error:
        failures.append(f"command Pico raw-REPL exit ({command_pico.port}): {error}")

    if failures:
        raise RuntimeError("Pico cleanup failed: " + " | ".join(failures))


def run_hardware(profile: SimulatorProfile, args: argparse.Namespace, event_log: EventLog) -> int:
    stop = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True
        # Abort a long Pico-side capture immediately. The surrounding finally
        # block then restores all feedback outputs to their safe LOW state.
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    states = {name: False for name in profile.feedback_signals}
    deployment_modes = configured_deployment_modes(profile, args)
    deployment_enables = configured_deployment_enables(profile, args)
    capture_pins = selected_command_pins(profile, deployment_modes)
    voltage_enabled = args.v_ch_feedback == "yes"
    current_enabled = args.i_ch_feedback == "yes"
    ext_adc_enabled = voltage_enabled or current_enabled
    enforce_width = True
    deployment_gate = build_deployment_gate(profile, deployment_modes)
    edge_code = build_edge_stream_code(capture_pins)
    main_red_pairs = configured_main_red_pairs(profile, deployment_modes)
    main_red_attempt_counter = {deployment: 0 for deployment in main_red_pairs}
    main_red_attempt_by_pin: dict[int, int] = {}
    active_main_red_units: set[str] = set()
    last_ext_adc_mask = 0
    stream_buffer = b""
    auto_stow_deadline: float | None = None
    simulator_deadline: float | None = None
    required_devices = {
        "command Pico": args.command_pico,
        "status Pico": args.feedback_pico,
    }
    if ext_adc_enabled:
        required_devices["external-ADC Pico"] = args.ext_adc_pico
    validate_required_pico_paths(required_devices)

    with ExitStack() as stack:
        print(f"Opening command Pico: {args.command_pico}")
        command_pico = stack.enter_context(RawRepl(args.command_pico))
        print("Command Pico serial port opened.")
        print(f"Opening status Pico: {args.feedback_pico}")
        feedback_pico = stack.enter_context(RawRepl(args.feedback_pico))
        print("Status Pico serial port opened.")
        if ext_adc_enabled:
            print(f"Opening external-ADC Pico: {args.ext_adc_pico}")
        ext_adc_pico = (
            stack.enter_context(RawRepl(args.ext_adc_pico))
            if ext_adc_enabled
            else None
        )
        if ext_adc_pico is not None:
            print("External-ADC Pico serial port opened.")
        stack.callback(
            cleanup_hardware_picos,
            profile,
            args,
            event_log,
            command_pico,
            feedback_pico,
            ext_adc_pico,
        )
        print("All required Pico ports opened; safe-output cleanup armed.")
        command_pico.enter_raw()
        feedback_pico.enter_raw()
        if ext_adc_pico is not None:
            ext_adc_pico.enter_raw()

        # Establish a known-safe status state before V/I setup or edge capture,
        # even when every deployment output is disabled by the test scenario.
        drive_low_and_verify(
            "status Pico",
            feedback_pico,
            tuple(feedback.pin for feedback in profile.feedback_signals.values()),
        )
        if ext_adc_pico is not None:
            ext_adc_pico.execute_raw(build_ext_adc_init_code(profile.ext_adc_pins))
            drive_low_and_verify(
                "external-ADC Pico",
                ext_adc_pico,
                profile.ext_adc_pins,
            )
        command_pico.start_raw_program(edge_code)
        simulator_deadline = time.monotonic() + profile.simulator_timeout_s
        event_log.write(
            "started",
            command_pico=args.command_pico,
            feedback_pico=args.feedback_pico,
            ext_adc_pico=args.ext_adc_pico,
            deployment_modes=deployment_modes,
            deployment_enables=deployment_enables,
            v_ch_feedback=args.v_ch_feedback,
            i_ch_feedback=args.i_ch_feedback,
            capture_pins=capture_pins,
            capture_method="gpio_irq_state_mask",
            simulator_timeout_s=profile.simulator_timeout_s,
            strict_width=enforce_width,
            states=states,
        )
        print(
            f"Running deployment modes {deployment_modes} on "
            f"{', '.join('GP' + str(pin) for pin in capture_pins)}; "
            f"V feedback={args.v_ch_feedback}, I feedback={args.i_ch_feedback}, "
            f"deployment enables={deployment_enables}."
        )
        try:
            while not stop:
                assert simulator_deadline is not None
                remaining_s = simulator_deadline - time.monotonic()
                if remaining_s <= 0:
                    print(
                        f"SIMULATOR TIMER EXCEEDED ({profile.simulator_timeout_s:g} s); "
                        "BEGINNING CLEANUP"
                    )
                    event_log.write(
                        "simulator_timeout",
                        timeout_s=profile.simulator_timeout_s,
                        states=states,
                    )
                    stop = True
                    break
                stream_buffer += command_pico.read_stream(
                    timeout_s=min(0.1, remaining_s)
                )
                while b"\n" in stream_buffer:
                    raw_line, stream_buffer = stream_buffer.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip("\r\x04> ")
                    if EDGE_OVERFLOW_MARKER in line:
                        dropped_text = line.split(EDGE_OVERFLOW_MARKER, 1)[1]
                        dropped = int(dropped_text.strip())
                        event_log.write("edge_queue_overflow", dropped=dropped)
                        print(f"WARNING: command Pico dropped {dropped} edge snapshot(s)")
                        continue
                    if EDGE_MARKER not in line:
                        continue
                    snapshot = parse_edge_payload(profile, line.split(EDGE_MARKER, 1)[1])
                    host_received_ns = time.monotonic_ns()

                    newly_active_main_red: list[str] = []
                    for deployment, pair in main_red_pairs.items():
                        pair_mask = (1 << pair[0]) | (1 << pair[1])
                        both_high = snapshot.state_mask & pair_mask == pair_mask
                        if both_high:
                            if deployment not in active_main_red_units:
                                active_main_red_units.add(deployment)
                                main_red_attempt_counter[deployment] += 1
                                attempt_id = main_red_attempt_counter[deployment]
                                for pin in pair:
                                    main_red_attempt_by_pin[pin] = attempt_id
                                newly_active_main_red.append(deployment)
                        else:
                            active_main_red_units.discard(deployment)

                    qualified_state = qualified_command_state(
                        profile, snapshot.state_mask, deployment_modes
                    )
                    desired_ext_adc_mask = ext_adc_mask_for_state(
                        profile, qualified_state,
                        voltage_enabled=voltage_enabled,
                        current_enabled=current_enabled,
                    )
                    ext_update_ms: float | None = None
                    if (
                        ext_adc_pico is not None
                        and desired_ext_adc_mask != last_ext_adc_mask
                    ):
                        update_code = build_ext_adc_update_code(
                            last_ext_adc_mask, desired_ext_adc_mask
                        )
                        update_started_ns = time.monotonic_ns()
                        ext_adc_pico.execute_raw(update_code)
                        ext_update_ms = (
                            time.monotonic_ns() - update_started_ns
                        ) / 1_000_000.0
                        last_ext_adc_mask = desired_ext_adc_mask

                    edge_records: list[dict[str, object]] = []
                    for pin in capture_pins:
                        bit = 1 << pin
                        if not snapshot.changed_mask & bit:
                            continue
                        level = "HIGH" if snapshot.state_mask & bit else "LOW"
                        mapping = profile.command_signals[pin]
                        voltage_level = (
                            "HIGH"
                            if desired_ext_adc_mask & (1 << mapping.voltage_pin)
                            else "LOW"
                        )
                        current_level = (
                            "HIGH"
                            if desired_ext_adc_mask & (1 << mapping.current_pin)
                            else "LOW"
                        )
                        edge_records.append(
                            {
                                "command_pin": pin,
                                "signal": mapping.signal,
                                "level": level,
                                "voltage_pin": mapping.voltage_pin,
                                "current_pin": mapping.current_pin,
                                "voltage_level": voltage_level,
                                "current_level": current_level,
                                "pico_timestamp_us": snapshot.timestamp_us,
                                "command_state_mask": snapshot.state_mask,
                                "changed_mask": snapshot.changed_mask,
                                "ext_adc_mask": desired_ext_adc_mask,
                                "ext_adc_update_ms": (
                                round(ext_update_ms, 3)
                                if ext_update_ms is not None
                                else None
                                ),
                                "host_received_monotonic_ns": host_received_ns,
                            }
                        )

                    overlap_pulses: list[Pulse] = []
                    unpaired_main_red_pulses: list[Pulse] = []
                    pulse_attempt_ids: dict[int, int] = {}
                    completed_main_red_attempts: set[str] = set()
                    for pulse in snapshot.falling_pulses:
                        deployment_unit = deployment_unit_for_command(profile, pulse.pin)
                        if deployment_modes[deployment_unit] != "main-red":
                            overlap_pulses.append(pulse)
                        elif pulse.pin in main_red_attempt_by_pin:
                            overlap_pulses.append(pulse)
                            pulse_attempt_ids[pulse.pin] = main_red_attempt_by_pin.pop(
                                pulse.pin
                            )
                            attempt_id = pulse_attempt_ids[pulse.pin]
                            if not any(
                                main_red_attempt_by_pin.get(pair_pin) == attempt_id
                                for pair_pin in main_red_pairs[deployment_unit]
                            ):
                                completed_main_red_attempts.add(deployment_unit)
                        else:
                            unpaired_main_red_pulses.append(pulse)
                    pulses = [
                        pulse
                        for pulse in overlap_pulses
                        if pulse.width_ms >= args.min_pulse_ms
                    ]
                    glitches = [
                        pulse
                        for pulse in overlap_pulses
                        if pulse.width_ms < args.min_pulse_ms
                    ]
                    events: list[dict[str, object]] = []
                    feedback_changed = False
                    if pulses:
                        previous_states = dict(states)
                        states, events = apply_pulses(
                            profile, states,
                            pulses,
                            strict_width=enforce_width,
                            width_tolerance_ms=args.width_tolerance_ms,
                            deployment_enabled_by_unit=deployment_enables,
                            deployment_modes=deployment_modes,
                            pulse_attempt_ids=pulse_attempt_ids,
                            deployment_gate=deployment_gate,
                        )
                        feedback_changed = states != previous_states
                        if feedback_pico is not None and feedback_changed:
                            # Hardware status is updated before any reporting I/O.
                            feedback_pico.execute_raw(build_feedback_code(profile, states))

                    # Reporting is intentionally after V/I and deployment-status updates.
                    for deployment in newly_active_main_red:
                        enabled_feedback = "/".join(
                            name
                            for name, enabled in (
                                ("V", voltage_enabled),
                                ("I", current_enabled),
                            )
                            if enabled
                        )
                        feedback_message = (
                            f"{enabled_feedback} FEEDBACK COMMANDED"
                            if enabled_feedback
                            else "V/I FEEDBACK DISABLED BY TEST SCENARIO"
                        )
                        print(
                            f"{deployment} MAIN+REDUNDANT HIGH DETECTED; "
                            f"{feedback_message}"
                        )
                        event_log.write(
                            "main_red_overlap_detected",
                            deployment_unit=deployment,
                            command_pins=main_red_pairs[deployment],
                            ext_adc_mask=desired_ext_adc_mask,
                        )
                    for edge in edge_records:
                        print(
                            f"Edge {edge['signal']} GP{edge['command_pin']} "
                            f"{edge['level']}; V=GP{edge['voltage_pin']} "
                            f"({edge['voltage_level'] if voltage_enabled else 'disabled'}), "
                            f"I=GP{edge['current_pin']} "
                            f"({edge['current_level'] if current_enabled else 'disabled'})"
                        )
                        event_log.write(
                            "edge",
                            **{
                                **edge,
                                "level": str(edge["level"]).lower(),
                            },
                        )
                    for pulse in glitches:
                        event_log.write(
                            "pulse_glitch_ignored",
                            command_pin=pulse.pin,
                            width_us=pulse.width_us,
                            minimum_ms=args.min_pulse_ms,
                        )
                    for pulse in unpaired_main_red_pulses:
                        deployment_unit = deployment_unit_for_command(profile, pulse.pin)
                        print(
                            f"IGNORED {deployment_unit} "
                            f"{profile.command_signals[pulse.pin].path.upper()} PULSE: "
                            "MAIN AND REDUNDANT WERE NOT HIGH TOGETHER"
                        )
                        event_log.write(
                            "main_red_pulse_ignored",
                            deployment_unit=deployment_unit,
                            command_pin=pulse.pin,
                            width_us=pulse.width_us,
                            reason="no_main_red_high_overlap",
                        )
                    report_events(events)
                    for event in events:
                        event_log.write("pulse", **event)
                    if feedback_changed:
                        event_log.write("feedback", states=states)
                    if args.stow_after_seconds > 0 and any(
                        event["feedback_accepted"] for event in events
                    ):
                        auto_stow_deadline = (
                            time.monotonic() + args.stow_after_seconds
                        )
                        print(
                            f"Holding deployed feedback for "
                            f"{args.stow_after_seconds:g} seconds..."
                        )
                    completed_single_path = any(
                        deployment_modes[deployment_unit_for_command(profile, pulse.pin)]
                        != "main-red"
                        for pulse in pulses
                    )
                    if args.once and (
                        completed_single_path or completed_main_red_attempts
                    ):
                        stop = True
                        break

                if (
                    auto_stow_deadline is not None
                    and time.monotonic() >= auto_stow_deadline
                ):
                    assert feedback_pico is not None
                    drive_low_and_verify(
                        "status Pico",
                        feedback_pico,
                        tuple(feedback.pin for feedback in profile.feedback_signals.values()),
                    )
                    states = {name: False for name in profile.feedback_signals}
                    event_log.write(
                        "automatic_stow_verified",
                        hold_seconds=args.stow_after_seconds,
                        states=states,
                    )
                    print(
                        "Feedback returned to STOWED/LOW and Pico latch verified; "
                        "OBC reset remains external."
                    )
                    auto_stow_deadline = None
        except KeyboardInterrupt:
            print("Stopping deployment simulator...")
    return 0


def run_cli(profile: SimulatorProfile, argv: Sequence[str] | None = None) -> int:
    parser = build_parser(profile)
    args = parser.parse_args(argv)
    if args.show_mapping:
        print_mapping(profile)
        if (
            not args.command_pico
            and args.dry_run_pulses is None
            and not args.stow_feedback
        ):
            return 0
    validate_args(profile, parser, args)
    event_log = EventLog(args.log)
    if args.stow_feedback:
        return run_stow_feedback(profile, args, event_log)
    if args.dry_run_pulses is not None:
        return run_dry(profile, args, event_log)
    return run_hardware(profile, args, event_log)


def main(profile: SimulatorProfile, argv: Sequence[str] | None = None) -> int:
    try:
        return run_cli(profile, argv)
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
