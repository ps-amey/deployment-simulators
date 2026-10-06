# =============================================================================
#  uhfAntDeploymentSimulator.py
#
#  UHF antenna deployment simulator for the Raspberry Pi Pico (RP2040), running
#  MicroPython. The Pico acts as an I2C *slave/target* exposing a single 8-bit
#  register (ANT_REGISTER) that a flight-computer "master" reads and writes to
#  drive/monitor a simulated 4-antenna, 2-thermal-cutter deployment sequence.
#
#  ---------------------------------------------------------------------------
#  ANT_REGISTER (1 byte)
#    bit0  FB1  RO   ANT1 status   0 = deployed, 1 = stored
#    bit1  FB2  RO   ANT2 status   0 = deployed, 1 = stored
#    bit2  FB3  RO   ANT3 status   0 = deployed, 1 = stored
#    bit3  FB4  RO   ANT4 status   0 = deployed, 1 = stored
#    bit4  TC1  R/W  Thermal cutter 1   1 = ON, 0 = OFF
#    bit5  TC2  R/W  Thermal cutter 2   1 = ON, 0 = OFF
#    bit6  ---  --   unused (always 0)
#    bit7  ---  --   unused (always 0, flight-like)
#
#  Master interaction (single-register device, no register-pointer byte):
#    - Master WRITE of 1 byte  -> command; only TC bits (0x30) are applied.
#    - Master READ  of 1 byte  -> current telemetry (freshly assembled).
#  Examples:  write 0x10 -> TC1 on;  write 0x20 -> TC2 on;  write 0x30 -> both.
#  Staged sequential reads include:
#       0x0F: all antennas stored
#       0x1F: all stored, TC1 commanded
#       0x1C: ANT1/ANT2 deployed, ANT3/ANT4 stored, TC1 commanded
#       0x2C: first pair deployed, TC2 commanded
#       0x20: all four deployed, TC2 commanded
#
#  ---------------------------------------------------------------------------
#  ONE TARGET ON PICO I2C0 / SELECTABLE MAIN OR REDUNDANT ADDRESS
#  The RP2040 I2C hardware block matches exactly ONE slave address (single
#  IC_SAR register); it has no native multi-address support. Every session
#  therefore exposes one target on I2C0 (GP20/GP21); I2C1 stays disabled:
#
#       UHF testNN : 0x45 (main) or 0x46 (redundant)
#       AIS aisNN  : 0x47 (main) or 0x48 (redundant)
#
#  Scenarios that start on a redundant address leave main absent, so the
#  OBC's main attempt is NACKed before it retries through redundant. Failover
#  and shared-bus scenarios re-address I2C0 mid-session on an idle bus.
#
#  AIS has a single thermal cutter (TC1). TC1 deploys the antenna bits in
#  AIS_TC1_DEPLOY_MASK; there is no AIS TC2 and power never deploys AIS.
#
#  ---------------------------------------------------------------------------
#  Register-level slave driver adapted from kyordhel/i2cslave (MIT License,
#  Mauricio Matamoros). Verified against the RP2040 datasheet register map.
# =============================================================================

import gc
import json
import machine
import select
import time
import sys

# =============================================================================
#  CONFIGURATION  --  edit this block for your wiring and test scenario
# =============================================================================

# --- Which slave implementation to use -------------------------------------
#   "register"  : low-level RP2040 register driver. Version-independent, fully
#                 verified, works on stock MicroPython. RECOMMENDED / default.
#   "i2ctarget" : uses machine.I2CTarget (MicroPython v1.24+ only). Cleaner but
#                 the API must match your firmware -- verify before relying on it.
DRIVER = "register"

# --- Addressing -------------------------------------------------------------
# Every session uses Pico I2C0 only; I2C1 is never enabled.
PRIMARY_I2C_ID   = 0         # the single target hardware block
PRIMARY_MAIN_ADDRESS = 0x45       # UHF DCS1/main
PRIMARY_REDUNDANT_ADDRESS = 0x46  # UHF DCS2/backup
PRIMARY_SDA      = 20        # GP20 (I2C0 SDA pins: 0,4,8,12,16,20)
PRIMARY_SCL      = 21        # GP21 (I2C0 SCL pins: 1,5,9,13,17,21)

AIS_MAIN_ADDRESS = 0x47           # AIS DCS1/main
AIS_REDUNDANT_ADDRESS = 0x48      # AIS DCS2/backup

# --- AIS antenna model -----------------------------------------------------
# AIS has a single thermal cutter (TC1). Once TC1 has been commanded for
# AIS_TC1_DEPLOY_DELAY_S (with power present), the feedback bits in
# AIS_TC1_DEPLOY_MASK change from stored (1) to deployed (0). The mask covers
# only the antenna feedback bits 0-3; TC and NC bits are unaffected.
# The OBC treats AIS as deployed only when all four bits 0-3 read 0
# (AISMgr: status & 0x0F == 0), so this mask must match the AIS success mask
# in the OBC firmware. Keep 0x0F until the TC1-only AIS firmware says otherwise.
AIS_TC1_DEPLOY_MASK = 0x0F
AIS_TC1_DEPLOY_DELAY_S = 5   # TBD: replace with the agreed AIS TC1 burn time
# ais05 deliberately deploys only part of the bits to exercise the OBC's
# incomplete-deployment handling.
AIS_PARTIAL_DEPLOY_MASK = 0x03

# shared_i2c_deployment uses only Pico I2C0. It starts as UHF main (0x45)
# and, after serving the final UHF deployed status, re-addresses the same
# hardware block as AIS main (0x47). The harness must put both OBC transactions
# on this physical bus; software cannot bridge two electrically separate buses.
#
# shared_i2c_deployment_redundant is the same handoff, but both boards answer
# on their redundant addresses instead (UHF 0x46, then AIS 0x48).
SHARED_UHF_ADDRESS = PRIMARY_MAIN_ADDRESS
SHARED_AIS_ADDRESS = AIS_MAIN_ADDRESS
SHARED_UHF_REDUNDANT_ADDRESS = PRIMARY_REDUNDANT_ADDRESS
SHARED_AIS_REDUNDANT_ADDRESS = AIS_REDUNDANT_ADDRESS
SHARED_HANDOFF_DELAY_MS = 20

# Shared mode keeps UHF stored on power alone, so UHF proceeds through its
# TC1/TC2 fallback. After the handoff AIS follows the TC1-only AIS model.
SHARED_UHF_POWER_DEPLOY_SUCCESS = False

# tc2_failover_main_to_red (test20) uses only Pico I2C0. TC1 is accepted
# and deploys on the main address. TC2 is then ACKed on main and shown in the
# register, but never deploys. Once that TC2 write has been received and the
# bus has been idle for FAILOVER_SWITCH_DELAY_MS, the same hardware block is
# re-addressed to the redundant address (UHF 0x45 -> 0x46).
# The OBC polls about once a second, so the switch lands on an idle bus; its
# status reads then fail on main and fall back to redundant. After the OBC's
# TC2 timeout it re-sends TC2 to redundant, which deploys ANT3/ANT4.
FAILOVER_SWITCH_DELAY_MS = 20

# main_lost_after_tc1 (test21) models the main controller disappearing
# mid-deployment. TC1 deploys on main as normal. Once the OBC has read the TC1
# pair deployed, and the bus has been idle for FAILOVER_SWITCH_DELAY_MS, I2C0
# moves to redundant before the OBC's next tick. The OBC's TC2 write to main is
# therefore NACKed (it never reaches the Pico) and the OBC retries TC2 on
# redundant on the next tick, which deploys ANT3/ANT4. A TC2 that does reach
# main means the switch was late; the report flags it as unexpected.

INTERNAL_PULLUPS =True
# Bus data rate is set by the MASTER (target 100 kHz). A slave does not drive
# the clock, so there is nothing to configure here; just run the master at 100k.

# --- ANT_POWER digital input ------------------------------------------------
DI_PIN            = 15       # GPIO used to sense antenna power
DI_ACTIVE_HIGH    = True     # True: pin HIGH  -> ANT_POWER_ON
                             #       pin LOW   -> ANT_POWER_LOW
DI_PULL           = "down"     # "down" | "up" | None  (internal pull on DI pin)

# --- Board profile ----------------------------------------------------------
#   The same simulator serves two boards with the same register map:
#     "UHF" (testNN): antennas may deploy on power-on or on TC1 then TC2.
#     "AIS" (aisNN) : one cutter only; antennas deploy on TC1 alone, never on
#                     power and never on TC2.
#   The profile is chosen by the command prefix; UHF scenarios are rejected
#   under the AIS profile and AIS scenarios under the UHF profile.
BOARD_PROFILE = "UHF"        # default when a request names no profile

# Configure one test session over USB CDC before enabling I2C.
USB_SCENARIO_CONTROL = True
DEFAULT_SESSION_DURATION_S = 400
# A test command may carry its own duration: "test01 500" runs 500 s and
# "test01 none" runs until a runtime "end"/"abort". Numeric durations are
# capped so ticks_diff() (valid for ~6.2 days of ms ticks) stays correct.
MAX_SESSION_DURATION_S = 500000
USB_COMMAND_MAX_BYTES = 256

# Runtime USB commands accepted while a session is running (status, end,
# abort, testNN/aisNN). stdin is polled without blocking, one character
# per service-loop pass. A completed line is executed right after the next
# served I2C read (the OBC reads ~1/s, so the bus is then idle), or after
# RUNTIME_COMMAND_DEFER_MS if no read arrives.
RUNTIME_COMMANDS = True
RUNTIME_COMMAND_DEFER_MS = 1200

# Does a thermal cutter need ANT_POWER present to fire? A real burn-wire does,
# so deployment is gated on power by default. Set False only if your test bench
# asserts the TC bits without driving the ANT_POWER (DI) pin.
TC_REQUIRES_POWER = True

# --- Deployment scenario ----------------------------------------------------
#   POWER_ON_ALL deploys all four antennas after power remains present.
#
#   SEQUENTIAL_TC models the OBC deployment sequence:
#     1. TC1 must remain commanded for tc1_delay_s, then ANT1/ANT2 deploy.
#     2. The simulator waits for TC2.
#     3. TC2 must remain commanded for tc2_delay_s, then ANT3/ANT4 deploy.
#
#   The TC1/TC2 values remain configurable because the flight timings are TBD.
#   There is deliberately no TC1_AND_TC2 scenario. A simultaneous 0x30 command
#   can still progress through the same two stages; it is not a separate mode.
#
#   redundant_deploy uses the exact same sequential behavior but binds the
#   targets to backup addresses 0x46 and 0x48 instead of 0x45 and 0x47.
#
#   PAIR_TEST provides focused partial-deployment and failure scenarios. Each
#   scenario independently selects which pair, if any, deploys on power-on;
#   which cutter commands are accepted; and which accepted commands actually
#   deploy their associated antenna pair. Every PAIR_TEST starts with all four
#   feedback bits in the stored state.
TC1_DEPLOY_DELAY_S = 5       # TBD: replace with the agreed TC1 deployment time
TC2_DEPLOY_DELAY_S = 5       # TBD: replace with the agreed TC2 deployment time
PAIR_POWER_DEPLOY_DELAY_S = 5

SCENARIOS = {
    "test01_power_on": {
        "mode": "POWER_ON_ALL", "delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "address_set": "main"
    },
    "test02_sequential_deploy": {
        "mode": "SEQUENTIAL_TC",
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    "test03_no_deploy": {
        "mode": "NO_DEPLOY", "address_set": "main"
    },
    # Only TC1 is accepted. ANT1/ANT2 deploy after the TC1 delay; ANT3/ANT4
    # remain stored regardless of TC2 writes.
    "test04__power_no_deploy_then_only_tc1": {
        "mode": "PAIR_TEST",
        "power_pair": None,
        "accept_tc1": True,
        "accept_tc2": False,
        "deploy_on_tc1": True,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Only TC2 is accepted. ANT3/ANT4 deploy after the TC2 delay; ANT1/ANT2
    # remain stored regardless of TC1 writes.
    "test05_power_no_deploy_then_only_tc2": {
        "mode": "PAIR_TEST",
        "power_pair": None,
        "accept_tc1": False,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": True,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Power deploys the TC1 pair first. An accepted TC2 command then deploys
    # the TC2 pair.
    "test06_power_tc1_then_tc2_deploy": {
        "mode": "PAIR_TEST",
        "power_pair": "TC1",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": True,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": True,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Power deploys the TC1 pair first. TC2 is accepted and reported in the
    # register, but the TC2 pair deliberately remains stored.
    "test07_power_tc1_then_tc2_no_deploy": {
        "mode": "PAIR_TEST",
        "power_pair": "TC1",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": True,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Power deploys the TC2 pair first. An accepted TC1 command then deploys
    # the TC1 pair.
    "test08_power_tc2_then_tc1_deploy": {
        "mode": "PAIR_TEST",
        "power_pair": "TC2",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": True,
        "accept_tc2": True,
        "deploy_on_tc1": True,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Power deploys the TC2 pair first. TC1 is accepted and reported in the
    # register, but the TC1 pair deliberately remains stored.
    "test09_power_tc2_then_tc1_no_deploy": {
        "mode": "PAIR_TEST",
        "power_pair": "TC2",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": True,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main",
    },
    # Power-on deploys the TC2 pair on redundant addresses. Only TC1 is
    # accepted afterward and deploys ANT1/ANT2.
    "test10_power_tc2_deploy_then_tc1_at_red": {
        "mode": "PAIR_TEST",
        "power_pair": "TC2",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": True,
        "accept_tc2": False,
        "deploy_on_tc1": True,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    # Power-on deploys the TC1 pair on redundant addresses. Only TC2 is
    # accepted afterward and deploys ANT3/ANT4.
    "test11_power_tc1_deploy_then_tc2_at_red": {
        "mode": "PAIR_TEST",
        "power_pair": "TC1",
        "power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "accept_tc1": False,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": True,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    # Redundant addresses only. TC1 deploys ANT1/ANT2 and TC2 then deploys
    # ANT3/ANT4 using the normal sequential state machine.
    "test12_redundant_tc1_tc2_deploy": {
        "mode": "SEQUENTIAL_TC",
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    # Both cutter commands are accepted on the redundant addresses, but only
    # the TC1 pair deploys. TC2 remains visible in the register but has no
    # deployment effect.
    "test13_redundant_tc1_only_tc2_ignored": {
        "mode": "PAIR_TEST",
        "power_pair": None,
        "accept_tc1": True,
        "accept_tc2": True,
        "deploy_on_tc1": True,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    # TC1 is rejected on the redundant addresses. TC2 is accepted and deploys
    # only ANT3/ANT4.
    "test14_redundant_tc1_ignored_tc2_deploy": {
        "mode": "PAIR_TEST",
        "power_pair": None,
        "accept_tc1": False,
        "accept_tc2": True,
        "deploy_on_tc1": False,
        "deploy_on_tc2": True,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    # Both cutter commands are rejected on the redundant addresses and all
    # four antennas remain stored.
    "test15_redundant_ignore_all": {
        "mode": "PAIR_TEST",
        "power_pair": None,
        "accept_tc1": False,
        "accept_tc2": False,
        "deploy_on_tc1": False,
        "deploy_on_tc2": False,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    "redundant_deploy": {
        "mode": "SEQUENTIAL_TC",
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "redundant",
    },
    "shared_i2c_deployment": {
        "mode": "SHARED_I2C_DEPLOYMENT",
        "uhf_power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "shared",
    },
    # Same UHF->AIS handoff as shared_i2c_deployment, but both boards answer
    # on their redundant addresses (UHF 0x46, then AIS 0x48) instead of main.
    "shared_i2c_deployment_redundant": {
        "mode": "SHARED_I2C_DEPLOYMENT",
        "uhf_power_delay_s": PAIR_POWER_DEPLOY_DELAY_S,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "shared_redundant",
    },
    # TC1 deploys on main. TC2 is ACKed on main but does not deploy; the
    # target then moves to the redundant address, where the OBC's timed-out
    # TC2 retry deploys ANT3/ANT4.
    "tc2_failover_main_to_red": {
        "mode": "TC2_FAILOVER",
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main_to_red",
    },
    # TC1 deploys on main, then main disappears: the OBC's TC2 write to main
    # is NACKed and its immediate TC2 retry on redundant deploys ANT3/ANT4.
    "main_lost_after_tc1": {
        "mode": "MAIN_LOST_FAILOVER",
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "main_to_red",
    },
    # --- AIS (single cutter, TC1 only) --------------------------------------
    #   start        : address I2C0 starts on ("main" 0x47 or "red" 0x48)
    #   main_tc1     : what main does with TC1 ("deploy" or "ignore")
    #   red_tc1      : what redundant does with TC1 ("deploy" or "ignore")
    #   switch_after_main_tc1 : move to redundant once TC1 has reached main
    #                  and the bus is idle (the OBC then times out on main and
    #                  retries TC1 on redundant)
    #   deploy_mask  : feedback bits TC1 clears; None = AIS_TC1_DEPLOY_MASK
    #   expect       : what the report checks the run against
    # TC1 on main deploys.
    "ais01_tc1_main_deploy": {
        "mode": "AIS_TC1", "address_set": "ais", "start": "main",
        "main_tc1": "deploy", "red_tc1": "deploy",
        "switch_after_main_tc1": False, "deploy_mask": None,
        "tc1_delay_s": AIS_TC1_DEPLOY_DELAY_S,
        "expect": {"tc1_main": True, "tc1_red": False, "switch": False,
                   "final": "deployed"},
    },
    # TC1 is ACKed on main but never deploys; I2C0 then moves to redundant,
    # where the OBC's timed-out TC1 retry deploys.
    "ais02_tc1_main_ignored_red_deploy": {
        "mode": "AIS_TC1", "address_set": "ais", "start": "main",
        "main_tc1": "ignore", "red_tc1": "deploy",
        "switch_after_main_tc1": True, "deploy_mask": None,
        "tc1_delay_s": AIS_TC1_DEPLOY_DELAY_S,
        "expect": {"tc1_main": True, "tc1_red": True, "switch": True,
                   "final": "deployed"},
    },
    # Only redundant is present: TC1 to main is NACKed, TC1 on redundant
    # deploys.
    "ais03_redundant_only_tc1_deploy": {
        "mode": "AIS_TC1", "address_set": "ais", "start": "red",
        "main_tc1": "deploy", "red_tc1": "deploy",
        "switch_after_main_tc1": False, "deploy_mask": None,
        "tc1_delay_s": AIS_TC1_DEPLOY_DELAY_S,
        "expect": {"tc1_main": False, "tc1_red": True, "switch": False,
                   "final": "deployed"},
    },
    # TC1 is ignored on main and on redundant: the OBC times out on both and
    # reports a deployment failure.
    "ais04_tc1_ignored_both": {
        "mode": "AIS_TC1", "address_set": "ais", "start": "main",
        "main_tc1": "ignore", "red_tc1": "ignore",
        "switch_after_main_tc1": True, "deploy_mask": None,
        "tc1_delay_s": AIS_TC1_DEPLOY_DELAY_S,
        "expect": {"tc1_main": True, "tc1_red": True, "switch": True,
                   "final": "stored"},
    },
    # TC1 on main deploys only AIS_PARTIAL_DEPLOY_MASK, so deployment never
    # completes and the OBC reports a failure.
    "ais05_tc1_partial_deploy": {
        "mode": "AIS_TC1", "address_set": "ais", "start": "main",
        "main_tc1": "deploy", "red_tc1": "deploy",
        "switch_after_main_tc1": False, "deploy_mask": AIS_PARTIAL_DEPLOY_MASK,
        "tc1_delay_s": AIS_TC1_DEPLOY_DELAY_S,
        "expect": {"tc1_main": True, "tc1_red": False, "switch": False,
                   "final": "partial"},
    },
}

# Short USB command names mapped to the existing descriptive scenarios.
TEST_COMMAND_LOOKUP = {
    "test01": "test01_power_on",
    "test02": "test02_sequential_deploy",
    "test03": "test03_no_deploy",
    "test04": "test04__power_no_deploy_then_only_tc1",
    "test05": "test05_power_no_deploy_then_only_tc2",
    "test06": "test06_power_tc1_then_tc2_deploy",
    "test07": "test07_power_tc1_then_tc2_no_deploy",
    "test08": "test08_power_tc2_then_tc1_deploy",
    "test09": "test09_power_tc2_then_tc1_no_deploy",
    "test10": "test10_power_tc2_deploy_then_tc1_at_red",
    "test11": "test11_power_tc1_deploy_then_tc2_at_red",
    "test12": "test12_redundant_tc1_tc2_deploy",
    "test13": "test13_redundant_tc1_only_tc2_ignored",
    "test14": "test14_redundant_tc1_ignored_tc2_deploy",
    "test15": "test15_redundant_ignore_all",
    "test16": "redundant_deploy",
    "test17": "shared_i2c_deployment",
    "test18": "shared_i2c_deployment_redundant",
    "test20": "tc2_failover_main_to_red",
    "test21": "main_lost_after_tc1",
    # AIS has its own TC1-only tests. The old ais01-ais16/ais20/ais21 aliases
    # of the UHF tests are retired; the shared UHF->AIS handoff stays in
    # test17/test18.
    "ais01": "ais01_tc1_main_deploy",
    "ais02": "ais02_tc1_main_ignored_red_deploy",
    "ais03": "ais03_redundant_only_tc1_deploy",
    "ais04": "ais04_tc1_ignored_both",
    "ais05": "ais05_tc1_partial_deploy",
}

ACTIVE_SCENARIO = "test02_sequential_deploy"

# Host/debug reset command. This is intentionally outside the real writable
# TC bit range so ordinary cutter writes cannot re-arm the simulator.
SIM_RESET_COMMAND = 0x80

# --- Debug ------------------------------------------------------------------
DEBUG        = False     # prints add service-loop latency; enable only for bench debug
DEBUG_READS  = False     # also print every master read (can be very chatty)
DEBUG_COUNTS = False     # enable only for periodic bench activity counters
POLL_MS      = 0         # 0 = busy-loop; minimizes I2C clock-stretch latency

# --- USB serial test report ------------------------------------------------
# MicroPython print() output is carried by the Pico micro-USB CDC/REPL serial
# connection. Keep command history bounded so a noisy or long run cannot
# consume unbounded Pico RAM. The report is emitted once, after the session
# ends (duration expiry, runtime "end", or a runtime testNN switch) and I2C is
# disabled. REPORT_AFTER_S is not used by the code.
# Master switch for command/response collection and the timed USB report.
# Set REPORT = False to disable the complete reporting feature.
REPORT = True
REPORT_AFTER_S = 200
REPORT_COMMAND_LIMIT = 64
REPORT_RESPONSE_LIMIT = 64


# =============================================================================
#  ANT_REGISTER bit positions
# =============================================================================
FB1 = 0
FB2 = 1
FB3 = 2
FB4 = 3
TC1 = 4
TC2 = 5
# bit6 unused
RTN = 7

TC_WRITE_MASK = (1 << TC1) | (1 << TC2)   # 0x30 -- only bits the master may set


# =============================================================================
#  Register-level RP2040 I2C slave driver (poll-based, non-blocking)
#  Adapted from kyordhel/i2cslave (MIT). Offsets per RP2040 datasheet.
# =============================================================================
class PicoI2CSlave:
    # RP2040 memory-mapped peripheral base addresses.
    _IO_BANK0_BASE = 0x40014000
    _PADS_BANK0_BASE = 0x4001c000
    _I2C0_BASE     = 0x40044000
    _I2C1_BASE     = 0x40048000

    _ATOM_SET = 0x2000
    _ATOM_CLR = 0x3000

    _GPIO_FUNC_I2C = 0x03

    # PADS_BANK0 GPIO pad control bits.
    _PAD_IE = 0x40
    _PAD_PUE = 0x08
    _PAD_PDE = 0x04

    _IC_CON           = 0x00
    _IC_SAR           = 0x08
    _IC_DATA_CMD      = 0x10
    _IC_RAW_INTR_STAT = 0x34
    _IC_CLR_RD_REQ    = 0x50
    _IC_CLR_TX_ABRT   = 0x54
    _IC_ENABLE        = 0x6c
    _IC_STATUS        = 0x70
    _IC_RXFLR         = 0x78

    # IC_STATUS bits
    _ST_TFNF = 0x02   # Tx FIFO not full
    _ST_RFNE = 0x08   # Rx FIFO not empty
    _ST_SLV_ACTIVITY = 0x40   # slave FSM not idle (transfer in progress)
    # IC_RAW_INTR_STAT bits
    _IRQ_RD_REQ = 0x20

    _SDA_PINS = {0: (0, 4, 8, 12, 16, 20), 1: (2, 6, 10, 14, 18, 26)}
    _SCL_PINS = {0: (1, 5, 9, 13, 17, 21), 1: (3, 7, 11, 15, 19, 27)}

    def __init__(self, i2c_id, address, sda, scl):
        if i2c_id not in (0, 1):
            raise ValueError("i2c_id must be 0 or 1")
        if sda not in self._SDA_PINS[i2c_id]:
            raise ValueError("SDA pin %d invalid for I2C%d" % (sda, i2c_id))
        if scl not in self._SCL_PINS[i2c_id]:
            raise ValueError("SCL pin %d invalid for I2C%d" % (scl, i2c_id))
        if not (0 <= address <= 0x7f):
            raise ValueError("address must be a 7-bit value")

        self.id = i2c_id
        self.address = address
        self._base = self._I2C0_BASE if i2c_id == 0 else self._I2C1_BASE

        self._setup_pin(sda)
        self._setup_pin(scl)

        # Datasheet slave-enable sequence:
        # 1. disable block
        self._clr(self._IC_ENABLE, 0x0001)
        # 2. program slave address into IC_SAR
        self._clr(self._IC_SAR, 0x03ff)
        self._set(self._IC_SAR, address)
        # 3. clear MASTER_MODE(0), 10BIT_SLAVE(3), IC_SLAVE_DISABLE(6) -> 0x49
        self._clr(self._IC_CON, 0x0049)
        # 4. re-enable block
        self._set(self._IC_ENABLE, 0x0001)

    def set_address(self, address):
        """Re-address this target while its I2C block is idle."""
        if not (0 <= address <= 0x7f):
            raise ValueError("address must be a 7-bit value")
        if address == self.address:
            return

        # Disable before changing IC_SAR, clear stale RX/abort/request state,
        # then re-enable. The shared-scenario controller invokes this only
        # after a completed status response and a handoff guard interval.
        self._clr(self._IC_ENABLE, 0x0001)
        while self._rd(self._IC_STATUS, self._ST_RFNE):
            self._rd(self._IC_DATA_CMD, 0xff)
        self._rd(self._IC_CLR_TX_ABRT)
        self._rd(self._IC_CLR_RD_REQ)
        self._clr(self._IC_SAR, 0x03ff)
        self._set(self._IC_SAR, address)
        self.address = address
        self._set(self._IC_ENABLE, 0x0001)

    def is_busy(self):
        """True while a transfer addressed to this target is in progress."""
        return bool(self._rd(self._IC_STATUS, self._ST_SLV_ACTIVITY))

    def disable(self):
        """Disable this I2C target between test sessions."""
        self._clr(self._IC_ENABLE, 0x0001)

    # -- low-level helpers ---------------------------------------------------
    def _setup_pin(self, pin):
        # PADS_BANK0 GPIOx starts at base + 4 + 4*pin. Configure the pad before
        # switching FUNCSEL so the bus never sees the reset pull-down.
        pad = self._PADS_BANK0_BASE + 4 + 4 * pin
        machine.mem32[pad | self._ATOM_CLR] = self._PAD_PDE | self._PAD_PUE
        machine.mem32[pad | self._ATOM_SET] = self._PAD_IE
        if INTERNAL_PULLUPS:
            machine.mem32[pad | self._ATOM_SET] = self._PAD_PUE

        # GPIOx_CTRL lives at IO_BANK0_BASE + 8*pin + 4; FUNCSEL=3 selects I2C.
        ctrl = self._IO_BANK0_BASE + 8 * pin + 4
        machine.mem32[ctrl | self._ATOM_CLR] = 0x1f
        machine.mem32[ctrl | self._ATOM_SET] = self._GPIO_FUNC_I2C

    def _set(self, reg, mask):
        machine.mem32[self._base | self._ATOM_SET | reg] = mask

    def _clr(self, reg, mask):
        machine.mem32[self._base | self._ATOM_CLR | reg] = mask

    def _rd(self, reg, mask=0xffffffff):
        return machine.mem32[self._base | reg] & mask

    def _wr(self, reg, value):
        machine.mem32[self._base | reg] = value

    # -- non-blocking service ------------------------------------------------
    def read_pending(self):
        """Return list of bytes the master has written (may be empty)."""
        out = []
        while self._rd(self._IC_STATUS, self._ST_RFNE):
            out.append(self._rd(self._IC_DATA_CMD, 0xff))
        return out

    def read_requested(self):
        """True if the master is currently requesting a byte from us."""
        return bool(self._rd(self._IC_RAW_INTR_STAT, self._IRQ_RD_REQ))

    def send_byte(self, value):
        """Answer an outstanding master read request with one byte."""
        # IC_CLR_TX_ABRT is read-to-clear. Do not clear RD_REQ until after the
        # data byte is queued; clearing RD_REQ can release clock stretching.
        self._rd(self._IC_CLR_TX_ABRT)
        # wait (briefly) for Tx FIFO space -- hardware clock-stretches the master
        t0 = time.ticks_ms()
        while not self._rd(self._IC_STATUS, self._ST_TFNF):
            if time.ticks_diff(time.ticks_ms(), t0) > 5:
                break
        self._wr(self._IC_DATA_CMD, value & 0xff)
        self._rd(self._IC_CLR_RD_REQ)          # read-to-clear RD_REQ


def make_slave(i2c_id, address, sda, scl):
#    if DRIVER == "i2ctarget":
#        return TargetI2CSlave(i2c_id, address, sda, scl)
    return PicoI2CSlave(i2c_id, address, sda, scl)


# =============================================================================
#  Simulator state / engine
# =============================================================================
class DeploymentSim:
    def __init__(self, scenario):
        self.scenario = scenario
        self.reset()

    def reset(self):
        # antenna "stored" flags: True = stored (bit=1), False = deployed (bit=0)
        self.stored = [True, True, True, True]
        self.tc1 = False
        self.tc2 = False
        self.deployed = False
        # Sequential mode phase: 0 waits for TC1, 1 waits for TC2, 2 complete.
        self.sequence_phase = 0
        self._timer_start = None      # ms tick when countdown began
        self._counting = False
        self._timer_trigger = None

    # -- register assembly / command application ----------------------------
    def assemble(self):
        val = 0
        if self.stored[0]: val |= (1 << FB1)
        if self.stored[1]: val |= (1 << FB2)
        if self.stored[2]: val |= (1 << FB3)
        if self.stored[3]: val |= (1 << FB4)
        if self.tc1:       val |= (1 << TC1)
        if self.tc2:       val |= (1 << TC2)
        # bits 6 and 7 are unused and read 0.
        return val

    def apply_command(self, cmd):
        """Apply a master write; TC bits retain their last commanded values."""
        if cmd == SIM_RESET_COMMAND:
            self.reset()
            if DEBUG:
                print("[RESET] simulator re-armed by command 0x%02X" % cmd)
            return
        if self.deployed:
            return
        scen = self.scenario
        accept_tc1 = scen.get("accept_tc1", True)
        accept_tc2 = scen.get("accept_tc2", True)
        new_tc1 = bool(cmd & (1 << TC1)) if accept_tc1 else False
        new_tc2 = bool(cmd & (1 << TC2)) if accept_tc2 else False
        if (new_tc1, new_tc2) != (self.tc1, self.tc2):
            self.tc1, self.tc2 = new_tc1, new_tc2
            if DEBUG:
                print("[WRITE] cmd=0x%02X -> TC1=%d TC2=%d"
                      % (cmd, self.tc1, self.tc2))

    def _deploy_all(self):
        self.stored = [False, False, False, False]  # all deployed, permanent
#        self.tc1 = False                             # auto-clear thermal cutters
#        self.tc2 = False
        self.deployed = True
        self.sequence_phase = 2
        self._counting = False
        self._timer_start = None
        self._timer_trigger = None
        if DEBUG:
            print("[DEPLOY] all antennas deployed; TC1/TC2 latched -> "
                  "reg=0x%02X" % self.assemble())

    def _deploy_first_pair(self):
        """TC1 success: deploy only ANT1/ANT2, then wait for TC2."""
        self.stored[0] = False
        self.stored[1] = False
        self.sequence_phase = 1
        self._reset_timer(silent=True)
        if DEBUG:
            print("[DEPLOY] TC1 complete: ANT1/ANT2 deployed; waiting for TC2 "
                  "-> reg=0x%02X" % self.assemble())

    def _deploy_second_pair(self):
        """TC2 success: deploy ANT3/ANT4 and complete the sequence."""
        self.stored[2] = False
        self.stored[3] = False
        self.deployed = True
        self.sequence_phase = 2
        self._reset_timer(silent=True)
        if DEBUG:
            print("[DEPLOY] TC2 complete: ANT3/ANT4 deployed; sequence complete "
                  "-> reg=0x%02X" % self.assemble())

    def _deploy_pair_for_test(self, pair_name, trigger_name):
        """Permanently deploy one pair for a focused PAIR_TEST scenario."""
        if pair_name == "TC1":
            self.stored[0] = False
            self.stored[1] = False
            pair_text = "ANT1/ANT2"
        elif pair_name == "TC2":
            self.stored[2] = False
            self.stored[3] = False
            pair_text = "ANT3/ANT4"
        else:
            raise ValueError("Unsupported antenna pair: %r" % pair_name)

        # Only mark the simulator fully deployed when both pairs are out.
        self.deployed = not any(self.stored)
        if self.deployed:
            self.sequence_phase = 2
        self._reset_timer(silent=True)
        if DEBUG:
            print("[DEPLOY] %s deployed by %s -> reg=0x%02X"
                  % (pair_text, trigger_name, self.assemble()))

    def _update_timed_condition(self, condition, trigger_name, delay_s, now_ms,
                                on_complete):
        if condition:
            if not self._counting or self._timer_trigger != trigger_name:
                self._counting = True
                self._timer_trigger = trigger_name
                self._timer_start = now_ms
                if DEBUG:
                    print("[TIMER] %s condition met; action in %ss"
                          % (trigger_name, delay_s))
            elif time.ticks_diff(now_ms, self._timer_start) >= delay_s * 1000:
                on_complete()
        else:
            self._reset_timer()

    # -- deployment engine ---------------------------------------------------
    def update(self, power_on, now_ms):
        if self.deployed:
            return

        scen = self.scenario
        mode = scen["mode"]

        if mode == "NO_DEPLOY":
            self._reset_timer(silent=True)
            return

        if mode == "POWER_ON_ALL":
            self._update_timed_condition(
                power_on, "POWER_ON", scen["delay_s"], now_ms,
                self._deploy_all)
            return

        if mode == "SEQUENTIAL_TC":
            self.update_sequential(power_on, now_ms, scen)
            return

        if mode == "PAIR_TEST":
            self.update_pair_test(power_on, now_ms, scen)
            return

        if mode == "SHARED_I2C_DEPLOYMENT":
            raise ValueError(
                "SHARED_I2C_DEPLOYMENT must be run by the shared-bus controller")

        if mode == "TC2_FAILOVER":
            raise ValueError(
                "TC2_FAILOVER must be run by the failover controller")

        if mode == "MAIN_LOST_FAILOVER":
            raise ValueError(
                "MAIN_LOST_FAILOVER must be run by the main-lost controller")

        raise ValueError("Unsupported scenario mode: %r" % mode)

    def update_sequential(self, power_on, now_ms, scen):
        """Advance TC1-first-pair then TC2-second-pair deployment."""
        if self.deployed:
            return
        tc_gate = power_on or not TC_REQUIRES_POWER
        if self.sequence_phase == 0:
            self._update_timed_condition(
                self.tc1 and tc_gate,
                "TC1",
                scen["tc1_delay_s"],
                now_ms,
                self._deploy_first_pair,
            )
        elif self.sequence_phase == 1:
            self._update_timed_condition(
                self.tc2 and tc_gate,
                "TC2",
                scen["tc2_delay_s"],
                now_ms,
                self._deploy_second_pair,
            )

    def update_pair_test(self, power_on, now_ms, scen):
        """Run a configurable partial-pair deployment/failure scenario."""
        if self.deployed:
            return

        power_pair = scen.get("power_pair")
        if power_pair == "TC1" and (self.stored[0] or self.stored[1]):
            self._update_timed_condition(
                power_on,
                "POWER_TC1_PAIR",
                scen["power_delay_s"],
                now_ms,
                lambda: self._deploy_pair_for_test("TC1", "POWER_ON"),
            )
            return
        if power_pair == "TC2" and (self.stored[2] or self.stored[3]):
            self._update_timed_condition(
                power_on,
                "POWER_TC2_PAIR",
                scen["power_delay_s"],
                now_ms,
                lambda: self._deploy_pair_for_test("TC2", "POWER_ON"),
            )
            return

        tc_gate = power_on or not TC_REQUIRES_POWER
        if scen.get("deploy_on_tc1") and (self.stored[0] or self.stored[1]):
            self._update_timed_condition(
                self.tc1 and tc_gate,
                "TC1",
                scen["tc1_delay_s"],
                now_ms,
                lambda: self._deploy_pair_for_test("TC1", "TC1"),
            )
            return
        if scen.get("deploy_on_tc2") and (self.stored[2] or self.stored[3]):
            self._update_timed_condition(
                self.tc2 and tc_gate,
                "TC2",
                scen["tc2_delay_s"],
                now_ms,
                lambda: self._deploy_pair_for_test("TC2", "TC2"),
            )
            return

        # Commands may still be accepted and exposed in the register for a
        # deliberate no-deploy case; they simply do not start a timer.
        self._reset_timer(silent=True)

    def update_tc2_failover(self, power_on, now_ms, scen, tc2_on_redundant):
        """TC1 deploys as usual; TC2 deploys only once commanded on redundant."""
        if self.deployed:
            return
        if self.sequence_phase == 0:
            self.update_sequential(power_on, now_ms, scen)
            return
        tc_gate = power_on or not TC_REQUIRES_POWER
        self._update_timed_condition(
            self.tc2 and tc2_on_redundant and tc_gate,
            "TC2",
            scen["tc2_delay_s"],
            now_ms,
            self._deploy_second_pair,
        )

    def update_shared_uhf(self, power_on, now_ms, scen):
        """Model UHF power-on success or its TC1/TC2 fallback."""
        if self.deployed:
            return

        # Receiving TC1 commits the simulation to the OBC fallback path. Once
        # the first pair is deployed, only TC2 can complete the sequence.
        if self.tc1 or self.sequence_phase > 0:
            self.update_sequential(power_on, now_ms, scen)
            return

        if SHARED_UHF_POWER_DEPLOY_SUCCESS:
            self._update_timed_condition(
                power_on,
                "POWER_ON",
                scen["uhf_power_delay_s"],
                now_ms,
                self._deploy_all,
            )
        else:
            # Intentionally remain stored throughout the OBC initial poll.
            self._reset_timer(silent=True)

    def _reset_timer(self, silent=False):
        if self._counting and not silent and DEBUG:
            print("[TIMER] condition lost (power off / TC cleared) -> reset")
        self._counting = False
        self._timer_start = None
        self._timer_trigger = None


class AisDeploymentSim:
    """AIS board: a single thermal cutter (TC1); no TC2, never power-deployed.

    TC1 held for tc1_delay_s with power present clears the feedback bits in
    deploy_mask. TC1 fires once; later writes cannot change the antennas.
    """

    def __init__(self, deploy_mask, tc1_delay_s):
        self.deploy_mask = deploy_mask
        self.tc1_delay_s = tc1_delay_s
        self.reset()

    def reset(self):
        self.stored = 0x0F            # bits 0-3: 1 = stored, 0 = deployed
        self.tc1 = False
        self.fired = False            # TC1 burn completed
        self.sequence_phase = 0       # 0 waiting for TC1, 2 TC1 complete
        self._timer_start = None

    @property
    def deployed(self):
        return self.stored == 0

    def assemble(self):
        val = self.stored
        if self.tc1:       val |= (1 << TC1)
        # bit5 (TC2) does not exist on AIS; bits 5-7 always read 0.
        return val

    def apply_command(self, cmd):
        """Apply a master write. Only the TC1 bit has any effect on AIS."""
        if cmd == SIM_RESET_COMMAND:
            self.reset()
            return
        if self.fired:
            return
        self.tc1 = bool(cmd & (1 << TC1))
        if not self.tc1:
            self._timer_start = None

    def update(self, power_on, now_ms):
        if self.fired:
            return
        if not (self.tc1 and (power_on or not TC_REQUIRES_POWER)):
            self._timer_start = None
            return
        if self._timer_start is None:
            self._timer_start = now_ms
        elif time.ticks_diff(now_ms, self._timer_start) >= self.tc1_delay_s * 1000:
            self.stored &= ~self.deploy_mask & 0x0F
            self.fired = True
            self.sequence_phase = 2
            self._timer_start = None


# =============================================================================
#  Main
# =============================================================================

PRIMARY_WRITES = 0
PRIMARY_READS = 0
SECONDARY_WRITES = 0
SECONDARY_READS = 0
COMMAND_HISTORY = []
COMMAND_HISTORY_DROPPED = 0
RESPONSE_HISTORY = []
RESPONSE_HISTORY_DROPPED = 0
LAST_RESPONSE_BY_TARGET = {}
# (elapsed_ms, from_address, to_address, reason) for each Pico re-address
ADDRESS_EVENTS = []


def command_name(command):
    if command == SIM_RESET_COMMAND:
        return "SIM_RESET"
    if command == 0x00:
        return "TC_OFF"
    if command == (1 << TC1):
        return "TC1"
    if command == (1 << TC2):
        return "TC2"
    if command == TC_WRITE_MASK:
        return "TC1_TC2"
    return "OTHER"


def record_obc_command(now_ms, start_ms, target, address, command):
    """Record a bounded chronological command trace for the USB report."""
    global COMMAND_HISTORY_DROPPED
    if not REPORT:
        return
    if len(COMMAND_HISTORY) < REPORT_COMMAND_LIMIT:
        elapsed_ms = time.ticks_diff(now_ms, start_ms)
        COMMAND_HISTORY.append(
            (elapsed_ms, target, address, command, command_name(command)))
    else:
        COMMAND_HISTORY_DROPPED += 1


def _deployment_state(status, target="UHF"):
    feedback = status & 0x0f
    if feedback == 0x0f:
        return "ALL STORED"
    if feedback == 0x00:
        return "ALL DEPLOYED"
    if target == "AIS":
        return "PARTIAL (stored bits 0x%02X)" % feedback
    if feedback == 0x0c:
        return "TC1 PAIR DEPLOYED"
    if feedback == 0x03:
        return "TC2 PAIR DEPLOYED"
    return "MIXED FEEDBACK"


def _deployed_count(status):
    feedback = status & 0x0f
    return sum(1 for bit in range(4) if not feedback & (1 << bit))


def _print_target_report(target, writes, reads):
    print("-" * 64)
    print(" %s OBC COMMANDS (%d received)" % (target, writes))
    displayed_commands = 0
    for entry in COMMAND_HISTORY:
        elapsed_ms, entry_target, address, command, name = entry
        if entry_target != target:
            continue
        displayed_commands += 1
        print("  %02d. %7.3f s | 0x%02X | %-9s (0x%02X)" %
              (displayed_commands, elapsed_ms / 1000,
               address, name, command))
    if displayed_commands == 0:
        print("  None recorded")

    print("-" * 64)
    print(" %s PICO RESPONSES (%d status reads)" % (target, reads))
    displayed_responses = 0
    for entry in RESPONSE_HISTORY:
        first_ms, last_ms, entry_target, address, status, response_reads = entry
        if entry_target != target:
            continue
        displayed_responses += 1
        print("  %02d. %7.3f-%7.3f s | 0x%02X | status 0x%02X | %s" %
              (displayed_responses, first_ms / 1000, last_ms / 1000,
               address, status, _deployment_state(status, target)))
        if target == "AIS":
            print("      Antennas deployed: %d/4 | reads: %d" %
                  (_deployed_count(status), response_reads))
        else:
            print("      TC1 pair: %-12s | TC2 pair: %-12s | reads: %d" %
                  ("DEPLOYED" if status & 0x03 == 0 else "NOT DEPLOYED",
                   "DEPLOYED" if status & 0x0c == 0 else "NOT DEPLOYED",
                   response_reads))
    if displayed_responses == 0:
        print("  No status byte was returned to the OBC")

    # Report the last byte actually served for this target. Calling assemble()
    # here could report state which the OBC never observed.
    target_result = None
    for key, response in LAST_RESPONSE_BY_TARGET.items():
        entry_target, address = key
        status, _, response_ms = response
        if entry_target == target:
            if target_result is None or response_ms >= target_result[0]:
                target_result = (response_ms, address, status)
    print("-" * 64)
    print(" FINAL %s RESULT" % target)
    if target_result is None:
        print("  No status register response was read by the OBC")
        return
    response_ms, address, status = target_result
    print("  Address    : 0x%02X" % address)
    print("  Last read  : %.3f s" % (response_ms / 1000))
    print("  Status     : 0x%02X" % status)
    print("  Deployment : %s" % _deployment_state(status, target))
    if target == "AIS":
        print("  Antennas   : %d/4 deployed" % _deployed_count(status))
        return
    print("  TC1 pair   : %s" %
          ("DEPLOYED" if status & 0x03 == 0 else "NOT DEPLOYED"))
    print("  TC2 pair   : %s" %
          ("DEPLOYED" if status & 0x0c == 0 else "NOT DEPLOYED"))


def _print_failover_report(target, main_address, red_address):
    """Summarise the TC2 main -> redundant failover path for test20."""
    def first_command(address, bit):
        for elapsed_ms, entry_target, entry_address, command, _ in COMMAND_HISTORY:
            if (entry_target == target and entry_address == address and
                    command != SIM_RESET_COMMAND and command & (1 << bit)):
                return elapsed_ms
        return None

    def at(elapsed_ms):
        return "NOT SEEN" if elapsed_ms is None else "seen at %.3f s" % (elapsed_ms / 1000)

    tc1_main = first_command(main_address, TC1)
    tc2_main = first_command(main_address, TC2)
    tc2_red = first_command(red_address, TC2)
    final = None
    for (entry_target, _), response in LAST_RESPONSE_BY_TARGET.items():
        if entry_target == target and (final is None or response[2] >= final[1]):
            final = (response[0], response[2])
    all_deployed = final is not None and final[0] & 0x0f == 0

    print("-" * 64)
    print(" %s ADDRESS FAILOVER (0x%02X -> 0x%02X)" % (target, main_address, red_address))
    for elapsed_ms, from_address, to_address, reason in ADDRESS_EVENTS:
        print("  %7.3f s | 0x%02X -> 0x%02X | %s" %
              (elapsed_ms / 1000, from_address, to_address, reason))
    if not ADDRESS_EVENTS:
        print("  No address switch happened")
    print("  TC1 on main      : %s" % at(tc1_main))
    print("  TC2 on main      : %s" % at(tc2_main))
    print("  TC2 on redundant : %s" % at(tc2_red))
    if tc2_main is not None and tc2_red is not None:
        print("  TC2 main->red gap: %.3f s (OBC TC2 timeout)" %
              ((tc2_red - tc2_main) / 1000))
    passed = (tc1_main is not None and tc2_main is not None and
              tc2_red is not None and len(ADDRESS_EVENTS) > 0 and all_deployed)
    verdict = "PASS" if passed else "INCOMPLETE"
    print("  Failover result  : %s" % verdict)
    if COMMAND_HISTORY_DROPPED:
        print("  Warning: command history overflowed; result may be incomplete")
    return verdict


def _print_main_lost_report(target, main_address, red_address):
    """Summarise the main-lost-after-TC1 failover path for test21."""
    # Judge only the run after the last bench reset, if there was one.
    commands = [entry for entry in COMMAND_HISTORY if entry[1] == target]
    after_ms = 0
    for index in range(len(commands) - 1, -1, -1):
        if commands[index][3] == SIM_RESET_COMMAND:
            after_ms = commands[index][0]
            commands = commands[index + 1:]
            break
    switches = [event for event in ADDRESS_EVENTS
                if event[1] == main_address and event[2] == red_address]

    def first_command(address, bit):
        for elapsed_ms, _, entry_address, command, _ in commands:
            if entry_address == address and command & (1 << bit):
                return elapsed_ms
        return None

    def at(elapsed_ms):
        return "NOT SEEN" if elapsed_ms is None else "seen at %.3f s" % (elapsed_ms / 1000)

    tc1_main = first_command(main_address, TC1)
    tc2_main = first_command(main_address, TC2)
    tc2_red = first_command(red_address, TC2)
    final = None
    for (entry_target, _), response in LAST_RESPONSE_BY_TARGET.items():
        if entry_target == target and (final is None or response[2] >= final[1]):
            final = (response[0], response[2])
    all_deployed = (final is not None and final[1] >= after_ms and
                    final[0] & 0x0f == 0)

    print("-" * 64)
    print(" %s MAIN LOST AFTER TC1 (0x%02X -> 0x%02X)" % (target, main_address, red_address))
    for elapsed_ms, from_address, to_address, reason in ADDRESS_EVENTS:
        print("  %7.3f s | 0x%02X -> 0x%02X | %s" %
              (elapsed_ms / 1000, from_address, to_address, reason))
    if not switches:
        print("  No main -> redundant switch happened")
    print("  TC1 on main      : %s (expected)" % at(tc1_main))
    print("  TC2 on main      : %s (%s)" %
          (at(tc2_main), "expected: NACKed, main absent" if tc2_main is None
           else "UNEXPECTED: switch was late, test20 path taken"))
    print("  TC2 on redundant : %s (expected)" % at(tc2_red))
    if switches and tc2_red is not None:
        print("  Switch->TC2 red  : %.3f s (OBC retries on its next tick)" %
              ((tc2_red - switches[-1][0]) / 1000))
    passed = (tc1_main is not None and tc2_main is None and
              tc2_red is not None and len(switches) > 0 and all_deployed)
    verdict = "PASS" if passed else "INCOMPLETE"
    print("  Failover result  : %s" % verdict)
    if COMMAND_HISTORY_DROPPED:
        print("  Warning: command history overflowed; result may be incomplete")
    return verdict


def _print_ais_report(scenario, main_address, red_address):
    """Check an aisNN run against the scenario's expected path."""
    expect = scenario["expect"]
    deploy_mask = scenario["deploy_mask"] or AIS_TC1_DEPLOY_MASK
    # AIS counts as deployed when every bit of AIS_TC1_DEPLOY_MASK (the bits
    # the OBC firmware checks) reads 0, whatever this scenario's TC1 clears.
    success_mask = AIS_TC1_DEPLOY_MASK
    # Judge only the run after the last bench reset, if there was one.
    commands = [entry for entry in COMMAND_HISTORY if entry[1] == "AIS"]
    after_ms = 0
    for index in range(len(commands) - 1, -1, -1):
        if commands[index][3] == SIM_RESET_COMMAND:
            after_ms = commands[index][0]
            commands = commands[index + 1:]
            break
    switches = [event for event in ADDRESS_EVENTS
                if event[1] == main_address and event[2] == red_address]

    def first_command(address, bit):
        for elapsed_ms, _, entry_address, command, _ in commands:
            if entry_address == address and command & (1 << bit):
                return elapsed_ms
        return None

    def at(elapsed_ms):
        return "NOT SEEN" if elapsed_ms is None else "seen at %.3f s" % (elapsed_ms / 1000)

    def tag(seen, expected):
        if seen == expected:
            return "expected"
        return "UNEXPECTED, expected %s" % ("seen" if expected else "not seen")

    tc1_main = first_command(main_address, TC1)
    tc1_red = first_command(red_address, TC1)
    tc2_any = [entry for entry in commands if entry[3] & (1 << TC2)]
    final = None
    for (entry_target, _), response in LAST_RESPONSE_BY_TARGET.items():
        if entry_target == "AIS" and (final is None or response[2] >= final[1]):
            final = (response[0], response[2])
    if final is None or final[1] < after_ms:
        final_state = "no read"
    elif final[0] & success_mask == 0:
        final_state = "deployed"
    elif final[0] & 0x0f == 0x0f:
        final_state = "stored"
    elif final[0] & 0x0f == 0x0f & ~deploy_mask:
        final_state = "partial"
    else:
        final_state = "mixed"

    checks = [
        ("TC1 on main      ", tc1_main, expect["tc1_main"]),
        ("TC1 on redundant ", tc1_red, expect["tc1_red"]),
    ]
    print("-" * 64)
    print(" AIS TC1-ONLY CHECK (main 0x%02X, redundant 0x%02X)" %
          (main_address, red_address))
    print("  TC1 clears bits 0x%02X; deployed means bits 0x%02X all 0" %
          (deploy_mask, success_mask))
    for elapsed_ms, from_address, to_address, reason in ADDRESS_EVENTS:
        print("  %7.3f s | 0x%02X -> 0x%02X | %s" %
              (elapsed_ms / 1000, from_address, to_address, reason))
    print("  Address switch   : %s (%s)" %
          ("yes" if switches else "no", tag(bool(switches), expect["switch"])))
    for label, seen_ms, expected in checks:
        print("  %s: %s (%s)" % (label, at(seen_ms), tag(seen_ms is not None, expected)))
    print("  Final antennas   : %s (expected %s)" % (final_state, expect["final"]))
    if tc2_any:
        # The TC1-only AIS firmware must never send TC2. The current firmware
        # still sends TC2 if TC1 leaves bits 2-3 stored (ais05).
        print("  WARNING: %d TC2 write(s) reached AIS (no AIS TC2 exists; ignored)"
              % len(tc2_any))
    as_expected = (all((seen_ms is not None) == expected for _, seen_ms, expected in checks)
                   and bool(switches) == expect["switch"]
                   and final_state == expect["final"])
    verdict = "AS EXPECTED" if as_expected else "UNEXPECTED"
    print("  Scenario result  : %s" % verdict)
    if COMMAND_HISTORY_DROPPED:
        print("  Warning: command history overflowed; result may be incomplete")
    return verdict


def _delay_text(value_s, from_command):
    if value_s is None:
        return "n/a"
    return "%s s (%s)" % (value_s, "command" if from_command else "default")


def print_test_report(now_ms, start_ms, session):
    """Print completed-session logs for one board, or both in shared mode.

    Returns the scenario verdict, or None for tests that have no verdict.
    """
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000
    scenario_name = session["name"]
    scenario = session["scenario"]
    board_profile = session["board_profile"]
    shared_mode = scenario["mode"] == "SHARED_I2C_DEPLOYMENT"
    targets = ("UHF", "AIS") if shared_mode else (board_profile,)

    print("")
    print("=" * 64)
    print(" ANTENNA DEPLOYMENT TEST REPORT")
    print("=" * 64)
    print(" Test       : %s" % scenario_name)
    print(" Mode       : %s" % scenario["mode"])
    print(" Board      : %s" % ("UHF + AIS" if shared_mode else board_profile))
    print(" Addresses  : %s" % scenario["address_set"])
    print(" Duration   : %s" % _duration_text(session))
    print(" Elapsed    : %.3f s" % elapsed_s)
    print(" TC delay   : %s" % _delay_text(
        scenario.get("tc1_delay_s"), session["tc_delay_s"] is not None))
    print(" Power delay: %s" % _delay_text(
        _session_power_delay_s(session), session["power_delay_s"] is not None))

    for target in targets:
        writes = PRIMARY_WRITES if target == "UHF" else SECONDARY_WRITES
        reads = PRIMARY_READS if target == "UHF" else SECONDARY_READS
        _print_target_report(target, writes, reads)
    verdict = None
    if scenario["mode"] == "TC2_FAILOVER":
        main_address, red_address = scenario_i2c_addresses(scenario)
        verdict = _print_failover_report(board_profile, main_address, red_address)
    elif scenario["mode"] == "MAIN_LOST_FAILOVER":
        main_address, red_address = scenario_i2c_addresses(scenario)
        verdict = _print_main_lost_report(board_profile, main_address, red_address)
    elif scenario["mode"] == "AIS_TC1":
        main_address, red_address = scenario_i2c_addresses(scenario)
        verdict = _print_ais_report(scenario, main_address, red_address)

    if COMMAND_HISTORY_DROPPED:
        print("  Note: %d command(s) exceeded the history limit" %
              COMMAND_HISTORY_DROPPED)
    if RESPONSE_HISTORY_DROPPED:
        print("  Note: %d response transition(s) exceeded the history limit" %
              RESPONSE_HISTORY_DROPPED)
    print("=" * 64)
    print(" END OF REPORT")
    print("=" * 64)
    return verdict


def print_session_result(end_ms, start_ms, session, outcome, verdict):
    """Print the session summary as one JSON line for host tools.

    Printed before SESSION_COMPLETE / SESSION_ABORTED. "final" holds the last
    status byte actually returned to each board (empty when REPORT is False).
    """
    scenario = session["scenario"]
    if scenario["mode"] == "SHARED_I2C_DEPLOYMENT":
        targets = ("UHF", "AIS")
    else:
        targets = (session["board_profile"],)
    final = {}
    for (target, address), response in LAST_RESPONSE_BY_TARGET.items():
        status, _, response_ms = response
        previous = final.get(target)
        if previous is None or response_ms >= previous[0]:
            final[target] = (response_ms, address, status)
    result = {
        "test": session["command_id"],
        "scenario": session["name"],
        "outcome": outcome,
        "verdict": verdict,
        "duration_s": session["duration_s"],
        "elapsed_s": round(time.ticks_diff(end_ms, start_ms) / 1000, 3),
        "tc_delay_s": scenario.get("tc1_delay_s"),
        "power_delay_s": _session_power_delay_s(session),
        "obc_writes": {},
        "status_reads": {},
        "final": {},
        "address_switches": [
            {"t_s": round(elapsed_ms / 1000, 3), "from": "0x%02X" % from_address,
             "to": "0x%02X" % to_address, "reason": reason}
            for elapsed_ms, from_address, to_address, reason in ADDRESS_EVENTS],
        "history_overflow": bool(COMMAND_HISTORY_DROPPED or RESPONSE_HISTORY_DROPPED),
    }
    for target in targets:
        result["obc_writes"][target] = PRIMARY_WRITES if target == "UHF" else SECONDARY_WRITES
        result["status_reads"][target] = PRIMARY_READS if target == "UHF" else SECONDARY_READS
        if target in final:
            response_ms, address, status = final[target]
            result["final"][target] = {
                "address": "0x%02X" % address,
                "status": "0x%02X" % status,
                "state": _deployment_state(status, target),
                "last_read_s": round(response_ms / 1000, 3),
            }
    print("RESULT " + json.dumps(result))

def validate_i2c_target_config(name, i2c_id, address, sda, scl):
    if i2c_id not in (0, 1):
        raise ValueError("%s I2C id must be 0 or 1" % name)
    if not (0 <= address <= 0x7f):
        raise ValueError("%s I2C address must be a 7-bit value" % name)
    if sda == scl:
        raise ValueError("%s SDA and SCL cannot use the same GPIO" % name)
    if sda not in PicoI2CSlave._SDA_PINS[i2c_id]:
        raise ValueError("%s SDA pin %d invalid for I2C%d" % (name, sda, i2c_id))
    if scl not in PicoI2CSlave._SCL_PINS[i2c_id]:
        raise ValueError("%s SCL pin %d invalid for I2C%d" % (name, scl, i2c_id))


def print_idle_status():
    """Answer "status" at CONFIG_READY: alive, idle, and the antenna power."""
    try:
        power = 1 if _read_power(_make_di_pin()) else 0
    except Exception:
        # Never let a pin problem stop the idle loop; report power as unknown.
        power = None
    print("STATUS " + json.dumps({"state": "idle", "power": power}))


def wait_for_usb_configuration():
    print("")
    print("CONFIG_READY")
    # Skip blank lines: a runtime "end\r\n" is completed at "\r", leaving
    # its "\n" queued for this read. "status" is answered here and the Pico
    # keeps waiting for a test command.
    command = ""
    while not command:
        line = sys.stdin.readline()
        if not line:
            raise ValueError("No USB command received")
        command = line.strip().lower()
        if command == "status":
            print_idle_status()
            command = ""
    if len(command) > USB_COMMAND_MAX_BYTES:
        raise ValueError("USB command is too long")
    return make_test_request(command)


def _command_delay_s(text, label):
    # int() raises ValueError for non-numeric text such as "5s".
    value = int(text)
    if not (0 <= value <= MAX_SESSION_DURATION_S):
        raise ValueError("%s must be 0..%d" % (label, MAX_SESSION_DURATION_S))
    return value


def make_test_request(command):
    """Parse "testNN [seconds|none] [tc_delay_s] [power_delay_s]".

    Every field after the test ID is optional and positional; "-" keeps that
    field's default, so "test01 - - 100" changes only the power delay.
    """
    parts = command.split()
    if not parts or len(parts) > 4:
        raise ValueError("Expected: testNN [seconds|none] [tc_delay_s] [power_delay_s]")
    test_id = parts[0]
    scenario_name = TEST_COMMAND_LOOKUP.get(test_id)
    if scenario_name is None:
        raise ValueError("Unknown test command: %s" % test_id)
    request = {
        "command_id": test_id,
        "scenario": scenario_name,
        "board_profile": "AIS" if test_id.startswith("ais") else "UHF",
    }
    if len(parts) > 1 and parts[1] != "-":
        if parts[1] == "none":
            request["duration_s"] = None
        else:
            # int() raises ValueError for non-numeric text such as "5s".
            request["duration_s"] = int(parts[1])
    if len(parts) > 2 and parts[2] != "-":
        request["tc_delay_s"] = _command_delay_s(parts[2], "TC delay")
    if len(parts) > 3 and parts[3] != "-":
        request["power_delay_s"] = _command_delay_s(parts[3], "power delay")
    return request


def build_session(request):
    scenario_name = request.get("scenario")
    if scenario_name not in SCENARIOS:
        raise ValueError("Unknown scenario: %r" % scenario_name)
    scenario = dict(SCENARIOS[scenario_name])
    # A delay sent with the command replaces the scenario's default. The one
    # TC delay covers every cutter (UHF TC1 and TC2, AIS TC1); the power delay
    # covers every power-only deployment.
    tc_delay_s = request.get("tc_delay_s")
    if tc_delay_s is not None:
        for field in ("tc1_delay_s", "tc2_delay_s"):
            if field in scenario:
                scenario[field] = tc_delay_s
    power_delay_s = request.get("power_delay_s")
    if power_delay_s is not None:
        for field in ("delay_s", "power_delay_s", "uhf_power_delay_s"):
            if field in scenario:
                scenario[field] = power_delay_s
    return {
        "command_id": request["command_id"],
        "name": scenario_name,
        "scenario": scenario,
        "board_profile": request.get("board_profile", BOARD_PROFILE),
        "duration_s": request.get("duration_s", DEFAULT_SESSION_DURATION_S),
        "tc_delay_s": tc_delay_s,
        "power_delay_s": power_delay_s,
    }


def _session_power_delay_s(session):
    """Power-only deployment delay, or None when the scenario has none."""
    scenario = session["scenario"]
    mode = scenario["mode"]
    if mode == "POWER_ON_ALL":
        return scenario["delay_s"]
    if mode == "PAIR_TEST" and scenario.get("power_pair") is not None:
        return scenario["power_delay_s"]
    if mode == "SHARED_I2C_DEPLOYMENT" and SHARED_UHF_POWER_DEPLOY_SUCCESS:
        return scenario["uhf_power_delay_s"]
    return None


def _shared_ais_tc1_delay_s(session):
    """AIS half of test17/test18: the command's TC delay, else the AIS default."""
    if session.get("tc_delay_s") is not None:
        return session["tc_delay_s"]
    return AIS_TC1_DEPLOY_DELAY_S


def scenario_i2c_addresses(scenario):
    """Return the two addresses a scenario uses on Pico I2C0.

    main / redundant  : (UHF target address, None)
    shared(_redundant): (UHF address, AIS address after the handoff)
    main_to_red       : (UHF main, UHF redundant)
    ais               : (AIS main, AIS redundant)
    """
    address_set = scenario.get("address_set")
    if address_set == "main":
        return PRIMARY_MAIN_ADDRESS, None
    if address_set == "redundant":
        return PRIMARY_REDUNDANT_ADDRESS, None
    if address_set == "shared":
        return SHARED_UHF_ADDRESS, SHARED_AIS_ADDRESS
    if address_set == "shared_redundant":
        return SHARED_UHF_REDUNDANT_ADDRESS, SHARED_AIS_REDUNDANT_ADDRESS
    if address_set == "main_to_red":
        return PRIMARY_MAIN_ADDRESS, PRIMARY_REDUNDANT_ADDRESS
    if address_set == "ais":
        return AIS_MAIN_ADDRESS, AIS_REDUNDANT_ADDRESS
    raise ValueError("Unsupported address_set: %r" % address_set)


def _validate_deploy_mask(mask, label):
    """An AIS deploy mask selects antenna feedback bits 0-3 only."""
    if isinstance(mask, bool) or not isinstance(mask, int) or not (0 < mask <= 0x0F):
        raise ValueError("%s must be 0x01..0x0F (antenna bits 0-3 only)" % label)


def _nonnegative_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
        raise ValueError("%s must be a non-negative number" % label)


def validate_session(session):
    scenario_name = session["name"]
    scenario = session["scenario"]
    board_profile = session["board_profile"]
    duration_s = session["duration_s"]
    if board_profile not in ("UHF", "AIS"):
        raise ValueError("board_profile must be UHF or AIS")
    if duration_s is not None and (
            isinstance(duration_s, bool) or
            not isinstance(duration_s, (int, float)) or
            not (0 < duration_s <= MAX_SESSION_DURATION_S)):
        raise ValueError("duration_s must be 1..%d or None" % MAX_SESSION_DURATION_S)
    if DRIVER != "register":
        raise ValueError("Only DRIVER='register' is supported")
    if POLL_MS < 0 or REPORT_COMMAND_LIMIT < 0 or REPORT_RESPONSE_LIMIT < 0:
        raise ValueError("poll and report limits must be non-negative")
    if not isinstance(REPORT, bool):
        raise ValueError("REPORT must be True or False")

    mode = scenario.get("mode")
    if mode not in ("POWER_ON_ALL", "SEQUENTIAL_TC", "NO_DEPLOY",
                    "PAIR_TEST", "SHARED_I2C_DEPLOYMENT", "TC2_FAILOVER",
                    "MAIN_LOST_FAILOVER", "AIS_TC1"):
        raise ValueError("Unsupported mode: %r" % mode)
    # aisNN runs only AIS scenarios, testNN only UHF/shared ones.
    if (board_profile == "AIS") != (mode == "AIS_TC1"):
        raise ValueError("%s is not a %s scenario" % (scenario_name, board_profile))
    scenario_i2c_addresses(scenario)
    # The AIS model is used by aisNN and by the AIS half of test17/test18.
    _validate_deploy_mask(AIS_TC1_DEPLOY_MASK, "AIS_TC1_DEPLOY_MASK")
    _nonnegative_number(AIS_TC1_DEPLOY_DELAY_S, "AIS_TC1_DEPLOY_DELAY_S")
    if mode == "POWER_ON_ALL":
        _nonnegative_number(scenario.get("delay_s"), "POWER_ON_ALL delay_s")
    if mode in ("SEQUENTIAL_TC", "SHARED_I2C_DEPLOYMENT", "TC2_FAILOVER",
                "MAIN_LOST_FAILOVER"):
        _nonnegative_number(scenario.get("tc1_delay_s"), "tc1_delay_s")
        _nonnegative_number(scenario.get("tc2_delay_s"), "tc2_delay_s")
    if mode == "PAIR_TEST":
        if scenario.get("power_pair") not in (None, "TC1", "TC2"):
            raise ValueError("PAIR_TEST power_pair must be None, TC1, or TC2")
        if scenario.get("power_pair") is not None:
            _nonnegative_number(scenario.get("power_delay_s"), "power_delay_s")
        for field in ("accept_tc1", "accept_tc2", "deploy_on_tc1", "deploy_on_tc2"):
            if not isinstance(scenario.get(field), bool):
                raise ValueError("PAIR_TEST %s must be bool" % field)
        if scenario.get("deploy_on_tc1") and not scenario.get("accept_tc1"):
            raise ValueError("PAIR_TEST cannot deploy on a rejected TC1 command")
        if scenario.get("deploy_on_tc2") and not scenario.get("accept_tc2"):
            raise ValueError("PAIR_TEST cannot deploy on a rejected TC2 command")
        _nonnegative_number(scenario.get("tc1_delay_s"), "tc1_delay_s")
        _nonnegative_number(scenario.get("tc2_delay_s"), "tc2_delay_s")

    if mode == "SHARED_I2C_DEPLOYMENT":
        _nonnegative_number(scenario.get("uhf_power_delay_s"), "uhf_power_delay_s")
        shared_uhf_address, shared_ais_address = scenario_i2c_addresses(scenario)
        validate_i2c_target_config("SHARED", PRIMARY_I2C_ID, shared_uhf_address, PRIMARY_SDA, PRIMARY_SCL)
        validate_i2c_target_config("SHARED", PRIMARY_I2C_ID, shared_ais_address, PRIMARY_SDA, PRIMARY_SCL)
        if DI_PIN in (PRIMARY_SDA, PRIMARY_SCL):
            raise ValueError("DI_PIN must not collide with shared I2C GPIOs")
        _nonnegative_number(SHARED_HANDOFF_DELAY_MS, "SHARED_HANDOFF_DELAY_MS")
        return True

    if mode == "TC2_FAILOVER":
        if scenario.get("address_set") != "main_to_red":
            raise ValueError("TC2_FAILOVER requires address_set main_to_red")
        main_address, red_address = scenario_i2c_addresses(scenario)
        if main_address == red_address:
            raise ValueError("TC2_FAILOVER main and redundant addresses must differ")
        validate_i2c_target_config("FAILOVER", PRIMARY_I2C_ID, main_address, PRIMARY_SDA, PRIMARY_SCL)
        validate_i2c_target_config("FAILOVER", PRIMARY_I2C_ID, red_address, PRIMARY_SDA, PRIMARY_SCL)
        if DI_PIN in (PRIMARY_SDA, PRIMARY_SCL):
            raise ValueError("DI_PIN must not collide with failover I2C GPIOs")
        _nonnegative_number(FAILOVER_SWITCH_DELAY_MS, "FAILOVER_SWITCH_DELAY_MS")
        return True

    if mode == "MAIN_LOST_FAILOVER":
        if scenario.get("address_set") != "main_to_red":
            raise ValueError("MAIN_LOST_FAILOVER requires address_set main_to_red")
        main_address, red_address = scenario_i2c_addresses(scenario)
        if main_address == red_address:
            raise ValueError("MAIN_LOST_FAILOVER main and redundant addresses must differ")
        validate_i2c_target_config("MAIN_LOST", PRIMARY_I2C_ID, main_address, PRIMARY_SDA, PRIMARY_SCL)
        validate_i2c_target_config("MAIN_LOST", PRIMARY_I2C_ID, red_address, PRIMARY_SDA, PRIMARY_SCL)
        if DI_PIN in (PRIMARY_SDA, PRIMARY_SCL):
            raise ValueError("DI_PIN must not collide with main-lost I2C GPIOs")
        _nonnegative_number(FAILOVER_SWITCH_DELAY_MS, "FAILOVER_SWITCH_DELAY_MS")
        return True

    if mode == "AIS_TC1":
        if scenario.get("start") not in ("main", "red"):
            raise ValueError("AIS_TC1 start must be main or red")
        for field in ("main_tc1", "red_tc1"):
            if scenario.get(field) not in ("deploy", "ignore"):
                raise ValueError("AIS_TC1 %s must be deploy or ignore" % field)
        if not isinstance(scenario.get("switch_after_main_tc1"), bool):
            raise ValueError("AIS_TC1 switch_after_main_tc1 must be bool")
        if scenario.get("start") == "red" and scenario.get("switch_after_main_tc1"):
            raise ValueError("AIS_TC1 cannot switch to redundant when starting there")
        if scenario.get("switch_after_main_tc1") and scenario.get("main_tc1") == "deploy":
            # Main would deploy and the OBC would never reach redundant.
            raise ValueError("AIS_TC1 switch_after_main_tc1 needs main_tc1 ignore")
        if scenario.get("deploy_mask") is not None:
            _validate_deploy_mask(scenario["deploy_mask"], "AIS_TC1 deploy_mask")
        expect = scenario.get("expect")
        if not isinstance(expect, dict):
            raise ValueError("AIS_TC1 expect must be a dict")
        for field in ("tc1_main", "tc1_red", "switch"):
            if not isinstance(expect.get(field), bool):
                raise ValueError("AIS_TC1 expect %s must be bool" % field)
        if expect.get("final") not in ("deployed", "stored", "partial"):
            raise ValueError("AIS_TC1 expect final must be deployed, stored or partial")
        if expect["final"] == "partial":
            # A partial test must leave at least one checked bit stored, or it
            # silently becomes a full deployment.
            partial_mask = scenario.get("deploy_mask")
            if partial_mask is None or not AIS_TC1_DEPLOY_MASK & ~partial_mask:
                raise ValueError("partial deploy mask must leave a bit of "
                                 "AIS_TC1_DEPLOY_MASK stored")
        if expect["final"] == "deployed":
            # A full-deploy test must clear every bit the OBC checks.
            effective_mask = scenario.get("deploy_mask") or AIS_TC1_DEPLOY_MASK
            if AIS_TC1_DEPLOY_MASK & ~effective_mask:
                raise ValueError("deploy mask must cover AIS_TC1_DEPLOY_MASK "
                                 "when the expected result is deployed")
        _nonnegative_number(scenario.get("tc1_delay_s"), "tc1_delay_s")
        main_address, red_address = scenario_i2c_addresses(scenario)
        validate_i2c_target_config("AIS", PRIMARY_I2C_ID, main_address, PRIMARY_SDA, PRIMARY_SCL)
        validate_i2c_target_config("AIS", PRIMARY_I2C_ID, red_address, PRIMARY_SDA, PRIMARY_SCL)
        if DI_PIN in (PRIMARY_SDA, PRIMARY_SCL):
            raise ValueError("DI_PIN must not collide with AIS I2C GPIOs")
        _nonnegative_number(FAILOVER_SWITCH_DELAY_MS, "FAILOVER_SWITCH_DELAY_MS")
        return True

    # Normal UHF sessions: one target on I2C0.
    primary_address, _ = scenario_i2c_addresses(scenario)
    validate_i2c_target_config("PRIMARY", PRIMARY_I2C_ID, primary_address,
                               PRIMARY_SDA, PRIMARY_SCL)
    if DI_PIN in (PRIMARY_SDA, PRIMARY_SCL):
        raise ValueError("I2C and DI GPIO assignments must not collide")
    return True


def reset_session_report():
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    global COMMAND_HISTORY_DROPPED, RESPONSE_HISTORY_DROPPED
    PRIMARY_WRITES = PRIMARY_READS = 0
    SECONDARY_WRITES = SECONDARY_READS = 0
    COMMAND_HISTORY[:] = []
    COMMAND_HISTORY_DROPPED = 0
    RESPONSE_HISTORY[:] = []
    RESPONSE_HISTORY_DROPPED = 0
    LAST_RESPONSE_BY_TARGET.clear()
    ADDRESS_EVENTS[:] = []


def _make_di_pin():
    if DI_PULL == "up":
        return machine.Pin(DI_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    if DI_PULL == "down":
        return machine.Pin(DI_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
    return machine.Pin(DI_PIN, machine.Pin.IN)


def configure_session_hardware(session):
    scenario = session["scenario"]
    board_profile = session["board_profile"]
    primary_address, secondary_address = scenario_i2c_addresses(scenario)
    slaves = []
    try:
        if scenario["mode"] == "SHARED_I2C_DEPLOYMENT":
            slaves.append(make_slave(
                PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL))
        elif scenario["mode"] == "TC2_FAILOVER":
            # One block only: it starts on main and later moves to redundant.
            # I2C1 stays disabled so no second target is exposed.
            slaves.append(make_slave(
                PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL))
        elif scenario["mode"] == "MAIN_LOST_FAILOVER":
            # Same single block as TC2_FAILOVER: main first, then redundant.
            slaves.append(make_slave(
                PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL))
        elif scenario["mode"] == "AIS_TC1":
            # AIS on I2C0, starting on main (0x47) or redundant (0x48).
            start = secondary_address if scenario["start"] == "red" else primary_address
            slaves.append(make_slave(
                PRIMARY_I2C_ID, start, PRIMARY_SDA, PRIMARY_SCL))
        else:
            # Normal UHF session: one target on I2C0; I2C1 stays disabled.
            slaves.append(make_slave(
                PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL))
        return slaves, _make_di_pin()
    except Exception:
        _disable_slaves(slaves)
        raise


def disable_all_i2c_blocks():
    """Leave both hardware targets inaccessible while awaiting USB config."""
    for base in (PicoI2CSlave._I2C0_BASE, PicoI2CSlave._I2C1_BASE):
        machine.mem32[base | PicoI2CSlave._ATOM_CLR |
                      PicoI2CSlave._IC_ENABLE] = 0x0001


def print_session_ack(session):
    print("ACK=%s" % session["command_id"])


def _read_power(di_pin):
    level = di_pin.value()
    return (level == 1) if DI_ACTIVE_HIGH else (level == 0)


def _record_response(now, start_ms, target, slave, reg):
    global RESPONSE_HISTORY_DROPPED
    if not REPORT:
        return
    response_ms = time.ticks_diff(now, start_ms)
    key = (target, slave.address)
    previous = LAST_RESPONSE_BY_TARGET.get(key)
    if previous is not None and previous[0] == reg:
        if previous[1] is not None:
            entry = RESPONSE_HISTORY[previous[1]]
            entry[1] = response_ms
            entry[5] += 1
        LAST_RESPONSE_BY_TARGET[key] = (reg, previous[1], response_ms)
    elif len(RESPONSE_HISTORY) < REPORT_RESPONSE_LIMIT:
        RESPONSE_HISTORY.append([response_ms, response_ms, target, slave.address, reg, 1])
        LAST_RESPONSE_BY_TARGET[key] = (reg, len(RESPONSE_HISTORY) - 1, response_ms)
    else:
        RESPONSE_HISTORY_DROPPED += 1
        LAST_RESPONSE_BY_TARGET[key] = (reg, None, response_ms)


class RuntimeConsole:
    """Non-blocking USB line reader for commands issued during a session."""

    def __init__(self):
        self._poll = select.poll()
        self._poll.register(sys.stdin, select.POLLIN)
        self._buf = ""
        self._overflow = False
        self._pending = None
        self._pending_ms = 0

    def _stdin_ready(self):
        # ipoll() reuses its result object; poll() would allocate a list on
        # every loop pass and bring on more frequent GC pauses.
        for _ in self._poll.ipoll(0):
            return True
        return False

    def finish(self):
        """Call once I2C is disabled: name a command that was still waiting,
        or only partly received, when the session ended."""
        if self._pending is not None:
            print("RT_DROPPED=%s" % self._pending)
            self._pending = None
        if self._buf or self._overflow:
            # A line cut off by the session end: consume the rest of it so it
            # is not misread as a test command at CONFIG_READY.
            while self._stdin_ready():
                ch = sys.stdin.read(1)
                if ch == "\r" or ch == "\n":
                    break
                if len(self._buf) < USB_COMMAND_MAX_BYTES:
                    self._buf += ch
            print("RT_DROPPED=%s" % self._buf.strip().lower())
            self._buf, self._overflow = "", False

    def service(self, now, read_served):
        """Return a completed command once it is safe to run, else None.

        Reads at most one character per call so a long line never adds more
        than one character's worth of latency to a single loop pass.
        """
        if self._pending is None and self._stdin_ready():
            ch = sys.stdin.read(1)
            if ch == "\r" or ch == "\n":
                line = self._buf.strip().lower()
                overflow = self._overflow
                self._buf, self._overflow = "", False
                if overflow:
                    print("RT_NACK=TOO_LONG")
                elif line:
                    self._pending, self._pending_ms = line, now
            elif len(self._buf) < USB_COMMAND_MAX_BYTES:
                self._buf += ch
            else:
                self._overflow = True
        if self._pending is not None and (
                read_served or
                time.ticks_diff(now, self._pending_ms) >= RUNTIME_COMMAND_DEFER_MS):
            line, self._pending = self._pending, None
            return line
        return None


def _print_runtime_status(session, start_ms, now, power_on, targets, loop_max_us):
    """One-line snapshot; targets is a list of (name, address, sim, active)."""
    parts = ["RT_STATUS t=%.3fs duration=%s test=%s power=%d loop_max_us=%d" %
             (time.ticks_diff(now, start_ms) / 1000, _duration_text(session),
              session["command_id"], power_on, loop_max_us)]
    for name, address, sim, active in targets:
        reads = PRIMARY_READS if name == "UHF" else SECONDARY_READS
        writes = PRIMARY_WRITES if name == "UHF" else SECONDARY_WRITES
        parts.append("%s%s 0x%02X reg=0x%02X phase=%d deployed=%d r=%d w=%d" %
                     (name, "*" if active else "", address, sim.assemble(),
                      sim.sequence_phase, sim.deployed, reads, writes))
    print(" | ".join(parts))


def handle_runtime_command(line, session, start_ms, now, power_on, targets,
                           loop_max_us):
    """Run one runtime command.

    Returns None to keep going; "end" or "abort" for the loop to act on; or
    a validated session to switch to once the current one ends.
    """
    if line == "status":
        _print_runtime_status(session, start_ms, now, power_on, targets,
                              loop_max_us)
    elif line in ("end", "abort"):
        print("RT_ACK=%s" % line)
        return line
    elif line.split()[0] in TEST_COMMAND_LOOKUP:
        try:
            next_session = build_session(make_test_request(line))
            validate_session(next_session)
        except (ValueError, TypeError, KeyError):
            print("RT_NACK=%s" % line)
            return None
        print("RT_ACK=%s" % line)
        return next_session
    else:
        print("RT_NACK=%s" % line)
    return None


def _session_duration_ms(session):
    """Session length in ms, or None to run until end/abort."""
    duration_s = session["duration_s"]
    return None if duration_s is None else duration_s * 1000


def _duration_text(session):
    duration_s = session["duration_s"]
    return "none" if duration_s is None else "%ds" % duration_s


def run_normal_session(session, slaves, di, console):
    """UHF testNN session: one UHF target on I2C0."""
    global PRIMARY_WRITES, PRIMARY_READS
    primary_sim = DeploymentSim(session["scenario"])
    primary_target = "UHF"
    outcome = None          # None, "abort", or the session to switch to
    loop_max_us = 0
    start_ms = time.ticks_ms()
    duration_ms = _session_duration_ms(session)
    last_count_report = start_ms
    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms):
        pass_start_us = time.ticks_us()
        now = time.ticks_ms()
        power_on = _read_power(di)
        read_served = False
        for cmd in slaves[0].read_pending():
            PRIMARY_WRITES += 1
            record_obc_command(now, start_ms, primary_target, slaves[0].address, cmd)
            primary_sim.apply_command(cmd)
        if slaves[0].read_requested():
            reg = primary_sim.assemble()
            slaves[0].send_byte(reg)
            read_served = True
            PRIMARY_READS += 1
            _record_response(now, start_ms, primary_target, slaves[0], reg)
        primary_sim.update(power_on, now)
        if console is not None:
            line = console.service(now, read_served)
            if line is not None:
                targets = [(primary_target, slaves[0].address, primary_sim, True)]
                action = handle_runtime_command(
                    line, session, start_ms, now, power_on, targets, loop_max_us)
                if action == "end":
                    break
                elif action is not None:
                    outcome = action
                    break
        if DEBUG_COUNTS and time.ticks_diff(now, last_count_report) >= 7000:
            print("[I2C_COUNTS] UHF r=%d w=%d" % (PRIMARY_READS, PRIMARY_WRITES))
            last_count_report = now
        loop_max_us = max(loop_max_us, time.ticks_diff(time.ticks_us(), pass_start_us))
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms(), outcome


def run_shared_i2c_deployment(session, slave, di, console):
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    scenario = session["scenario"]
    uhf_address, ais_address = scenario_i2c_addresses(scenario)
    uhf_sim = DeploymentSim(scenario)
    # After the handoff AIS follows the single-cutter model: TC1 deploys it.
    ais_sim = AisDeploymentSim(AIS_TC1_DEPLOY_MASK, _shared_ais_tc1_delay_s(session))
    active_target, active_sim = "UHF", uhf_sim
    handoff_at = None
    outcome = None          # None, "abort", or the session to switch to
    loop_max_us = 0
    start_ms = time.ticks_ms()
    duration_ms = _session_duration_ms(session)
    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms):
        pass_start_us = time.ticks_us()
        now = time.ticks_ms()
        power_on = _read_power(di)
        read_served = False
        for cmd in slave.read_pending():
            record_obc_command(now, start_ms, active_target, slave.address, cmd)
            if active_target == "UHF": PRIMARY_WRITES += 1
            else: SECONDARY_WRITES += 1
            if cmd == SIM_RESET_COMMAND:
                uhf_sim.reset(); ais_sim.reset()
                active_target, active_sim, handoff_at = "UHF", uhf_sim, None
                if slave.address != uhf_address:
                    ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                           slave.address, uhf_address, "SIM_RESET"))
                slave.set_address(uhf_address)
            else:
                active_sim.apply_command(cmd)
        if slave.read_requested():
            reg = active_sim.assemble()
            slave.send_byte(reg)
            read_served = True
            _record_response(now, start_ms, active_target, slave, reg)
            if active_target == "UHF":
                PRIMARY_READS += 1
                if uhf_sim.deployed and handoff_at is None:
                    handoff_at = time.ticks_add(now, SHARED_HANDOFF_DELAY_MS)
            else:
                SECONDARY_READS += 1
        if console is not None:
            line = console.service(now, read_served)
            if line is not None:
                targets = [("UHF", uhf_address, uhf_sim, active_target == "UHF"),
                           ("AIS", ais_address, ais_sim, active_target == "AIS")]
                action = handle_runtime_command(
                    line, session, start_ms, now, power_on, targets, loop_max_us)
                if action == "end":
                    break
                elif action is not None:
                    outcome = action
                    break
        if active_target == "UHF":
            uhf_sim.update_shared_uhf(power_on, now, scenario)
        else:
            ais_sim.update(power_on, now)
        if active_target == "UHF" and handoff_at is not None and time.ticks_diff(now, handoff_at) >= 0:
            slave.set_address(ais_address)
            active_target, active_sim, handoff_at = "AIS", ais_sim, None
            ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                   uhf_address, ais_address,
                                   "UHF deployed read, handoff to AIS"))
        loop_max_us = max(loop_max_us, time.ticks_diff(time.ticks_us(), pass_start_us))
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms(), outcome


def run_tc2_failover_session(session, slave, di, console):
    """TC1 on main; TC2 ACKed on main without deploying, then on redundant."""
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    scenario = session["scenario"]
    main_address, red_address = scenario_i2c_addresses(scenario)
    target = "UHF"          # test20/test21 are UHF-only
    sim = DeploymentSim(scenario)
    switch_at = None        # tick to move to redundant; None = no switch pending
    tc2_on_red = False      # TC2 has been commanded through the redundant address
    outcome = None          # None, "abort", or the session to switch to
    loop_max_us = 0
    start_ms = time.ticks_ms()
    duration_ms = _session_duration_ms(session)
    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms):
        pass_start_us = time.ticks_us()
        now = time.ticks_ms()
        power_on = _read_power(di)
        read_served = False
        bus_activity = False
        for cmd in slave.read_pending():
            bus_activity = True
            record_obc_command(now, start_ms, target, slave.address, cmd)
            if target == "UHF": PRIMARY_WRITES += 1
            else: SECONDARY_WRITES += 1
            if cmd == SIM_RESET_COMMAND:
                sim.reset()
                switch_at, tc2_on_red = None, False
                if slave.address != main_address:
                    ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                           slave.address, main_address, "SIM_RESET"))
                    slave.set_address(main_address)
                continue
            sim.apply_command(cmd)
            if slave.address == main_address:
                # Arm the switch only for TC2 after the TC1 pair is out, so an
                # unexpected early TC2 cannot move the target off main.
                if (cmd & (1 << TC2)) and sim.sequence_phase == 1 and switch_at is None:
                    switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
            elif cmd & (1 << TC2):
                tc2_on_red = True
        if slave.read_requested():
            bus_activity = True
            reg = sim.assemble()
            slave.send_byte(reg)
            read_served = True
            _record_response(now, start_ms, target, slave, reg)
            if target == "UHF": PRIMARY_READS += 1
            else: SECONDARY_READS += 1
        if console is not None:
            line = console.service(now, read_served)
            if line is not None:
                targets = [(target, slave.address, sim, True)]
                action = handle_runtime_command(
                    line, session, start_ms, now, power_on, targets, loop_max_us)
                if action == "end":
                    break
                elif action is not None:
                    outcome = action
                    break
        sim.update_tc2_failover(power_on, now, scenario, tc2_on_red)
        if switch_at is not None:
            if bus_activity or slave.is_busy():
                # Never re-address mid-transfer: restart the idle guard.
                switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
            elif time.ticks_diff(now, switch_at) >= 0:
                slave.set_address(red_address)
                switch_at = None
                ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                       main_address, red_address,
                                       "TC2 ACKed on main, bus idle"))
        loop_max_us = max(loop_max_us, time.ticks_diff(time.ticks_us(), pass_start_us))
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms(), outcome


def run_main_lost_session(session, slave, di, console):
    """TC1 on main; main then disappears so TC2 is NACKed there and goes to red."""
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    scenario = session["scenario"]
    main_address, red_address = scenario_i2c_addresses(scenario)
    target = "UHF"          # test20/test21 are UHF-only
    sim = DeploymentSim(scenario)
    switch_at = None        # tick to move to redundant; None = no switch pending
    tc2_on_red = False      # TC2 has been commanded through the redundant address
    outcome = None          # None, "abort", or the session to switch to
    loop_max_us = 0
    start_ms = time.ticks_ms()
    duration_ms = _session_duration_ms(session)
    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms):
        pass_start_us = time.ticks_us()
        now = time.ticks_ms()
        power_on = _read_power(di)
        read_served = False
        bus_activity = False
        for cmd in slave.read_pending():
            bus_activity = True
            record_obc_command(now, start_ms, target, slave.address, cmd)
            if target == "UHF": PRIMARY_WRITES += 1
            else: SECONDARY_WRITES += 1
            if cmd == SIM_RESET_COMMAND:
                sim.reset()
                switch_at, tc2_on_red = None, False
                if slave.address != main_address:
                    ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                           slave.address, main_address, "SIM_RESET"))
                    slave.set_address(main_address)
                continue
            # A TC2 that reaches main is still applied (it shows in the
            # register) but cannot deploy; the report flags it as a late switch.
            sim.apply_command(cmd)
            if slave.address == red_address and cmd & (1 << TC2):
                tc2_on_red = True
        if slave.read_requested():
            bus_activity = True
            reg = sim.assemble()
            slave.send_byte(reg)
            read_served = True
            _record_response(now, start_ms, target, slave, reg)
            if target == "UHF": PRIMARY_READS += 1
            else: SECONDARY_READS += 1
            # Arm the switch once the OBC has seen the TC1 pair deployed on
            # main, so main is gone before its TC2 write on the next tick.
            if (slave.address == main_address and switch_at is None and
                    sim.sequence_phase == 1 and reg & 0x03 == 0):
                switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
        if console is not None:
            line = console.service(now, read_served)
            if line is not None:
                targets = [(target, slave.address, sim, True)]
                action = handle_runtime_command(
                    line, session, start_ms, now, power_on, targets, loop_max_us)
                if action == "end":
                    break
                elif action is not None:
                    outcome = action
                    break
        sim.update_tc2_failover(power_on, now, scenario, tc2_on_red)
        if switch_at is not None:
            if bus_activity or slave.is_busy():
                # Never re-address mid-transfer: restart the idle guard.
                switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
            elif time.ticks_diff(now, switch_at) >= 0:
                slave.set_address(red_address)
                switch_at = None
                ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                       main_address, red_address,
                                       "TC1 pair read on main, bus idle"))
        loop_max_us = max(loop_max_us, time.ticks_diff(time.ticks_us(), pass_start_us))
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms(), outcome


def run_ais_session(session, slave, di, console):
    """AIS aisNN session: single cutter (TC1) on I2C0, main and/or redundant."""
    global SECONDARY_WRITES, SECONDARY_READS
    scenario = session["scenario"]
    main_address, red_address = scenario_i2c_addresses(scenario)
    start_address = red_address if scenario["start"] == "red" else main_address
    sim = AisDeploymentSim(scenario["deploy_mask"] or AIS_TC1_DEPLOY_MASK,
                           scenario["tc1_delay_s"])
    switch_at = None        # tick to move to redundant; None = no switch pending
    outcome = None          # None, "abort", or the session to switch to
    loop_max_us = 0
    start_ms = time.ticks_ms()
    duration_ms = _session_duration_ms(session)
    while (duration_ms is None or
           time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms):
        pass_start_us = time.ticks_us()
        now = time.ticks_ms()
        power_on = _read_power(di)
        read_served = False
        bus_activity = False
        for cmd in slave.read_pending():
            bus_activity = True
            record_obc_command(now, start_ms, "AIS", slave.address, cmd)
            SECONDARY_WRITES += 1
            if cmd == SIM_RESET_COMMAND:
                sim.reset()
                switch_at = None
                if slave.address != start_address:
                    ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                           slave.address, start_address, "SIM_RESET"))
                    slave.set_address(start_address)
                continue
            on_main = slave.address == main_address
            policy = scenario["main_tc1"] if on_main else scenario["red_tc1"]
            # An ignored TC1 is ACKed by the hardware but has no effect, as a
            # controller whose cutter never fires. TC2 has no effect on AIS.
            if not (cmd & (1 << TC1) and policy == "ignore"):
                sim.apply_command(cmd)
            if (on_main and cmd & (1 << TC1) and scenario["switch_after_main_tc1"]
                    and switch_at is None):
                switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
        if slave.read_requested():
            bus_activity = True
            reg = sim.assemble()
            slave.send_byte(reg)
            read_served = True
            _record_response(now, start_ms, "AIS", slave, reg)
            SECONDARY_READS += 1
        if console is not None:
            line = console.service(now, read_served)
            if line is not None:
                targets = [("AIS", slave.address, sim, True)]
                action = handle_runtime_command(
                    line, session, start_ms, now, power_on, targets, loop_max_us)
                if action == "end":
                    break
                elif action is not None:
                    outcome = action
                    break
        sim.update(power_on, now)
        if switch_at is not None:
            if bus_activity or slave.is_busy():
                # Never re-address mid-transfer: restart the idle guard.
                switch_at = time.ticks_add(now, FAILOVER_SWITCH_DELAY_MS)
            elif time.ticks_diff(now, switch_at) >= 0:
                slave.set_address(red_address)
                switch_at = None
                ADDRESS_EVENTS.append((time.ticks_diff(now, start_ms),
                                       main_address, red_address,
                                       "TC1 reached main, bus idle"))
        loop_max_us = max(loop_max_us, time.ticks_diff(time.ticks_us(), pass_start_us))
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms(), outcome


def _disable_slaves(slaves):
    for slave in slaves:
        slave.disable()


def run_session(session):
    """Run one session; return the next session if one was requested at runtime."""
    reset_session_report()
    # Collect the previous session's garbage now rather than mid-session.
    gc.collect()
    slaves = []
    try:
        slaves, di = configure_session_hardware(session)
        console = RuntimeConsole() if RUNTIME_COMMANDS else None
        print_session_ack(session)
        if session["scenario"]["mode"] == "SHARED_I2C_DEPLOYMENT":
            start_ms, end_ms, outcome = run_shared_i2c_deployment(
                session, slaves[0], di, console)
        elif session["scenario"]["mode"] == "TC2_FAILOVER":
            start_ms, end_ms, outcome = run_tc2_failover_session(
                session, slaves[0], di, console)
        elif session["scenario"]["mode"] == "MAIN_LOST_FAILOVER":
            start_ms, end_ms, outcome = run_main_lost_session(
                session, slaves[0], di, console)
        elif session["scenario"]["mode"] == "AIS_TC1":
            start_ms, end_ms, outcome = run_ais_session(
                session, slaves[0], di, console)
        else:
            start_ms, end_ms, outcome = run_normal_session(
                session, slaves, di, console)
        _disable_slaves(slaves)
        slaves = []
        if console is not None:
            console.finish()
        # I2C is already off. A failure while printing must not lose the
        # completion line or a runtime testNN switch, so each print is guarded.
        verdict = None
        if outcome != "abort" and REPORT:
            try:
                verdict = print_test_report(end_ms, start_ms, session)
            except Exception as error:
                print("REPORT_ERROR reason=%s" % error)
        try:
            print_session_result(end_ms, start_ms, session,
                                 "aborted" if outcome == "abort" else "complete",
                                 verdict)
        except Exception as error:
            print("RESULT_ERROR reason=%s" % error)
        if outcome == "abort":
            print("SESSION_ABORTED")
            return None
        print("SESSION_COMPLETE")
        return outcome
    finally:
        _disable_slaves(slaves)


def main():
    disable_all_i2c_blocks()
    next_session = None
    while True:
        session, next_session = next_session, None
        if session is None:
            try:
                request = (wait_for_usb_configuration()
                           if USB_SCENARIO_CONTROL else {
                               "command_id": "configured",
                               "scenario": ACTIVE_SCENARIO,
                           })
                session = build_session(request)
                validate_session(session)
            except (ValueError, TypeError, KeyError):
                print("NACK=INVALID")
                continue
        try:
            next_session = run_session(session)
        except Exception as error:
            print("RUNTIME_ERROR reason=%s" % error)


if __name__ == "__main__":
    main()
