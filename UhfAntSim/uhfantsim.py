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
#    bit7  SIG  RO   bench signature when READ_SIGNATURE=True
#
#  Master interaction (single-register device, no register-pointer byte):
#    - Master WRITE of 1 byte  -> command; only TC bits (0x30) are applied.
#    - Master READ  of 1 byte  -> current telemetry (freshly assembled).
#  Examples:  write 0x10 -> TC1 on;  write 0x20 -> TC2 on;  write 0x30 -> both.
#  With READ_SIGNATURE=True, staged sequential reads include:
#       0x8F: all antennas stored
#       0x9F: all stored, TC1 commanded
#       0x9C: ANT1/ANT2 deployed, ANT3/ANT4 stored, TC1 commanded
#       0xAC: first pair deployed, TC2 commanded
#       0xA0: all four deployed, TC2 commanded
#
#  ---------------------------------------------------------------------------
#  DUAL TARGETS / SELECTABLE MAIN OR REDUNDANT ADDRESSES
#  The RP2040 I2C hardware block matches exactly ONE slave address (single
#  IC_SAR register); it has no native multi-address support. This simulator
#  therefore selects one address per hardware block at startup:
#
#       normal scenarios: I2C0 @ 0x45 and I2C1 @ 0x47
#       redundant_deploy: I2C0 @ 0x46 and I2C1 @ 0x48
#
#  In redundant_deploy the main addresses are intentionally absent, allowing
#  the OBC's DCS1 attempt to fail before it retries TC1 and TC2 through DCS2.
#
#  ---------------------------------------------------------------------------
#  Register-level slave driver adapted from kyordhel/i2cslave (MIT License,
#  Mauricio Matamoros). Verified against the RP2040 datasheet register map.
# =============================================================================

import machine
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
DUAL_ADDRESS = True          # True: expose one target on each I2C block.
                             # False: expose only the primary block.

PRIMARY_I2C_ID   = 0         # UHF target hardware block
PRIMARY_MAIN_ADDRESS = 0x45       # UHF DCS1/main
PRIMARY_REDUNDANT_ADDRESS = 0x46  # UHF DCS2/backup
PRIMARY_SDA      = 20        # GP20 (I2C0 SDA pins: 0,4,8,12,16,20)
PRIMARY_SCL      = 21        # GP21 (I2C0 SCL pins: 1,5,9,13,17,21)

SECONDARY_I2C_ID = 1         # AIS target hardware block
SECONDARY_MAIN_ADDRESS = 0x47       # AIS DCS1/main
SECONDARY_REDUNDANT_ADDRESS = 0x48  # AIS DCS2/backup
SECONDARY_SDA    = 26        # GP26 (I2C1 SDA pins: 2,6,10,14,18,26)
SECONDARY_SCL    = 27        # GP27 (I2C1 SCL pins: 3,7,11,15,19,27)

# shared_i2c_deployment uses only Pico I2C1. It starts as UHF main (0x45)
# and, after serving the final UHF deployed status, re-addresses the same
# hardware block as AIS main (0x47). The harness must put both OBC transactions
# on this physical bus; software cannot bridge two electrically separate buses.
SHARED_I2C_ID = 1
SHARED_SDA = 26
SHARED_SCL = 27
SHARED_UHF_ADDRESS = PRIMARY_MAIN_ADDRESS
SHARED_AIS_ADDRESS = SECONDARY_MAIN_ADDRESS
SHARED_HANDOFF_DELAY_MS = 20

# True exercises successful UHF power-on deployment. False leaves UHF stored
# during the OBC's power-on poll so that the OBC proceeds to TC1 then TC2.
SHARED_UHF_POWER_DEPLOY_SUCCESS = True

INTERNAL_PULLUPS =True
# Bus data rate is set by the MASTER (target 100 kHz). A slave does not drive
# the clock, so there is nothing to configure here; just run the master at 100k.

# --- ANT_POWER digital input ------------------------------------------------
DI_PIN            = 15       # GPIO used to sense antenna power
DI_ACTIVE_HIGH    = True     # True: pin HIGH  -> ANT_POWER_ON
                             #       pin LOW   -> ANT_POWER_LOW
DI_PULL           = "down"     # "down" | "up" | None  (internal pull on DI pin)

# --- Board profile ----------------------------------------------------------
#   The same simulator serves two boards. They share an identical register map
#   and driver; only which triggers are legitimate differs:
#     "UHF" : antennas may deploy on power-on OR on thermal cutters.
#     "AIS" : antennas NEVER deploy on power-on -- only on TC1/TC2/both.
#   Under the "AIS" profile, selecting a POWER_ON-triggered scenario is rejected
#   at startup so an AIS board can't be power-deployed by mistake.
BOARD_PROFILE = "UHF"        # "UHF" | "AIS"

# Configure one finite test session over USB CDC before enabling I2C.
USB_SCENARIO_CONTROL = True
DEFAULT_SESSION_DURATION_S = 200
USB_COMMAND_MAX_BYTES = 256

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
        "mode": "POWER_ON_ALL", "delay_s": 5, "address_set": "main"
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
    # Power-on alone does nothing. Only TC1 is accepted on red addr and deploys ANT1/ANT2.
    # power-on deploys tc2 pair
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
    # Power-on alone does nothing. Only TC2 is accepted on red addrnand deploys ANT3/ANT4.
    # power-on deploys tc1.
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
        "uhf_power_delay_s": 5,
        "tc1_delay_s": TC1_DEPLOY_DELAY_S,
        "tc2_delay_s": TC2_DEPLOY_DELAY_S,
        "address_set": "shared",
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
}

ACTIVE_SCENARIO = "test02_sequential_deploy"

# Host/debug reset command. This is intentionally outside the real writable
# TC bit range so ordinary cutter writes cannot re-arm the simulator.
SIM_RESET_COMMAND = 0x80

# --- Debug ------------------------------------------------------------------
DEBUG        = False     # prints add service-loop latency; enable only for bench debug
DEBUG_READS  = False     # also print every master read (can be very chatty)
DEBUG_COUNTS = False     # enable only for periodic bench activity counters
READ_SIGNATURE = True    # set bit7 on reads so OBC first=0x8f proves Pico data
POLL_MS      = 0         # 0 = busy-loop; minimizes I2C clock-stretch latency

# --- USB serial test report ------------------------------------------------
# MicroPython print() output is carried by the Pico micro-USB CDC/REPL serial
# connection. Keep command history bounded so a noisy or long run cannot
# consume unbounded Pico RAM. The report is emitted once, 200 seconds after
# the simulator starts servicing the bus.
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
        if READ_SIGNATURE: val |= 0x80
        # bit6 unused = 0; bit7 is an optional bench-only signature.
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


def print_test_report(now_ms, start_ms, session):
    """Print a concise report for this completed session on USB serial."""
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000
    scenario_name = session["name"]
    scenario = session["scenario"]
    board_profile = session["board_profile"]
    profile_writes = (PRIMARY_WRITES if board_profile == "UHF"
                      else SECONDARY_WRITES)
    profile_reads = (PRIMARY_READS if board_profile == "UHF"
                     else SECONDARY_READS)

    print("")
    print("=" * 64)
    print(" ANTENNA DEPLOYMENT TEST REPORT")
    print("=" * 64)
    print(" Test       : %s" % scenario_name)
    print(" Mode       : %s" % scenario["mode"])
    print(" Board      : %s" % board_profile)
    print(" Addresses  : %s" % scenario["address_set"])
    print(" Elapsed    : %.3f s" % elapsed_s)

    print("-" * 64)
    print(" OBC COMMANDS (%d received by %s)" %
          (profile_writes, board_profile))
    displayed_commands = 0
    for entry in COMMAND_HISTORY:
        elapsed_ms, target, address, command, name = entry
        if target != board_profile:
            continue
        displayed_commands += 1
        print("  %02d. %7.3f s | 0x%02X | %-9s (0x%02X)" %
              (displayed_commands, elapsed_ms / 1000,
               address, name, command))
    if displayed_commands == 0:
        print("  None recorded")
    if COMMAND_HISTORY_DROPPED:
        print("  Note: %d command(s) exceeded the history limit" %
              COMMAND_HISTORY_DROPPED)

    print("-" * 64)
    print(" PICO RESPONSES (%d status reads by %s)" %
          (profile_reads, board_profile))
    displayed_responses = 0
    for entry in RESPONSE_HISTORY:
        first_ms, last_ms, target, address, status, reads = entry
        if target != board_profile:
            continue
        displayed_responses += 1
        feedback = status & 0x0f
        if feedback == 0x0f:
            state = "ALL STORED"
        elif feedback == 0x0c:
            state = "TC1 PAIR DEPLOYED"
        elif feedback == 0x03:
            state = "TC2 PAIR DEPLOYED"
        elif feedback == 0x00:
            state = "ALL DEPLOYED"
        else:
            state = "MIXED FEEDBACK"
        print("  %02d. %7.3f-%7.3f s | 0x%02X | status 0x%02X | %s" %
              (displayed_responses, first_ms / 1000, last_ms / 1000,
               address, status, state))
        print("      TC1 pair: %-12s | TC2 pair: %-12s | reads: %d" %
              ("DEPLOYED" if status & 0x03 == 0 else "NOT DEPLOYED",
               "DEPLOYED" if status & 0x0c == 0 else "NOT DEPLOYED", reads))
    if displayed_responses == 0:
        print("  No status byte was returned to the OBC")
    if RESPONSE_HISTORY_DROPPED:
        print("  Note: %d response transition(s) exceeded the history limit" %
              RESPONSE_HISTORY_DROPPED)

    # BOARD_PROFILE selects the board whose last actually-served register byte
    # is promoted as the test result. Do not call assemble() here: that could
    # report a state which the OBC never read.
    profile_result = None
    for key, response in LAST_RESPONSE_BY_TARGET.items():
        target, address = key
        status, _, response_ms = response
        if target == board_profile:
            if profile_result is None or response_ms >= profile_result[0]:
                profile_result = (response_ms, address, status)
    if profile_result is None:
        print("-" * 64)
        print(" FINAL %s RESULT" % board_profile)
        print("  No status register response was read by the OBC")
    else:
        response_ms, address, status = profile_result
        feedback = status & 0x0f
        if feedback == 0x0f:
            state = "ALL STORED"
        elif feedback == 0x0c:
            state = "TC1 PAIR DEPLOYED"
        elif feedback == 0x03:
            state = "TC2 PAIR DEPLOYED"
        elif feedback == 0x00:
            state = "ALL DEPLOYED"
        else:
            state = "MIXED FEEDBACK"
        print("-" * 64)
        print(" FINAL %s RESULT" % board_profile)
        print("  Address    : 0x%02X" % address)
        print("  Last read  : %.3f s" % (response_ms / 1000))
        print("  Status     : 0x%02X" % status)
        print("  Deployment : %s" % state)
        print("  TC1 pair   : %s" %
              ("DEPLOYED" if status & 0x03 == 0 else "NOT DEPLOYED"))
        print("  TC2 pair   : %s" %
              ("DEPLOYED" if status & 0x0c == 0 else "NOT DEPLOYED"))
    print("=" * 64)
    print(" END OF REPORT")
    print("=" * 64)


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


def wait_for_usb_configuration():
    print("")
    print("CONFIG_READY")
    line = sys.stdin.readline()
    if line is None:
        raise ValueError("No USB command received")
    command = line.strip().lower()
    if not command:
        raise ValueError("Empty USB command")
    if len(command) > USB_COMMAND_MAX_BYTES:
        raise ValueError("USB command is too long")
    scenario_name = TEST_COMMAND_LOOKUP.get(command)
    if scenario_name is None:
        raise ValueError("Unknown test command: %s" % command)
    return {"command_id": command, "scenario": scenario_name}


def build_session(request):
    scenario_name = request.get("scenario")
    if scenario_name not in SCENARIOS:
        raise ValueError("Unknown scenario: %r" % scenario_name)
    scenario = dict(SCENARIOS[scenario_name])
    for field in ("tc1_delay_s", "tc2_delay_s", "power_delay_s",
                  "uhf_power_delay_s"):
        if field in request:
            scenario[field] = request[field]
    return {
        "command_id": request["command_id"],
        "name": scenario_name,
        "scenario": scenario,
        "board_profile": request.get("board_profile", BOARD_PROFILE),
        "duration_s": request.get("duration_s", DEFAULT_SESSION_DURATION_S),
    }


def scenario_i2c_addresses(scenario):
    address_set = scenario.get("address_set")
    if address_set == "main":
        return PRIMARY_MAIN_ADDRESS, SECONDARY_MAIN_ADDRESS
    if address_set == "redundant":
        return PRIMARY_REDUNDANT_ADDRESS, SECONDARY_REDUNDANT_ADDRESS
    if address_set == "shared":
        return SHARED_UHF_ADDRESS, SHARED_AIS_ADDRESS
    raise ValueError("Unsupported address_set: %r" % address_set)


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
    if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) or duration_s <= 0:
        raise ValueError("duration_s must be greater than zero")
    if DRIVER != "register":
        raise ValueError("Only DRIVER='register' is supported")
    if POLL_MS < 0 or REPORT_COMMAND_LIMIT < 0 or REPORT_RESPONSE_LIMIT < 0:
        raise ValueError("poll and report limits must be non-negative")
    if not isinstance(REPORT, bool):
        raise ValueError("REPORT must be True or False")

    mode = scenario.get("mode")
    if mode not in ("POWER_ON_ALL", "SEQUENTIAL_TC", "NO_DEPLOY",
                    "PAIR_TEST", "SHARED_I2C_DEPLOYMENT"):
        raise ValueError("Unsupported mode: %r" % mode)
    scenario_i2c_addresses(scenario)
    if mode == "POWER_ON_ALL":
        _nonnegative_number(scenario.get("delay_s"), "POWER_ON_ALL delay_s")
    if mode in ("SEQUENTIAL_TC", "SHARED_I2C_DEPLOYMENT"):
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
        validate_i2c_target_config("SHARED", SHARED_I2C_ID, SHARED_UHF_ADDRESS, SHARED_SDA, SHARED_SCL)
        validate_i2c_target_config("SHARED", SHARED_I2C_ID, SHARED_AIS_ADDRESS, SHARED_SDA, SHARED_SCL)
        if DI_PIN in (SHARED_SDA, SHARED_SCL):
            raise ValueError("DI_PIN must not collide with shared I2C GPIOs")
        _nonnegative_number(SHARED_HANDOFF_DELAY_MS, "SHARED_HANDOFF_DELAY_MS")
        return True

    if board_profile == "AIS" and not DUAL_ADDRESS:
        raise ValueError("AIS profile requires DUAL_ADDRESS=True")
    primary_address, secondary_address = scenario_i2c_addresses(scenario)
    targets = [("PRIMARY", PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL)]
    if DUAL_ADDRESS:
        targets.append(("SECONDARY", SECONDARY_I2C_ID, secondary_address, SECONDARY_SDA, SECONDARY_SCL))
    for target in targets:
        validate_i2c_target_config(*target)
    ids = [target[1] for target in targets]
    addresses = [target[2] for target in targets]
    pins = [pin for target in targets for pin in target[3:5]]
    if len(set(ids)) != len(ids):
        raise ValueError("AIS and UHF must use different Pico I2C blocks")
    if len(set(addresses)) != len(addresses):
        raise ValueError("AIS and UHF I2C addresses must differ")
    if len(set(pins)) != len(pins) or DI_PIN in pins:
        raise ValueError("I2C and DI GPIO assignments must not collide")
    if board_profile == "AIS" and mode == "POWER_ON_ALL":
        raise ValueError("AIS profile never deploys on power-on (%s)" % scenario_name)
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


def _make_di_pin():
    if DI_PULL == "up":
        return machine.Pin(DI_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    if DI_PULL == "down":
        return machine.Pin(DI_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
    return machine.Pin(DI_PIN, machine.Pin.IN)


def configure_session_hardware(session):
    scenario = session["scenario"]
    primary_address, secondary_address = scenario_i2c_addresses(scenario)
    slaves = []
    try:
        if scenario["mode"] == "SHARED_I2C_DEPLOYMENT":
            slaves.append(make_slave(
                SHARED_I2C_ID, primary_address, SHARED_SDA, SHARED_SCL))
        else:
            slaves.append(make_slave(
                PRIMARY_I2C_ID, primary_address, PRIMARY_SDA, PRIMARY_SCL))
            if DUAL_ADDRESS:
                slaves.append(make_slave(
                    SECONDARY_I2C_ID, secondary_address,
                    SECONDARY_SDA, SECONDARY_SCL))
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


def run_normal_session(session, slaves, di):
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    primary_sim = DeploymentSim(session["scenario"])
    secondary_sim = DeploymentSim(session["scenario"])
    start_ms = time.ticks_ms()
    duration_ms = session["duration_s"] * 1000
    last_count_report = start_ms
    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        now = time.ticks_ms()
        power_on = _read_power(di)
        for cmd in slaves[0].read_pending():
            PRIMARY_WRITES += 1
            record_obc_command(now, start_ms, "UHF", slaves[0].address, cmd)
            primary_sim.apply_command(cmd)
        if slaves[0].read_requested():
            reg = primary_sim.assemble()
            slaves[0].send_byte(reg)
            PRIMARY_READS += 1
            _record_response(now, start_ms, "UHF", slaves[0], reg)
        if len(slaves) > 1:
            for cmd in slaves[1].read_pending():
                SECONDARY_WRITES += 1
                record_obc_command(now, start_ms, "AIS", slaves[1].address, cmd)
                secondary_sim.apply_command(cmd)
            if slaves[1].read_requested():
                reg = secondary_sim.assemble()
                slaves[1].send_byte(reg)
                SECONDARY_READS += 1
                _record_response(now, start_ms, "AIS", slaves[1], reg)
        primary_sim.update(power_on, now)
        if len(slaves) > 1:
            secondary_sim.update(power_on, now)
        if DEBUG_COUNTS and time.ticks_diff(now, last_count_report) >= 7000:
            print("[I2C_COUNTS] primary r=%d w=%d | secondary r=%d w=%d" %
                  (PRIMARY_READS, PRIMARY_WRITES, SECONDARY_READS, SECONDARY_WRITES))
            last_count_report = now
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms()


def run_shared_i2c_deployment(session, slave, di):
    global PRIMARY_WRITES, PRIMARY_READS, SECONDARY_WRITES, SECONDARY_READS
    scenario = session["scenario"]
    uhf_sim = DeploymentSim(scenario)
    ais_sim = DeploymentSim(scenario)
    active_target, active_sim = "UHF", uhf_sim
    handoff_at = None
    start_ms = time.ticks_ms()
    duration_ms = session["duration_s"] * 1000
    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        now = time.ticks_ms()
        power_on = _read_power(di)
        for cmd in slave.read_pending():
            record_obc_command(now, start_ms, active_target, slave.address, cmd)
            if active_target == "UHF": PRIMARY_WRITES += 1
            else: SECONDARY_WRITES += 1
            if cmd == SIM_RESET_COMMAND:
                uhf_sim.reset(); ais_sim.reset()
                active_target, active_sim, handoff_at = "UHF", uhf_sim, None
                slave.set_address(SHARED_UHF_ADDRESS)
            else:
                active_sim.apply_command(cmd)
        if slave.read_requested():
            reg = active_sim.assemble()
            slave.send_byte(reg)
            _record_response(now, start_ms, active_target, slave, reg)
            if active_target == "UHF":
                PRIMARY_READS += 1
                if uhf_sim.deployed and handoff_at is None:
                    handoff_at = time.ticks_add(now, SHARED_HANDOFF_DELAY_MS)
            else:
                SECONDARY_READS += 1
        if active_target == "UHF":
            uhf_sim.update_shared_uhf(power_on, now, scenario)
        else:
            ais_sim.update_sequential(power_on, now, scenario)
        if active_target == "UHF" and handoff_at is not None and time.ticks_diff(now, handoff_at) >= 0:
            slave.set_address(SHARED_AIS_ADDRESS)
            active_target, active_sim, handoff_at = "AIS", ais_sim, None
        time.sleep_ms(POLL_MS)
    return start_ms, time.ticks_ms()


def _disable_slaves(slaves):
    for slave in slaves:
        slave.disable()


def run_session(session):
    reset_session_report()
    slaves = []
    try:
        slaves, di = configure_session_hardware(session)
        print_session_ack(session)
        if session["scenario"]["mode"] == "SHARED_I2C_DEPLOYMENT":
            start_ms, end_ms = run_shared_i2c_deployment(session, slaves[0], di)
        else:
            start_ms, end_ms = run_normal_session(session, slaves, di)
        _disable_slaves(slaves)
        slaves = []
        if REPORT:
            print_test_report(end_ms, start_ms, session)
        print("SESSION_COMPLETE")
    finally:
        _disable_slaves(slaves)


def main():
    disable_all_i2c_blocks()
    while True:
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
            run_session(session)
        except Exception as error:
            print("RUNTIME_ERROR reason=%s" % error)


if __name__ == "__main__":
    main()
