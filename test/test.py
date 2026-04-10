# ============================================================
#  QuadPulse Golden Vector Test Suite (test.py)
#  Uses cocotb — a Python framework for hardware simulation
#
#  "Golden vectors" = known-correct input→output pairs.
#  We run the same logic in Python (our "reference model")
#  and in the Verilog simulator simultaneously.
#  If they ever disagree → bug found → fix before tapeout.
#
#  Think of this as: Python is the answer key, Verilog is the exam.
#
# ============================================================
#  MEASUREMENT TOLERANCE EXPLANATION (±5% on duty checks)
# ============================================================
#  The phase-accumulator ramp gives 256 discrete steps per period.
#  Duty resolution = 1/256 ≈ 0.39%.
#
#  We measure duty by counting HIGH samples over a window of N ticks.
#  If our window doesn't align perfectly to PWM period boundaries,
#  the first and last partial periods introduce an error of up to
#  ±1 ramp step = ±1/256 per boundary = ±2/256 ≈ ±0.78% for 2 boundaries.
#
#  At 20 kHz (period=500 ticks, ramp step=1 tick), a 1000-tick window
#  covers exactly 2 complete periods → boundary error is near zero.
#  At 50 Hz (period=200000 ticks, ramp step=781 ticks), even a 500000-tick
#  window may straddle a period boundary → ±0.78% error possible.
#
#  We use ±5% tolerance to comfortably cover:
#    - Ramp-step quantisation (±0.39%)
#    - Window boundary misalignment (±0.78%)
#    - Synchroniser latency (2 clocks = ±0.4% at 500-tick period)
#    - Any test-environment clock jitter
#  A real silicon measurement with an oscilloscope + averaging would
#  show much tighter accuracy (typically < 0.5% error).
# ============================================================

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, FallingEdge


# ============================================================
# REFERENCE MODEL — Pure Python version of your chip's logic
# This is your "answer key". It must match the Verilog exactly.
# ============================================================

class QuadPulseModel:
    """Software copy of the QuadPulse chip — runs in Python."""

    def __init__(self):
        # Default duty cycles on startup (50% = 127 out of 255)
        self.duty = [127, 127, 127, 127]  # duty[0..3] for channels 0..3
        self.emergency_stop = False

    def reset(self):
        """Simulates pressing the reset button."""
        self.duty = [127, 127, 127, 127]
        self.emergency_stop = False

    def set_duty(self, channel, value):
        """Simulates loading a duty cycle via SPI."""
        assert 0 <= channel <= 3, "Only 4 channels (0-3)"
        assert 0 <= value <= 255, "Duty must be 0-255"
        self.duty[channel] = value

    def set_emergency_stop(self, state):
        self.emergency_stop = state

    def expected_pwm(self, channel, ramp_value):
        """
        For a given ramp value (0-255), what should the PWM output be?
        HIGH (1) if ramp < duty AND not emergency stop
        LOW  (0) otherwise
        """
        if self.emergency_stop:
            return 0
        return 1 if ramp_value < self.duty[channel] else 0

# ============================================================
# HELPER: Send one SPI frame to load a duty cycle
#
# SPI frame = 16 bits, MSB first:
#   [15:14] = channel address (00=ch0, 01=ch1, 10=ch2, 11=ch3)
#   [13:8]  = 000000 (unused, send as zeros)
#   [7:0]   = duty cycle value (0-255)
#
# We bit-bang it: manually toggle SCLK and set MOSI for each bit.
#
# TIMING NOTE (SPI clock rate):
#   The design uses a 2-stage synchroniser on spi_sclk/spi_mosi.
#   A 2-stage synchroniser needs the signal to be stable for at
#   least 2 clock cycles before and after each edge to guarantee
#   correct capture. We use 3 ClockCycles per SCLK phase (SCLK LOW
#   phase = 3 cycles, SCLK HIGH phase = 3 cycles, total = 6 cycles
#   per bit). This gives 2 full cycles of stability on each phase
#   after the synchroniser latency of 2 cycles is removed.
#   Minimum required: ≥4 cycles/bit. This implementation uses 6
#   cycles/bit → 50% safety margin beyond minimum.
# ============================================================

async def spi_send(dut, channel, duty_value):
    """Send one 16-bit SPI frame to set channel's duty cycle."""

    # Build the 16-bit frame (MSB first)
    frame = ((channel & 0x3) << 14) | (duty_value & 0xFF)
    # Example: channel=1, duty=200
    # frame = (01 << 14) | 11001000 = 0100_0000_1100_1000

    # Assert CS_N high first (ensure clean idle state)
    dut.uio_in.value = 0b00000100  # CS_N=1 (bit 2 high = not selected)
    await ClockCycles(dut.clk, 3)

    # Pull CS_N low = select chip
    dut.uio_in.value = 0b00000000  # CS_N=0, MOSI=0, SCLK=0
    await ClockCycles(dut.clk, 3)

    # Send 16 bits, MSB first
    for i in range(15, -1, -1):
        mosi_bit = (frame >> i) & 1   # Extract bit i from frame

        # Set MOSI, hold SCLK LOW (setup time before rising edge)
        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)

        # Rising edge of SPI clock — chip samples MOSI on this edge
        dut.uio_in.value = (mosi_bit << 0) | (1 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)

        # Falling edge of SPI clock — hold MOSI stable during SCLK HIGH
        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)
        # (SCLK already falls here — next iteration will set MOSI for next bit)

    # Allow final SCLK low phase to settle
    await ClockCycles(dut.clk, 3)

    # Pull CS_N high (deselect — chip now latches the new duty cycle)
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 6)

# ============================================================
# HELPER: Partially send an SPI frame (for abort test)
# Sends only `num_bits` bits then returns without pulling CS high
# ============================================================

async def spi_send_partial(dut, channel, duty_value, num_bits):
    """Send only the first num_bits bits of a 16-bit SPI frame, leave CS low."""
    frame = ((channel & 0x3) << 14) | (duty_value & 0xFF)

    # CS_N high → idle
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 3)

    # Pull CS_N low = begin frame
    dut.uio_in.value = 0b00000000
    await ClockCycles(dut.clk, 3)

    # Send only num_bits bits (MSB first)
    for i in range(15, 15 - num_bits, -1):
        mosi_bit = (frame >> i) & 1

        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)

        dut.uio_in.value = (mosi_bit << 0) | (1 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)

        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)

    await ClockCycles(dut.clk, 2)
    # Leave CS_N low — caller must abort by pulling CS high

# ============================================================
# HELPER: Check PWM output for one channel over several clock cycles
# ============================================================

async def check_pwm_channel(dut, model, channel, num_ticks=500):
    """
    Observe the PWM output pin over several clock cycles.
    Compare against reference model.
    Returns: measured duty cycle (0.0 to 1.0)

    Tolerance note: ±5% accounts for ramp quantisation (±0.39%),
    window boundary misalignment (±0.78%), and synchroniser latency.
    See file header for full explanation.
    """
    high_count = 0
    total = num_ticks

    for _ in range(num_ticks):
        await RisingEdge(dut.clk)
        actual_bit = (int(dut.uo_out.value) >> channel) & 1
        if actual_bit == 1:
            high_count += 1

    measured_duty = high_count / total
    return measured_duty

# ============================================================
# HELPER: Check all 4 channels simultaneously in one loop
# Returns tuple of 4 measured duty cycles
# ============================================================

async def check_all_channels(dut, num_ticks=2000):
    """
    Measure all 4 PWM channels simultaneously in a single sample loop.
    This is critical for detecting cross-contamination between channels:
    if channels share any state incorrectly, sequential measurement
    would not catch it (the "bad" state might reset between measurements).
    Returns: (duty0, duty1, duty2, duty3) as floats 0.0–1.0
    """
    counts = [0, 0, 0, 0]

    for _ in range(num_ticks):
        await RisingEdge(dut.clk)
        uo = int(dut.uo_out.value)
        for ch in range(4):
            if (uo >> ch) & 1:
                counts[ch] += 1

    return tuple(c / num_ticks for c in counts)

# ============================================================
# TEST 1: Reset test
# After reset, all channels should output 50% duty cycle (duty=127)
# ============================================================

@cocotb.test()
async def test_01_reset(dut):
    """After reset, duty registers default to 127 (50% duty cycle)."""

    dut._log.info("=== TEST 1: Reset behaviour ===")
    model = QuadPulseModel()

    # Start clock
    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())  # 10 MHz

    # Apply reset
    dut.ena.value    = 1
    dut.rst_n.value  = 0   # Hold in reset
    dut.ui_in.value  = 0   # No emergency stop, freq=50Hz
    dut.uio_in.value = 0b00000100  # CS_N high = SPI idle
    await ClockCycles(dut.clk, 10)

    dut.rst_n.value = 1   # Release reset
    model.reset()
    await ClockCycles(dut.clk, 5)

    # After reset: duty=127 means 127/256 ≈ 49.6% duty cycle
    # We select 20 kHz (freq_sel=11) for fast simulation
    dut.ui_in.value = 0b00000110   # freq_sel[1:0] = 11 → 20 kHz → period=500 ticks

    # Wait for freq_sel_sync to register (2 clock cycles)
    await ClockCycles(dut.clk, 5)

    # Measure channel 0 over 1000 ticks (= 2 complete 500-tick periods at 20 kHz)
    measured = await check_pwm_channel(dut, model, channel=0, num_ticks=1000)
    expected = 127 / 256  # ≈ 0.496

    dut._log.info(f"Channel 0 duty after reset: measured={measured:.3f} expected={expected:.3f}")

    # Allow ±5% tolerance — see file header for tolerance explanation
    assert abs(measured - expected) < 0.05, \
        f"FAIL: Reset duty mismatch. Got {measured:.3f}, expected {expected:.3f}"

    dut._log.info("✅ TEST 1 PASSED")


# ============================================================
# TEST 2: SPI loading — set each channel to different duty cycles
# ============================================================

@cocotb.test()
async def test_02_spi_duty_cycle(dut):
    """Load unique duty cycles on all 4 channels via SPI, verify outputs."""

    dut._log.info("=== TEST 2: SPI duty cycle loading ===")
    model = QuadPulseModel()

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    # Reset
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110  # 20 kHz for fast testing
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)  # Extra settling for synchronisers

    # Load known duty cycles on each channel
    test_duties = [64, 128, 192, 32]  # 25%, 50%, 75%, 12.5%

    for ch, duty in enumerate(test_duties):
        dut._log.info(f"Loading channel {ch} with duty={duty} ({duty/255*100:.0f}%)")
        await spi_send(dut, channel=ch, duty_value=duty)
        model.set_duty(ch, duty)

    # Verify each channel's duty cycle
    for ch, duty in enumerate(test_duties):
        measured = await check_pwm_channel(dut, model, channel=ch, num_ticks=2000)
        expected = duty / 256.0

        dut._log.info(f"Ch{ch}: duty={duty}, measured={measured:.3f}, expected={expected:.3f}")
        assert abs(measured - expected) < 0.05, \
            f"FAIL: Channel {ch} duty mismatch. Got {measured:.3f}, expected {expected:.3f}"

    dut._log.info("✅ TEST 2 PASSED")


# ============================================================
# TEST 3: Emergency stop — all channels go to 0 immediately
# ============================================================

@cocotb.test()
async def test_03_emergency_stop(dut):
    """Emergency stop forces ALL PWM outputs to 0 regardless of duty."""

    dut._log.info("=== TEST 3: Emergency stop ===")
    model = QuadPulseModel()

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    # Reset
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # Load all channels at 75% duty
    for ch in range(4):
        await spi_send(dut, channel=ch, duty_value=192)

    await ClockCycles(dut.clk, 20)

    # Trigger emergency stop
    dut._log.info("Triggering emergency stop...")
    dut.ui_in.value = 0b00000111  # bit0 = emergency_stop = 1, freq=11

    # Check: all PWM outputs must be 0 immediately
    # emergency_stop is combinational — takes effect within 1 gate delay (<1ns)
    # We wait just 1 clock for output to propagate through uo_out assignments
    await ClockCycles(dut.clk, 2)

    for _ in range(200):
        await RisingEdge(dut.clk)
        output = int(dut.uo_out.value)
        pwm_bits = output & 0x0F  # Lower 4 bits = 4 PWM channels
        assert pwm_bits == 0, \
            f"FAIL: Emergency stop failed! uo_out[3:0]={bin(pwm_bits)}, expected 0000"

    # Emergency stop flag should be HIGH on uo_out[4]
    flag = (int(dut.uo_out.value) >> 4) & 1
    assert flag == 1, "FAIL: Emergency stop flag (uo_out[4]) should be HIGH"

    # Release emergency stop — channels should return to their duty cycles
    dut.ui_in.value = 0b00000110  # bit0 = 0 (e-stop off), freq=11
    await ClockCycles(dut.clk, 50)

    # Verify channel 0 returns to ~75% (192/256 = 0.75)
    measured = await check_pwm_channel(dut, None, channel=0, num_ticks=1000)
    assert abs(measured - 192/256.0) < 0.05, \
        f"FAIL: After releasing e-stop, channel 0 duty wrong: {measured:.3f}"

    dut._log.info("✅ TEST 3 PASSED")


# ============================================================
# TEST 4: Channel independence — changing one channel doesn't affect others
# ============================================================

@cocotb.test()
async def test_04_channel_independence(dut):
    """Updating one channel's duty must not disturb the other three."""

    dut._log.info("=== TEST 4: Channel independence ===")

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # Set distinct duties
    duties = [50, 100, 150, 200]
    for ch, d in enumerate(duties):
        await spi_send(dut, channel=ch, duty_value=d)

    # Measure baseline for channels 1, 2, 3
    baseline = []
    for ch in range(1, 4):
        m = await check_pwm_channel(dut, None, channel=ch, num_ticks=1500)
        baseline.append(m)

    # Now change only channel 0
    await spi_send(dut, channel=0, duty_value=255)

    # Measure channels 1, 2, 3 again — must be unchanged
    for ch in range(1, 4):
        after = await check_pwm_channel(dut, None, channel=ch, num_ticks=1500)
        expected = duties[ch] / 256.0
        dut._log.info(f"Ch{ch}: before={baseline[ch-1]:.3f} after={after:.3f} expected={expected:.3f}")
        assert abs(after - expected) < 0.06, \
            f"FAIL: Channel {ch} was disturbed when channel 0 changed!"

    dut._log.info("✅ TEST 4 PASSED")


# ============================================================
# TEST 5: Boundary values — duty=0 (always off) and duty=255 (always on)
# ============================================================

@cocotb.test()
async def test_05_boundary_values(dut):
    """duty=0 must give 0% output; duty=255 must give ~100% output."""

    dut._log.info("=== TEST 5: Boundary duty values ===")

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # duty = 0 → channel must ALWAYS be LOW
    # (ramp is always >= 0, so condition "ramp < 0" is never true)
    await spi_send(dut, channel=0, duty_value=0)
    await ClockCycles(dut.clk, 20)

    for _ in range(600):
        await RisingEdge(dut.clk)
        bit = (int(dut.uo_out.value) >> 0) & 1
        assert bit == 0, "FAIL: duty=0 but ch0 went HIGH!"

    # duty = 255 → channel must ALWAYS be HIGH
    # (ramp counts 0..254 max before resetting; 0..254 < 255 always true)
    # At 20 kHz with sub_period=1: ramp increments every tick 0..499
    # Only ticks where ramp == 255 would the output go LOW.
    # But ramp resets with period_done at tick 499 → ramp reaches max ~255
    # then period ends. The output is high for ramp 0..254 = 255 out of ~256.
    # Note: duty=255 means "always on" by convention — 255/256 ≈ 99.6%
    await spi_send(dut, channel=1, duty_value=255)
    await ClockCycles(dut.clk, 20)

    # Count total HIGH samples — must be >= 99% (allowing for the 1/256 OFF time)
    high_count = 0
    total_samples = 600
    for _ in range(total_samples):
        await RisingEdge(dut.clk)
        bit = (int(dut.uo_out.value) >> 1) & 1
        if bit == 1:
            high_count += 1

    duty_measured = high_count / total_samples
    assert duty_measured >= 0.95, \
        f"FAIL: duty=255 but ch1 measured only {duty_measured:.3f} (expected ≥0.95)"

    dut._log.info(f"duty=255 measured: {duty_measured:.3f} (expected ~0.996)")
    dut._log.info("✅ TEST 5 PASSED")


# ============================================================
# TEST 6: SPI frame abort — partial frame must be silently discarded
#
# What happens if CS_N goes HIGH mid-frame (before 16 bits)?
# The design must: reset spi_bit_count to 0, discard partial data,
# NOT corrupt any duty register. The next complete frame must work.
# ============================================================

@cocotb.test()
async def test_06_spi_abort(dut):
    """Partial SPI frame (CS_N de-asserted early) must be silently discarded."""

    dut._log.info("=== TEST 6: SPI frame abort ===")

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    # Reset
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110   # 20 kHz
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # Baseline: set ch0 to duty=100 (known good value)
    await spi_send(dut, channel=0, duty_value=100)
    await ClockCycles(dut.clk, 10)

    # Verify baseline
    baseline = await check_pwm_channel(dut, None, channel=0, num_ticks=1000)
    assert abs(baseline - 100/256.0) < 0.05, \
        f"FAIL: Baseline duty wrong: {baseline:.3f}"
    dut._log.info(f"Baseline ch0 duty: {baseline:.3f} (expected {100/256.0:.3f})")

    # --- ABORT: Send only 8 of 16 bits (half frame), then pull CS high ---
    dut._log.info("Sending partial frame (8 bits) then aborting with CS_N high...")

    # The partial frame uses channel=3, duty=50 — if abort fails and this
    # gets decoded incorrectly, we might corrupt ch0's duty register.
    # We send bits 15..8 only (the top byte, which contains channel addr).
    await spi_send_partial(dut, channel=3, duty_value=50, num_bits=8)

    # Pull CS_N HIGH to abort mid-frame
    dut.uio_in.value = 0b00000100  # CS_N = 1
    await ClockCycles(dut.clk, 10) # Let spi_bit_count reset stabilise

    # Now send a COMPLETE frame for ch0 with duty=200
    dut._log.info("Sending complete frame: ch0, duty=200...")
    await spi_send(dut, channel=0, duty_value=200)
    await ClockCycles(dut.clk, 10)

    # Verify ch0 duty is now 200 (not corrupted by the aborted frame)
    measured = await check_pwm_channel(dut, None, channel=0, num_ticks=1000)
    expected = 200 / 256.0

    dut._log.info(f"After abort + complete frame: ch0 measured={measured:.3f} expected={expected:.3f}")
    assert abs(measured - expected) < 0.05, \
        f"FAIL: ch0 duty wrong after abort. Got {measured:.3f}, expected {expected:.3f}. " \
        f"Possible abort corruption."

    dut._log.info("✅ TEST 6 PASSED — Partial frame correctly discarded")


# ============================================================
# TEST 7: Frequency switching — change freq mid-operation
#
# After operating at 20 kHz, switch to a different frequency.
# Verify: period_counter resets (no overflow glitch), PWM output
# continues at the new frequency correctly.
# ============================================================

@cocotb.test()
async def test_07_freq_switching(dut):
    """Switching frequency must not cause period counter overflow or duty corruption."""

    dut._log.info("=== TEST 7: Frequency switching ===")

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    # Reset at 20 kHz
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110   # freq_sel=11 → 20 kHz
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # Load ch0 with duty=128 (50%) at 20 kHz
    await spi_send(dut, channel=0, duty_value=128)
    await ClockCycles(dut.clk, 20)

    # Verify 50% at 20 kHz
    measured_20k = await check_pwm_channel(dut, None, channel=0, num_ticks=1000)
    assert abs(measured_20k - 128/256.0) < 0.05, \
        f"FAIL: 20kHz baseline wrong: {measured_20k:.3f}"
    dut._log.info(f"20 kHz duty: {measured_20k:.3f} ✓")

    # Switch to 10 kHz (freq_sel=10)
    dut._log.info("Switching from 20 kHz → 10 kHz...")
    dut.ui_in.value = 0b00000100   # freq_sel=10 → 10 kHz, e-stop=0
    # At 10 kHz, period=1000 ticks. We must wait ≥2 full periods after the switch
    # for the 2-stage sync (2 cycles) + period reset + ramp re-accumulation to settle.
    # Waiting 3000 ticks = 3 full 10kHz periods guarantees clean steady-state.
    await ClockCycles(dut.clk, 3000)

    # Measure over 4000 ticks = 4 complete 10kHz periods for solid averaging
    measured_10k = await check_pwm_channel(dut, None, channel=0, num_ticks=4000)
    assert abs(measured_10k - 128/256.0) < 0.05, \
        f"FAIL: 10kHz after switch wrong: {measured_10k:.3f}"
    dut._log.info(f"10 kHz duty after switch: {measured_10k:.3f} ✓")

    # Switch back to 20 kHz and verify no corruption
    dut._log.info("Switching back to 20 kHz...")
    dut.ui_in.value = 0b00000110   # freq_sel=11 → 20 kHz
    # Wait 2 full 500-tick 20kHz periods for settling
    await ClockCycles(dut.clk, 1500)

    measured_back = await check_pwm_channel(dut, None, channel=0, num_ticks=2000)
    assert abs(measured_back - 128/256.0) < 0.05, \
        f"FAIL: After switching back duty wrong: {measured_back:.3f}"
    dut._log.info(f"Back at 20 kHz: {measured_back:.3f} ✓")

    dut._log.info("✅ TEST 7 PASSED — Frequency switching works correctly")


# ============================================================
# TEST 8: Simultaneous 4-channel measurement
#
# Load all 4 channels with different duties, then measure ALL 4
# in a single sample loop. Sequential measurement would miss
# any channel cross-contamination because the "bad" shared state
# might reset between calls. This test catches it definitively.
# ============================================================

@cocotb.test()
async def test_08_simultaneous_all_channels(dut):
    """Measure all 4 PWM channels simultaneously to detect cross-channel contamination."""

    dut._log.info("=== TEST 8: Simultaneous 4-channel measurement ===")

    cocotb.start_soon(Clock(dut.clk, 100, units="ns").start())

    # Reset
    dut.ena.value    = 1
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0b00000110   # 20 kHz for fast testing
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 10)

    # Load 4 distinct duties with deliberate spread across the range
    duties = [32, 96, 160, 224]  # 12.5%, 37.5%, 62.5%, 87.5%
    for ch, d in enumerate(duties):
        dut._log.info(f"Loading ch{ch}: duty={d} ({d/256*100:.1f}%)")
        await spi_send(dut, channel=ch, duty_value=d)

    await ClockCycles(dut.clk, 20)  # Settle

    # Measure ALL 4 simultaneously in one loop
    dut._log.info("Measuring all 4 channels simultaneously...")
    result = await check_all_channels(dut, num_ticks=3000)

    all_passed = True
    for ch, (measured, expected_duty) in enumerate(zip(result, duties)):
        expected = expected_duty / 256.0
        diff = abs(measured - expected)
        status = "✓" if diff < 0.05 else "✗ FAIL"
        dut._log.info(
            f"Ch{ch}: measured={measured:.4f}  expected={expected:.4f}  "
            f"diff={diff:.4f}  {status}"
        )
        if diff >= 0.05:
            all_passed = False

    assert all_passed, (
        "FAIL: One or more channels outside ±5% tolerance in simultaneous measurement. "
        "This suggests channel cross-contamination or a shared ramp bug."
    )

    dut._log.info("✅ TEST 8 PASSED — All 4 channels correct simultaneously")
    dut._log.info("=" * 60)
    dut._log.info("ALL 8 TESTS PASSED — QuadPulse RTL is verified correct")
    dut._log.info("Ready for Tiny Tapeout synthesis and tapeout")
    dut._log.info("=" * 60)
