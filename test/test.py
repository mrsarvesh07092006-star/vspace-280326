import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

# REFERENCE MODEL (Golden Vector Generator)
class StopwatchModel:
    def __init__(self):
        self.digit = 0
        self.running = False
        # 7-segment hex values for 0-9
        self.segments = [
            0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110, 
            0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111
        ]

    def reset(self):
        self.digit = 0
        self.running = False

    def set_run_state(self, state):
        self.running = bool(state)

    def tick(self):
        if self.running:
            if self.digit == 9:
                self.digit = 0
            else:
                self.digit += 1

    def get_expected_output(self):
        return self.segments[self.digit]

# TEST SUITE
@cocotb.test()
async def test_stopwatch_golden_vectors(dut):
    dut._log.info("Starting Golden Vector Stopwatch Test")
    
    # Initialize software reference model
    model = StopwatchModel()

    # Set the clock period (Using 10us to match the official TT template)
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Phase 1: Reset Sequence
    dut._log.info("Resetting design")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    model.reset()
    await ClockCycles(dut.clk, 2)
    
    assert int(dut.uo_out.value) == model.get_expected_output(), "Failed at Reset!"

    # Phase 2: Counting Sequence
    dut._log.info("Pressing Start Button")
    dut.ui_in.value = 1
    model.set_run_state(True)

    # Test 15 'seconds' (Our tb.v overrides 1 second to equal 10 clocks)
    for simulated_second in range(15):
        await ClockCycles(dut.clk, 10)
        await FallingEdge(dut.clk)
        model.tick()
        
        expected = model.get_expected_output()
        actual = int(dut.uo_out.value)
        
        dut._log.info(f"Sec {simulated_second + 1}: Expected {bin(expected)}, Got {bin(actual)}")
        assert actual == expected, f"Mismatch at second {simulated_second + 1}!"

    # Phase 3: Pause Sequence
    dut._log.info("Pressing Pause Button")
    dut.ui_in.value = 0
    model.set_run_state(False)
    
    # Wait 5 'seconds' to ensure it doesn't count while paused
    for _ in range(5):
        await ClockCycles(dut.clk, 10)
        await FallingEdge(dut.clk)
        model.tick()
        
    assert int(dut.uo_out.value) == model.get_expected_output(), "Pause failed! HW kept counting."
    dut._log.info("All Golden Vector tests passed perfectly!")
