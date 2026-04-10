`default_nettype none
`timescale 1ns / 1ps
// ^^^ timescale tells the simulator: 1 unit = 1 nanosecond, precision = 1ps
//     This doesn't affect your real chip — only the simulation timing

// ============================================================
//  QuadPulse Testbench (tb.v)
//
//  Think of this file as a "virtual lab bench":
//  - It creates fake wires and a fake clock
//  - It plugs your chip module into those wires
//  - It dumps all signal values to a file so you can view waveforms
//
//  This file is NEVER synthesised into silicon.
//  It only exists for simulation (checking correctness in software).
// ============================================================

module tb ();

    // --- WAVEFORM DUMP ---
    // This saves all signal values to tb.fst so you can open it
    // in GTKWave and see the actual PWM waveform visually
    initial begin
        $dumpfile("tb.fst");
        $dumpvars(0, tb);
        #1; // Wait 1ns for stability before simulation starts
    end

    // --- DECLARE THE FAKE WIRES ---
    // "reg" = a wire we can drive from the testbench (like a signal generator)
    // "wire" = a wire we just observe (the chip drives these)
    reg        clk;       // We'll generate the clock ourselves
    reg        rst_n;     // We'll control reset
    reg        ena;       // We'll keep this = 1 always
    reg  [7:0] ui_in;     // We control the input pins (switches/buttons)
    reg  [7:0] uio_in;    // We control bidir pins (SPI signals go here)
    wire [7:0] uo_out;    // Chip controls these — we just watch
    wire [7:0] uio_out;   // Chip controls (unused, will be 0)
    wire [7:0] uio_oe;    // Chip controls (will be 0 — all inputs)

    // Gate-level simulation power ports (only used during GLS, ignore for now)
`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    // --- INITIAL VALUES ---
    // IMPORTANT: Without this block, all regs start as X (unknown).
    // X propagates through your design like a virus — X AND 1 = X, etc.
    // This initial block drives known-good values onto all inputs BEFORE
    // the clock starts, guaranteeing a clean simulation start.
    initial begin
        clk     = 0;         // Clock starts LOW
        rst_n   = 0;         // Start in reset (active LOW = 0 means reset)
        ena     = 1;         // Always enabled
        ui_in   = 8'b0;      // No emergency stop, freq=50Hz
        uio_in  = 8'b00000100; // CS_N HIGH (bit 2 = 1) = SPI idle, MOSI/SCLK=0
    end

    // --- INSTANTIATE YOUR CHIP ---
    // This "plugs in" your tt_um_quadpulse_pwm module
    // like inserting a chip into a breadboard socket
    tt_um_quadpulse_pwm user_project (
`ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
`endif
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // --- CLOCK GENERATOR ---
    // Creates a 10 MHz clock: HIGH for 50ns, LOW for 50ns = 100ns period
    // 1/100ns = 10,000,000 Hz = 10 MHz ✓
    always #50 clk = ~clk; // Every 50ns, flip the clock

endmodule
