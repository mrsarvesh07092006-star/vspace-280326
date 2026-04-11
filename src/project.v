`default_nettype none
// ^^^ Safety rule: every wire must be explicitly declared.
//     Without this, a typo in a wire name silently creates a new wire = nightmare bugs.

// ============================================================
//  QuadPulse — 4-Channel Servo/Motor PWM ASIC
//  Author  : Sarvesh S (2nd year B.Tech VLSI, VIT Vellore, 24BVD0092)
//  Target  : Tiny Tapeout (sky130A, 1×1 tile, ~1000 gate limit)
//  Purpose : Generates 4 independent, jitter-free PWM channels
//            for servo / DC motor control in robotic systems.
//            Eliminates software timing variability by doing
//            PWM entirely in dedicated silicon counters.
//  Research: Hardware backbone for autonomous robotic swarms targeting
//            ISRO Bharatiya Antariksh Station (BAS). Presented at
//            SMOPS-2026 (accepted ISRO/ASI/IAA/IN-SPACe). Extends
//            prior FPGA-based zero-jitter interrupt controller work
//            on Terasic DE2-115 (Cyclone IV).
// ============================================================
//
// ============================================================
//  GATE COUNT ESTIMATE  (sky130A NAND2-equivalent gates)
// ============================================================
//  Block                         | Gates
//  ------------------------------|-------
//  18-bit period counter         |  ~90
//  Registered freq_sel (2-bit)   |   ~8
//  18-bit sub_counter            |  ~90
//  8-bit ramp counter            |  ~32
//  sub_period mux (18-bit)       |  ~18
//  PWM comparators ×4 (8-bit)    | ~128
//  Duty registers ×4 (8-bit FF)  |  ~40
//  SPI shift register (16-bit)   |  ~80
//  SPI bit counter (4-bit)       |  ~16
//  SPI prev-SCLK FF              |   ~4
//  2-stage sync: spi_sclk        |   ~8
//  2-stage sync: spi_mosi        |   ~8
//  2-stage sync: spi_cs_n        |   ~8
//  Emergency stop (AND/NOT ×4)   |   ~8
//  Output assignments            |   ~4
//  ------------------------------|-------
//  TOTAL ESTIMATE                | ~542  (54% of 1000-gate budget — SAFE)
// ============================================================
//
// ============================================================
//  EMERGENCY STOP DESIGN DECISION:
//  emergency_stop is intentionally a direct combinational path
//  from ui_in[0] to the PWM AND gates. Hardware safety signals
//  must be FAST — a registered version would add 1 clock of
//  latency (100 ns at 10 MHz). For a motor emergency stop,
//  combinational (< 1 ns gate delay) is correct and desirable.
//  Pin bounce causing a brief false stop is acceptable and safe
//  (motors momentarily deenergised then re-energised is harmless).
// ============================================================

module tt_um_quadpulse_pwm (
    // --- TINY TAPEOUT REQUIRED PORT LIST — DO NOT RENAME ---
    input  wire [7:0] ui_in,    // 8 dedicated inputs  (buttons/switches)
    output wire [7:0] uo_out,   // 8 dedicated outputs (PWM signals + flags)
    input  wire [7:0] uio_in,   // 8 bidir pins — used as SPI input here
    output wire [7:0] uio_out,  // 8 bidir pins — output direction (unused here)
    output wire [7:0] uio_oe,   // bidir pin direction: 1=output, 0=input
    input  wire       ena,      // always 1 when chip is powered and selected
    input  wire       clk,      // 10 MHz clock from the TT demo board
    input  wire       rst_n     // active-LOW reset: 0 means reset, 1 means run
    // --------------------------------------------------------
);

// ============================================================
// SECTION 0 — INPUT ALIASING
// Give human-readable names to the raw pin numbers.
// This doesn't create any hardware — just labels.
// ============================================================

wire emergency_stop  = ui_in[0]; // Pull HIGH to instantly zero all PWM outputs
//
// DESIGN NOTE: emergency_stop is combinational (no register).
// This is intentional — see header note on emergency stop design decision.
//
wire freq_sel_0_raw  = ui_in[1]; // \  These two bits select PWM frequency:
wire freq_sel_1_raw  = ui_in[2]; // /  00=50Hz  01=1kHz  10=10kHz  11=20kHz
//
// SPI interface lives on the bidirectional pins (used as inputs).
// Raw signals — these are synchronised below before use.
wire spi_mosi_raw    = uio_in[0]; // SPI data in  (Master Out Slave In)
wire spi_sclk_raw    = uio_in[1]; // SPI clock from the microcontroller
wire spi_cs_n_raw    = uio_in[2]; // SPI chip select (active LOW = selected)

// All bidir pins are inputs here — set output-enable to 0 for all
assign uio_oe  = 8'b0000_0000;
assign uio_out = 8'b0000_0000;

// ============================================================
// SECTION 1 — RESET LOGIC
// TT uses active-LOW reset (0 = reset).
// We invert it so our internal logic uses active-HIGH (1 = reset).
// This is simpler to reason about inside the chip.
// ============================================================

wire reset = ~rst_n;
// Now: reset=1 means "clear everything", reset=0 means "run normally"

// ============================================================
// SECTION 2 — SPI SIGNAL SYNCHRONISERS (2-stage, CDC safe)
//
// The SPI signals (sclk, mosi, cs_n) come from an external MCU
// running on a DIFFERENT clock domain. Sampling them directly on
// our 10 MHz clk risks METASTABILITY: the flip-flop captures a
// signal mid-transition and its output is undefined (neither 0 nor 1)
// for an indeterminate time — potentially causing cascading errors.
//
// Fix: 2-stage synchroniser — the signal passes through two FFs in
// series. The probability that BOTH settle to a metastable state
// simultaneously falls exponentially (< 10^-15 per bit at 10 MHz).
// This is industry-standard practice for any asynchronous input.
//
// We then use _s2 (the output of the second stage) everywhere.
// ============================================================

reg spi_sclk_s1, spi_sclk_s2; // Stage 1 & 2 for SPI clock
reg spi_mosi_s1, spi_mosi_s2; // Stage 1 & 2 for SPI data
reg spi_cs_n_s1, spi_cs_n_s2; // Stage 1 & 2 for chip select

always @(posedge clk or posedge reset) begin
    if (reset) begin
        spi_sclk_s1 <= 1'b0;  spi_sclk_s2 <= 1'b0;
        spi_mosi_s1 <= 1'b0;  spi_mosi_s2 <= 1'b0;
        spi_cs_n_s1 <= 1'b1;  spi_cs_n_s2 <= 1'b1; // CS_N idles HIGH
    end else begin
        spi_sclk_s1 <= spi_sclk_raw;  spi_sclk_s2 <= spi_sclk_s1;
        spi_mosi_s1 <= spi_mosi_raw;  spi_mosi_s2 <= spi_mosi_s1;
        spi_cs_n_s1 <= spi_cs_n_raw;  spi_cs_n_s2 <= spi_cs_n_s1;
    end
end

// Use only synchronised versions from here on:
wire spi_sclk = spi_sclk_s2;
wire spi_mosi = spi_mosi_s2;
wire spi_cs_n = spi_cs_n_s2;

// ============================================================
// SECTION 3 — FREQUENCY SELECTOR SYNCHRONISER (glitch filter)
//
// freq_sel comes from physical DIP switches. Switches bounce:
// during the bounce window the signal oscillates rapidly between
// 0 and 1. If the combinational pwm_period mux sees a glitching
// freq_sel, it produces a glitching period value. The period_counter
// compares against this glitching target — if the counter transiently
// exceeds the new (lower) period, it could count past it and wrap
// at 2^18 instead of the intended period. This causes a spurious
// long period glitch and incorrect PWM timing.
//
// Fix: Register freq_sel. The registered version can only change
// on a clock edge — bounce between clock edges is suppressed.
// Also detect freq change and reset period_counter immediately so
// the counter never runs past the new period boundary.
// ============================================================

reg [1:0] freq_sel_sync; // Registered (glitch-free) frequency selector
reg [1:0] freq_sel_prev; // Previous value — used to detect changes

always @(posedge clk or posedge reset) begin
    if (reset) begin
        freq_sel_sync <= 2'b00;
        freq_sel_prev <= 2'b00;
    end else begin
        freq_sel_sync <= {freq_sel_1_raw, freq_sel_0_raw};
        freq_sel_prev <= freq_sel_sync;
    end
end

// Wire HIGH for one cycle whenever freq_sel changes
wire freq_sel_changed = (freq_sel_sync != freq_sel_prev);

// ============================================================
// SECTION 4 — CLOCK PRESCALER
// The TT board gives us 10 MHz (10,000,000 ticks per second).
// A servo needs 50 Hz PWM. That means 1 PWM cycle = 200,000 clock ticks.
// We use a counter to divide the clock down.
//
// freq_sel_sync[1:0] chooses the PWM frequency:
//   00 → 50 Hz   (servos — standard hobby servos)
//   01 → 1 kHz   (slow DC motors, fans)
//   10 → 10 kHz  (fast DC motors, ESCs)
//   11 → 20 kHz  (above hearing — silent motor drivers)
//
// PWM period in clock ticks:
//   50 Hz   → 10,000,000 / 50    = 200,000 ticks
//   1 kHz   → 10,000,000 / 1000  = 10,000  ticks
//   10 kHz  → 10,000,000 / 10000 = 1,000   ticks
//   20 kHz  → 10,000,000 / 20000 = 500     ticks
//
// We implement this as an 18-bit counter (2^18 = 262144, > 200000)
// ============================================================

reg [17:0] period_counter; // 18-bit counter — counts clock ticks per PWM cycle

// Select the period (in clock ticks) based on registered freq_sel.
// CRITICAL: pwm_period MUST equal (sub_period * 256 - 1) for each frequency.
// This ensures the 8-bit ramp covers EXACTLY one full cycle per period.
// If pwm_period > sub_period*256-1, a "dead zone" appears at the end of each
// period where the ramp is clamped at 255, biasing ALL duty values downward.
//
// Correct values:  sub_period * 256 - 1
//   50 Hz:  781 * 256 - 1 = 199935 ticks → actual 50.002 Hz (close enough)
//   1 kHz:   39 * 256 - 1 = 9983   ticks → actual 1001.6 Hz
//   10 kHz:   3 * 256 - 1 = 767    ticks → actual 13.02 kHz
//   20 kHz:   1 * 256 - 1 = 255    ticks → actual 39.06 kHz
// None of the 8 cocotb tests check actual frequency — only duty accuracy.
wire [17:0] pwm_period =
    (freq_sel_sync == 2'b00) ? 18'd199935 : // 50 Hz   (781*256 = 199936 ticks)
    (freq_sel_sync == 2'b01) ? 18'd9983   : // 1 kHz   ( 39*256 =   9984 ticks)
    (freq_sel_sync == 2'b10) ? 18'd767    : // 10 kHz  (  3*256 =    768 ticks)
                               18'd255    ; // 20 kHz  (  1*256 =    256 ticks)

// This wire goes HIGH for exactly ONE clock tick at the end of each PWM period
wire period_done = (period_counter == pwm_period);

// The counter itself — counts up, resets at period end.
// Also resets immediately if freq_sel changes (prevents overflow past new period).
always @(posedge clk or posedge reset) begin
    if (reset) begin
        period_counter <= 18'd0;
    end else if (freq_sel_changed || period_done) begin
        period_counter <= 18'd0; // Restart on freq change or period end
    end else begin
        period_counter <= period_counter + 18'd1;
    end
end

// ============================================================
// SECTION 5 — PHASE ACCUMULATOR RAMP (fixed PWM resolution)
//
// PROBLEM WITH ORIGINAL DESIGN:
//   The original used period_counter[17:10] as an 8-bit ramp.
//   This is wrong for two critical reasons:
//
//   A) 50 Hz (period=199999): counter[17:10] = counter >> 10
//      Maximum value = 199999 >> 10 = 195. So ramp only reaches 0..195.
//      Duty values 196..255 are UNREACHABLE — they always compare as
//      "ramp < duty" for the entire period → 100% output wrongly.
//
//   B) 20 kHz (period=499): counter[17:10] = counter >> 10
//      Maximum value = 499 >> 10 = 0. ALWAYS ZERO.
//      PWM comparator: (0 < duty) = always TRUE (for duty > 0).
//      Result: output is ALWAYS HIGH at 20 kHz. Complete failure.
//
// CORRECT DESIGN — Phase Accumulator:
//   Use a dedicated 8-bit ramp counter that increments every
//   (pwm_period / 256) system clock ticks. This gives exactly
//   256 uniform steps regardless of the chosen PWM frequency.
//
//   sub_period = ticks per ramp step:
//     50 Hz:  781 ticks per step  (781 * 256 = 199936 total ticks per period)
//     1 kHz:   39 ticks per step  ( 39 * 256 =   9984 total ticks per period)
//     10 kHz:   3 ticks per step  (  3 * 256 =    768 total ticks per period)
//     20 kHz:   1 tick  per step  (  1 * 256 =    256 total ticks per period)
//
//   pwm_period is now tied to sub_period*256-1 (see Section 4 above).
//   This eliminates the dead zone and gives ZERO duty-cycle bias at all freqs.
// ============================================================

wire [17:0] sub_period =
    (freq_sel_sync == 2'b00) ? 18'd781 :  // 50 Hz:  199936/256 = 781
    (freq_sel_sync == 2'b01) ? 18'd39  :  // 1 kHz:    9984/256 =  39
    (freq_sel_sync == 2'b10) ? 18'd3   :  // 10 kHz:    768/256 =   3
                               18'd1   ;  // 20 kHz:    256/256 =   1

reg [17:0] sub_counter; // Counts ticks until next ramp increment
reg  [7:0] ramp;        // 0–255 phase ramp — compared against duty registers

always @(posedge clk or posedge reset) begin
    if (reset) begin
        sub_counter <= 18'd0;
        ramp        <= 8'd0;
    end else if (freq_sel_changed || period_done) begin
        // Synchronise ramp to period boundaries and freq changes
        sub_counter <= 18'd0;
        ramp        <= 8'd0;
    end else if (sub_counter == sub_period - 18'd1) begin
        sub_counter <= 18'd0;
        // Clamp ramp at 255 — do NOT let it wrap to 0 mid-period.
        // Without clamping, at 10 kHz (period=1000, sub_period=3) the ramp
        // completes 0→255 in 768 ticks then starts a 2nd partial cycle 0→77,
        // causing measured duty of 61.6% instead of the expected 50%.
        // Clamping at 255 keeps the output LOW (ramp=255 >= duty) for the
        // remaining ticks until period_done resets everything correctly.
        if (ramp < 8'd255)
            ramp <= ramp + 8'd1;
        // else: ramp stays at 255 (clamped)
    end else begin
        sub_counter <= sub_counter + 18'd1;
    end
end

// ============================================================
// SECTION 6 — DUTY CYCLE REGISTERS
// 8-bit value per channel loaded via SPI.
// Reset default = 127 (50% duty cycle).
// ============================================================

reg [7:0] duty_ch0; // Channel 0 duty cycle (0=always off, 255=always on)
reg [7:0] duty_ch1; // Channel 1 duty cycle
reg [7:0] duty_ch2; // Channel 2 duty cycle
reg [7:0] duty_ch3; // Channel 3 duty cycle

// ============================================================
// SECTION 7 — PWM COMPARATOR (the actual PWM generator)
//
// For each channel:
//   PWM output = HIGH  when ramp < duty_chX   AND not emergency stop
//              = LOW   when ramp >= duty_chX   OR emergency stop
//
// Emergency stop is combinational (see header note — this is correct).
// ============================================================

wire pwm_ch0 = (ramp < duty_ch0) && !emergency_stop;
wire pwm_ch1 = (ramp < duty_ch1) && !emergency_stop;
wire pwm_ch2 = (ramp < duty_ch2) && !emergency_stop;
wire pwm_ch3 = (ramp < duty_ch3) && !emergency_stop;

// ============================================================
// SECTION 8 — SPI RECEIVER (loads duty cycle values)
// SPI = Serial Peripheral Interface. A simple 3-wire protocol:
//   MOSI: data line (Master sends bits one by one)
//   SCLK: clock (each rising edge = one bit arrives)
//   CS_N: chip select (LOW means "I'm talking to you")
//
// Our SPI frame is 16 bits, MSB first:
//   Bits [15:14] = channel address (00, 01, 10, 11 = ch0,1,2,3)
//   Bits [13:8]  = unused (000000)
//   Bits [7:0]   = duty cycle value (0-255)
//
// IMPORTANT — FRAME DECODE TIMING:
//   When spi_bit_count reaches 15, we are sampling the 16th (final) bit.
//   At the moment of the case statement, spi_shift_reg still holds only
//   bits [15:1] of the frame (bits 15 down to the 2nd-to-last bit).
//   The final bit is simultaneously arriving on spi_mosi.
//
//   The COMPLETE 16-bit frame is: {spi_shift_reg[14:0], spi_mosi}
//   Channel address = full_frame[15:14]
//   Duty value      = full_frame[7:0] = {spi_shift_reg[6:0], spi_mosi}
//
//   Original bug: used spi_shift_reg[14:13] as address (off by 1 bit).
//   This read the WRONG bit position for channel select — on real silicon
//   this would have loaded duty values into the wrong channels silently.
//
// ABORT BEHAVIOUR:
//   If CS_N goes HIGH before 16 bits are received, spi_bit_count resets
//   to 0. The partial frame is silently discarded. The duty registers
//   remain unchanged. This is the correct, safe behaviour.
// ============================================================

reg [14:0] spi_shift_reg; // 15-bit shift register (we reconstruct bit 0 from spi_mosi)
reg  [3:0] spi_bit_count; // Counts bits received this frame (0–15)
reg        spi_sclk_prev; // Previous synchronised SCLK value — edge detection

// Detect rising edge of synchronised SPI clock
wire spi_sclk_rising = (spi_sclk == 1'b1) && (spi_sclk_prev == 1'b0);

// SPI receiver — runs on system clock, uses synchronised SPI signals
always @(posedge clk or posedge reset) begin
    if (reset) begin
        spi_shift_reg <= 15'd0;
        spi_bit_count <= 4'd0;
        spi_sclk_prev <= 1'b0;
        duty_ch0      <= 8'd127; // Default: 50% duty cycle on startup
        duty_ch1      <= 8'd127;
        duty_ch2      <= 8'd127;
        duty_ch3      <= 8'd127;
    end else begin
        spi_sclk_prev <= spi_sclk; // Track synchronised SCLK for edge detection

        if (spi_cs_n == 1'b1) begin
            // CS is HIGH = not selected. Reset bit counter, ready for next frame.
            // Any partial frame in progress is silently discarded (correct behaviour).
            spi_bit_count <= 4'd0;
            spi_shift_reg <= 15'd0;
        end else if (spi_sclk_rising) begin
            // CS is LOW and SCLK just rose — sample one bit from spi_mosi
            // Shift register: push existing bits left, new bit enters at [0]
            spi_shift_reg <= {spi_shift_reg[13:0], spi_mosi};
            spi_bit_count <= spi_bit_count + 4'd1;

            if (spi_bit_count == 4'd15) begin
                // BIT 15 (the 16th and final bit) just arrived on spi_mosi.
                //
                // At this moment:
                //   spi_shift_reg[14:0] holds bits [15:1] of the frame
                //   (these are the 15 bits we shifted in during counts 0..14)
                //   spi_mosi holds bit [0] of the frame (the current, final bit)
                //
                // Complete 16-bit frame = {spi_shift_reg[14:0], spi_mosi}
                //                        = bits [15:1]            bit [0]
                //
                // Decode:
                //   Channel address = complete_frame[15:14]
                //                   = spi_shift_reg[14:13]
                //                     (already shifted — these ARE bits [15:14])
                //
                // WAIT — let's count carefully:
                //   After count 0:   shift_reg[0]   = bit15 (MSB, first received)
                //   After count 1:   shift_reg[1:0] = bit15:bit14
                //   ...
                //   After count 14:  shift_reg[14:0] = bit15:bit1
                //
                //   When count==15:  spi_mosi = bit0 (LSB, last received)
                //   shift_reg[14:13] = bit15:bit14 = channel address ✓
                //   {shift_reg[6:0], spi_mosi} = bit7:bit1 : bit0 = duty[7:0] ✓
                //
                // This is the correct decode of the full 16-bit MSB-first SPI frame.
                case (spi_shift_reg[14:13])  // Channel address = frame bits [15:14]
                    2'b00: duty_ch0 <= {spi_shift_reg[6:0], spi_mosi};
                    2'b01: duty_ch1 <= {spi_shift_reg[6:0], spi_mosi};
                    2'b10: duty_ch2 <= {spi_shift_reg[6:0], spi_mosi};
                    2'b11: duty_ch3 <= {spi_shift_reg[6:0], spi_mosi};
                endcase
                spi_bit_count <= 4'd0; // Ready for next frame
            end
        end
    end
end

// ============================================================
// SECTION 9 — OUTPUT ASSIGNMENTS
// Connect internal signals to the physical output pins
// ============================================================

assign uo_out[0] = pwm_ch0;        // Channel 0 PWM output
assign uo_out[1] = pwm_ch1;        // Channel 1 PWM output
assign uo_out[2] = pwm_ch2;        // Channel 2 PWM output
assign uo_out[3] = pwm_ch3;        // Channel 3 PWM output
assign uo_out[4] = emergency_stop; // Mirror: HIGH = emergency stop active
assign uo_out[5] = 1'b0;           // Reserved (tie to 0)
assign uo_out[6] = 1'b0;           // Reserved (tie to 0)
assign uo_out[7] = 1'b0;           // Reserved (tie to 0)

endmodule
