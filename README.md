# QuadPulse — 4-Channel Servo/Motor PWM ASIC

**A dedicated silicon PWM generator for deterministic robotic motor control.**  
**Platform:** Tiny Tapeout (sky130A, 1×1 tile) · **Author:** Sarvesh S (VIT Vellore, 24BVD0092)

---

## Problem Statement

In robotic systems — particularly autonomous swarms — motion control depends on **precise, repeatable PWM signals** to servos and motors. Software PWM running on a general-purpose microcontroller introduces **timing jitter**: the CPU gets interrupted mid-waveform, delayed by other tasks, or stalled by memory operations. The result is inconsistent servo positioning and motor speed that worsens under load as the CPU handles more tasks simultaneously.

This is not a software bug that can be patched — it is a fundamental property of shared-resource CPUs. The only correct fix is hardware:

> **Hardware counters cannot be preempted, delayed, or jittered. They count.**

QuadPulse moves PWM generation into dedicated silicon. Four completely independent hardware counters generate four independent PWM waveforms at cycle-accurate precision. No CPU involvement occurs after initial configuration. The chip is always on time, regardless of what the MCU is doing.

This is the same philosophy as ARM Cortex-M hardware timers — except as a standalone open-source ASIC you can actually tape out.

---

## Architecture

```
                      External MCU (STM32 / ESP32 / RP2040)
                             │
              ┌──────────────┼────────────────┐
              │  SPI (16-bit frame, 3-wire)   │  Emergency Stop (pin)
              │  MOSI / SCLK / CS_N           │  ui_in[0]
              │                               │
              ▼                               │ (combinational — instant)
   ┌──────────────────────────────────────────┴──────────────┐
   │                   QuadPulse ASIC                        │
   │                                                         │
   │  ┌─────────────────────────────────────────────────┐   │
   │  │             2-Stage CDC Synchronisers           │   │
   │  │  spi_mosi_raw → [FF1→FF2] → spi_mosi           │   │
   │  │  spi_sclk_raw → [FF1→FF2] → spi_sclk           │   │
   │  │  spi_cs_n_raw → [FF1→FF2] → spi_cs_n           │   │
   │  └───────────────────┬─────────────────────────────┘   │
   │                      │                                  │
   │  ┌───────────────────▼───────────────┐                 │
   │  │         SPI Receiver (16-bit)     │                 │
   │  │  Frame: [15:14]=ch_addr [7:0]=duty│                 │
   │  └──┬────────┬────────┬────────┬────┘                 │
   │     │        │        │        │                        │
   │  ┌──▼──┐  ┌──▼──┐  ┌──▼──┐  ┌──▼──┐  Duty registers  │
   │  │ D0  │  │ D1  │  │ D2  │  │ D3  │  (8-bit each)    │
   │  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘                  │
   │     │        │        │        │  compare              │
   │  ┌──▼────────▼────────▼────────▼──────────────────┐   │
   │  │          PWM Comparator × 4                    │   │
   │  │    pwm_chX = (ramp < duty_chX) && !e_stop      │   │
   │  └──┬────────┬────────┬────────┬──────────────────┘   │
   │     │        │        │        │   ▲                    │
   │  ┌──▼────────▼────────▼────────▼───┴────────────────┐  │
   │  │        Phase Accumulator Ramp (0→255)            │  │
   │  │   sub_counter ticks → ramp++ every sub_period    │  │
   │  └─────────────────────────────┬──────────────────┘  │
   │                                │                        │
   │  ┌─────────────────────────────▼──────────────────┐   │
   │  │  Period Counter (18-bit) + Freq Mux             │   │
   │  │  Registered freq_sel_sync from DIP switches     │   │
   │  │  00=50Hz 01=1kHz 10=10kHz 11=20kHz             │   │
   │  └────────────────────────────────────────────────┘   │
   └─────────────────────────────────────────────────────────┘
              │        │        │        │
           PWM_CH0  PWM_CH1  PWM_CH2  PWM_CH3
           (servo / DC motor / ESC drivers)
```

---

## Pin Reference

| Pin | Direction | Function |
|-----|-----------|----------|
| `ui_in[0]` | Input | **Emergency Stop** — HIGH instantly forces all PWM outputs to 0 |
| `ui_in[1]` | Input | **Freq select bit 0** — with bit 1: selects PWM frequency |
| `ui_in[2]` | Input | **Freq select bit 1** — `00`=50 Hz, `01`=1 kHz, `10`=10 kHz, `11`=20 kHz |
| `ui_in[7:3]` | Input | _Unused_ |
| `uo_out[0]` | Output | **PWM Channel 0** (Servo 1 / Motor 1) |
| `uo_out[1]` | Output | **PWM Channel 1** (Servo 2 / Motor 2) |
| `uo_out[2]` | Output | **PWM Channel 2** (Servo 3 / Motor 3) |
| `uo_out[3]` | Output | **PWM Channel 3** (Servo 4 / Motor 4) |
| `uo_out[4]` | Output | **Emergency Stop flag** — mirrors input, HIGH when e-stop active |
| `uo_out[7:5]` | Output | _Reserved (tied LOW)_ |
| `uio[0]` | Input | **SPI MOSI** — serial data from MCU to chip |
| `uio[1]` | Input | **SPI SCLK** — SPI clock from MCU |
| `uio[2]` | Input | **SPI CS_N** — chip select, active LOW |
| `uio[7:3]` | Input | _Unused_ |

---

## SPI Protocol

Send a **16-bit frame, MSB first**, to set any channel's duty cycle.

```
Bit 15–14 : Channel address    (00=CH0, 01=CH1, 10=CH2, 11=CH3)
Bit 13–8  : Reserved           (send as 000000)
Bit 7–0   : Duty cycle value   (0 = 0% off,  127 = ~50%,  255 = ~100% on)
```

**Frame timing:**
```
CS_N  ‾\___________________________________________/‾
SCLK     ‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\_‾\
MOSI     [15][14][13][12][11][10][ 9][ 8][ 7]...[ 0]
               ↑ CH addr ↑              ↑ duty ↑
```

**Example: Set Channel 2 to 75% duty cycle:**
```
Channel addr = 2 = 0b10
Duty value   = 192 = 0xC0 (192/256 = 75%)
Frame        = 0b10_000000_11000000 = 0x80C0
```

**Example: Set Channel 0 to 0% (always off):**
```
Frame = 0b00_000000_00000000 = 0x0000
```

**Abort behaviour:** If CS_N goes HIGH before all 16 bits are received, the partial frame is silently discarded. Duty registers are unchanged.

---

## Demo on TT Demoboard (no oscilloscope required)

### Hardware setup:
1. Connect `uo_out[0]` → 470 Ω resistor → LED → GND
2. The LED brightness directly shows PWM duty cycle (physics: LEDs average PWM)

### Step-by-step demo:

| Step | DIP switch setting | What you observe |
|------|--------------------|-----------------|
| 1. Power on | `ui_in = 00000110` (20 kHz, no e-stop) | LED at ~50% brightness (default 127 duty) |
| 2. Send SPI: ch0 duty=0 | CS→MOSI→SCLK sequence | LED turns OFF |
| 3. Send SPI: ch0 duty=255 | CS→MOSI→SCLK sequence | LED at maximum brightness |
| 4. Send SPI: ch0 duty=128 | CS→MOSI→SCLK sequence | LED at half brightness |
| 5. Set freq=50 Hz | `ui_in = 00000000` | LED visibly flickers (50 Hz within flicker range) |
| 6. Emergency stop | `ui_in[0] = 1` | LED goes off instantly |
| 7. Release stop | `ui_in[0] = 0` | LED returns to previous brightness |

### Servo demo:
1. Connect `uo_out[0]` to a hobby servo signal wire (orange/white)
2. Set freq = 50 Hz (`ui_in[2:1] = 00`)
3. Send SPI: ch0 duty=114 → servo centers (1500 µs pulse)
4. Send SPI: ch0 duty=76  → servo goes left (1000 µs pulse)
5. Send SPI: ch0 duty=152 → servo goes right (2000 µs pulse)

---

## Gate Count

| Block | Gates (NAND2-equiv) |
|-------|---------------------|
| 18-bit period counter | ~90 |
| Registered freq_sel sync (2-bit) | ~8 |
| 18-bit sub_counter (ramp accumulator) | ~90 |
| 8-bit ramp register | ~32 |
| sub_period mux (18-bit, 4-way) | ~18 |
| PWM comparators × 4 (8-bit) | ~128 |
| Duty registers × 4 (8-bit FF) | ~40 |
| SPI shift register (15-bit) | ~75 |
| SPI bit counter (4-bit) | ~16 |
| SPI prev-SCLK FF | ~4 |
| 2-stage CDC sync: spi_sclk | ~8 |
| 2-stage CDC sync: spi_mosi | ~8 |
| 2-stage CDC sync: spi_cs_n | ~8 |
| Emergency stop AND gates × 4 | ~8 |
| Output assignments | ~4 |
| **TOTAL** | **~537 (~54% of 1000-gate budget)** |

The design fits comfortably within the 1×1 tile budget with 46% headroom.

---

## How to Run Tests

```bash
# 1. Clone the TT template (first time only)
git clone https://github.com/TinyTapeout/tt10-verilog-template.git tt-quadpulse
cd tt-quadpulse

# 2. Copy project files
cp path/to/project.v src/project.v
cp path/to/tb.v      test/tb.v
cp path/to/test.py   test/test.py

# 3. Install cocotb
pip install cocotb

# 4. Run all 8 tests
cd test
make

# Expected output:
# PASS  test_01_reset
# PASS  test_02_spi_duty_cycle
# PASS  test_03_emergency_stop
# PASS  test_04_channel_independence
# PASS  test_05_boundary_values
# PASS  test_06_spi_abort
# PASS  test_07_freq_switching
# PASS  test_08_simultaneous_all_channels
```

All 8 golden vector tests must pass before submitting to Tiny Tapeout.

---

## Research Context

This chip is the hardware backbone of ongoing research on **autonomous robotic swarms** for ISRO's **Bharatiya Antariksh Station (BAS)**, presented at **SMOPS-2026** (accepted by ISRO/ASI/IAA/IN-SPACe).

The central research question: can a swarm of small robots with commodity MCUs achieve the **deterministic, multi-channel motor timing** required for coordinated locomotion in a space station environment? The answer from simulation was: not with software PWM. MCU timer interrupts under load showed jitter of 5–50 µs — enough to cause servo positioning errors of 0.5–5° per step, accumulating into significant trajectory deviation over time.

QuadPulse solves this directly: silicon counters have zero jitter by physical law. The chip extends prior work on an **FPGA-assisted zero-jitter interrupt controller** implemented on the Terasic DE2-115 (Cyclone IV FPGA), which demonstrated that hardware-enforced timing determinism is achievable and significantly outperforms software approaches. QuadPulse brings this same philosophy to a fully custom ASIC form — open source, fabricatable, and reproducible.

**Why Tiny Tapeout:** Open-source silicon (sky130A PDK, OpenROAD toolchain) makes academic hardware research publishable and reproducible. Any institution can take this design from this repository, run the same toolflow, and fabricate an identical chip.

---

## What the GitHub CI Checks

When you push to GitHub, the Actions workflow runs automatically:

| CI Step | What it checks | Why it matters |
|---------|---------------|----------------|
| **Lint** | Verilog syntax, undeclared nets, multiple drivers | Catches typos before wasting synthesis time |
| **Simulation** | Runs all 8 cocotb tests with iverilog | Proves RTL behaves correctly in software |
| **Synthesis** | Yosys maps RTL → sky130A cell library | Confirms the design is actually synthesisable |
| **Gate count** | Checks synthesised netlist cell count ≤ 1000 | Enforces tile-size budget |
| **Place & Route** | OpenROAD routes wires within 1×1 tile | Confirms physical feasibility |
| **DRC** | Magic VLSI checks design rule compliance | Ensures manufacturability per sky130A rules |
| **LVS** | Netgen checks schematic vs. layout match | Confirms the GDS physically matches the RTL |
| **GDS render** | KLayout renders final layout image | Visual confirmation of the chip |

A green checkmark on every step means: this chip is ready to submit to Tiny Tapeout for the next shuttle run.

---

*QuadPulse is open-source hardware, released under Apache 2.0 License.*  
*Designed using the open sky130A PDK and OpenROAD EDA toolchain.*
