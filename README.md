# Advanced Industrial PID Controller — FPGA (Verilog)

A fully synthesizable, closed-loop PID control system implemented in Verilog, designed for industrial applications requiring precise, real-time control. The system integrates SPI-based ADC sensing, runtime-configurable PID gains over UART, and 12-bit PWM output — all in a single, modular FPGA design.

---

## System Architecture

```
                        ┌─────────────────────────────────────────┐
                        │            pid_system_top                │
                        │                                          │
  [Sensor / ADC] ──SPI──►  spi_adc_master  ──► pid_controller  ──►  pwm_generator ──► [Motor/Actuator]
                        │                          ▲                                         │
  [Host PC] ──UART RX──►  pid_config_handler ──────┘                                        │
             UART TX◄──── uart_tx ◄──────────────────── [ADC Raw Data] ◄───────────────────┘
                        └─────────────────────────────────────────┘
```

The top-level module (`pid_system_top`) integrates 5 sub-modules into a single, self-contained control loop running at 50 MHz.

---

## Key Features

| Feature | Detail |
|---|---|
| **Arithmetic** | Q16.16 fixed-point (32-bit) — no floating-point unit required |
| **Anti-Windup** | Conditional integration: halts when output is saturated |
| **Derivative Filter** | First-order IIR low-pass filter with tunable alpha (Kd_filter) |
| **Dead-band** | Configurable threshold to suppress noise near setpoint |
| **Output Saturation** | Symmetric clamping with configurable min/max limits |
| **Runtime Tuning** | Kp, Ki, Kd updatable live via UART (115200 baud, 8N1) |
| **SPI ADC Interface** | Configurable clock divider, compatible with MCP3201-type 12-bit ADCs |
| **PWM Output** | 12-bit resolution (4096 steps), 50 MHz base clock |

---

## Module Breakdown

### `pid_controller_advanced.v`
The core control engine. Implements a 9-state FSM that executes one full PID calculation per enable pulse.

**FSM Pipeline:**
```
IDLE → CALC_ERROR → CALC_P → CALC_I → CALC_D → FILTER_D → CALC_SUM → SATURATE → OUTPUT
```

- **P Term:** `Kp × error` (Q16.16 multiply, upper 32 bits extracted)
- **I Term:** Accumulates error with anti-windup guard; `Ki × integral`
- **D Term:** `Kd × (error - error_prev)`, then IIR filtered: `D_filt = D_filt + α × (D_raw - D_filt)`
- **Sum:** Overflow-safe signed addition using sign-extended operands
- **Saturate:** Clamps to `[out_min, out_max]`, feeds back for anti-windup decision

### `spi_adc_master.v`
SPI Mode 0 master. Reads 16 bits from an ADC and extracts the 12-bit data word.

- Configurable clock divider via `CLK_DIV` parameter
- 4-state FSM: `IDLE → CS_LOW → TRANSFER → CS_HIGH`
- Generates `data_valid` strobe to trigger PID computation

### `pid_config_handler.v`
UART-driven runtime parameter updater. Parses a simple 5-byte serial protocol:

```
[CMD: 'P'/'I'/'D'] [BYTE3] [BYTE2] [BYTE1] [BYTE0]
```
Reassembles the 4 data bytes into a 32-bit Q16.16 gain value and updates the corresponding register. Default gains are set at synthesis time.

### `uart_rx.v` / `uart_tx.v`
Standard UART transceivers parameterized by `CLK_FREQ` and `BAUD_RATE`. Baud rate counter is calculated at elaboration time — no manual timing constants.

### `pwm_generator.v`
12-bit up-counter comparator. Generates a PWM signal whose duty cycle directly tracks `pid_controller_advanced`'s `pwm_duty` output.

---

## Parameters Reference

| Parameter | Module | Default | Description |
|---|---|---|---|
| `DATA_WIDTH` | pid_controller_advanced | 32 | Bit width of all data paths |
| `FRAC_BITS` | pid_controller_advanced | 16 | Fractional bits in Q format |
| `PWM_BITS` | pid_controller_advanced | 12 | PWM resolution |
| `INTEGRAL_BITS` | pid_controller_advanced | 48 | Accumulator width (prevents overflow) |
| `CLK_DIV` | spi_adc_master | 50 | SPI clock = sys_clk / CLK_DIV |
| `CLK_FREQ` | uart_rx / uart_tx | 50,000,000 | System clock frequency in Hz |
| `BAUD_RATE` | uart_rx / uart_tx | 115200 | Serial baud rate |
| `PWM_BITS` | pwm_generator | 12 | Counter/comparator width |

---

## Fixed-Point Number Format (Q16.16)

All PID gains and signals use Q16.16 format:

```
Bit 31          Bit 16  Bit 15          Bit 0
[  Integer Part  ] . [  Fractional Part  ]
```

**Example — Setting Kp = 2.0:**
```
2.0 × 2^16 = 131072 = 0x00020000
```

**Default gains (set in `pid_config_handler.v`):**
- `Kp = 0x00020000` → 2.0
- `Ki = 0x00000100` → ~0.0039
- `Kd = 0x00001000` → ~0.0625

---

## Simulation

The testbench (`tb_pid_system_top.v`) exercises the full system with:
- Simulated SPI ADC responses
- UART gain update sequences
- Step-response verification

**Run with ModelSim / QuestaSim:**
```bash
vlib work
vlog pid_controller_advanced.v spi_adc_master.v uart_rx.v uart_tx.v \
     pwm_generator.v pid_config_handler.v pid_system_top.v tb_pid_system_top.v
vsim -t 1ns tb_pid_system_top
run -all
```

---

## File Structure

```
├── pid_system_top.v          # Top-level integration module
├── pid_controller_advanced.v # Core PID FSM (Q16.16, anti-windup, D-filter)
├── spi_adc_master.v          # SPI Mode 0 ADC reader
├── pid_config_handler.v      # UART-driven runtime gain updater
├── uart_rx.v                 # UART receiver
├── uart_tx.v                 # UART transmitter
├── pwm_generator.v           # 12-bit PWM generator
└── tb_pid_system_top.v       # System-level testbench
```

---

## Target Platform

- **Board:** Basys3 (Xilinx Artix-7)
- **Toolchain:** Vivado 2023.x
- **Simulation:** ModelSim / QuestaSim
- **Clock:** 50 MHz system clock

---

## Potential Applications

- DC motor speed control
- Temperature regulation (PID thermostat)
- Power supply regulation
- Any closed-loop industrial control application requiring real-time FPGA processing

---

## Author

**Parsa hoseinzadeh** — Embedded Systems & FPGA Design Engineer  
[LinkedIn](parsa hoseinzadeh) | [GitHub](https://github.com/Parsajan)
