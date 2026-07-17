// =============================================================================
// Project   : Advanced Industrial PID Controller
// Module    : Top Level Integration
// Board     : Basys3 (Xilinx Artix-7 XC7A35T)
// Toolchain : Vivado 2023.x
// Clock     : 100 MHz input (W5) -> 50 MHz via Clocking Wizard (MMCM)
//
// PIN MAP (Basys3):
//   clk_100m     -> W5   (100 MHz onboard oscillator)
//   rst_n        -> U18  (Center button BTNC, active-low)
//
//   [SPI ADC — Pmod JA]
//   adc_cs_n     -> J1   (JA1, Pin 1)  Chip Select (active-low)
//   adc_sclk     -> L2   (JA3, Pin 3)  SPI Clock (~1 MHz)
//   adc_miso     -> L1   (JA2, Pin 2)  Master In Slave Out
//
//   [PWM Output]
//   pwm_signal   -> U16  (LD0)         Demo on LED; route to JA4 for motor driver
//
//   [UART — USB-UART bridge]
//   uart_rx_pin  -> B18  (USB-UART RXD) Receive Kp/Ki/Kd from host PC
//   uart_tx_pin  -> A18  (USB-UART TXD) Stream ADC feedback to host PC
//
// CLOCK ARCHITECTURE:
//   clk_100m (W5) --> [clk_wiz_0 / MMCM] --> clk_50m (internal, buffered)
//                                         --> mmcm_locked
//   All internal logic runs on clk_50m.
//   Reset is held asserted until mmcm_locked goes high (rst_n_int).
//
// UART PROTOCOL (pid_config_handler):
//   Send 5 bytes to update a gain: ['P'|'I'|'D'] [B3] [B2] [B1] [B0]
//   Example — set Kp = 2.0 (Q16.16 = 0x00020000):
//     'P' 0x00 0x02 0x00 0x00
// =============================================================================

`timescale 1ns / 1ps

module pid_system_top (
    // Clock & Reset
    input  wire clk_100m,     // W5  — 100 MHz onboard oscillator
    input  wire rst_n,        // U18 — Active-low reset (BTNC)

    // SPI ADC Interface (Pmod JA) — MCP3201 or compatible 12-bit ADC
    output wire adc_cs_n,     // J1  — Chip Select, active-low
    output wire adc_sclk,     // L2  — SPI clock (50 MHz / CLK_DIV = 1 MHz)
    input  wire adc_miso,     // L1  — Serial data from ADC

    // PWM Output — connect to motor driver / power stage
    output wire pwm_signal,   // U16 — 12-bit PWM (LD0 for demo; JA4 for real use)

    // UART — Host PC Interface (115200 baud, 8N1)
    input  wire uart_rx_pin,  // B18 — Receive gain updates (Kp, Ki, Kd)
    output wire uart_tx_pin   // A18 — Transmit ADC feedback for host-side plotting
);

// =============================================================================
// Internal Parameters
// =============================================================================
// SAMPLE_COUNT: clock cycles between ADC reads @ 50 MHz
//   50 MHz / 50000 = 1 kHz sample rate (1 ms period)
localparam SAMPLE_COUNT = 50000;

// =============================================================================
// Clock & Reset Signals
// =============================================================================
wire clk_50m;       // 50 MHz buffered clock — output of MMCM
wire mmcm_locked;   // MMCM lock indicator: 1 = clock stable and valid

// Reset synchronizer: generates a clean reset pulse after MMCM locks
// Sequence:
//   1. After power-up: rst_n_int held low until mmcm_locked goes high
//   2. Once locked: rst_n_int released (goes high) and stays high
//   3. Manual reset: pressing BTNC drives rst_n low, which directly resets logic
reg [3:0] rst_sync_counter;
wire rst_n_int;

always @(posedge clk_50m or negedge rst_n or negedge mmcm_locked) begin
    if (!rst_n || !mmcm_locked) begin
        // Active reset: button pressed OR MMCM not locked
        rst_sync_counter <= 4'd0;
    end else if (rst_sync_counter < 4'd15) begin
        // Count up while MMCM is locked and button not pressed
        // This ensures internal logic sees a stable rising edge of rst_n_int
        rst_sync_counter <= rst_sync_counter + 1'b1;
    end
end

// rst_n_int released only after MMCM locked AND counter saturated
assign rst_n_int = (rst_sync_counter == 4'd15) & rst_n;

// =============================================================================
// 0. Clocking Wizard (MMCM)
//    Divides 100 MHz input clock to 50 MHz for all internal logic
//    Generated IP: clk_wiz_0 (Vivado IP Catalog -> Clocking Wizard)
//    Settings: Input = 100 MHz, clk_out1 = 50 MHz, Primitive = MMCM
// =============================================================================
clk_wiz_0 u_clk_wiz (
    .clk_in1  (clk_100m),   // 100 MHz from Basys3 oscillator (W5)
    .clk_out1 (clk_50m),    // 50 MHz buffered output to all modules
    .locked   (mmcm_locked) // Asserted when output clock is stable
);

// =============================================================================
// Internal Signals
// =============================================================================
reg  [15:0] sample_timer;
reg         adc_start_trigger;

wire [11:0] raw_adc_data;   // 12-bit raw ADC reading
wire        adc_ready;      // Strobe: new ADC sample available
wire [31:0] feedback_q16;   // ADC data scaled to Q16.16 format

wire [11:0] pid_pwm_duty;   // 12-bit PWM duty cycle from PID core

wire [7:0]  uart_byte;      // Received UART byte
wire        uart_valid;     // Strobe: new UART byte received

wire [31:0] dynamic_Kp;     // Runtime-configurable gains (Q16.16)
wire [31:0] dynamic_Ki;
wire [31:0] dynamic_Kd;

// =============================================================================
// 1. Sampling Timer
//    Generates a 1-clock-wide start pulse at 1 kHz (every 50000 cycles @ 50 MHz)
// =============================================================================
always @(posedge clk_50m or negedge rst_n_int) begin
    if (!rst_n_int) begin
        sample_timer      <= 16'd0;
        adc_start_trigger <= 1'b0;
    end else begin
        if (sample_timer == SAMPLE_COUNT - 1) begin
            sample_timer      <= 16'd0;
            adc_start_trigger <= 1'b1;  // One-cycle pulse to trigger SPI read
        end else begin
            sample_timer      <= sample_timer + 1'b1;
            adc_start_trigger <= 1'b0;
        end
    end
end

// =============================================================================
// 2. SPI ADC Reader
//    Reads 12-bit data from MCP3201-compatible ADC via SPI Mode 0
//    SPI clock = 50 MHz / CLK_DIV = 50 MHz / 50 = 1 MHz
// =============================================================================
spi_adc_master #(
    .CLK_DIV(50)
) u_adc_reader (
    .clk        (clk_50m),
    .rst_n      (rst_n_int),
    .start      (adc_start_trigger),
    .cs_n       (adc_cs_n),
    .sclk       (adc_sclk),
    .miso       (adc_miso),
    .adc_data   (raw_adc_data),
    .data_valid (adc_ready)
);

// Scale 12-bit ADC output to Q16.16 fixed-point format
// raw_adc_data[11:0] placed at bits [27:16] — integer part only, fraction = 0
assign feedback_q16 = {16'd0, raw_adc_data, 4'd0};

// =============================================================================
// 3. UART Receiver
//    Receives gain update commands from host PC (Python script / serial terminal)
//    Protocol: ['P'|'I'|'D'] + 4 data bytes (32-bit Q16.16 value, MSB first)
// =============================================================================
uart_rx #(
    .CLK_FREQ  (50_000_000),
    .BAUD_RATE (115_200)
) u_rx (
    .clk     (clk_50m),
    .rst_n   (rst_n_int),
    .rx      (uart_rx_pin),
    .rx_data (uart_byte),
    .rx_done (uart_valid)
);

// =============================================================================
// 4. PID Config Handler
//    Parses 5-byte UART frames and updates Kp, Ki, Kd registers live
//    Default gains at reset: Kp=2.0, Ki=~0.004, Kd=~0.063 (Q16.16)
// =============================================================================
pid_config_handler u_config (
    .clk      (clk_50m),
    .rst_n    (rst_n_int),
    .rx_byte  (uart_byte),
    .rx_valid (uart_valid),
    .Kp       (dynamic_Kp),
    .Ki       (dynamic_Ki),
    .Kd       (dynamic_Kd)
);

// =============================================================================
// 5. Advanced PID Controller Core
//    Q16.16 fixed-point arithmetic, 9-state pipeline FSM
//    Features: Anti-Windup, Derivative IIR Filter, Dead-band, Output Saturation
// =============================================================================
pid_controller_advanced #(
    .DATA_WIDTH    (32),
    .FRAC_BITS     (16),
    .PWM_BITS      (12),
    .INTEGRAL_BITS (48)
) u_pid_core (
    .clk          (clk_50m),
    .rst_n        (rst_n_int),
    .enable       (adc_ready),          // Trigger on every new ADC sample
    .setpoint     (32'h00008000),       // Default setpoint = 0.5 (Q16.16)
    .feedback     (feedback_q16),
    .Kp           (dynamic_Kp),
    .Ki           (dynamic_Ki),
    .Kd           (dynamic_Kd),
    .Kd_filter    (32'h00003333),       // IIR alpha ~= 0.2
    .deadband_thr (32'h00000020),       // Dead-band threshold ~= 0.0005
    .out_max      (32'h0000FFFF),       // Max output limit (Q16.16)
    .out_min      (32'h00000000),       // Min output limit
    .pwm_duty     (pid_pwm_duty)
);

// =============================================================================
// 6. PWM Generator
//    12-bit resolution: 50 MHz / 4096 = ~12.2 kHz PWM frequency
//    duty = 0 -> 0% | duty = 4095 -> 100%
// =============================================================================
pwm_generator #(
    .PWM_BITS(12)
) u_pwm_gen (
    .clk        (clk_50m),
    .rst_n      (rst_n_int),
    .duty_cycle (pid_pwm_duty),
    .pwm_out    (pwm_signal)
);

// =============================================================================
// 7. UART Transmitter
//    Streams 8 MSBs of ADC reading to host PC for real-time plotting
//    Fires on every adc_ready strobe (1 kHz)
// =============================================================================
uart_tx #(
    .CLK_FREQ  (50_000_000),
    .BAUD_RATE (115_200)
) u_tx (
    .clk      (clk_50m),
    .rst_n    (rst_n_int),
    .tx_data  (raw_adc_data[11:4]),    // 8 MSBs of 12-bit ADC value
    .tx_start (adc_ready),
    .tx       (uart_tx_pin)
);

endmodule
