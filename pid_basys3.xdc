## =============================================================================
## Constraints File : Advanced PID Controller — Basys3 (Artix-7 XC7A35T)
## Project          : FPGA Industrial PID Controller
## Toolchain        : Vivado 2023.x
## Clock Input      : 100 MHz (W5) — divided to 50 MHz internally via MMCM
## =============================================================================

## ============================================================
## CLOCK — 100 MHz onboard oscillator
## period = 10 ns (100 MHz)
## The MMCM (clk_wiz_0) derives 50 MHz internally.
## Vivado automatically creates a 50 MHz constraint from the MMCM output.
## DO NOT add a second create_clock for clk_50m.
## ============================================================
set_property PACKAGE_PIN W5 [get_ports clk_100m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_100m]
create_clock -period 10.000 -name sys_clk_100 -waveform {0.000 5.000} [get_ports clk_100m]

## ============================================================
## RESET — Center button BTNC (active-low in RTL)
## Note: Basys3 buttons output high when pressed.
##       rst_n_int in RTL = rst_n & mmcm_locked, so pressing
##       BTNC (high) drives rst_n low -> active reset.
## ============================================================
set_property PACKAGE_PIN U18 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

## ============================================================
## SPI ADC Interface — Pmod JA Header
## Target: MCP3201 or compatible 12-bit SPI ADC
##
##   JA Pin | Signal     | FPGA Pin
##   -------|------------|----------
##   JA1    | adc_cs_n   | J1
##   JA2    | adc_miso   | L1
##   JA3    | adc_sclk   | L2
##   JA4    | (reserved) | J2   <- can be used for MOSI if ADC needs it
## ============================================================
set_property PACKAGE_PIN J1 [get_ports adc_cs_n]
set_property IOSTANDARD LVCMOS33 [get_ports adc_cs_n]

set_property PACKAGE_PIN L1 [get_ports adc_miso]
set_property IOSTANDARD LVCMOS33 [get_ports adc_miso]

set_property PACKAGE_PIN L2 [get_ports adc_sclk]
set_property IOSTANDARD LVCMOS33 [get_ports adc_sclk]

## ============================================================
## PWM OUTPUT
## Primary: LD0 (onboard LED) — for demo and visual verification
## For real motor control: move to Pmod JA4 (J2) or JB
## ============================================================
set_property PACKAGE_PIN U16 [get_ports pwm_signal]
set_property IOSTANDARD LVCMOS33 [get_ports pwm_signal]

## Alternate — Pmod JA4 for external motor driver:
## set_property PACKAGE_PIN J2 [get_ports pwm_signal]
## set_property IOSTANDARD LVCMOS33 [get_ports pwm_signal]

## ============================================================
## UART — USB-UART Bridge (onboard FT2232HQ chip)
## Baud: 115200, 8N1
## RX: receive Kp/Ki/Kd updates from host PC
## TX: stream ADC feedback data for real-time plotting
## ============================================================
set_property PACKAGE_PIN B18 [get_ports uart_rx_pin]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx_pin]

set_property PACKAGE_PIN A18 [get_ports uart_tx_pin]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx_pin]

## ============================================================
## CONFIGURATION & BITSTREAM
## ============================================================
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

## ============================================================
## TIMING EXCEPTIONS
## ============================================================
## Reset path is asynchronous — no timing closure needed
set_false_path -from [get_ports rst_n]

## UART RX is an asynchronous external input
set_false_path -from [get_ports uart_rx_pin]

## ADC MISO is synchronous to SPI clock (generated internally),
## but treat as false path from the FPGA I/O perspective
set_false_path -from [get_ports adc_miso]
