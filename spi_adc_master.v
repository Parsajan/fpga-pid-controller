// =============================================================================
// SPI ADC Master - نسخه نهایی تأیید‌شده v3
// =============================================================================
// اصلاحات:
//   1. شرط خروج از TRANSFER: if(bit_cnt == 15) صحیح است (نه 14)
//      چون bit_cnt مقدار فعلی را چک می‌کند (قبل از +1)، پس 16 بیت کامل می‌شود
//   2. خواندن MISO در لبه بالارونده (clk_cnt == HALF)
//   3. Slice داده: shift_reg[14:3] برای MCP3201
//      (بیت 15 = null، بیت‌های 14:3 = داده 12 بیتی)
// =============================================================================

`timescale 1ns / 1ps

module spi_adc_master #(
    parameter CLK_DIV = 50
)(
    input wire clk,
    input wire rst_n,
    input wire start,

    output reg cs_n,
    output reg sclk,
    input  wire miso,

    output reg [11:0] adc_data,
    output reg        data_valid
);

localparam IDLE     = 2'd0;
localparam CS_LOW   = 2'd1;
localparam TRANSFER = 2'd2;
localparam CS_HIGH  = 2'd3;

localparam HALF = CLK_DIV / 2; // = 25 برای CLK_DIV=50

reg [1:0] state;
reg [15:0] shift_reg;
reg [7:0]  clk_cnt;
reg [4:0]  bit_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= IDLE;
        cs_n       <= 1'b1;
        sclk       <= 1'b0;
        adc_data   <= 12'd0;
        data_valid <= 1'b0;
        clk_cnt    <= 8'd0;
        bit_cnt    <= 5'd0;
        shift_reg  <= 16'd0;
    end else begin
        data_valid <= 1'b0;

        case (state)
            // ------------------------------------------------------------------
            IDLE: begin
                cs_n    <= 1'b1;
                sclk    <= 1'b0;
                clk_cnt <= 8'd0;
                bit_cnt <= 5'd0;
                shift_reg <= 16'd0;
                if (start)
                    state <= CS_LOW;
            end

            // ------------------------------------------------------------------
            CS_LOW: begin
                cs_n <= 1'b0;
                if (clk_cnt == HALF - 1) begin
                    clk_cnt <= 8'd0;
                    state   <= TRANSFER;
                end else begin
                    clk_cnt <= clk_cnt + 8'd1;
                end
            end

            // ------------------------------------------------------------------
            TRANSFER: begin
                clk_cnt <= clk_cnt + 8'd1;

                // لبه بالارونده: SCLK=1 و نمونه‌برداری MISO
                if (clk_cnt == HALF - 1) begin
                    sclk      <= 1'b1;
                    shift_reg <= {shift_reg[14:0], miso};

                // لبه پایین‌رونده: SCLK=0 و شمارش بیت
                end else if (clk_cnt == CLK_DIV - 1) begin
                    sclk    <= 1'b0;
                    clk_cnt <= 8'd0;

                    // ✅ چک با مقدار فعلی bit_cnt (قبل از +1)
                    // وقتی bit_cnt=15، یعنی 16 بیت کامل خونده شده
                    if (bit_cnt == 5'd15) begin
                        state <= CS_HIGH;
                    end else begin
                        bit_cnt <= bit_cnt + 5'd1;
                    end
                end
            end

            // ------------------------------------------------------------------
            CS_HIGH: begin
                cs_n <= 1'b1;
                // MCP3201 فرمت 16 بیتی:
                // bit15 = null bit (0)
                // bit14..3 = داده 12 بیتی (MSB اول)
                // bit2..0  = filler
                adc_data   <= shift_reg[14:3];
                data_valid <= 1'b1;
                state      <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule