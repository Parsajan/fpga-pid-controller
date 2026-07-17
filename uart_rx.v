// =============================================================================
// UART Receiver - اصلاح‌شده
// اصلاحات:
//   1. نمونه‌برداری در وسط هر بیت (mid-point sampling) برای پایداری بیشتر
//   2. ریست صحیح تمام رجیسترها
//   3. بررسی stop bit برای جلوگیری از دریافت داده‌های نویزی
// =============================================================================

module uart_rx #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rx,

    output reg [7:0] rx_data,
    output reg       rx_done
);

// تعداد کلاک برای هر بیت
localparam integer WAIT_COUNT     = CLK_FREQ / BAUD_RATE;
// نیمه دوره برای شروع نمونه‌برداری از وسط بیت Start
localparam integer HALF_WAIT      = WAIT_COUNT / 2;

reg [15:0] timer;
reg [3:0]  bit_idx;
reg [1:0]  state;

localparam IDLE  = 2'd0;
localparam START = 2'd1;
localparam DATA  = 2'd2;
localparam STOP  = 2'd3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state   <= IDLE;
        rx_done <= 1'b0;
        rx_data <= 8'd0;
        timer   <= 16'd0;
        bit_idx <= 4'd0;
    end else begin
        rx_done <= 1'b0; // پالس یک‌کلاکه

        case (state)
            // ----------------------------------------------------------------
            IDLE: begin
                if (rx == 1'b0) begin
                    // لبه پایین‌رونده: شروع بیت Start
                    // تایمر را روی نصف دوره قرار می‌دهیم تا
                    // نمونه‌برداری بعدی در وسط بیت Start باشد
                    state <= START;
                    timer <= HALF_WAIT - 1;
                end
            end

            // ----------------------------------------------------------------
            START: begin
                if (timer == 16'd0) begin
                    // بررسی اینکه خط هنوز 0 است (تایید Start Bit)
                    if (rx == 1'b0) begin
                        state   <= DATA;
                        timer   <= WAIT_COUNT - 1;
                        bit_idx <= 4'd0;
                    end else begin
                        // نویز بود، برگشت به IDLE
                        state <= IDLE;
                    end
                end else begin
                    timer <= timer - 16'd1;
                end
            end

            // ----------------------------------------------------------------
            DATA: begin
                if (timer == 16'd0) begin
                    // نمونه‌برداری از وسط هر بیت داده
                    rx_data[bit_idx] <= rx;
                    timer            <= WAIT_COUNT - 1;

                    if (bit_idx == 4'd7) begin
                        state <= STOP;
                    end else begin
                        bit_idx <= bit_idx + 4'd1;
                    end
                end else begin
                    timer <= timer - 16'd1;
                end
            end

            // ----------------------------------------------------------------
            STOP: begin
                if (timer == 16'd0) begin
                    // ▶ اصلاح: تنها در صورت معتبر بودن Stop Bit داده را اعلام می‌کنیم
                    if (rx == 1'b1) begin
                        rx_done <= 1'b1;
                    end
                    // در هر صورت به IDLE برمی‌گردیم
                    state <= IDLE;
                end else begin
                    timer <= timer - 16'd1;
                end
            end

            default: state <= IDLE;
        endcase
    end
end

endmodule