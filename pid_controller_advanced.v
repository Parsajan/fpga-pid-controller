// =============================================================================
// Advanced Industrial PID Controller for FPGA - نسخه نهایی اصلاح‌شده v2
// اصلاحات:
//   1. Race Condition: هر ضرب به دو state (MUL + REG) تقسیم شد
//   2. Anti-Windup: pid_clamped_prev به‌جای مقدار لحظه‌ای
//   3. SATURATE: sign extension صحیح برای out_min و out_max
//   4. enable latch: پالس یک‌کلاکه enable از دست نمی‌رود
// =============================================================================

`timescale 1ns / 1ps

module pid_controller_advanced #(
    parameter DATA_WIDTH    = 32,
    parameter FRAC_BITS     = 16,
    parameter PWM_BITS      = 12,
    parameter INTEGRAL_BITS = 48
)(
    input wire clk,
    input wire rst_n,
    input wire enable,

    input wire [DATA_WIDTH-1:0] setpoint,
    input wire [DATA_WIDTH-1:0] feedback,

    input wire [DATA_WIDTH-1:0] Kp,
    input wire [DATA_WIDTH-1:0] Ki,
    input wire [DATA_WIDTH-1:0] Kd,
    input wire [DATA_WIDTH-1:0] Kd_filter,
    input wire [DATA_WIDTH-1:0] deadband_thr,

    input wire [DATA_WIDTH-1:0] out_max,
    input wire [DATA_WIDTH-1:0] out_min,

    output reg [PWM_BITS-1:0]   pwm_duty,
    output reg [DATA_WIDTH-1:0] pid_out,
    output reg                  out_valid
);

localparam IDLE        = 4'd0,
           CALC_ERROR  = 4'd1,
           CALC_P_MUL  = 4'd2,
           CALC_P_REG  = 4'd3,
           CALC_I_MUL  = 4'd4,
           CALC_I_REG  = 4'd5,
           CALC_D_MUL  = 4'd6,
           CALC_D_REG  = 4'd7,
           FILTER_MUL  = 4'd8,
           FILTER_REG  = 4'd9,
           CALC_SUM    = 4'd10,
           SATURATE    = 4'd11,
           OUTPUT      = 4'd12;

reg [3:0] state, next_state;

reg signed [DATA_WIDTH-1:0]    error, error_prev;
reg signed [DATA_WIDTH-1:0]    p_term, i_term, d_term, d_term_filt;
reg signed [INTEGRAL_BITS-1:0] integral_acc;
reg signed [2*DATA_WIDTH-1:0]  mult_temp;
reg signed [DATA_WIDTH+1:0]    pid_sum;
reg signed [DATA_WIDTH-1:0]    pid_clamped;
reg signed [DATA_WIDTH-1:0]    pid_clamped_prev;

// ▶ لچ برای نگه داشتن پالس enable
reg enable_latch;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state            <= IDLE;
        enable_latch     <= 1'b0;
        error            <= 0;
        error_prev       <= 0;
        integral_acc     <= 0;
        d_term_filt      <= 0;
        pid_clamped      <= 0;
        pid_clamped_prev <= 0;
        out_valid        <= 0;
        pwm_duty         <= 0;
        pid_out          <= 0;
        mult_temp        <= 0;
        p_term           <= 0;
        i_term           <= 0;
        d_term           <= 0;
        pid_sum          <= 0;
    end else begin

        // ثبت پالس enable - تا شروع CALC_ERROR نگه داشته می‌شود
        if (enable)
            enable_latch <= 1'b1;
        else if (state == CALC_ERROR)
            enable_latch <= 1'b0;

        state <= next_state;

        case (state)
            IDLE: begin
                out_valid <= 1'b0;
            end

            CALC_ERROR: begin
                if (($signed(setpoint) - $signed(feedback)) > $signed(deadband_thr))
                    error <= $signed(setpoint) - $signed(feedback);
                else if (($signed(setpoint) - $signed(feedback)) < -$signed(deadband_thr))
                    error <= $signed(setpoint) - $signed(feedback);
                else
                    error <= 0;
            end

            CALC_P_MUL: begin
                mult_temp <= $signed(Kp) * $signed(error);
            end
            CALC_P_REG: begin
                p_term <= mult_temp[FRAC_BITS +: DATA_WIDTH];
            end

            CALC_I_MUL: begin
                // Anti-Windup با pid_clamped_prev
                if (!( ($signed(pid_clamped_prev) >= $signed(out_max) && $signed(error) > 0) ||
                       ($signed(pid_clamped_prev) <= $signed(out_min) && $signed(error) < 0) )) begin
                    integral_acc <= integral_acc +
                        $signed({{(INTEGRAL_BITS-DATA_WIDTH){error[DATA_WIDTH-1]}}, error});
                end
                mult_temp <= $signed(Ki) * $signed(integral_acc[INTEGRAL_BITS-1:FRAC_BITS]);
            end
            CALC_I_REG: begin
                i_term <= mult_temp[FRAC_BITS +: DATA_WIDTH];
            end

            CALC_D_MUL: begin
                mult_temp  <= $signed(Kd) * ($signed(error) - $signed(error_prev));
                error_prev <= error;
            end
            CALC_D_REG: begin
                d_term <= mult_temp[FRAC_BITS +: DATA_WIDTH];
            end

            FILTER_MUL: begin
                mult_temp <= $signed(Kd_filter) * ($signed(d_term) - $signed(d_term_filt));
            end
            FILTER_REG: begin
                d_term_filt <= $signed(d_term_filt) + $signed(mult_temp[FRAC_BITS +: DATA_WIDTH]);
            end

            CALC_SUM: begin
                pid_sum <= $signed({{2{p_term[DATA_WIDTH-1]}},     p_term})     +
                           $signed({{2{i_term[DATA_WIDTH-1]}},     i_term})     +
                           $signed({{2{d_term_filt[DATA_WIDTH-1]}},d_term_filt});
            end

            SATURATE: begin
                // ▶ sign extension صحیح برای هر دو حد
                if (pid_sum >= $signed({{2{out_max[DATA_WIDTH-1]}}, out_max}))
                    pid_clamped <= out_max;
                else if (pid_sum <= $signed({{2{out_min[DATA_WIDTH-1]}}, out_min}))
                    pid_clamped <= out_min;
                else
                    pid_clamped <= pid_sum[DATA_WIDTH-1:0];

                pid_clamped_prev <= pid_clamped;
            end

            OUTPUT: begin
                pid_out   <= pid_clamped;
                pwm_duty  <= pid_clamped[FRAC_BITS +: PWM_BITS];
                out_valid <= 1'b1;
            end

            default: ;
        endcase
    end
end

always @(*) begin
    case (state)
        IDLE:       next_state = (enable || enable_latch) ? CALC_ERROR : IDLE;
        CALC_ERROR: next_state = CALC_P_MUL;
        CALC_P_MUL: next_state = CALC_P_REG;
        CALC_P_REG: next_state = CALC_I_MUL;
        CALC_I_MUL: next_state = CALC_I_REG;
        CALC_I_REG: next_state = CALC_D_MUL;
        CALC_D_MUL: next_state = CALC_D_REG;
        CALC_D_REG: next_state = FILTER_MUL;
        FILTER_MUL: next_state = FILTER_REG;
        FILTER_REG: next_state = CALC_SUM;
        CALC_SUM:   next_state = SATURATE;
        SATURATE:   next_state = OUTPUT;
        OUTPUT:     next_state = IDLE;
        default:    next_state = IDLE;
    endcase
end

endmodule