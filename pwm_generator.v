// =============================================================================
// PWM Generator - اصلاح‌شده
// اصلاحات:
//   1. duty_cycle = 0 → خروجی همیشه 0 (بدون glitch)
//   2. duty_cycle = MAX → خروجی همیشه 1
//   3. کامنت‌گذاری بهتر
// =============================================================================

module pwm_generator #(
    parameter PWM_BITS = 12
)(
    input wire clk,
    input wire rst_n,
    input wire [PWM_BITS-1:0] duty_cycle,
    output reg pwm_out
);

localparam MAX_COUNT = (1 << PWM_BITS) - 1; // 4095 برای 12 بیت

reg [PWM_BITS-1:0] counter;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= {PWM_BITS{1'b0}};
        pwm_out <= 1'b0;
    end else begin
        counter <= counter + {{(PWM_BITS-1){1'b0}}, 1'b1};

        // ▶ اصلاح: مدیریت حالت‌های مرزی
        if (duty_cycle == {PWM_BITS{1'b0}}) begin
            // duty = 0%: خروجی همیشه خاموش
            pwm_out <= 1'b0;
        end else if (duty_cycle == {PWM_BITS{1'b1}}) begin
            // duty = 100%: خروجی همیشه روشن
            pwm_out <= 1'b1;
        end else begin
            // حالت عادی: مقایسه شمارنده با duty cycle
            pwm_out <= (counter < duty_cycle) ? 1'b1 : 1'b0;
        end
    end
end

endmodule