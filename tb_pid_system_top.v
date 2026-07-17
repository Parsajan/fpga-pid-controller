// =============================================================================
// Testbench PID System - نسخه v6
// بهبود: تست تغییر setpoint از طریق UART (دستور 'S')
// =============================================================================

`timescale 1ns / 1ps

module tb_pid_system_top;

reg clk_50m;
reg rst_n;
reg uart_rx_pin;
reg adc_miso;

wire adc_cs_n;
wire adc_sclk;
wire pwm_signal;
wire uart_tx_pin;

reg        tb_inject_valid;
reg [11:0] tb_inject_data;

pid_system_top u_top (
    .clk_50m            (clk_50m),
    .rst_n              (rst_n),
    .adc_cs_n           (adc_cs_n),
    .adc_sclk           (adc_sclk),
    .adc_miso           (adc_miso),
    .pwm_signal         (pwm_signal),
    .uart_rx_pin        (uart_rx_pin),
    .uart_tx_pin        (uart_tx_pin),
    .tb_adc_inject_valid(tb_inject_valid),
    .tb_adc_inject_data (tb_inject_data)
);

// کلاک 50MHz
initial clk_50m = 1'b0;
always  #10 clk_50m = ~clk_50m;

// ============================================================
// Task: تزریق مستقیم ADC
// ============================================================
task inject_adc;
    input [11:0] adc_val;
    begin
        @(posedge clk_50m);
        tb_inject_data  <= adc_val;
        tb_inject_valid <= 1'b1;
        @(posedge clk_50m);
        tb_inject_valid <= 1'b0;
    end
endtask

// ============================================================
// Task: ارسال یک بایت UART
// ============================================================
task send_uart_byte;
    input [7:0] data_byte;
    integer i;
    begin
        uart_rx_pin = 1'b0; #8680;
        for (i = 0; i < 8; i = i + 1) begin
            uart_rx_pin = data_byte[i]; #8680;
        end
        uart_rx_pin = 1'b1; #8680;
        #5000;
    end
endtask

// ============================================================
// Task: ارسال پارامتر (ضرایب یا setpoint)
// ============================================================
task send_param;
    input [7:0]  cmd;
    input [31:0] val;
    begin
        $display("[TB  @ %0t us] '%s' = 0x%08X  (= %.4f در Q16.16)",
                 $time/1000, cmd, val, val/65536.0);
        send_uart_byte(cmd);
        send_uart_byte(val[31:24]);
        send_uart_byte(val[23:16]);
        send_uart_byte(val[15: 8]);
        send_uart_byte(val[ 7: 0]);
        #200000;
    end
endtask

// Timeout
initial begin
    #40_000_000;
    $display("[TB] TIMEOUT!");
    $stop;
end

// ============================================================
// سناریوی اصلی
// ============================================================
initial begin
    $dumpfile("pid_sim.vcd");
    $dumpvars(0, tb_pid_system_top);

    rst_n           = 1'b0;
    uart_rx_pin     = 1'b1;
    adc_miso        = 1'b0;
    tb_inject_valid = 1'b0;
    tb_inject_data  = 12'd0;

    #500;
    rst_n = 1'b1;
    $display("[TB] ======================================");
    $display("[TB] سیستم PID با setpoint قابل تنظیم");
    $display("[TB] ======================================");

    #500_000;

    // ============================================================
    // تنظیم ضرایب اولیه
    // ============================================================
    $display("[TB] --- تنظیم ضرایب ---");
    send_param("P", 32'h00010000); // Kp = 1.0
    send_param("I", 32'h00000028); // Ki = 0.0006
    send_param("D", 32'h00008000); // Kd = 0.5

    // ============================================================
    // تست 1: setpoint = 64.0، feedback = 50 → error مثبت
    // ============================================================
    $display("[TB] --- تست 1: SP=64.0 | FB=50 ---");
    send_param("S", 32'h00400000); // setpoint = 64.0
    repeat(20) begin
        inject_adc(12'd800); // 800/16 = 50.0 در Q16.16
        #50_000;
    end

    // ============================================================
    // تست 2: setpoint = 200.0، feedback = 50 → error خیلی بزرگ
    // ============================================================
    $display("[TB] --- تست 2: SP=200.0 | FB=50 → اشباع ---");
    send_param("S", 32'h00C80000); // setpoint = 200.0
    repeat(20) begin
        inject_adc(12'd800);
        #50_000;
    end

    // ============================================================
    // تست 3: feedback = setpoint → error ≈ 0
    // ============================================================
    $display("[TB] --- تست 3: FB≈SP → error≈0 → pwm کم ---");
    send_param("S", 32'h00320000); // setpoint = 50.0
    repeat(20) begin
        inject_adc(12'd800); // feedback = 50.0 = setpoint!
        #50_000;
    end

    // ============================================================
    // تست 4: feedback > setpoint → error منفی → pwm=0
    // ============================================================
    $display("[TB] --- تست 4: FB>SP → error منفی → pwm=0 ---");
    send_param("S", 32'h00100000); // setpoint = 16.0
    repeat(20) begin
        inject_adc(12'd800); // feedback = 50 > setpoint = 16
        #50_000;
    end

    $display("[TB] همه تست‌ها تموم شد.");
    $finish;
end

// ============================================================
// مانیتور — نمایش وضعیت سیستم
// ============================================================
always @(posedge clk_50m) begin
    if (u_top.pid_out_valid) begin
        $display("[PID @ %0t us] SP=%.2f | FB=%.2f | ERR=%.2f | pwm=%0d/4095 (%.1f%%)",
                 $time/1000,
                 u_top.dynamic_setpoint / 65536.0,
                 u_top.feedback_q16     / 65536.0,
                 ($signed(u_top.dynamic_setpoint) - $signed(u_top.feedback_q16)) / 65536.0,
                 u_top.pid_pwm_duty,
                 u_top.pid_pwm_duty * 100.0 / 4095.0);
    end
end

// مانیتور تغییر setpoint
always @(u_top.dynamic_setpoint) begin
    if (rst_n)
        $display("[CFG @ %0t us] >>> Setpoint تغییر کرد به: %.4f (0x%08X)",
                 $time/1000,
                 u_top.dynamic_setpoint / 65536.0,
                 u_top.dynamic_setpoint);
end

endmodule