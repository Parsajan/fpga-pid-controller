// =============================================================================
// PID Config Handler - نسخه v2
// بهبود: اضافه شدن دستور 'S' برای تنظیم Setpoint از لپ‌تاپ
//
// پروتکل UART (5 بایت برای هر دستور):
//   'P' + [4 byte] → Kp      (Q16.16)
//   'I' + [4 byte] → Ki      (Q16.16)
//   'D' + [4 byte] → Kd      (Q16.16)
//   'S' + [4 byte] → Setpoint (Q16.16)
//
// مثال: تنظیم setpoint = 100.0
//   ارسال: 0x53, 0x00, 0x64, 0x00, 0x00
//   (0x53='S', 0x00640000 = 100.0 در Q16.16)
// =============================================================================

module pid_config_handler (
    input wire clk,
    input wire rst_n,
    input wire [7:0] rx_byte,
    input wire rx_valid,

    output reg [31:0] Kp,
    output reg [31:0] Ki,
    output reg [31:0] Kd,
    output reg [31:0] setpoint,    // ▶ جدید: setpoint از لپ‌تاپ
    output reg        config_valid  // پالس یک‌کلاکه برای اعلام آپدیت
);

reg [2:0] byte_cnt;
reg [7:0] cmd;
reg [23:0] temp_val; // فقط 3 بایت اول نگه داشته می‌شه

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        Kp           <= 32'h00020000; // Kp = 2.0
        Ki           <= 32'h00000100; // Ki = 0.004
        Kd           <= 32'h00001000; // Kd = 0.06
        setpoint     <= 32'h00800000; // setpoint = 128.0 (پیش‌فرض)
        byte_cnt     <= 3'd0;
        cmd          <= 8'd0;
        temp_val     <= 24'd0;
        config_valid <= 1'b0;
    end else begin
        config_valid <= 1'b0;

        if (rx_valid) begin

            // --- تشخیص کاراکتر دستور ---
            if (rx_byte == "P" || rx_byte == "I" ||
                rx_byte == "D" || rx_byte == "S") begin
                cmd      <= rx_byte;
                byte_cnt <= 3'd1;
                temp_val <= 24'd0;

            // --- جمع‌آوری 4 بایت داده ---
            end else if (byte_cnt >= 3'd1 && byte_cnt <= 3'd4) begin

                if (byte_cnt == 3'd4) begin
                    // بایت آخر رسید → مقدار کامل را ذخیره کن
                    case (cmd)
                        "P": Kp       <= {temp_val, rx_byte};
                        "I": Ki       <= {temp_val, rx_byte};
                        "D": Kd       <= {temp_val, rx_byte};
                        "S": setpoint <= {temp_val, rx_byte}; // ▶ جدید
                        default: ;
                    endcase
                    config_valid <= 1'b1;
                    byte_cnt     <= 3'd0;
                end else begin
                    // بایت‌های 1 تا 3: در temp_val شیفت می‌شن
                    temp_val <= {temp_val[15:0], rx_byte};
                    byte_cnt <= byte_cnt + 3'd1;
                end
            end
        end
    end
end

endmodule