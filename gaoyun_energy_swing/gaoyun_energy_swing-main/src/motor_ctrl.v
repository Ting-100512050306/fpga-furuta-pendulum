//=============================================================================
// 模块: motor_ctrl   TB6612FNG 单通道驱动(有符号控制量 -> 方向 + PWM)
//-----------------------------------------------------------------------------
//   cmd : 有符号控制量
//         > 0 -> ain1 = 1, ain2 = 0  (正转)
//         < 0 -> ain1 = 0, ain2 = 1  (反转)
//         = 0 -> ain1 = ain2 = 0     (停止)
//
//   占空比 = |cmd| >>> DUTY_SHIFT,上限 DUTY_MAX
//   载波  : 50MHz / 5400 ≈ 9.26kHz(与原工程保持一致)
//   映射  : duty * 20,即 duty=250 时高电平 5000 < 5400,接近满占空比
//=============================================================================
module motor_ctrl #(
    parameter [24:0] PWM_PERIOD = 25'd5400 ,  // PWM 载波周期(50MHz 时钟)
    parameter [3:0]  DUTY_SHIFT = 4'd4     ,  // |cmd| 到占空比的移位
    parameter [7:0]  DUTY_MAX   = 8'd250      // 占空比上限(0~255)
)(
    input                      clk_50m ,
    input                      rst_n   ,
    input       signed [15:0]  cmd     ,   // 有符号控制量:符号=方向,幅值=力度
    output reg                 ain1    ,
    output reg                 ain2    ,
    output                     pwm
);

    //------------------------------------------------------------------
    // PWM 载波
    //------------------------------------------------------------------
    reg [24:0] pwm_cnt;

    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)
            pwm_cnt <= 25'd0;
        else if(pwm_cnt >= PWM_PERIOD - 25'd1)
            pwm_cnt <= 25'd0;
        else
            pwm_cnt <= pwm_cnt + 25'd1;
    end

    //------------------------------------------------------------------
    // |cmd| -> 请求占空比
    //------------------------------------------------------------------
    wire [15:0] cmd_abs = cmd[15] ? (~cmd + 16'd1) : cmd[15:0];
    wire [16:0] duty_u  = {1'b0, cmd_abs} >> DUTY_SHIFT;
    wire [7:0]  duty_req = (duty_u > DUTY_MAX) ? DUTY_MAX : duty_u[7:0];

    //------------------------------------------------------------------
    // 周期边界锁存 + 反向死区
    //   正在带载时若命令符号改变，先输出整整一个 PWM 周期的 0 占空比，
    //   下一个周期边界才接通新方向。方向和比较值均不会在周期中途跳变。
    //------------------------------------------------------------------
    reg [7:0] duty_r;
    reg       dir_r;
    reg       reverse_dead;

    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            duty_r       <= 8'd0;
            dir_r        <= 1'b0;
            reverse_dead <= 1'b0;
        end else if(pwm_cnt == 25'd0) begin
            if(reverse_dead) begin
                // 已完成一个零占空比周期，采用此刻最新命令。
                reverse_dead <= 1'b0;
                dir_r        <= cmd[15];
                duty_r       <= duty_req;
            end else if(duty_req == 8'd0) begin
                duty_r       <= 8'd0;
            end else if((duty_r != 8'd0) && (cmd[15] != dir_r)) begin
                reverse_dead <= 1'b1;
                duty_r       <= 8'd0;
            end else begin
                dir_r        <= cmd[15];
                duty_r       <= duty_req;
            end
        end
    end

    wire [24:0] cmp = duty_r * 25'd20;

    always @(*) begin
        if(duty_r == 8'd0) begin
            ain1 = 1'b0;
            ain2 = 1'b0;
        end else if(dir_r) begin
            ain1 = 1'b0;
            ain2 = 1'b1;
        end else begin
            ain1 = 1'b1;
            ain2 = 1'b0;
        end
    end

    assign pwm = (pwm_cnt < cmp) ? 1'b1 : 1'b0;

endmodule
