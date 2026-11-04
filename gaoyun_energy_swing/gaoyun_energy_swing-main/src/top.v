//=============================================================================
// 模块: top   J280 姿态控制系统 —— 一级旋转倒立摆 自动起摆 + 倒立平衡
//-----------------------------------------------------------------------------
// 数据流
//   WDD35D4 电位器 -> 3PA1030 ADC -> adc.v (3ms 均值) -> angle_proc.v
//        -> phi / omega / cos_phi -> controller.v
//   MG531 霍尔编码器 -> encoder.v -> 旋转臂位置 -> controller.v / uart.v
//   controller.v -> motor_ctrl.v -> TB6612FNG -> MG531 电机
//
// 操作
//   SW5 : 复位(rst_n)
//   SW1 : 启动 / 停止 自动起摆
//   SW3 : 触发串口上报(与参考工程一致)
//   SW2 : 本设计未使用(保留管脚,便于后续扩展)
//   SW4 : 本设计未使用。其引脚 T20 是 GW2A-55 的 SSPI 专用配置脚,默认被
//         锁定为专用功能,直接使用会导致布局布线报 PR2017/PR2028,因此
//         本设计不使用 SW4。若后续确实需要,可在云源软件
//         Project -> Configuration -> Place & Route -> Dual-Purpose Pin
//         勾选 Use SSPI as regular IO 后再恢复该按键。
//
//   串口帧格式保持原样: AA 55 | 角度16bit | 编码器32bit | 0D 0A
//   其中“角度16bit”为有符号摆角 phi(0 = 倒立),编码器为 {臂位置, 备用}
//
//   串口接收(param_cfg.v):上位机可下发 6 字节参数帧在线改整定值,并可远程
//   起停,免去“改参数 -> 综合布线 -> 烧录”的循环。详见 param_cfg.v。
//=============================================================================
module top(
    input               clk_50m     ,   // 50MHz 时钟
    input               rst_n       ,   // 复位(SW5)

    input       [2:0]   key_sw      ,   // SW1 ~ SW3

    output              uart_tx     ,
    input               uart_rx     ,

    // ADC 3PA1030 + WDD35D4
    // 注:J280 底板未把 ADC 的 STBY 引脚引到 FPGA(板上已固定接好),
    //    故此处不定义 adc_stby_n,避免它被自动分配到板上的其他网络。
    output              adc_clk     ,
    output              adc_oe_n    ,
    input       [9:0]   adc_data_in ,

    // 编码器
    input               enc1_a      ,
    input               enc1_b      ,
    input               enc2_a      ,
    input               enc2_b      ,

    // MG531 + TB6612FNG
    output              AN1         ,
    output              AN2         ,
    output              PWMA        ,
    output              BN1         ,
    output              BN2         ,
    output              PWMB
);

    //------------------------------------------------------------------
    // 按键消抖
    //------------------------------------------------------------------
    wire [2:0] swing_key;
    // 提前声明跨区使用的信号，避免工具把首次引用误建为隐式 1bit 网线。
    wire              theta_vld;
    wire signed [15:0] p_arm_off;
    wire signed [15:0] p_traj_sw;

    genvar i;
    generate for(i = 0; i <= 2; i = i + 1) begin : gen_key

        key_filter #(
            .CNT_MAX (20'd1000_000)
        ) key_filter_inst (
            .sys_clk   (clk_50m     ),
            .sys_rst_n (rst_n       ),
            .key_in    (key_sw[i]   ),
            .key_flag  (swing_key[i])
        );

    end
    endgenerate

    //------------------------------------------------------------------
    // 拓展1 摆臂定点位置控制: SW2 按键循环选择档位
    //   [2026/10/23 已由 5 档改为 3 档; 2026/10/27 修正此注释与实际代码一致]
    //   档位: 0(中, 0deg) -> 1(左, +285=+90deg) -> 2(右, -285=-90deg) -> 0 ...
    //   编码器 1138 counts/圈: 90deg = 285 counts
    //   串口参数 arm_off(0x0E) 作为微调, 与按键档位叠加
    //------------------------------------------------------------------
    //==================================================================
    // [2026/10/23] 拓展2 轨迹控制: 按 SW3 切换正弦轨迹模式
    //   短按 SW2 = 三档静止位置循环
    //   短按 SW3 = 进入/退出正弦轨迹(12 秒周期, 幅度 400 counts)
    //==================================================================
    reg traj_en;

    // [2026/10/27] 轨迹模式使能 = SW3 按键切换  或  串口 traj_sw(0x0F) 非零
    //   原设计只能靠 SW3 进入; 增加串口通道后既便于远程演示, 也便于自动化测试
    wire traj_en_eff = traj_en | (p_traj_sw != 16'sd0);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)          traj_en <= 1'b0;
        else if (swing_key[2]) traj_en <= ~traj_en;     // SW3 单击切换
    end

    reg [2:0] pos_sel;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            pos_sel <= 3'd0;
        else if (swing_key[1])              // [2026/10/23] 只用 SW2 切档(去掉 SW3)
            pos_sel <= (pos_sel == 3'd2) ? 3'd0 : (pos_sel + 3'd1);   // [2026/10/23] 3 档: 中/左/右
    end

    reg signed [15:0] pos_tab;
    always @(*) begin
        case (pos_sel)
            3'd0: pos_tab =  16'sd0;
            3'd1: pos_tab =  16'sd285;   // 左: 命令 90 度 -> 实际约 50 度
            3'd2: pos_tab = -16'sd285;   // 右: 命令 -90 度 -> 实际约 -50 度
            3'd3: pos_tab =  16'sd0;     // (不用)
            3'd4: pos_tab =  16'sd0;     // (不用)
            default: pos_tab = 16'sd0;
        endcase
    end


    //------------------------------------------------------------------
    // E2 轨迹模式（第 6 档）: 目标 = 参考点 + TRAJ_A*cos(2*pi*t/TRAJ_T)
    //   TRAJ_A = 450 counts = 142 deg, 周期 10 秒
    //   复用 cos_lut(0~180deg, Q14): 相位 0..511 对应 0..360deg
    //------------------------------------------------------------------
    // [2026/10/27 实测结论] 摆臂只能跟到指令幅度的约 1/3 —— 位置环想拉摆臂(输出近饱和),
    //   但倾角环同时给出反向力矩抵掉大半, 两者对抗的结果就是 1/3。
    //   指令 400 -> 摆臂 ±43°(R²=1.00, BAL 100%, |phi|max 9.5°)  ✓ 安全
    //   指令 1000-> 摆杆直接倒(BAL 19%, 摆臂飞转 12.8 圈)          ✗ 不可用
    //   故取 400; 若要更大摆幅需同时放松倾角环, 会牺牲平衡性能, 不做。
    localparam signed [15:0] TRAJ_A = 16'sd400;   // 指令振幅 400 counts = 127 deg (实际摆臂约 ±43 deg)
    // 周期 12s = 4000 拍; 相位 0..511 对应 0..360deg
    //   PH_STEP = 512 * 2^17 / 4000 = 16777
    //   注: 位置环带宽 wn≈0.95 rad/s (周期 6.7s), 轨迹周期必须显著长于它才能跟得上。
    //       最初设 8s 时实测只出 ±190/周期3.3s(跟不上), 改 12s。
    localparam        [31:0] PH_STEP = 32'd10065;   // [2026/10/27] 12s -> 20s 周期: 512*2^17/6667=10065


    // [2026/10/27] ★ 软进入: 直接开轨迹会让目标从 0 跳到 +400(=127deg), 摆臂猛甩把摆杆甩倒
    //   实测: 开轨迹后 1.5 秒摆杆就掉了(BAL 17%), 摆臂飞转 13.8 圈。
    //   修法: (1) 进入瞬间相位归零(从余弦峰值起步, 该点速度最小)
    //         (2) 幅度用 traj_gain 在 2.3 秒内从 0 升到 256(满幅)
    //             -> 目标爬升速率 400/2.3 ≈ 174 counts/s < 斜坡上限 333 counts/s
    reg traj_en_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) traj_en_d <= 1'b0;
        else        traj_en_d <= traj_en_eff;
    end
    wire traj_start = traj_en_eff & ~traj_en_d;

    reg [8:0] traj_gain;
    reg [1:0] tg_div;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            traj_gain <= 9'd0;
            tg_div    <= 2'd0;
        end else if (theta_vld) begin
            if (!traj_en_eff) begin
                traj_gain <= 9'd0;
                tg_div    <= 2'd0;
            end else begin
                tg_div <= tg_div + 2'd1;
                if (tg_div == 2'd2 && traj_gain < 9'd256)
                    traj_gain <= traj_gain + 9'd1;
            end
        end
    end

    reg [31:0] ph_acc;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                        ph_acc <= 32'd0;
        else if (traj_start)               ph_acc <= 32'd0;
        else if (theta_vld && traj_en_eff) ph_acc <= ph_acc + PH_STEP;
    end

    wire [8:0]  traj_ph  = ph_acc[25:17];
    wire [7:0]  traj_idx = (traj_ph < 9'd256) ? traj_ph[7:0] : (traj_ph[7:0] - 8'd0);
    wire signed [15:0] traj_cos;
    cos_lut traj_lut (.idx(traj_idx), .cos_val(traj_cos));
    wire signed [15:0] traj_sin = (traj_ph < 9'd256) ? traj_cos : -traj_cos;
    wire signed [31:0] traj_mul = $signed(traj_sin) * $signed(TRAJ_A);
    wire signed [15:0] traj_val_next = traj_mul >>> 14;
    // LUT 与幅度软启动之间插入一级流水，避免两个乘法器串在同一 20ns 周期。
    reg signed [15:0] traj_val;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            traj_val <= 16'sd0;
        else if(theta_vld)
            traj_val <= traj_val_next;
    end
    // 幅度软启动: traj_val * traj_gain / 256
    wire signed [31:0] traj_mulg = $signed(traj_val) * $signed({1'b0, traj_gain});
    wire signed [15:0] traj_val_g = traj_mulg >>> 8;

    wire signed [15:0] arm_off_next = traj_en_eff ? (traj_val_g + p_arm_off)
                                                   : (pos_tab + p_arm_off);
    // 截断 ph_acc -> LUT -> 两级乘法 -> balance.arm_tgt 的长组合路径。
    // 目标只需随 3ms 控制拍更新，因此增加这一拍不会影响机械响应。
    reg signed [15:0] arm_off_eff;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            arm_off_eff <= 16'sd0;
        else if(theta_vld)
            arm_off_eff <= arm_off_next;
    end


    //------------------------------------------------------------------
    // 角度采集 + 摆角处理
    //------------------------------------------------------------------
    wire [11:0] theta_out;

    adc adc_inst(
        .clk_50m     (clk_50m     ),
        .rst_n       (rst_n       ),
        .adc_clk     (adc_clk     ),
        .adc_oe_n    (adc_oe_n    ),
        .adc_data_in (adc_data_in ),
        .theta_out   (theta_out   ),
        .theta_vld   (theta_vld   )
    );

    wire signed [15:0] phi;
    wire signed [15:0] omega;
    wire signed [15:0] cos_phi;

    // ★ ZERO_RAW 为“倒立(平衡)位置”对应的 ADC 原始值,已上板标定。
    //   标定方法:用手把摆杆扶到正倒立静止,读串口上报的 phi=X,
    //             则 ZERO_RAW = 旧值 + X。
    //   实测(2026-09):下垂位 raw≈904~922;倒立静止时 phi≈-122,
    //             故 3150 + (-122) = 3028。
    // [2026/10/17] 3028 -> 3001:上板复测自然下垂位 phi=1954(静止,峰峰<100),
    //   而按"下垂位对应 |phi|=1981"推算应读到 1981 -> 偏了 27 LSB(2.4°)。
    //   27 LSB 的静态倾角偏置经 KP=20 就是 540 cmd 的恒定力矩需求,而恒定力矩
    //   等于恒定角加速度,臂撑不住就会单向转 —— 这是"臂 1 圈/s 飞转"的源头之一。
    //   精确做法:烧录后把摆杆扶正倒立 5s,用 fpga_serial.ps1 -ZeroCal 读出 phi,
    //   再按 ZERO_RAW = 3001 + phi 微调。
    angle_proc #(
        .ZERO_RAW (12'd2860)      // [2026/10/27 重新标定] 实测: 平衡时 phi 均值 -68.4LSB(-6.01deg), 下垂读数 1983(理想2048)
                                  //   两路反算 ZERO_RAW = 2862 / 2859 -> 取 2860。
                                  //   注意: 零点会随温度/时间漂移, 每次上电后应复核"下垂时 phi 应约 ±2048"
    ) angle_proc_inst(
        .clk_50m   (clk_50m  ),
        .rst_n     (rst_n    ),
        .theta_raw (theta_out),
        .tick      (theta_vld),
        .phi       (phi      ),
        .omega     (omega    ),
        .cos_phi   (cos_phi  )
    );

    //------------------------------------------------------------------
    // 编码器:enc1 = 旋转臂(接 MG531),enc2 备用
    //------------------------------------------------------------------
    wire [15:0] enc_pos1;
    wire [15:0] enc_pos2;

    encoder encoder_1(
        .clk_50m (clk_50m ),
        .rst_n   (rst_n   ),
        .enc_a   (enc1_a  ),
        .enc_b   (enc1_b  ),
        .pos     (enc_pos1)
    );

    encoder encoder_2(
        .clk_50m (clk_50m ),
        .rst_n   (rst_n   ),
        .enc_a   (enc2_a  ),
        .enc_b   (enc2_b  ),
        .pos     (enc_pos2)
    );

    //------------------------------------------------------------------
    // 串口在线参数配置(上位机 -> FPGA)
    //   帧格式 AA 55 | idx | val_H | val_L | chk,详见 param_cfg.v
    //   有了这一路之后,整定参数不用再重新综合布线+烧录,发一帧即生效。
    //------------------------------------------------------------------
    wire signed [15:0] p_kp_theta   ;
    wire signed [15:0] p_kd_theta   ;
    wire signed [15:0] p_kp_pos     ;
    wire signed [15:0] p_kd_pos     ;
    wire signed [15:0] p_ki_theta   ;
    wire signed [15:0] p_ki_pos     ;
    wire signed [15:0] p_u_max_bal  ;
    wire signed [15:0] p_u_min_swing;
    wire signed [15:0] p_u_max_swing;
    wire signed [15:0] p_phi_catch  ;
    wire signed [15:0] p_omega_catch;
    wire signed [15:0] p_phi_top    ;
    wire signed [15:0] p_phi_fall   ;
    wire               p_start_pulse;
    wire               p_wr_tog     ;

    param_cfg param_cfg_inst(
        .clk_50m     (clk_50m     ),
        .rst_n       (rst_n       ),
        .rx_in       (uart_rx     ),
        .kp_theta    (p_kp_theta   ),
        .kd_theta    (p_kd_theta   ),
        .kp_pos      (p_kp_pos     ),
        .kd_pos      (p_kd_pos     ),
        .ki_theta    (p_ki_theta   ),
        .ki_pos      (p_ki_pos     ),
        .u_max_bal   (p_u_max_bal  ),
        .u_min_swing (p_u_min_swing),
        .u_max_swing (p_u_max_swing),
        .phi_catch   (p_phi_catch  ),
        .omega_catch (p_omega_catch),
        .phi_top     (p_phi_top    ),
        .phi_fall    (p_phi_fall   ),
        .arm_off     (p_arm_off    ),
        .traj_sw     (p_traj_sw    ),
        .start_pulse (p_start_pulse),
        .wr_tog      (p_wr_tog     )
    );

    //------------------------------------------------------------------
    // 起摆 / 平衡 状态机
    //   起停命令有两个来源:板上 SW1,以及串口(等价按一下 SW1)
    //------------------------------------------------------------------
    wire signed [15:0] motor_cmd;
    wire        [1:0]  ctrl_state;
wire signed [15:0] ctrl_dbg;      // [DEBUG] 诊断量(当前=arm_err)

    controller controller_inst(
        .clk_50m     (clk_50m     ),
        .rst_n       (rst_n       ),
        .tick        (theta_vld   ),
        .start       (swing_key[0] | p_start_pulse),
        .phi         (phi         ),
        .omega       (omega       ),
        .cos_phi     (cos_phi     ),
        .arm_pos     (enc_pos1    ),
        .phi_catch   (p_phi_catch  ),
        .omega_catch (p_omega_catch),
        .phi_top     (p_phi_top    ),
        .phi_fall    (p_phi_fall   ),
        .arm_off     (arm_off_eff  ),   // E1: 按键档位 + 串口微调
        .kp_theta    (p_kp_theta   ),
        .kd_theta    (p_kd_theta   ),
        .kp_pos      (p_kp_pos     ),
        .kd_pos      (p_kd_pos     ),
        .ki_theta    (p_ki_theta   ),
        .ki_pos      (p_ki_pos     ),
        .u_max_bal   (p_u_max_bal  ),
        .u_min_swing (p_u_min_swing),
        .u_max_swing (p_u_max_swing),
        .motor_cmd   (motor_cmd    ),
        .ctrl_dbg    (ctrl_dbg     ),
        .state       (ctrl_state  )
    );

    //------------------------------------------------------------------
    // 电机驱动:A、B 两路并联驱动同一台 MG531(与参考工程一致)
    //------------------------------------------------------------------
    motor_ctrl motor_ctrl_A(
        .clk_50m (clk_50m   ),
        .rst_n   (rst_n     ),
        .cmd     (motor_cmd ),
        .ain1    (AN1       ),
        .ain2    (AN2       ),
        .pwm     (PWMA      )
    );

    motor_ctrl motor_ctrl_B(
        .clk_50m (clk_50m   ),
        .rst_n   (rst_n     ),
        .cmd     (motor_cmd ),
        .ain1    (BN1       ),
        .ain2    (BN2       ),
        .pwm     (PWMB      )
    );

    //------------------------------------------------------------------
    // 串口上报(帧格式与原工程完全一致)
    //------------------------------------------------------------------
    wire [15:0] data_out;

    uart #(
        .CNT_1S (32'd149_999)         // [2026/10/17] 3ms 一帧(≈333Hz),与 3ms 控制节拍同步。
                                      //   原 999_999(20ms) + 9600bps:一帧 10 字节要 10.4ms,
                                      //   20ms 采样看不清 3ms 的动力学。配套 uart.v 已改 115200。
    ) uart_inst(
        .clk_50M    (clk_50m               ),
        .rst_n      (rst_n                 ),
        .key3       (swing_key[2]          ),
        .fpga_rx    (uart_rx               ),
        .fpga_tx    (uart_tx               ),
        .data_in    (phi                   ),
        // [DEBUG] 备用字段 = “旋转臂位置 16bit + 控制器状态 2bit + 参数写入标志 1bit + 电机指令低 13bit”,
        //         用于观测编码器计数、是否进入平衡态(S_BALANCE=2)、串口参数帧是否被接收
        //         (p_wr_tog 每收到一帧合法参数帧翻转一次)以及电机输出。
        //         正式版本请改回 {enc_pos1, enc_pos2}
        .encoder_in ({enc_pos1, ctrl_state, p_wr_tog, motor_cmd[12:0]}),
        .data_out   (data_out              )
    );

endmodule
