//=============================================================================
// 模块: controller   自动起摆 / 倒立平衡 状态机
//-----------------------------------------------------------------------------
//   IDLE    : 电机停转,等待 SW1
//   SWING   : 能量法起摆。摆杆摆到平衡位(|phi|<PHI_TOP)即停止起摆,
//             或在倒立捕获区(|phi| 小且 |omega| 小)内 -> BALANCE
//   BALANCE : 倾角环 + 位置环 PD 平衡。一旦 |phi| 超过失稳阈值 -> 回 SWING 重来
//
//   任何状态下再按一次 SW1 -> 回到 IDLE(电机停转)
//   SW5(rst_n) -> 全局复位
//-----------------------------------------------------------------------------
//   phi / omega / cos_phi 由 angle_proc 每 3ms 更新一次,arm_pos 来自编码器
//   两个子控制器的输出都是“有符号驱动力矩指令”,由状态机二选一送给电机
//=============================================================================
module controller (
    input                      clk_50m   ,
    input                      rst_n     ,
    input                      tick      ,   // 3ms 采样脉冲
    input                      start     ,   // SW1 按键脉冲 / 串口起停命令
    input       signed [15:0]  phi       ,
    input       signed [15:0]  omega     ,
    input       signed [15:0]  cos_phi   ,
    input              [15:0]  arm_pos   ,

    // ★ 以下阈值与子模块增益全部改成"在线可调":由 param_cfg.v 经串口写入,
    //   上电默认值写在 param_cfg.v 的 D_xxx 常量里(与之前的编译期数值一致)。
    input       signed [15:0]  phi_catch   ,   // 倒立捕获角阈值(ADC LSB)
    input       signed [15:0]  omega_catch ,   // 倒立捕获角速度阈值(LSB/3ms)
    input       signed [15:0]  phi_top     ,   // 顶部穿越判定:SWING 期间 |phi| 进此区即视为已到平衡位,立刻停止起摆
    input       signed [15:0]  phi_fall    ,   // 失稳判定角阈值(ADC LSB)
    input       signed [15:0] kp_theta    ,   // 倾角环 P
    input       signed [15:0] kd_theta    ,   // 倾角环 D
    input       signed [15:0] kp_pos      ,   // 位置环 P
    input       signed [15:0] kd_pos      ,   // 位置环 D
    input       signed [15:0] ki_theta    ,   // 倾角环 I
    input       signed [15:0] ki_pos      ,   // 位置环 I
    input       signed [15:0] u_max_bal   ,   // 平衡输出限幅
    input       signed [15:0] arm_off     ,   // E1: 臂目标偏移(counts)
    input       signed [15:0]  u_min_swing ,   // 起摆输出下限
    input       signed [15:0]  u_max_swing ,   // 起摆输出上限

    output reg  signed [15:0]  motor_cmd ,
    output reg          [1:0]  state,
    output     signed [15:0]  ctrl_dbg     // [DEBUG] 诊断量(当前=arm_err)
);

    localparam S_IDLE    = 2'd0;
    localparam S_SWING   = 2'd1;
    localparam S_BALANCE = 2'd2;

    //------------------------------------------------------------------
    // 子控制器
    //------------------------------------------------------------------
    // ★★★ [2026/10/17 修复] 下面两个信号必须在例化之前声明 ★★★
    //   原代码把 `wire bal_en = ...` 写在文件末尾(例化之后),Gowin 先按
    //   "隐式 1bit 网线" 处理第 75 行的用法,再对后面的显式声明报
    //   EX3638 'bal_en is already implicitly declared on line 75',
    //   最终综合结果是:
    //       WARN (EX1998): Net 'bal_en' does not have a driver
    //   即 en 恒为 0(网表里 balance 实例的 en 端口已被常量折叠删掉)。
    //   后果(实测确认):
    //     - balance.v 的 if(!en) 每个时钟都执行 -> arm_tgt 永远跟随 arm_pos
    //       -> arm_err ≡ 0 -> 位置环 P 项恒 0
    //     - i_phi / i_arm 每拍被清零 -> 两个积分项恒 0
    //   也就是"位置环 + 积分器"从未真正工作过。
    //   (位置环 D 项走 arm_dly 差分,不受 en 影响,一直是好的。)
    reg  [8:0] alpha_r;
    wire       bal_en = (state != S_IDLE) && (alpha_r != 9'd0);

    wire signed [15:0] u_swing;
    wire signed [15:0] u_brake;
    wire signed [15:0] u_bal;

    energy_swing energy_swing_inst(
        .clk_50m (clk_50m     ),
        .rst_n   (rst_n       ),
        .tick    (tick        ),
        .phi     (phi         ),
        .omega   (omega       ),
        .cos_phi (cos_phi     ),
        .u_min   (u_min_swing ),
        .u_max   (u_max_swing ),
        .u_out   (u_swing     ),
        .u_brake (u_brake     )
    );

    //==================================================================
    // 摆角速度的 8 拍(24ms)平滑估计  [2026/10/17]
    //------------------------------------------------------------------
    // 为什么需要:
    //   编码器 2048 线 + 3ms 采样 => 速度分辨率 = 1 LSB/3ms = 29 deg/s。
    //   而"摆杆立着时 ±7deg / 0.3Hz 的慢振荡"峰值角速度只有 13 deg/s(0.45 LSB),
    //   单拍差分完全看不见它 => D 项对慢振荡无效 => 只剩 P 项 => 等幅振荡。
    //   实测佐证: kp_pos / ki_pos / kd_pos 怎么调, 摆臂峰峰值都卡在 2.0~2.2 圈。
    // 做法:
    //   在 3ms tick 上移位, 8 拍差分 omega_f = phi - phi_dly[7],
    //   速度分辨率降到 3.6 deg/s, 慢振荡就能被 D 项看见。
    //   24ms 滞后对 3.4s 周期的慢振荡只有 0.7% 相位差, 可忽略。
    // 注意:
    //   omega_f ~= 8 * omega, 所以 KD_THETA 要除以 8 才保持原物理增益
    //   (原 kd_theta=12.0 -> 等效 1.5)。omega 本身不改, 捕获判定/能量法仍用单拍值。
    //==================================================================
    reg  signed [15:0] phi_dly [0:7];
    integer pd;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            for (pd = 0; pd < 8; pd = pd + 1) phi_dly[pd] <= 16'sd0;
        end else if (tick) begin
            phi_dly[0] <= phi;
            for (pd = 1; pd < 8; pd = pd + 1) phi_dly[pd] <= phi_dly[pd-1];
        end
    end
    //------------------------------------------------------------------
    // α-β 递推滤波器(两状态估计器)  [2026/10/18]
    //   目的: 给出一个"能看见慢摆"的摆角速度, 喂给倾角环的 D 项。
    //   为什么必须换掉单拍差分:
    //     单拍差分分辨率 = 29 deg/s; 而摆杆残余慢摆(±3deg @0.3Hz)的
    //     峰值角速度只有 13 deg/s -> 信号埋在量化噪声里, D 项实际是瞎的。
    //     实测证据: 0.3Hz 上 A_theta/A_phi = 22~26, 与理论下限 25~34 吻合,
    //     说明摆臂甩动完全是"维持那个倾角所必需", 不是摆臂自己乱晃;
    //     所以唯一的出路是把摆杆的慢摆压下去, 而这需要看得见的角速度。
    //   为什么不用 8 拍滑动平均:
    //     滞后 12ms, 对环路里 42Hz 的快抖动等于 180 度相位反转 -> 会失稳
    //     (已实测: 改用 8 拍平均后最长稳顶从 38s 掉到 0.6s)。
    //   α-β 的好处: 带模型的递推估计, 对 0.3Hz 几乎无衰减/滞后,
    //     对逐拍量化噪声和被丢弃的毛刺帧平滑得很好; 带宽由下面两个系数决定。
    //   系数是 Q8 (alpha = AB_ALPHA/256); 稳定性: 0<alpha<2, 0<beta<=4-2alpha
    //------------------------------------------------------------------
    localparam signed [15:0] AB_ALPHA = 16'sd64;   // 0.25  -> 位置估计带宽约 13Hz
    localparam signed [15:0] AB_BETA  = 16'sd20;   // 0.078 -> 约 2Hz (实测 8/20 无差别, 取回 20)
    reg  signed [15:0] phi_hat;
    reg  signed [15:0] w_hat;
    //   ★ 标准 α-β 递推必须先"用速度外推位置", 再用测量残差修正。
    //     我第一版漏了外推这一步, 结果 w_hat 退化成"误差的积分",
    //     对正弦输入相位差 90 度 -> 不是阻尼而是刚度项 -> 加进去必然失稳
    //     (实测: 增益 4.0 和 12.0 都会让 2/3 的运行崩掉, 正是这个原因)。
    wire signed [15:0] phi_pred = phi_hat + w_hat;        // ① 按速度外推
    // 角度是 12bit 环形量(-2048..2047)。跨过 ±2048 时若直接相减，
    // 残差会突然跳变约 4096，速度估计器会被误打出一个很大的尖峰。
    wire signed [16:0] ab_err_raw = $signed({phi[15], phi})
                                          - $signed({phi_pred[15], phi_pred});
    wire signed [16:0] ab_err_wrap = (ab_err_raw > 17'sd2047)  ? (ab_err_raw - 17'sd4096) :
                                     (ab_err_raw < -17'sd2048) ? (ab_err_raw + 17'sd4096) :
                                                                  ab_err_raw;
    wire signed [15:0] ab_err = ab_err_wrap[15:0];        // ② 环形测量残差
    wire signed [31:0] ab_p     = $signed(ab_err) * AB_ALPHA;
    wire signed [31:0] ab_w     = $signed(ab_err) * AB_BETA;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            phi_hat <= 16'sd0;
            w_hat   <= 16'sd0;
        end else if (tick) begin
            phi_hat <= phi_pred + (ab_p >>> 8);           // ③ 修正位置
            w_hat   <= w_hat    + (ab_w >>> 8);           // ④ 修正速度
        end
    end
    wire signed [15:0] omega_f = w_hat;
    balance balance_inst(
        .clk_50m  (clk_50m   ),
        .rst_n    (rst_n     ),
        .tick     (tick      ),
        // ★ en 不再只看状态机,而是"平衡环真正参与工作"= 非 IDLE 且混合权重
        //   不为 0。这样加权混合期间平衡环就已经生效(arm_tgt 就地锁存、积分
        //   器开始积累),而不是等到状态机跳到 BALANCE 才启动;反过来,短暂的
        //   SWING/BALANCE 往返也不会把积分器反复清零。
        .en       (bal_en    ),
        // ★ clr: 积分器只在 IDLE 时清零(SWING<->BALANCE 往返保持) [2026/10/17]
        .clr      (state == S_IDLE),
        .alpha    (alpha_r),          // 臂阻尼按混合权重分时生效
        .phi      (phi       ),
        .omega    (omega     ),
        .omega_f  (omega_f   ),   // 8 拍(24ms)平滑速度, 只给倾角环 D 项用
        .arm_pos  (arm_pos   ),
        .kp_theta (kp_theta  ),
        .kd_theta (kd_theta  ),
        .kp_pos   (kp_pos    ),
        .kd_pos   (kd_pos    ),
        .ki_theta (ki_theta  ),
        .ki_pos   (ki_pos    ),
        .u_max    (u_max_bal ),
        .u_out    (u_bal     )
        ,.arm_off (arm_off   )
        ,.dbg_out (ctrl_dbg  )
    );

    //------------------------------------------------------------------
    // 阈值判断(取绝对值比较)
    //------------------------------------------------------------------
    wire signed [15:0] phi_abs   = phi[15]   ? -phi   : phi;
    wire signed [15:0] omega_abs = omega[15] ? -omega : omega;

    wire in_catch = (phi_abs < phi_catch) && (omega_abs < omega_catch);
    wire is_fall  = (phi_abs > phi_fall);
    reg  [2:0] catch_cnt;

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            state <= S_IDLE;
            catch_cnt <= 3'd0;
        end else if(start) begin
            catch_cnt <= 3'd0;
            if(state == S_IDLE)
                state <= S_SWING;       // 按下 SW1:开始自动起摆
            else
                state <= S_IDLE;        // 再按一次:停止
        end else if(tick) begin
            case(state)
                S_IDLE: begin
                    state <= S_IDLE;
                    catch_cnt <= 3'd0;
                end
                S_SWING: begin
                    // 必须连续 5 拍(15ms)同时满足角度和速度条件，拒绝高速过顶。
                    if(in_catch) begin
                        if(catch_cnt == 3'd4) begin
                            state <= S_BALANCE;
                            catch_cnt <= 3'd0;
                        end else begin
                            catch_cnt <= catch_cnt + 3'd1;
                        end
                    end else begin
                        catch_cnt <= 3'd0;
                    end
                end
                S_BALANCE: begin
                    catch_cnt <= 3'd0;
                    if(is_fall)
                        state <= S_SWING;
                end
                default: begin
                    state <= S_IDLE;
                    catch_cnt <= 3'd0;
                end
            endcase
        end
    end

    //------------------------------------------------------------------
    // 起摆 <-> 平衡 加权混合  [2026/9/17 新增]
    //------------------------------------------------------------------
    //   原来 SWING / BALANCE 是"二选一"硬切换:切换那一拍指令从一个控制器
    //   的值直接跳成另一个控制器的值,对电机就是一个阶跃冲击。实测 BALANCE
    //   段里 phi 常年 30~40°、cmd 在 ±2000 之间整幅翻转,这个冲击有份。
    //   改成按权重 alpha 连续混合:
    //       u = ( alpha*u_bal + (256-alpha)*u_swing ) / 256 , alpha 为 Q8
    //   alpha 取两个"离倒立点有多近"的度量中较小的那个:
    //       alpha_phi = clamp( (PHI_FALL    - |phi|  ) << 2 , 0 , 256 )
    //       alpha_om  = clamp( (OMEGA_CATCH - |omega|) << 2 , 0 , 256 )
    //   |phi| <= PHI_FALL-64 时 alpha_phi 饱和到 256(纯平衡环),
    //   |phi| >= PHI_FALL    时线性降到 0  (纯起摆环),中间 64 LSB 过渡。
    //   全部是移位 + 比较 + 一次 16x10 乘法,没有除法,不拖时序。
    //   ★ alpha 只用于"输出混合",状态机判据(in_catch / PHI_FALL)一字未改,
    //     所以串口上报的 state 含义与以前完全一致,BALANCE 段依旧表示
    //     |phi| < PHI_TOP 且 |omega| < OMEGA_CATCH。
    //
    //   ★ 时序:alpha 从 phi/omega 算出来要串"取绝对值 -> 相减 -> 比较 -> 取小"
    //     四级逻辑,若再直接接上"乘 alpha -> 加 u_swing",controller 会变成
    //     全工程关键路径(实测 Fmax 由 55.7MHz 掉到 51.9MHz,逻辑级 21)。
    //     而 tick 是 3ms 一次(15 万个时钟),打拍对功能毫无影响,所以把 alpha
    //     单独寄存一拍,乘法从寄存器出发,关键路径一分为二。
    //     u_swing / u_bal 本来就是在 tick 上寄存的输出,alpha_r 与它们同拍,
    //     不存在错位。
    //------------------------------------------------------------------
    wire signed [16:0] ap_d = $signed({1'b0, phi_fall})    - $signed({1'b0, phi_abs});
    wire signed [16:0] om_soft_hi = $signed({omega_catch[15], omega_catch}) + 17'sd16;
    wire signed [16:0] om_margin  = om_soft_hi - $signed({1'b0, omega_abs});

    wire [8:0] alpha_phi = (ap_d <= 0)  ? 9'd0   :
                           (ap_d >= 64) ? 9'd256 : {ap_d[5:0], 2'b00};
    // 捕获速度阈值已经收紧到 10；混合区仍保留 16 LSB 的软过渡，避免
    // 阈值外完全关掉平衡环而使重捕获变慢。
    wire [8:0] alpha_om  = (omega_abs <= omega_catch) ? 9'd256 :
                           (om_margin <= 0)            ? 9'd0   :
                                                        {om_margin[4:0], 4'b0000};

    //------------------------------------------------------------------
    // ★ alpha 上限不再取满 256  [2026/10/17]
    //------------------------------------------------------------------
    //   3ms 全采样实测(cap4.csv, 3343 帧):
    //     顶部附近(|phi|<20deg) E>0 占 76%,|cmd| 均值 1686、饱和 57%;
    //     11 次离开顶部【每一次】离开前 E 都在 +2324 ~ +14314(能量过剩)。
    //   即:不是能量不够掉下来,而是能量排不掉。
    //   alpha=256 时输出 100% 是 u_bal(纯 PD),而 PD 没有抽能能力;唯一会抽能的
    //   u_swing 恰在此区域被混合权重完全掐断 -> 每次巡游攒的多余能量都留在系统里,
    //   越攒越多直到把摆杆甩出顶部。
    //   留 22% 的能量控制权限(alpha 封顶 200)= E>0 时始终有一份刹车,
    //   E<0 时也仍以 22% 参与泵能,不会破坏起摆。
    //------------------------------------------------------------------
    localparam [8:0] ALPHA_MAX = 9'd256;   // 256=完全交给平衡环(实测最好);200 试过,3 次试验平均变差,已回退
    wire [8:0] alpha_raw = (alpha_phi < alpha_om) ? alpha_phi : alpha_om;
    wire [8:0] alpha     = (alpha_raw > ALPHA_MAX) ? ALPHA_MAX : alpha_raw;

    // alpha_r / bal_en 的声明已上移到例化之前(见上面修复注释)
    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)      alpha_r <= 9'd0;
        else if(tick)   alpha_r <= alpha;
    end

    // u = u_swing + alpha*(u_bal - u_swing)/256 -> 整条混合链只要一个乘法器
    wire signed [15:0] u_dif   = u_bal - u_swing;
    wire signed [25:0] u_step  = u_dif * $signed({1'b0, alpha_r});
    wire signed [15:0] u_blend = u_swing + $signed(u_step[23:8]);

    //------------------------------------------------------------------
    // 输出选择:IDLE 强制 0,其余用混合结果
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // ★ 叠加"只刹车"项 [2026/10/17]
    //   u_brake 只在 E > E_REF 时非零,且不受 alpha 门控 —— 这正好补上
    //   "顶部附近 100% 交给 PD、而 PD 不会抽能"这个缺口。
    //   安全限幅 U_HARD=3000(占空比约 69%):u_max_bal=2000 时 PD 常饱和,
    //   若把总和也限在 2000,刹车在饱和段就完全无效。
    //------------------------------------------------------------------
    // ★ 刹车通道开关 [2026/10/17] 1=启用,0=关闭(默认)
    //   实测(5 次 x 12s, 与基线同一固件其余部分):
    //     基线(无刹车)       |phi|<10deg 25/16/39%  最长稳顶 1.04/1.10/1.27s
    //     刹车(alpha 补偿)   |phi|<10deg 0/28/5/17/38% 最长稳顶 0.01/1.68/0.75/1.14/1.69s
    //   刹车确实每帧都在算(|cmd| 峰值 3000, >2000 占 1%),但只在顶部一小段能补上,
    //   统计上与基线无差别。且它允许 3000 cmd(约 69% 占空比)的瞬态,
    //   没有可证明收益就不该承担电流风险 -> 默认关闭。
    //   真正的问题在别处: 泵能下限 u_min=700 远大于摩擦(~660),E 必然冲过 0,
    //   而"冲过 0 之后要抽掉"这一步发生在顶部、只持续几十毫秒,来不及。
    localparam BRAKE_EN = 1'b0;   // 位置环修好后复核: 4 次看似 +40%, 8 次后回落到与基线相同 -> 仍取 0
    localparam signed [16:0] U_HARD = 17'sd3000;

    // 若以后重新启用刹车，它应只补上被 alpha 混合削弱的那部分；当前开关为 0，
    // 必须严格输出零，不能像旧代码那样虽然声明“关闭”却仍持续叠加。
    wire signed [25:0] brk_mul     = $signed(u_brake) * $signed({1'b0, alpha_r});
    wire signed [15:0] u_brake_g   = BRAKE_EN ? (brk_mul >>> 8) : 16'sd0;
    wire signed [16:0] u_with_brake = $signed(u_blend) + $signed(u_brake_g);
    wire signed [15:0] u_final = (u_with_brake > U_HARD)  ? 16'sd3000 :
                                 (u_with_brake < -U_HARD) ? -16'sd3000 :
                                 u_with_brake[15:0];


    //------------------------------------------------------------------
    // 输出选择: IDLE 强制 0, 其余用混合结果
    //------------------------------------------------------------------
    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)
            motor_cmd <= 16'sd0;
        else if(state == S_IDLE)
            motor_cmd <= 16'sd0;
        else
            motor_cmd <= u_final;
    end

endmodule
