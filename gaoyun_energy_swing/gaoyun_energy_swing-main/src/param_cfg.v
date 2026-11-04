//=============================================================================
// 模块: param_cfg   串口在线参数配置(上位机 -> FPGA)
//-----------------------------------------------------------------------------
// 目的:把原来"改参数 -> 重新综合布线 -> 烧录 -> 按键"的循环,变成"发一帧串口,
//       参数下一个 3ms 采样拍立刻生效",便于在线整定。
//
// 帧格式(上位机 -> FPGA,6 字节):
//       AA 55 | idx | val_H | val_L | chk
//       chk = (idx + val_H + val_L) & 0xFF
//       val 为有符号 16 位补码,大端
//   波特率 9600-8-N-1,与上报帧一致。
//
// 参数表(增益类以 0.1 为单位下发:下发 205 表示 20.5;本模块内部把它换算成
//         1/16 为单位(Q4)再送给 balance.v,见下面第 3 节)
//       0x00 START        起停切换(等价按一下 SW1,val 任意)
//       0x01 KP_THETA     倾角环 P         (0.1)
//       0x02 KD_THETA     倾角环 D         (0.1)
//       0x03 KP_POS       位置环 P         (0.1)
//       0x04 KD_POS       位置环 D         (0.1)
//       0x05 U_MAX_BAL    平衡输出限幅      (cmd,不缩放)
//       0x06 U_MIN_SWING  起摆输出下限
//       0x07 U_MAX_SWING  起摆输出上限
//       0x08 PHI_CATCH    倒立捕获角阈值(ADC LSB)
//       0x09 OMEGA_CATCH  倒立捕获角速度阈值(LSB/3ms)
//       0x0A PHI_TOP      顶部穿越判定(ADC LSB)
//       0x0B PHI_FALL     失稳判定(ADC LSB)
//       0x0C KI_THETA     倾角环 I         (0.1)  ← 消恒定偏置力矩,见 balance.v
//       0x0D KI_POS       位置环 I         (0.1)
//   注:0x0C/0x0D 置 0 即退回纯 PD,行为与加积分之前完全一致。
//
// 回读:每成功写入一帧,wr_tog 翻转一次。该位挂在遥测帧的 bit13
//       (encoder_in = {臂位置16, 状态2, wr_tog1, 电机指令13}),上位机看到它
//       翻转就说明 FPGA 收到并接受了一帧合法参数;没翻转 = 帧被丢弃
//       (校验错/字节错位/波特率不对)。
//
// 默认值:上电/复位后等于下面的 D_xxx,与"编译期固化"的行为完全一致。
//=============================================================================
module param_cfg(
    input               clk_50m     ,
    input               rst_n       ,
    input               rx_in       ,   // FPGA 的 uart_rx

    // ---- 在线可调参数 ----
    output reg signed [15:0] kp_theta    ,
    output reg signed [15:0] kd_theta    ,
    output reg signed [15:0] kp_pos      ,
    output reg signed [15:0] kd_pos      ,
    output reg signed [15:0] ki_theta    ,
    output reg signed [15:0] ki_pos      ,
    output reg signed [15:0] u_max_bal   ,
    output reg signed [15:0] u_min_swing ,
    output reg signed [15:0] u_max_swing ,
    output reg signed [15:0] phi_catch   ,
    output reg signed [15:0] omega_catch ,
    output reg signed [15:0] phi_top     ,
    output reg signed [15:0] phi_fall    ,
    output reg signed [15:0] arm_off     ,   // E1: 臂目标偏移(counts, 相对平衡建立时的位置)
    output reg signed [15:0] traj_sw     ,   // E2: 轨迹模式远程开关(非0=启用, 与SW3或)

    output reg               start_pulse ,   // 宽度 1 个 50MHz 周期
    output reg               wr_tog          // 每成功写入一帧翻转一次
);

//-----------------------------------------------------------------------------
// 上电默认值 = 2026/9/17 第四轮在线整定的结果(串口逐组实验扫出来的)
//-----------------------------------------------------------------------------
// 整定过程(每组的 BALANCE 时间占比 / 单段最长时长):
//   5/140,  -2/-44, u_max 2000  46% / 0.3s    ← 上一版默认
//   15/240, 位置环断开          32% / 1.1s    ← 加大增益立刻变好
//   15/240, -2/-44              62% / 1.3s
//   20/320, +2/+44            63~78% / 4.3~5.6s  ★ 取这一组
//   30/480, +2/+44              54% / 2.8s    ← 再加反而变差
//   20/320, u_max 3000          72% / 3.3s    ← 占空比加大没用,故仍取 2000
//   5/400  (只加 D)             18% / 0.3s    ← 只加 D 不行
//   20/320, -2/-20 或 -3/-30    50~58%       ← 负位置环更差,且拦不住摆臂漂移
// 结论:
//   1) 倾角环增益必须比"换算值"大一截(KP 5 -> 20)。换算值按力矩模型推的,
//      实测偏乐观,偏小的话闭环极点落在虚轴右侧,只能勉强撑住几百毫秒。
//   2) 位置环符号取【正】。上一版按 corr(cmd,dEnc) 的正号推断"要取负",
//      那个相关性其实是倾角环自己造成的,推不出被控对象符号。
//   3) 摆臂缓慢单向漂移是"摆杆倒立时的反作用力"驱动的,和位置环符号无关:
//      位置环取 -2/-20 时漂移反而更快(-940 计数/s)。要压住它得另想办法
//      (或者干脆接受:比赛只看摆杆能不能立住)。
// 注:下面 4 个增益的存储值是"真实值 x 16"(Q4,见 balance.v 的控制律),所以
//     写成 "整数 * 16" 保持可读。串口下发仍按 0.1 为单位(20.5 -> 205),
//     由本模块第 3 节的换算逻辑转成 Q4 再存。
localparam signed [15:0] D_KP_THETA    = 16'sd20  * 16;   // 20.0
// ★ [2026/10/17] KD_THETA 32.0 -> 12.0
//   物理依据: 平衡环输出上限 u_max_bal=2000,而 D 项 = KD*omega,
//     故环能"抓住"的最大角速度 = 2000/KD:  KD=32 -> 6.25 LSB/3ms;
//     实测摆杆到顶部时角速度约 10 LSB/3ms -> 一到顶就饱和,没有减速余量。
//   实测(每次 12s,多次重复,看"最长连续稳顶"这个比赛真正关心的指标):
//     KD=32 (8 次) 平均 0.75s  <- 出厂默认,最差
//     KD=24 (3 次) 平均 1.19s
//     KD=16 (6 次) 平均 1.30s
//     KD=12 (3 次) 平均 1.23s
//     KD=8  (3 次) 平均 1.38s
//   低 KD 组稳定时间接近翻倍,取居中且表现稳定的 12.0。
// ★ [2026/10/17] kd_theta = 12.0
//   历史: 出厂 32.0 -> 首轮扫描(当时 kp_pos=0)得 12.0 -> 位置环恢复后复核 12/24 两批
//   结果互相矛盾(2.98s vs 1.29s), 已落到测量噪声底之下, 取已验证最好的 12.0。
//   若后续要精调, 必须先解决"批次间同配置差 4 倍"的重复性问题
//   (可能来自起始相位/摆臂初始位置), 否则单批对比没有意义。
// [2026/10/17] D 项改用 24ms(8 拍)平滑速度 omega_f, 其数值 ~= 8*单拍差值,
//   故为保持原物理增益需除以 8: 12.0 -> 1.5。默认取 3.0 (= 原来 24.0 的阻尼强度),
//   但现在是"干净"的速度估计, 可以放心加大而不放大量化噪声。
localparam signed [15:0] D_KD_THETA    = 16'sd90  * 16;   // 90.0 [2026/10/21 重扫: 120->90, |phi|1.75->1.44, A_theta 25->13]
                                 //   注: 起作用的是 controller.v 内的 alpha-beta 滤波器; pend_obs.v 未启用
// ★ [2026/10/17] 位置环 P 从 0 改回并放大到 16.0
//   这曾经是本轮最大的误判: 早期用"每组只跑 1 次 12s"的 A/B/C 测出
//   kp_pos=0 最好(45%/41%/25%),据此把位置环关掉了。但后来证明同样的配置
//   重复跑能差 4 倍,那个测试无效。
//   重新按每组 4 次测(KD=12):
//     kp_pos=0   平均最长稳顶 1.29s, 4 次里 2 次完全起不来
//     kp_pos=+2  3.36s (最好 6.13s)  <- 原作者的值,是对的
//     kp_pos=-2  0.90s               <- 符号反了明显更差
//     kp_pos=+8  3.01s
//     kp_pos=+16 3.68s (最好 6.55s), 臂净转角 0.04 圈/s(基本不漂)  <- 取这个
//     kp_pos=+32 0.44s (过冲,崩溃)
//   物理原因: 倾角环只能决定摆臂的"角加速度"; 摆杆带固定倾角 phi0 时,
//   维持平衡需要恒定的 theta_ddot = (g/r)tan(phi0)。没有位置环这个恒定加速度
//   会让摆臂一路转下去(实测臂 1 圈/s 飞转)。位置环 P 正是把摆臂角度拉回来的项,
//   "管住摆臂"才有平衡的立足点。
// [2026/10/19 修正] 曾把 kp_pos 从 16 提到 32, 依据是单次试验(position 误差 5~9% -> 2.3~2.5%)。
//   但用统一协议连测 16/24/32 三组后, 结论相反:
//     kp_pos=16: 定位误差 +9/-4 counts (3%/1%), |phi|均 2.09 deg  <- 最好
//     kp_pos=24: 定位误差 +11/-3        (4%/1%), |phi|均 2.81 deg
//     kp_pos=32: 定位误差 +20/-2        (7%/1%), |phi|均 3.35 deg
//   即 kp_pos 越大, 摆杆越晃, 定位精度也不升反降。故保持 16.0。
localparam signed [15:0] D_KP_POS      = 16'sd24  * 16;   // 24.0 [2026/10/23 由16提到24: 有位置偏移时摆臂峰峰 29.7->18.0, |phi| 4.79->1.08]   // 16.0  位置环 P
//  ↑ [2026/10/17] 由 +2.0 改为 0:上板实测(12s x 3 组)
//      kp_pos=0   -> 平衡占比 45%
//      kp_pos=-2  -> 41%
//      kp_pos=+2  -> 25%
//    两种符号都比关掉更差,说明当前位置环实现是负贡献(而且此前 bal_en 无驱动时
//    arm_err 恒 0,这条环路其实从未工作过,源码注释里"取正 63~78%"是死环下的假数据)。
//    先关掉,等倾角环稳了再逐个打开 kp_pos / kd_pos。
// ★ [2026/10/17] kd_pos 4.4 -> 50.0
//   必须有 kp_pos 才有意义: 位置环 P(16.0) 把摆臂角度拉回参考值, kd_pos 阻尼这个回归过程。
//   实测(每组 4 次 x 12s, kp_pos=16, kd_theta=12):
//     kd_pos=4.4  平均最长稳顶 1.15s / 2.48s
//     kd_pos=20   1.54s
//     kd_pos=50   3.71s / 1.66s (最好单次 7.79s)  <- 取这个
//     kd_pos=100  0.55s
//     kd_pos=200  0.19s  (阻尼过大, 把臂抱死, 起摆泵失效)
//   注: 早期在 kp_pos=0 的条件下测过 kd_pos=40 会锁死起摆泵 —— 那是因为没有位置环
//       做参照, 纯阻尼只是在抱死摆臂。有了 kp_pos 之后 kd_pos 才是有益的。
localparam signed [15:0] D_KD_POS      = 16'sd65  * 16;   // 65.0  臂速度阻尼 [配软启动: 捕获时 0.75x=49, 3秒后 1.0x=65]
localparam signed [15:0] D_KI_THETA    = 16'sd8   * 16;   // 8.0  倾角环 I(Ti≈2s,消恒定偏置力矩)
localparam signed [15:0] D_KI_POS      = 16'sd1   * 16;   // 1.0  位置环 I(Ti≈2s,消摆臂速度漂移)
localparam signed [15:0] D_U_MAX_BAL   = 16'sd2000;   // 平衡输出限幅(占空比 50%)
localparam signed [15:0] D_U_MIN_SWING = 16'sd700 ;   // 起摆输出下限(夹缝:500 卡在 37°,900 顶速过大平衡环拉不住)
localparam signed [15:0] D_U_MAX_SWING = 16'sd1800;   // 起摆输出上限
localparam signed [15:0] D_PHI_CATCH   = 16'sd150 ;   // 倒立捕获角阈值
localparam signed [15:0] D_OMEGA_CATCH = 16'sd10  ;   // 捕获角速度阈值(约 290deg/s)，拒绝高速过顶
localparam signed [15:0] D_ARM_OFF     = 16'sd0   ;   // E1: 臂目标偏移默认 0(不移动)
localparam signed [15:0] D_TRAJ_SW     = 16'sd0   ;   // E2: 轨迹模式默认关
localparam signed [15:0] D_PHI_TOP     = 16'sd150 ;   // 顶部穿越判定(|phi|<150=13.2° 才算到顶)
//  ↑ [2026/10/17] 200 -> 150:PHI_FALL 降到 220 后,若捕获判据仍是 200,
//    迟滞带只剩 [200,220]=20LSB(1.8°),状态机会来回抖。改成 150 后迟滞带 [150,220]=6.1°。
localparam signed [15:0] D_PHI_FALL    = 16'sd300 ;   // 26.4° —— [2026/10/17 最终] 回到原作者的值
//  ↑ [2026/10/17] 300 -> 220:按 b≈0.018、u_max_bal=2000、r=0.09 反算,
//    平衡环能回正的最大倾角约 tanφ = u*b*r/g = 2000*0.018*0.09/9.81 -> φ≈18°(205 LSB)。
//    原来判失稳在 300(26.4°),18°~26° 这一带平衡环已经饱和、什么也做不了,
//    实测每段平衡都以 |phi|≈30° 退出,就是在这一带空转。降到 220(19.3°)让状态机
//    更早承认失稳、回去重新起摆。

initial begin
    kp_theta    = D_KP_THETA   ;
    kd_theta    = D_KD_THETA   ;
    kp_pos      = D_KP_POS     ;
    kd_pos      = D_KD_POS     ;
    ki_theta    = D_KI_THETA   ;
    ki_pos      = D_KI_POS     ;
    u_max_bal   = D_U_MAX_BAL  ;
    u_min_swing = D_U_MIN_SWING;
    u_max_swing = D_U_MAX_SWING;
    phi_catch   = D_PHI_CATCH  ;
    omega_catch = D_OMEGA_CATCH;
    arm_off     = D_ARM_OFF    ;
    traj_sw     = D_TRAJ_SW    ;
    phi_top     = D_PHI_TOP    ;
    phi_fall    = D_PHI_FALL   ;
    start_pulse = 1'b0;
    wr_tog      = 1'b0;
end

//-----------------------------------------------------------------------------
// 1) UART 接收(9600-8-N-1,LSB first)
//    检测起始位下降沿 -> 等半个位对到起始位中点 -> 之后每 1 个位采一次,
//    共取 8 个数据位 -> 再等 1 个位(停止位)后输出。
//-----------------------------------------------------------------------------
localparam integer BPS  = 434;    // [2026/10/17] 50_000_000 / 115200 ≈ 434.03
localparam integer BPS2 = 217;    // 半位

reg  [12:0] bcnt;
reg  [3:0]  bnum;
reg  [7:0]  sh;
reg  [7:0]  rx_byte;
reg         rx_vld;
reg         rx_d0;
reg         rx_d1;

wire rx_fall = rx_d1 & ~rx_d0;

always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        rx_d0   <= 1'b1;
        rx_d1   <= 1'b1;
        bcnt    <= 13'd0;
        bnum    <= 4'd0;
        sh      <= 8'd0;
        rx_byte <= 8'd0;
        rx_vld  <= 1'b0;
    end else begin
        rx_d0  <= rx_in;
        rx_d1  <= rx_d0;
        rx_vld <= 1'b0;

        if(bnum == 4'd0) begin
            if(rx_fall) begin
                bcnt <= 13'd0;
                bnum <= 4'd1;
            end
        end else if(bcnt == ((bnum == 4'd1) ? (BPS2 - 1) : (BPS - 1))) begin
            bcnt <= 13'd0;
            if(bnum == 4'd1) begin
                // 起始位中点:必须还是低电平,否则是毛刺,丢弃
                bnum <= (rx_d0 == 1'b0) ? 4'd2 : 4'd0;
            end else if(bnum <= 4'd9) begin
                sh   <= {rx_d0, sh[7:1]};   // LSB first
                bnum <= bnum + 1'b1;
            end else begin
                rx_byte <= sh;
                rx_vld  <= 1'b1;
                bnum    <= 4'd0;
            end
        end else begin
            bcnt <= bcnt + 1'b1;
        end
    end
end

//-----------------------------------------------------------------------------
// 2) 帧解析:AA 55 idx vh vl chk -> 写参数
//-----------------------------------------------------------------------------
localparam [3:0] P_H0  = 4'd0;   // 等 AA
localparam [3:0] P_H1  = 4'd1;   // 等 55
localparam [3:0] P_IDX = 4'd2;
localparam [3:0] P_VH  = 4'd3;
localparam [3:0] P_VL  = 4'd4;
localparam [3:0] P_CHK = 4'd5;

reg [3:0] pos;
reg [7:0] idx_r;
reg [7:0] vh_r;
reg [7:0] vl_r;

wire [7:0]  chk_calc = idx_r + vh_r + vl_r;
wire [15:0] val_w    = {vh_r, vl_r};

//-----------------------------------------------------------------------------
// 3) 增益换算:上位机以 0.1 为单位下发,寄存器里存 1/16 为单位(Q4)
//-----------------------------------------------------------------------------
//   G = round(val * 16/10) = round(val * 8/5)
//   用 x52429 >> 15 实现 x1.6:52429/32768 = 1.6000061,相对误差 3.8e-6;
//   再 +16384(半个 LSB)做四舍五入。
//   ★ 之所以放在这里而不是 balance.v 的控制律里:参数是"写一次用很久",
//     缩放只需要在写参数这一拍算一次,这样控制律的关键路径上就只剩一次
//     >>4 移位(纯连线),不需要额外乘法器。balance.v 那边用 >>4 还原。
//   例:KP_THETA = 20.5 -> 下发 205 -> 存 328 (= 20.5 x 16)
//   有效范围:|增益| < 2047(16 位有符号 / 16),远超实际用量。
//-----------------------------------------------------------------------------
wire signed [31:0] q4_tmp = $signed(val_w) * 32'sd52429;
wire signed [15:0] val_q4 = (q4_tmp + 32'sd16384) >>> 15;

always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos         <= P_H0;
        idx_r       <= 8'd0;
        vh_r        <= 8'd0;
        vl_r        <= 8'd0;
        kp_theta    <= D_KP_THETA   ;
        kd_theta    <= D_KD_THETA   ;
        kp_pos      <= D_KP_POS     ;
        kd_pos      <= D_KD_POS     ;
        ki_theta    <= D_KI_THETA   ;
        ki_pos      <= D_KI_POS     ;
        u_max_bal   <= D_U_MAX_BAL  ;
        u_min_swing <= D_U_MIN_SWING;
        u_max_swing <= D_U_MAX_SWING;
        phi_catch   <= D_PHI_CATCH  ;
        omega_catch <= D_OMEGA_CATCH;
        arm_off     <= D_ARM_OFF    ;
        traj_sw     <= D_TRAJ_SW    ;
        phi_top     <= D_PHI_TOP    ;
        phi_fall    <= D_PHI_FALL   ;
        start_pulse <= 1'b0;
        wr_tog      <= 1'b0;
    end else begin
        start_pulse <= 1'b0;

        if(rx_vld) begin
            case(pos)
                P_H0 : pos <= (rx_byte == 8'hAA) ? P_H1 : P_H0;
                P_H1 : pos <= (rx_byte == 8'h55) ? P_IDX : P_H0;
                P_IDX: begin idx_r <= rx_byte; pos <= P_VH ; end
                P_VH : begin vh_r  <= rx_byte; pos <= P_VL ; end
                P_VL : begin vl_r  <= rx_byte; pos <= P_CHK; end
                default: begin                     // P_CHK
                    if(rx_byte == chk_calc) begin
                        case(idx_r)
                            8'h00: start_pulse <= 1'b1;
                            8'h01: kp_theta    <= val_q4;
                            8'h02: kd_theta    <= val_q4;
                            8'h03: kp_pos      <= val_q4;
                            8'h04: kd_pos      <= val_q4;
                            8'h05: u_max_bal   <= $signed(val_w);
                            8'h06: u_min_swing <= $signed(val_w);
                            8'h07: u_max_swing <= $signed(val_w);
                            8'h08: phi_catch   <= $signed(val_w);
                            8'h09: omega_catch <= $signed(val_w);
                            8'h0A: phi_top     <= $signed(val_w);
                            8'h0B: phi_fall    <= $signed(val_w);
                            8'h0C: ki_theta    <= val_q4;
                            8'h0D: ki_pos      <= val_q4;
                            8'h0E: arm_off     <= $signed(val_w);   // E1: 臂目标偏移(counts)
                            8'h0F: traj_sw     <= $signed(val_w);   // E2: 轨迹模式远程开关
                            default: ;                 // 未定义索引:忽略
                        endcase
                        wr_tog <= ~wr_tog;
                    end
                    pos <= P_H0;
                end
            endcase
        end
    end
end

endmodule
