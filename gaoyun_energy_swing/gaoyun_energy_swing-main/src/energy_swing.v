//=============================================================================
// 模块: energy_swing   能量法自动起摆控制器
//-----------------------------------------------------------------------------
// 原理
//   以“倒立位”为势能零点,摆杆的机械能为
//       E = 1/2 * J * phi_dot^2 + m*g*l * ( cos(phi) - 1 )
//   倒立静止时 E = 0 (即 E_REF);自然下垂静止时 E 最小(-2mgl)。
//   起摆的本质就是把 E 从最小值“泵”到 E_REF,再交给平衡环接管。
//
//   Furuta 摆的能量导数(拉格朗日推导,标准结果):
//       dE/dt = -m*l*r * phi_dot * theta_ddot * cos(phi)
//             ∝ -tau * phi_dot * cos(phi)
//   其中 theta_ddot 为臂的角加速度,与臂驱动力矩 tau 同号。
//   所以要让能量增加(E < E_REF 时),力矩方向应取 -sign(phi_dot*cos(phi))。
//   注意:符号还取决于机械极性(phi 的正方向、电机的正方向),
//        若上板后摆杆被“钉”在底部上不来,即本式符号与实际相反,
//        把 SIGN_FLIP 置 1 即可整体反号。
//
// 实现(全整数,避免浮点/开方)
//   E_proxy = KIN_GAIN * omega^2 + ( cos_phi - 16384 )
//   其中 cos_phi 为 Q14 定点(倒立 = +16384,下垂 = -16384),
//   omega 为 ADC LSB / 采样周期(3ms)。
//   归一化说明:把势能项系数定为 1(即 16384 对应 mgl 的量纲),
//   则动能项系数必须按物理关系配平,否则能量误差永远不为零、
//   控制器会一直满输出(现象:起摆冲顶速度过快、电机动力过剩),
//   或反过来——系数偏大时控制器"以为"能量已达标,输出在泵能/刹车之间
//   来回翻转,摆杆卡在等幅摆动上不来(现象:摆臂左右对称晃动、到不了顶)。
//
//   --- KIN_GAIN 实测标定(2026/9/17,以此为准) ---
//   摆杆自由摆动时机械能守恒:E = K*omega_LSB^2 + 16384*(cos(phi)-1) = 常数
//   取实测 (phi, omega) 序列两两代入解 K:
//       K = 16384*(cos(phi_j)-cos(phi_i)) / (omega_i^2 - omega_j^2)
//   上板 120s 数据得 36397 组有效样本:
//       中位数 K = 13.8   (10%分位 5.8,90%分位 20.7)
//   取 KIN_GAIN = 14。(按均匀杆模型反推等效摆长约 9.5cm;
//   早期按 l=0.15m 估出的 44 偏大 3 倍,是"泵到半路就停"的直接原因。)
//   若更换摆杆/配重,必须按上述方法重新标定。
//
//   控制律
//       dir = sign( omega * cos_phi )                 // 能量注入方向
//       u   = -sign( E - E_REF ) * |u| * dir          // 能量偏差决定正/负
//   |u| 随 |E-E_REF| 线性衰减,E 越接近目标输出越小,天然不会冲顶。
//
// 整定提示
//   KIN_GAIN  : 动能项权重(物理量纲配平系数),见上
//   E_SHIFT   : 起摆增益,减小 -> 起摆更猛
//   U_MIN / U_MAX : 输出限幅(单位与 motor_ctrl 的 cmd 一致)
//   COS_DEAD  : |cos(phi)| 死区。摆杆接近水平时 cos≈0,驱动力几乎无效,
//               此区间直接置零,避免电机来回抖动
//   SIGN_FLIP : 若上板后发现摆杆越"泵"越没劲,把该参数改成 1(整体反号)
//=============================================================================
module energy_swing #(
    // ★ E_REF 取 -300(目标势能略低于顶部):
    //   泵到 E=0 意味着"刚好以零速抵达顶部",但输出下限 u_min=700 远大于
    //   摩擦(约 660 cmd),接近目标时每一拍仍是 700 的整推力,实测把摆杆顶过
    //   顶部时速度约 10 LSB/3ms(~5 rad/s),而平衡环 D 项在 ω>6.25 LSB/3ms 就饱和
    //   (kd_theta=320 -> 320*6.25=2000=u_max_bal),必然抓不住。
    //   E_REF=-300 对应顶部下方约 11°，比旧值 -500 更容易进入严格的低速捕获窗，
    //   同时仍保留欠顶余量，减少带着较大速度冲顶后立即掉落。
    parameter signed [31:0] E_REF     = -32'sd300  , // 目标能量(倒立位为 0)
    parameter        [31:0] KIN_GAIN  = 32'd14     , // 动能项权重(实测标定,见下方“KIN_GAIN 实测标定”)
    parameter        [3:0]  E_SHIFT   = 4'd4       , // 增益:u = |E_REF-E| >>> E_SHIFT
    parameter        [15:0] COS_DEAD  = 16'd512    , // |cos| 死区
    parameter               SIGN_FLIP = 1'b1       , // 1: 输出整体反号(实测：摆杆被钉在底部,需反号)
    // ★ [2026/10/17] 泵能抖动调制(DITHER)
    //   问题: u_min=700 而摩擦约 660。能量接近目标时,定律只想给 70~330,
    //         但被 u_min 抬到 700 -> 每次都是整块推力,必然把 E 推过 0;
    //         而"冲过 0 后再抽回来"发生在顶部、只持续几十毫秒,来不及。
    //         实测 5 次试验里这种"冲过头"是全部失败的共同形态。
    //   做法: 把"还差多少"映射成占空比 ——
    //         |dE|>>E_SHIFT >= u_min 时占空比 100%(和原来一样,起摆段不受影响);
    //         越接近目标占空比越低(8 档),平均推力可降到 87~260 < 摩擦,
    //         但每个脉冲仍是 u_min(700),能瞬间破静摩擦。
    //         不用除法: 用 mag_raw*8 >= cnt*u_min 比较,占空比天然 ∝ 能量偏差,
    //         且 u_min 仍由串口在线可调(调它同时改变脉冲幅度和档位)。
    //   DITHER_N = 抖动周期(控制拍),8 拍 = 24ms。
    // ★ 抖动调制开关 [2026/10/17] 1=启用,0=关闭(默认)
    //   实测(5 次 x 12s): 启用后 5 次全部 |phi|<10deg = 0%,摆杆卡在离顶 20~30deg
    //   再也上不去(E 均值 -2531 -> |dE|>>4 约 125 -> 占空比约 18% -> 平均推力
    //   125~175 cmd,远低于摩擦 660,臂完全不动)。
    //   结论: 摩擦(~660)已经等于起摆最小推力(u_min=700)的 95%,
    //         平均推力要超过摩擦需占空比 >94%,细调空间只有 5% -> 抖动方案不可行。
    //   这是机械/摩擦层面的硬约束,不是控制算法能绕过的。
    parameter               DITHER_EN = 1'b0,
    parameter        [3:0]  DITHER_N  = 4'd8
)(
    input                      clk_50m ,
    input                      rst_n   ,
    input                      tick    ,   // 3ms 采样有效脉冲
    input       signed [15:0]  phi     ,   // 摆角
    input       signed [15:0]  omega   ,   // 摆角速度
    input       signed [15:0]  cos_phi ,   // cos(phi),Q14

    // ★ 输出限幅改成"在线可调"(由 param_cfg.v 经串口写入),上电默认值写在
    //   param_cfg.v 的 D_U_MIN_SWING / D_U_MAX_SWING。
    //   U_MIN 是末段驱动力下限:实测 300 时末段驱动力与摩擦打平,能量卡在
    //   E≈-3585(|phi|≈405 LSB,离顶部 35.6°)再也涨不上去,起摆够不到顶部。
    //   抬到 900 后末段驱动力是摩擦当量的约 3 倍,|E| 一旦小于 14400 就变成
    //   恒定幅值泵能(mag 恒为 U_MIN),一路推到 E_REF=0;E 超过 E_REF 后
    //   d_energy 变号自动反向抽能,能量自己收敛,不会冲顶。
    //   中间那段线性斜坡(mag 900~1800,|E|≥14400)不受影响。
    input       signed [15:0]  u_min   ,   // 输出幅值下限
    input       signed [15:0]  u_max   ,   // 输出幅值上限(duty 上限约 112/250)

    output reg  signed [15:0]  u_out   ,   // 驱动力矩指令(按混合权重参与)
    // ★ [2026/10/17] u_brake: 只抽能、不泵能的独立刹车输出
    //   3ms 全采样实测(cap4.csv): 11 次被甩出顶部,【每一次】之前 E 都 > 0
    //   (+2324 ~ +14314);顶部附近 E>0 占 76%。即故障不是"能量不够",而是"能量排不掉"。
    //   唯一会抽能的就是本模块,但顶部附近 alpha 饱和到 256,本模块输出被完全掐断,
    //   只剩没有抽能能力的 PD。所以这里单独引出一路:
    //       E > E_REF(能量过剩) -> 输出与 u_out 同向的刹车子(幅值仍按 u_min/u_max 限幅)
    //       E <= E_REF           -> 输出 0,完全不干扰起摆泵能
    //   controller.v 把它【叠加在混合结果之后】,不受 alpha 门控。
    //   方向自检: 顶部 cosφ>0 时 prod=ω·cosφ 与 ω 同号 -> 刹车方向与 PD 的 D 项同向,
    //   两者叠加是"加强阻尼",不会互相抵消。
    output reg  signed [15:0]  u_brake     // 仅在能量过剩时有效的刹车量
);

    //------------------------------------------------------------------
    // 1) 代理能量 E = KIN_GAIN*omega^2 + (cos_phi-16384)
    //------------------------------------------------------------------
    wire signed [31:0] w2     = $signed(omega) * $signed(omega);
    wire signed [31:0] e_kin  = KIN_GAIN * w2;
    wire signed [31:0] e_pot  = {{16{cos_phi[15]}}, cos_phi} - 32'sd16384;
    wire signed [31:0] energy = e_kin + e_pot;

    //------------------------------------------------------------------
    // 2) 能量误差与能量注入方向
    //------------------------------------------------------------------
    wire signed [31:0] d_energy = E_REF - energy;
    wire signed [31:0] prod     = $signed(omega) * $signed(cos_phi);

    wire [15:0] cos_abs = cos_phi[15] ? (~cos_phi + 16'd1) : cos_phi[15:0];
    wire        cos_gate = (cos_abs >= COS_DEAD);

    //------------------------------------------------------------------
    // 3) 幅值饱和 + 符号合成
    //------------------------------------------------------------------
    wire signed [31:0] de_abs  = d_energy[31] ? -d_energy : d_energy;
    wire signed [31:0] mag_raw = de_abs >>> E_SHIFT;
    wire signed [31:0] mag     = (mag_raw > $signed(u_max)) ? $signed(u_max) :
                                 (mag_raw < $signed(u_min)) ? $signed(u_min) : mag_raw;

    wire               sign_out = d_energy[31] ^ prod[31] ^ SIGN_FLIP;
    wire signed [31:0] u_val    = sign_out ? -mag : mag;

    //------------------------------------------------------------------
    // 抖动调制计数器与占空比判断
    //------------------------------------------------------------------
    reg  [3:0]  dith_cnt;
    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)          dith_cnt <= 4'd0;
        else if(tick)       dith_cnt <= (dith_cnt >= DITHER_N - 4'd1) ? 4'd0 : dith_cnt + 4'd1;
    end

    // mag_raw*8 >= cnt*u_min  -> 本拍输出脉冲
    //   mag_raw >= u_min 时恒成立(连续输出,起摆段行为不变)
    wire [31:0] mag_x8   = mag_raw << 3;
    wire [31:0] dith_thr = dith_cnt * $signed(u_min);
    wire        dith_on  = DITHER_EN ? (mag_x8 >= dith_thr) : 1'b1;

    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            u_out   <= 16'sd0;
            u_brake <= 16'sd0;
        end else if(tick) begin
            if(!cos_gate) begin
                u_out   <= 16'sd0;
                u_brake <= 16'sd0;
            end else begin
                u_out   <= dith_on ? u_val[15:0] : 16'sd0;      // 抖动门控
                // d_energy[31]=1 表示 E > E_REF(能量过剩) -> 才刹车
                u_brake <= d_energy[31] ? u_val[15:0] : 16'sd0;
            end
        end
    end

endmodule
