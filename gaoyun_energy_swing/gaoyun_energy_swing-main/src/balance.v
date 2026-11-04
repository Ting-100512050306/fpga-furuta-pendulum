//=============================================================================
// 模块: balance   倒立平衡控制器(倾角环 + 臂位置环 叠加,PD 结构)
//-----------------------------------------------------------------------------
//   u = KP_THETA*phi     + KD_THETA*omega        // 倾角环:把摆杆稳在 phi = 0
//     + KP_POS  *arm_err + KD_POS  *arm_vel      // 位置环:把旋转臂拉回目标位置
//
//   phi     : 摆角(0 = 倒立),单位 ADC LSB
//   omega   : 摆角速度,单位 LSB/采样周期(3ms)
//   arm_err : 旋转臂相对目标位置的偏差(编码器计数)
//   arm_vel : 旋转臂速度,单位 计数/采样周期(3ms)
//
//   en = 0 时位置目标 arm_tgt 持续跟随当前编码器值,arm_err 恒为 0,
//   此刻本模块只输出倾角环结果;en 拉高(切入平衡)时 arm_err 从 0 平滑
//   开始增长,即“就地平衡”:旋转臂停在哪儿就以哪儿为目标,不会在切换
//   瞬间因为一个巨大的虚假位置误差而踹一下电机。
//
// 整定提示
//   ★ 四个增益都声明为有符号。若上板后发现“越控越倒”,
//     把 KP_THETA / KD_THETA 的符号同时取反;若臂总是单向漂走,
//     把 KP_POS / KD_POS 的符号取反。
//   ★ KP_THETA 太小 -> 推不住,摆杆缓慢倒下;太大 -> 高频抖动。
//   ★ KD_THETA 用来抑制抖动,但太大会把 ADC/编码器噪声放大。
//=============================================================================
module balance #(
    // ★ 臂阻尼门控开关 [2026/10/17]
    //   1 = 按 alpha 分时生效;0 = 直接用 arm_vel(原行为,实测最好)
    //   3 次重复试验结论(每次 12s):
    //     门控关 + kd_pos=4.4  -> |phi|<10deg 占 21/40/10%,最长稳顶 1.52/1.58/1.24s  <= 取这个
    //     门控关 + kd_pos=40.0 -> 臂锁死 0.01 圈/s,顶部占比 0~5%
    //     门控开 + kd_pos=40.0 -> 3/9/0%,最长 0.44/0.34/0s(进捕获带就刹臂,起摆泵失效)
    //   原因: 摆杆每次经过捕获带都要靠臂把能量泵上去,任何在该带内生效的强臂阻尼
    //         都会掐断起摆。要解决臂漂移得靠消除恒定偏置力矩(积分器),不是臂阻尼。
    parameter GATE_ARM_DAMP = 1'b0
)(
    input                      clk_50m ,
    input                      rst_n   ,
    input                      tick    ,   // 3ms 采样有效脉冲
    input                      en      ,   // 平衡环使能
    input                      clr     ,   // 积分器清零(仅 IDLE 时有效) [2026/10/17 新增]
    input              [8:0]   alpha   ,   // 起摆/平衡混合权重 Q8 (0~256) [2026/10/17 新增]
    input       signed [15:0]  phi     ,
    input       signed [15:0]  omega   ,
input       signed [15:0]  omega_f ,   // 8 拍(24ms)平滑摆角速度, 供 D 项用; omega_f ~= 8*omega
    input              [15:0]  arm_pos ,

    // ★ 以下 5 个增益已改成"在线可调":由 param_cfg.v 经串口写入,上电默认值
    //   写在 param_cfg.v 的 D_xxx 常量里(与之前的编译期数值一致)。
    //   整定依据(2026/9/17):
    //   --- 倾角环 ---
    //   换算关系:k ≈ 0.0039 m/s² 每 cmd 单位(由实测 u=-2000 把 phi 稳在
    //   -225 LSB ≈ -0.345rad 反推,a = g*tan(0.345) = 3.53 m/s²),
    //   故 Kp_phys = 2.54*KP, Kd_phys = 0.00763*KD (m/s² 每 rad、每 rad/s)。
    //   等效摆长 l≈0.095m -> w0 = sqrt(g/l) ≈ 10 rad/s。
    //   旧值 KP=15/KD=12 换算得阻尼比 zeta≈0.028(几乎无阻尼) -> 摆杆一进
    //   平衡区就被踹飞。现目标 zeta≈1、闭环 wn≈5.5 rad/s(低于电机
    //   时间常数 ~120ms 对应的 1.3Hz,留足相位裕度)。
    //   ★ 上电默认 KP=20 / KD=320(2026/9/17 串口在线整定实测值)。
    //   注意:这两个数比按力矩模型换算出来的值大 4 倍。模型按
    //   k≈0.0039 m/s^2 每 cmd 推得 KP=5 就够(Kp_phys=12.7>g),但实测
    //   KP=5/KD=140 只能在倒立点撑住 ~0.3s;KP=20/KD=320 才能连续立住
    //   4~6s(KP=30/KD=480 又变差)。说明模型的 k 偏乐观,实际控制效率
    //   比推算的低,必须靠"加大增益"把闭环极点推到左半平面。
    //   极性已由实测确定:u 须与 phi 同号。
    //   --- 位置环 ---
    //   符号:上电默认取【正】(KP_POS=+2, KD_POS=+44),这是实测出来的。
    //   上一版曾按 corr(cmd, dEnc)=+0.891 推断"正 cmd 使摆臂正转,所以要
    //   取负增益",那个相关其实是倾角环自己造成的(倾角环为了扶住摆杆
    //   主动驱动摆臂),推不出被控对象的符号。实测把位置环取负(-2/-44)
    //   时平衡占比只有 30%,取正时 63~78%。
    //   ★ 另外:摆臂缓慢单向漂走是"摆杆倒立时的反作用力"驱动摆臂造成的,
    //     和位置环符号无关 —— 位置环取 -2/-20 时漂移反而更快(-940 计数/s)。
    //   量级:BALANCE 段对 (cmd, 摆臂角加速度) 做线性回归,斜率 7.24e-4
    //   计数/帧^2 每 cmd(帧周期 20ms),换成国际单位:
    //     b = 7.24e-4 * 2500 * (2*pi/1138) ≈ 0.010 rad/s^2 每 cmd
    //   取外环(摆臂位置环)目标 wn=1rad/s、zeta=1,比倾角环(wn≈5.5rad/s)
    //   慢 5 倍以上,保证两者频带分离、不互相抢。
    //     Kp = wn^2/b      = 100  cmd/rad = 0.552 cmd/计数
    //     Kd = 2*zeta*wn/b = 200  cmd/(rad/s) = 1.104 cmd per (计数/s)
    //   arm_err 用 >>>2 缩放到与 phi 同量级(1 计数≈3.6 phi-LSB):
    //     KP_POS = 0.552*4 ≈ 2   -> 等效 0.5 cmd/计数 -> wn≈0.95rad/s
    //   arm_vel 见下方 arm_dly 延迟线,它是 24ms(8 拍)的差分,即
    //   arm_vel = 角速度[计数/s] * 0.024,故
    //     KD_POS = 1.104*0.024 ≈ 0.026 -> 太小无法用整数表示,
    //   故把差分窗口拉长到 24ms 来"放大"这个系数:
    //     KD_POS = Kd / 0.024 = 46 -> 取 44 -> zeta≈1.0
    //   ★ 之所以用 24ms 差分:1 拍(3ms)差分的分辨率是 333 计数/s,而平衡段
    //     摆臂典型速度只有 ~130 计数/s,D 项会被量化吃光。
    input       signed [15:0]  kp_theta ,   // 倾角环 P(单位 1/16,20.5 存 328)
    input       signed [15:0]  kd_theta ,   // 倾角环 D(单位 1/16)
    input       signed [15:0]  kp_pos   ,   // 位置环 P(正,单位 1/16)
    input       signed [15:0] kd_pos   ,   // 位置环 D(正,配 24ms 差分,单位 1/16)
    input       signed [15:0] ki_theta ,   // 倾角环 I(单位 1/16,消恒定偏置力矩)
    input       signed [15:0] ki_pos   ,   // 位置环 I(单位 1/16,消摆臂速度漂移)
    input       signed [15:0] u_max    ,   // 输出限幅(单位 cmd,不缩放)
    input       signed [15:0]  arm_off  ,   // E1: 臂目标偏移(counts)

    output reg  signed [15:0]  u_out,
    output      signed [15:0]  dbg_out    // [DEBUG] 诊断量(当前接 arm_err)
);

    reg [15:0] arm_tgt;
    reg [15:0] arm_dly [0:7];       // 摆臂位置延迟线,第 [7] 项 = 24ms 前的值
    integer    di;

    // 16 位补码减法可自然处理编码器计数溢出
    wire signed [15:0] arm_err = arm_pos - arm_tgt;
    wire signed [15:0] arm_err_s = arm_err >>> 2;
    //==================================================================
    // E1 臂位置控制: 参考点 + 目标偏移 + 斜坡  [2026/10/19]
    //------------------------------------------------------------------
    //   目标 = arm_ref(平衡建立时的臂位置) + arm_off(串口 0x0E 设定)
    //   斜坡: 每 3ms 最多 1 count = 333 counts/s = 0.29 圈/s = 105 deg/s
    //         对应臂加速约 122 deg/s^2, 摆杆只需倾斜约 1.1 deg, 安全。
    //   安全: |phi| > 8 deg 时暂停目标移动, 防止把摆杆甩倒。
    //   参考点只在 IDLE(clr) 时失效; 平衡中途掉落不会冲掉已设定的目标。
    //==================================================================
    localparam signed [15:0] RAMP_STEP   = 16'sd1;
    localparam signed [15:0] MOVE_PHI_MAX = 16'sd170;   // 15 deg = 170 LSB [2026/10/23 由 8deg 放宽: 实测移动被反复暂停, 285counts 走了 20 秒]

    reg  [15:0] arm_ref;
    reg         ref_valid;

    wire signed [15:0] phi_abs_e = phi[15] ? -phi : phi;
    wire               move_ok   = (phi_abs_e < MOVE_PHI_MAX);
    wire        [15:0] tgt_final = arm_ref + arm_off;
    wire signed [15:0] tgt_diff  = tgt_final - arm_tgt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            arm_ref   <= 16'd0;
            ref_valid <= 1'b0;
            arm_tgt   <= 16'd0;
        end else begin
            // IDLE 时建立“归位点”；失稳重起摆期间只让临时目标跟随当前臂位，
            // 不覆盖归位点。这样重捕获后会平滑回到原方向，而不是把漂移位置
            // 误当成新的中心。
            if (clr) begin
                arm_ref   <= arm_pos;
                arm_tgt   <= arm_pos;
                ref_valid <= 1'b1;
            end else if (!en) begin
                arm_tgt   <= arm_pos;
            end else if (tick && ref_valid && move_ok) begin
                if (tgt_diff > RAMP_STEP)
                    arm_tgt <= arm_tgt + RAMP_STEP;
                else if (tgt_diff < -RAMP_STEP)
                    arm_tgt <= arm_tgt - RAMP_STEP;
                else
                    arm_tgt <= tgt_final;
            end
        end
    end

    // [DEBUG] 改送"位置环 P 项的实际出力"(已 >>4 到 cmd 单位)，用来核对 kp_pos 运行值
    wire signed [31:0] dbg_pos16 = $signed(kp_pos) * arm_err_s;
    wire signed [31:0] dbg_pos   = dbg_pos16 >>> 4;
    assign dbg_out = (dbg_pos >  32767) ?  16'sd32767 :
                     (dbg_pos < -32768) ? -16'sd32768 : dbg_pos[15:0];
    // ★ 摆臂速度用 8 拍(24ms)差分,而不是 1 拍(3ms)。原因:编码器约 1138
    //   计数/圈,1 拍差分的最小分辨率是 1 计数/3ms ≈ 333 计数/s ≈ 1.8rad/s,
    //   而平衡段摆臂的典型速度只有 ~130 计数/s,1 拍差分几乎全是量化台阶,
    //   D 项会退化成"全有全无"。改成 24ms 窗口后分辨率提高 8 倍(≈42 计数/s)。
    wire signed [15:0] arm_vel = arm_pos - arm_dly[7];

    //------------------------------------------------------------------
    // ★ 臂阻尼按 alpha 分时生效  [2026/10/17 新增]
    //------------------------------------------------------------------
    //   实测矛盾(都是 3ms 全采样测出来的):
    //     kd_pos=44  -> 臂 1.0~1.5 圈/s 飞转,离心力 r*θ̇² 把摆杆推出顶部;
    //                   此刻 kd_pos 只能给 2.75*41 ≈ 100 cmd,远低于摩擦阈值 ~660
    //     kd_pos=400 -> 臂被锁死到 0.01 圈/s,但起摆泵也被一起锁死,
    //                   顶部占比从 25% 掉到 0~5%,最长稳顶 1.45s -> 0.3s
    //   两者要的是同一个增益,所以必须让它【只在靠近顶部时生效】:
    //     alpha=256(|phi| 小,要平衡) -> 臂阻尼全开,压住飞转
    //     alpha=0  (|phi| 大,在泵能) -> 臂阻尼几乎关闭,让泵能把臂甩起来
    //   实现:先给 arm_vel 乘 alpha 再 >>8 还原(25 位中间积,不会溢出),
    //        后面仍只用一个 16x16 乘法器,不增加乘法器数量。
    //------------------------------------------------------------------
    wire signed [24:0] av_mul    = $signed(arm_vel) * $signed({1'b0, alpha});
    wire signed [15:0] arm_vel_g = av_mul >>> 8;
    wire signed [15:0] arm_vel_use = GATE_ARM_DAMP ? arm_vel_g : arm_vel;

    //------------------------------------------------------------------
    // 控制律
    //   arm_err 右移 2 位:编码器 1 计数 ≈ 3.6 phi-LSB,先缩放到与 phi 同量级
    //   再乘增益。arm_vel 是 24ms 差分,不再右移。
    //
    //   ★ 增益存储的单位是 1/16(Q4),即 存进来的数 = 真实增益 x 16。
    //     这个 x16 由 param_cfg.v 在"写参数的那一拍"换算好(上位机仍按 0.1
    //     为单位下发,205 表示 20.5),所以这里求完各项乘积之和,只要一次
    //     算术右移 >>4 就还原成 cmd 单位。
    //     => 关键路径上既没有除法器,也没有额外的乘法器,只有 6 个 16x16
    //        乘法(走 DSP)加一棵加法树,末尾接一根连线。
    //     (更早的版本是按 0.1 为单位求和后再 x3277>>15 除 10,那个多出来的
    //      32x12 乘法器把关键路径拖到 25.8ns、Fmax 只有 46.6MHz。)
    //   分辨率:Q4 的最小步进是 0.0625,比 0.1 更细,0.1 的步进都能表示。
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // 积分项 [2026/9/18 新增] —— 这是"摆臂一直旋转、立一会儿就倒"的根因
    //------------------------------------------------------------------
    //   实测摆杆倒立时 phi 的稳态均值有 +37 LSB(≈3.3°)固定偏置,说明系统里
    //   存在一个恒定偏置力矩(齿轮摩擦 / 静不平衡 / 电机死区),约合 660 cmd。
    //   纯 PD 对常值扰动是 I 型系统,只能靠"常驻一个固定倾角"去顶它。
    //
    //   而 Furuta 摆有个硬约束:摆杆一旦带固定倾角 φ0,平衡条件就要求摆臂
    //   持续产生 θ̈ = (g/r)*tan(φ0) 的角加速度 —— 等于给摆臂下了一个"永远
    //   加速"的指令;摆臂被齿轮摩擦限速后表现为单向匀速旋转
    //   (实测 -523 计数/s ≈ -0.46 圈/s)。
    //   摆臂一转,位置环 P 项为了把它拉回来吃掉 68% 的 u_max,倾角环只剩
    //   32% 权限 -> 立得住一会儿就倒。再叠加摆臂 2.9rad/s 的转速让枢轴离心
    //   加速度达 r*θ̇²≈0.84m/s²(等效重力偏差 8.6%),平衡彻底破坏。
    //
    //   加了积分之后:偏置力矩由积分器吸收,φ 的直流分量被压到 0,θ̈ 需求
    //   随之消失,摆臂不再有持续旋转的驱动力。
    //
    //   定点:I 项与 P 项共用 Q4(寄存器值 = 真实增益 x16),
    //       u_I = ki_real * (Σe) / 2^I_SHIFT ,  Σe 为每 3ms 累加一次的误差和
    //       ∫e dt ≈ 0.003*Σe  =>  等效连续积分增益 = ki_real/(2^8 * 0.003)
    //                                              = 1.30 * ki_real
    //     取 ki_real ≈ KP/2.6,即复位时间 Ti ≈ 2s(远大于闭环周期,不抢相位)。
    //   I_SHIFT=8,累加器限幅 ±32767*256,保证 (Σe)>>8 落在 16 位,乘法器
    //   维持 16x16,不额外加宽数据通路;乘法输入全部来自寄存器,
    //   不给关键路径添深度。
    //------------------------------------------------------------------
    localparam integer       I_SHIFT = 8;                 // integer 是"有符号"类型,
                                                          // 保证 i_xxx >>> I_SHIFT 走算术移位
    localparam signed [31:0] I_LIM   = 32'sd8388352;      // 32767*256

    reg  signed [31:0] i_phi;
    reg  signed [31:0] i_arm;

    wire signed [15:0] i_phi_s = i_phi >>> I_SHIFT;
    wire signed [15:0] i_arm_s = i_arm >>> I_SHIFT;

    //==================================================================
    // 双通道摆角速度  [2026/10/17]
    //   快通道: omega   (单拍差分, 零滞后) -> 保快回路相位裕度
    //   慢通道: omega_avg = omega_f/8 (24ms 平均, 有 ~12ms 滞后)
    //           -> 只补低频阻尼, 把 ±7deg/0.3Hz 的慢振荡压下去;
    //              滞后的相位损失由小增益承担, 不进快回路。
    //   两者相加: 既不丢快回路稳定性, 又能看见测量分辨率以下的慢运动。
    //   注: KD_SLOW_T 是编译期常数(Q4), 单位与 kd_theta 相同(=真实值*16)。
    //==================================================================
    localparam signed [15:0] KD_SLOW_T = 16'sd8;       // 0.5；配合未缩放的 α-β 速度，等效增益保持不变
    // omega_f 已经是每拍角速度估计，不再错误除以 8；同步把慢通道增益除以 8。
    wire signed [15:0] omega_avg = $signed(omega_f);
    // [2026/10/21] kd_pos 软启动: 捕获瞬间 0.75 倍(约49, 捕获可靠), 3 秒内渐变到 1.0 倍(65, 摆臂安静)
    reg [9:0] pos_ramp;
    wire signed [31:0] kd_pos_wide = $signed(kd_pos);
    wire signed [15:0] kd_pos_eff = (kd_pos_wide * $signed({1'b0, pos_ramp[9:2]})
                                   + (kd_pos_wide <<< 9) + (kd_pos_wide <<< 8)) >>> 10;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                            pos_ramp <= 10'd0;
        else if (!en || clr)                   pos_ramp <= 10'd0;
        else if (tick && pos_ramp != 10'd1023) pos_ramp <= pos_ramp + 10'd1;
    end

    wire signed [31:0] u_sum16 = $signed(kp_theta) * $signed(phi)
                               + $signed(KD_SLOW_T) * $signed(omega_avg)
                               + $signed(kd_theta) * $signed(omega)
                               + $signed(kp_pos)   * arm_err_s
                               + $signed(kd_pos_eff) * $signed(arm_vel_use)
                               + $signed(ki_theta) * i_phi_s
                               + $signed(ki_pos)   * i_arm_s;

    wire signed [31:0] u_sum   = u_sum16 >>> 4;      // Q4 -> cmd 单位,纯连线
    wire signed [31:0] u_max_s = {{16{u_max[15]}}, u_max};

    // 抗饱和(条件积分):输出已经顶到限幅、而误差还想往同方向推时停止累加;
    // 反向的误差仍允许积分,这样能自己退饱和,不会长时间卡在饱和上。
    wire sat_hi = (u_sum >  u_max_s);
    wire sat_lo = (u_sum < -u_max_s);

    wire int_phi_en = !(sat_hi && (phi > 0))         && !(sat_lo && (phi < 0));
    wire int_arm_en = !(sat_hi && (arm_err_s > 0))   && !(sat_lo && (arm_err_s < 0));

    always@(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            for(di = 0; di < 8; di = di + 1)
                arm_dly[di] <= 16'd0;
            i_phi   <= 32'sd0;
            i_arm   <= 32'sd0;
            u_out   <= 16'sd0;
        end else begin

            //------------------------------------------------------------------
            // ★ 积分器清零策略 [2026/10/17 修改]
            //   原来写成 if(!en) i_phi<=0,而 en 在 |phi|>PHI_FALL 时会掉 0,
            //   等于每次掉出平衡区都把积分清空 -> 积分永远学不会恒定偏置力矩。
            //   实测证据(2026/10/17 上板):
            //     ki_theta=0  -> 臂 4.42 圈/s,12s 转 53 圈,平衡占比 21%
            //     ki_theta=24 -> 臂 0.76 圈/s,平衡占比 43%
            //     ki_theta=48 -> 臂 0.17 圈/s,但平衡占比掉到 8%(积分在捕获区就顶死)
            //   积分能压住臂的旋转,说明"恒定偏置力矩"确实存在;但每段平衡只有
            //   0.2~1s,现积现用根本积不够。这与 controller.v 里那句
            //   "短暂的 SWING/BALANCE 往返也不会把积分器反复清零"的设计意图一致
            //   —— 现在才真的做到。
            //   只在 clr(IDLE)时清零;SWING<->BALANCE 往返保持。
            //   注:clr 与 en 互斥(en 要求 state!=IDLE),同一拍不会互相覆盖。
            //------------------------------------------------------------------
            if(clr)
                i_phi <= 32'sd0;

            // 摆角积分保留已学习的静态偏置；位置积分与临时目标绑定，掉落后
            // 必须清零，避免重捕获时残留 I 项把摆臂推向旧误差方向。
            if(clr || !en)
                i_arm <= 32'sd0;

            if(tick) begin
                arm_dly[0] <= arm_pos;
                for(di = 1; di < 8; di = di + 1)
                    arm_dly[di] <= arm_dly[di-1];

                if(en && !clr) begin
                    if(int_phi_en)
                        i_phi <= (i_phi + phi > I_LIM)  ?  I_LIM :
                                 (i_phi + phi < -I_LIM) ? -I_LIM : (i_phi + phi);
                    if(int_arm_en)
                        i_arm <= (i_arm + arm_err_s > I_LIM)  ?  I_LIM :
                                 (i_arm + arm_err_s < -I_LIM) ? -I_LIM : (i_arm + arm_err_s);
                end

                if(u_sum > u_max_s)
                    u_out <= u_max;
                else if(u_sum < -u_max_s)
                    u_out <= -u_max;
                else
                    u_out <= u_sum[15:0];
            end
        end
    end

endmodule

