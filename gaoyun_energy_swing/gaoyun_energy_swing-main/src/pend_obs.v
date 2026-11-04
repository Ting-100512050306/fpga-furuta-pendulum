//=============================================================================
// ★★★ 本模块当前【未启用】—— 仅供设计参考，请勿直接加入工程 ★★★
//   现状 (2026/10/27 核查): 整个工程没有任何一处例化 pend_obs。
//   实际在跑的是 controller.v 内部的 alpha-beta 滤波器 (phi_pred/w_hat)。
//   若要启用本观测器, 需先在 controller.v 里例化它, 并把 omega_f 换成它的 w_hat,
//   再重新整定 kd_theta (本文件注释里推算的目标是 kd_theta/kp_theta ≈ 227)。
//   已从 Auto_Swing.gprj 的编译列表中移除, 避免"改了却没生效"的误解。
//=============================================================================
//======================================================================
// pend_obs.v —— 摆杆角度/角速度 模型式观测器(离散 Luenberger / 简化 Kalman)
//   [2026/10/21] 为解决"速度信号被量化淹没"而加
//
// 背景(见实测记录 §28)：
//   摆杆在 0.33Hz 摆动时峰值角速度只有 0.138 LSB/3ms, 而量化步长是 1 LSB。
//   LQR 设计要求 kd_theta/kp_theta = 227 (手调是 0.6), 但按此放大后
//   速度噪声 ±1 LSB × 2902 = ±2902 cmd, 直接饱和。
//   => 必须先把速度估计的噪声降下来。
//
// 原理：
//   现有 alpha-beta 滤波器没有"模型", 只能靠两帧差分+平滑, 带宽受限于噪声。
//   本模块加入摆杆的物理模型(不稳定极点 + 摆臂加速的耦合), 用模型做预测,
//   实测只用于修正 -> 同带宽下噪声更低, 可把速度估计的可用增益提高约一个量级。
//
// 离散方程(周期 DT=3ms)：
//   phi_hat[k+1] = phi_hat + DT*w_hat + L1*(phi_meas - phi_hat)
//   w_hat[k+1]   = w_hat   + DT*(W0SQ*phi_hat + KC*theta_dd) + L2*(phi_meas - phi_hat)
//
// 参数(由 lqr_design.py 的 plant 得出, 见 plant_params):
//   W0SQ = 不稳定极点平方 ≈ 5.48^2 = 30.0   [1/s^2]
//   KC   = 摆臂角加速度对摆杆角加速度的耦合系数
//   L1, L2 = 观测器增益, 由期望带宽 wn 决定:
//        L1 = 2*zeta*wn*DT,  L2 = wn^2*DT
//        取 wn = 15 rad/s (约 2.4Hz), zeta=0.8
//        L1 = 2*0.8*15*0.003 = 0.072
//        L2 = 15*15*0.003     = 0.675
//
// 定点格式：
//   phi_meas / phi_hat : 有符号 16 位, 单位 ADC LSB
//   w_hat              : 有符号 16 位, 单位 LSB/3ms
//   theta_dd           : 有符号 16 位, 摆臂角加速度, 单位 (LSB/3ms)/3ms
//   系数用 Q12 定点
//======================================================================
module pend_obs #(
    parameter signed [31:0] W0SQ_Q12 = 32'sd401408,  // 98.0 * 4096 (= E/Jp = 0.06615/6.75e-4, 即 w0=9.9 rad/s)
    parameter signed [15:0] KC_Q12   = 16'sd0,        // 暂置 0: 第一阶段只加不稳定极点模型, 耦合项留待验证后加
    parameter signed [15:0] L1_Q12   = 16'sd1024,    // 0.250 * 4096 (对齐 alpha-beta 的 alpha)
    parameter signed [15:0] L2_Q12   = 16'sd319      // 0.078 * 4096 (对齐 alpha-beta 的 beta)
)(
    input  wire               clk_50m,
    input  wire               rst_n,
    input  wire               tick,        // 3ms 使能
    input  wire signed [15:0] phi_meas,    // 摆杆角度(ADC, LSB)
    input  wire signed [15:0] theta_dd,    // 摆臂角加速度
    output reg  signed [15:0] phi_hat,     // 估计角度
    output reg  signed [15:0] w_hat        // 估计角速度(LSB/3ms)
);
    wire signed [15:0] innov = phi_meas - phi_hat;          // 新息
    wire signed [31:0] cp = $signed(L1_Q12) * $signed(innov);
    wire signed [31:0] cw = $signed(L2_Q12) * $signed(innov);
    wire signed [15:0] corr_p = cp >>> 12;                  // L1*innov
    wire signed [31:0] corr_w_full = cw >>> 12;             // L2*innov (可能超 16 位)

    // 模型项: DT*(W0SQ*phi_hat + KC*theta_dd) = (W0SQ*phi_hat + KC*theta_dd) / 333
    wire signed [31:0] mp = $signed(W0SQ_Q12) * $signed(phi_hat);
    wire signed [31:0] mc = $signed(KC_Q12)   * $signed(theta_dd);
    wire signed [31:0] model_acc = (mp + mc) >>> 12;        // 1/s^2 * LSB
    // w_hat 单位是 LSB/拍; acc 单位是 LSB/s^2; Delta_w = acc*DT/333 = acc/333^2
    wire signed [31:0] model_vel = model_acc / 111089;      // 333^2

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            phi_hat <= 16'sd0;
            w_hat   <= 16'sd0;
        end else if (tick) begin
            // 位置: 用当前速度预测 + 新息修正
            phi_hat <= phi_hat + w_hat + corr_p;
            // 速度: 模型加速度积分 + 新息修正
            w_hat   <= w_hat + model_vel[15:0] + corr_w_full[15:0];
        end
    end

endmodule
