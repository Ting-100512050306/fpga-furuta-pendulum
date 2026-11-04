# 基于 FPGA 的旋转倒立摆实时姿态控制系统

> 高云 GW2A-55C · 单片 FPGA 纯硬件逻辑实现 · 3 ms（333 Hz）硬实时控制
>
> 自动起摆 · 倒立自平衡 · 抗外部扰动 · 摆臂定点位置控制 · 正弦轨迹运动（摆杆保持直立）

---

## 项目简介

本项目在单片高云 GW2A-55C FPGA 内实现一级旋转倒立摆（Furuta 摆）的全部实时控制逻辑。
**不使用软核处理器、不依赖外部 MCU**：采样、状态估计、控制律运算与 PWM 输出全部由硬件逻辑完成，
控制节拍严格固定为 3 ms。

控制策略上：

- **起摆**：能量法泵能，按能量偏差决定泵能/抽能，按 `ω·cosφ` 决定力矩方向；
- **平衡**：倾角环 PD+I + 位置环 PD+I；
- **过渡**：两路输出按"距倒立点远近"连续加权混合，无硬切换冲击；
- **测速**：α-β 滤波器估计摆杆角速度，解决低速段微分项被量化噪声淹没的问题；
- **拓展**：摆臂按键定点控制（斜坡逼近 + 倾角安全限幅）、20 秒周期正弦轨迹（相位归零 + 幅度软启动）。

---

## 目录结构

```
.
├── README.md                                  ← 本文件
├── .gitignore
├── 技术文档_基于FPGA的旋转倒立摆实时姿态控制系统.pdf   ← 完整技术文档（含实测数据）
└── gaoyun_energy_swing/
    ├── gaoyun_energy_swing-main/              ← 高云工程
    │   ├── src/                               源码（14 个 .v + .cst 引脚约束 + .sdc 时序约束）
    │   ├── Auto_Swing.gprj                    高云工程文件
    │   └── README.md                          ★ 工程详细说明（编译/烧录/按键/参数/串口协议）
    └── tools/
        ├── live_plot.py                       实时波形显示（串口遥测 → 曲线 + CSV）
        └── smooth_check.py                    正弦轨迹平滑度分析（R²/谐波/颤振）
```

> **注意**：仓库中不含 `impl/`（综合/布线/位流产物，约 24 MB）。
> 请克隆后用高云 IDE 打开 `Auto_Swing.gprj` 执行 `Run All` 重新生成；
> 或从本仓库的 **Releases** 页面下载现成的 `Auto_Swing.fs` 直接烧录。

---

## 快速上手

### 1. 编译

用高云云源 IDE 打开 `gaoyun_energy_swing/gaoyun_energy_swing-main/Auto_Swing.gprj`，双击 **Run All**。

命令行方式：

```tcl
open_project {<你的路径>/Auto_Swing.gprj}
run all
```

**编译后必须检查两项**：

1. 综合日志里**不能有 `TA1132`**（未约束时钟）
2. 时序报告里 **TNS = 0.000、Setup/Hold 违例端点 = 0**

### 2. 烧录

```powershell
programmer_cli.exe --device GW2A-55C --cable-index 0 --operation_index 2 `
                   --fsFile "<你的路径>\impl\pnr\Auto_Swing.fs"
```

> `--operation_index 2` = SRAM 下载（掉电丢失）。

### 3. 操作

| 按键 | 功能 |
|---|---|
| **SW5** | 全局复位 |
| **SW1** | 启动 / 停止 自动起摆 |
| **SW2** | 摆臂位置档循环（中 → 左 → 右）|
| **SW3** | 进出正弦轨迹模式 |

详细说明见 [`gaoyun_energy_swing/gaoyun_energy_swing-main/README.md`](gaoyun_energy_swing/gaoyun_energy_swing-main/README.md)。

---

## 硬件平台

| 部件 | 型号 |
|---|---|
| FPGA | 高云 **GW2A-55C**（GW2A-LV55PG484C8/I7）|
| 开发平台 | J280 姿态控制系统开发平台 |
| 摆角传感器 | WDD35D4 电位器 + 3PA1030 10 位 ADC |
| 电机 | MG531 直流减速电机（带增量编码器，1138 counts/圈）|
| 电机驱动 | TB6612FNG（A/B 两路并联）|
| 调试接口 | 115200 bps 串口（CH340）|

---

## 实测性能

| 指标 | 实测值 |
|---|---|
| 倒立平衡状态占比 | 98% ~ 100% |
| 摆杆倾角平均偏差 | 1.7° ~ 2.4°（峰值 < 10°）|
| 连续稳定时间 | ≥ 70 秒（最长记录 119 秒）|
| 起摆时间 | 3 ~ 10 秒捕获 |
| 位置控制平衡占比 | 100% |
| 轨迹正弦拟合优度 | R² = 0.993 ~ 0.998 |
| 时序收敛 | Fmax 50.14 MHz，TNS 0.000，违例 0 |
| FPGA 资源占用 | 逻辑 5%，寄存器 3% |

---

## 已知限制

| 限制 | 说明 |
|---|---|
| 摆臂存在固有往复摆动 | 一级旋转倒立摆是**欠驱动系统**（1 电机控 2 自由度），扶住摆杆必然需要摆臂运动。赛题指标均针对**摆杆** |
| ADC 零点需现场标定 | 零点偏差 6° 就会导致摆臂持续单向旋转，标定方法见工程 README |
| 摆杆存在约 5.5 Hz 微颤 | 速度估计相位滞后引起的环路极限环（RMS ≈ 1.2°），是当前精度主要限制 |
| 长时间运行需注意线缆 | 摆臂累计转动圈数过多会绕线，建议单次演示 ≤ 2 分钟 |

---

## 辅助工具

```powershell
pip install pyserial matplotlib numpy

# 实时波形（同时存 CSV）
python gaoyun_energy_swing/tools/live_plot.py COM4

# 正弦平滑度分析
python gaoyun_energy_swing/tools/smooth_check.py traj.csv
```

---

## 许可

本项目仅用于学习与竞赛，欢迎参考。如用于发表或商用，请自行确认相关器件手册与算法的授权条款。
