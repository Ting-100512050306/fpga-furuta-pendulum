# -*- coding: utf-8 -*-
"""
==============================================================================
 live_plot.py —— 旋转倒立摆 实时轨迹显示窗口
----------------------------------------------------------------------------
 作用: 从串口读 3ms 遥测, 实时画出
        上: 摆臂角度 (度)          下: 摆杆角度 (度)
       并在控制台打印统计值。
       演示时把本窗口放在旁边, 评委能直接看到"摆臂在跟踪什么轨迹"。

 用法:
    python live_plot.py                 # 自动找串口
    python live_plot.py COM4            # 指定串口
    python live_plot.py COM4 --test     # 不开窗口, 只打印解析统计(自检)

 显示比例可在命令行临时改（不用改代码）:
    --window 40     横轴显示最近 40 秒（默认 40 = 2 个 20 秒周期）
    --xtick 2       横轴刻度间隔 2 秒（默认 2）
    --cycles 2      便捷写法: 窗口 = 2 个轨迹周期（等价 --window 40）
    --period 20     轨迹周期(秒)，仅用于标题显示（默认 20，对应 PH_STEP=10065）
    --armfix 200    摆臂纵轴固定 ±200°（默认 auto 自动缩放；固定后若摆臂累计转动
                    超过 ±200° 波形会跑出窗口，仅短时间演示时建议用）
    --ymax 12       摆杆纵轴最小半量程 ±12°（默认 12，改小=曲线更陡）

 依赖: pyserial, matplotlib, numpy
       pip install pyserial matplotlib numpy
==============================================================================
"""
import sys, time, threading, collections
import numpy as np

BAUD     = 115200
WINDOW_S = 40.0      # 窗口显示最近多少秒  [2026-10-31] 60 -> 40 = 正好 2 个 20 秒周期
                     #   想看 K 个周期:  python live_plot.py COM4 --cycles K
                     #   窗口必须 > 轨迹周期 才能看全完整波形
# 轨迹周期（秒）—— 只影响标题显示，不影响固件！
#   固件里由 top.v 的 PH_STEP 决定: 周期 = 512*2^17/PH_STEP * 3ms
#     PH_STEP = 16777 -> 12.0 秒
#     PH_STEP = 10065 -> 20.0 秒   <-- 当前固件就是这个
TRAJ_PERIOD_S = 20.0
X_TICK_S     = 2.0   # 横轴刻度间隔(秒)  [2026-10-31] 5 -> 2
DECIMATE     = 3     # 每 3 帧取 1 帧显示 (333Hz -> 111Hz)
REF_PER_REV  = 1138.0  # 编码器每圈计数
LSB_DEG      = 180.0 / 2048.0

# --- 画布 / 纵轴 ---
FIG_W, FIG_H = 13.0, 9.0   # 画布尺寸（原来 12x6.5 太扁）
# 摆臂纵轴: 默认 auto(随数据缩放, 永远看得见波形)
#   ★ 不能默认固定量程: arm_accum 是从程序启动累计的, 起摆/掉落时摆臂转过几圈后
#     累计值会跑到 ±200 以外, 固定量程下波形就整条跑出窗口看不到。
#   需要固定量程时用命令行 --armfix 200
ARM_Y_MODE   = 'auto'  # 'auto' = 随数据缩放 ; 'fix' = 固定 ±ARM_Y_LIM
ARM_Y_LIM    = 200.0   # 固定模式下的半量程(度)
ARM_Y_TICK   = 50.0    # 固定模式下的刻度间隔(度)
ARM_Y_STEP   = 10.0    # auto 模式: 量程吸附到 10° 的整数倍(防抖动)
ARM_Y_PAD    = 0.08    # auto 模式: 留白 = 数据跨度的 8%
ARM_Y_MIN_H  = 30.0    # auto 模式: 最小高度(度)
PHI_Y_FLOOR  = 12.0    # 摆杆纵轴最小半量程(度): 平衡时 ±12 让曲线占满, 掉落时自动放大

HDR = b'\xAA\x55'
TAIL = b'\x0D\x0A'


def find_port():
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
        return ports[0] if ports else None
    except Exception:
        return None


def parse_frames(buf):
    """从字节流里取出完整帧, 返回 [(phi, enc, state, cmd)]"""
    out = []
    i = 0
    n = len(buf)
    while i + 9 < n:
        if buf[i:i + 2] == HDR and buf[i + 8:i + 10] == TAIL:
            phi = int.from_bytes(buf[i + 2:i + 4], 'big', signed=True)
            enc = int.from_bytes(buf[i + 4:i + 6], 'big', signed=False)
            w = int.from_bytes(buf[i + 6:i + 8], 'big')
            state = (w >> 14) & 0x3
            cmd = w & 0x1FFF
            if cmd >= 0x1000:
                cmd -= 0x2000
            out.append((phi, enc, state, cmd))
            i += 10
        else:
            i += 1
    return out


class Reader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self.buf = b''
        self.frames = collections.deque(maxlen=2000)
        self.n_ok = 0
        self.n_bad = 0
        self.stop = False
        self.t0 = time.time()

    def run(self):
        while not self.stop:
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            self.buf += chunk
            fs = parse_frames(self.buf)
            if fs:
                now = time.time() - self.t0
                for f in fs:
                    self.frames.append((now,) + f)
                    self.n_ok += 1
                k = self.buf.rfind(TAIL)
                self.buf = self.buf[k + 2:] if k >= 0 else self.buf[-16:]
            else:
                if len(self.buf) > 4096:
                    self.n_bad += len(self.buf) - 16
                    self.buf = self.buf[-16:]


def main():
    global WINDOW_S, PHI_Y_FLOOR, ARM_Y_LIM, ARM_Y_MODE, TRAJ_PERIOD_S
    argv = sys.argv[1:]
    test_mode = '--test' in argv

    def take_opt(name):
        """取出形如 --window 10 的选项，返回其值并从 argv 删除"""
        if name in argv:
            i = argv.index(name)
            if i + 1 < len(argv):
                v = argv[i + 1]
                del argv[i:i + 2]
                return v
        return None

    _w = take_opt('--window')
    if _w:
        WINDOW_S = float(_w)
    _y = take_opt('--ymax')
    if _y:
        PHI_Y_FLOOR = float(_y)
    _a = take_opt('--armmax')
    if _a:
        ARM_Y_LIM = float(_a)
        ARM_Y_MODE = 'fix'
    _af = take_opt('--armfix')          # 显式指定固定量程, 例如 --armfix 200
    if _af:
        ARM_Y_LIM = float(_af)
        ARM_Y_MODE = 'fix'
    _t = take_opt('--xtick')
    xtick = float(_t) if _t else X_TICK_S
    _p = take_opt('--period')
    if _p:
        TRAJ_PERIOD_S = float(_p)
    _c = take_opt('--cycles')           # 便捷: --cycles 2 等价于 --window 2*period
    if _c:
        WINDOW_S = float(_c) * TRAJ_PERIOD_S
        if not _t:
            xtick = max(1.0, round(TRAJ_PERIOD_S / 10.0))

    args = [a for a in argv if not a.startswith('--')]
    port = args[0] if args else find_port()
    if not port:
        print('找不到串口'); return 1
    print('打开 %s @ %d' % (port, BAUD))
    rd = Reader(port)
    rd.start()
    time.sleep(1.0)

    if test_mode:
        time.sleep(3.0)
        rd.stop = True
        time.sleep(0.2)
        print('解析成功 %d 帧, 丢弃字节 %d, 缓冲 %d 帧' % (rd.n_ok, rd.n_bad, len(rd.frames)))
        if rd.frames:
            t, phi, enc, st, cmd = rd.frames[-1]
            print('最后一帧: t=%.2fs phi=%.1f° enc=%d state=%d cmd=%d' % (t, phi * LSB_DEG, enc, st, cmd))
            states = collections.Counter(f[3] for f in rd.frames)
            print('状态分布:', dict(states))
        return 0

    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib import rcParams
    rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
    rcParams['axes.unicode_minus'] = False

    # 同时把遥测写成 CSV, 便于事后分析
    import csv as _csv
    _csvpath = 'live_%s.csv' % time.strftime('%H%M%S')
    _csvf = open(_csvpath, 'w', newline='', encoding='utf-8')
    _cw = _csv.writer(_csvf); _cw.writerow(['t_ms', 'phi', 'enc', 'state', 'cmd'])
    _rows = []                 # 批量缓冲: 每 200 帧刷一次盘
    print('同时记录到: ' + _csvpath)
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(FIG_W, FIG_H), sharex=True)
    fig.canvas.manager.set_window_title('旋转倒立摆 实时轨迹')
    # 横轴刻度
    from matplotlib.ticker import MultipleLocator
    ax2.xaxis.set_major_locator(MultipleLocator(xtick))
    ax2.xaxis.set_minor_locator(MultipleLocator(xtick / 5.0))
    l1, = ax1.plot([], [], lw=1.2, color='#2980b9')
    l2, = ax2.plot([], [], lw=0.9, color='#c0392b')
    ax1.set_ylabel('摆臂角度 (度)'); ax1.grid(alpha=0.3, which='both')
    ax1.set_title('摆臂位置 —— 轨迹模式(SW3)时应呈 %.0f 秒正弦' % TRAJ_PERIOD_S)
    # fix 模式在这里固定纵轴; auto 模式在循环里随数据缩放
    if ARM_Y_MODE == 'fix':
        ax1.set_ylim(-ARM_Y_LIM, ARM_Y_LIM)
        ax1.yaxis.set_major_locator(MultipleLocator(ARM_Y_TICK))
    ax1.axhline(0, ls='-', c='gray', lw=0.6, alpha=0.6)
    ax2.set_ylabel('摆杆角度 (度)'); ax2.set_xlabel('时间 (秒)'); ax2.grid(alpha=0.3, which='both')
    ax2.axhline(10, ls='--', c='gray', lw=0.7); ax2.axhline(-10, ls='--', c='gray', lw=0.7)
    ax2.set_title('摆杆倾角 —— 全程应在 ±10° 内')
    txt = ax2.text(0.01, 0.05, '', transform=ax2.transAxes, fontsize=10,
                   bbox=dict(facecolor='w', alpha=0.8, edgecolor='gray'))

    enc_prev = None
    arm_accum = 0.0
    tbuf = collections.deque()
    abuf = collections.deque()
    pbuf = collections.deque()
    k = 0
    last_draw = 0.0
    t_last = None

    try:
        while plt.fignum_exists(fig.number):
            if not rd.frames:
                plt.pause(0.02); continue
            while rd.frames:
                t, phi, enc, st, cmd = rd.frames.popleft()
                t_last = t
                # 编码器解卷绕
                if enc_prev is not None:
                    d = enc - enc_prev
                    if d > 32768: d -= 65536
                    elif d < -32768: d += 65536
                    arm_accum += d
                enc_prev = enc
                _rows.append([int(t * 1000), int(phi), int(enc), int(st), int(cmd)])
                if len(_rows) >= 200:
                    _cw.writerows(_rows); _rows.clear()
                k += 1
                if k % DECIMATE: continue
                tbuf.append(t); abuf.append(arm_accum * 360.0 / REF_PER_REV); pbuf.append(phi * LSB_DEG)
            # 裁剪窗口
            while tbuf and t_last - tbuf[0] > WINDOW_S:
                tbuf.popleft(); abuf.popleft(); pbuf.popleft()
            now = time.time()
            if now - last_draw < 0.05:
                plt.pause(0.005); continue
            last_draw = now
            if len(tbuf) > 2:
                ta = np.fromiter(tbuf, float); aa = np.fromiter(abuf, float); pp = np.fromiter(pbuf, float)
                l1.set_data(ta, aa); l2.set_data(ta, pp)
                ax1.set_xlim(ta[0], ta[-1] + 0.1)
                # 摆臂纵轴: auto 模式随数据缩放(永远看得见); fix 模式已固定, 这里不动
                if ARM_Y_MODE == 'auto':
                    a_lo, a_hi = float(aa.min()), float(aa.max())
                    pad = max((a_hi - a_lo) * ARM_Y_PAD, 2.0)
                    y_lo = np.floor((a_lo - pad) / ARM_Y_STEP) * ARM_Y_STEP
                    y_hi = np.ceil((a_hi + pad) / ARM_Y_STEP) * ARM_Y_STEP
                    if y_hi - y_lo < ARM_Y_MIN_H:
                        c = 0.5 * (y_lo + y_hi)
                        y_lo, y_hi = c - ARM_Y_MIN_H / 2.0, c + ARM_Y_MIN_H / 2.0
                    if ax1.get_ylim() != (y_lo, y_hi):
                        ax1.set_ylim(y_lo, y_hi)
                # 摆杆纵轴: 平时 ±PHI_Y_FLOOR 占满, 掉落时自动放大 (吸附 2° 防抖动)
                p_max = max(PHI_Y_FLOOR, np.ceil(float(np.abs(pp).max()) * 1.15 / 2.0) * 2.0)
                if abs(ax2.get_ylim()[1] - p_max) > 1e-6:
                    ax2.set_ylim(-p_max, p_max)
                txt.set_text('|phi|均 %.2f°   最大 %.1f°   摆臂峰峰 %.0f°'
                             % (np.abs(pp).mean(), np.abs(pp).max(), aa.max() - aa.min()))
                fig.canvas.draw_idle()
            plt.pause(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        rd.stop = True
        try:
            if _rows: _cw.writerows(_rows); _rows.clear()
            _csvf.flush(); _csvf.close(); print('CSV 已保存: ' + _csvpath)
        except Exception: pass
        plt.ioff()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
