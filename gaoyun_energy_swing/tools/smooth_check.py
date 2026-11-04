# -*- coding: utf-8 -*-
"""
==============================================================================
 smooth_check.py —— 轨迹正弦平滑度评估
----------------------------------------------------------------------------
 输入: live_plot.py 或 traj_test2.ps1 产出的 CSV
       列: t_ms/t_s, phi, enc, state, cmd  (分隔符自动识别 , 或 tab)
 输出:
   1) 摆杆高频颤振强度: 4~7 Hz 频段能量占比 + 高频 RMS
   2) 摆臂正弦拟合质量: R2、1/2/3/4 次谐波幅度、残差 RMS
   3) 平滑度评分: 越高越好
 用法:
   python smooth_check.py traj_kd90.csv traj_kd60.csv
==============================================================================
"""
import sys, csv
import numpy as np

FS = 1.0 / 0.003        # 333 Hz
LSB_DEG = 180.0 / 2048.0
CNT2DEG = 360.0 / 1138.0


def load(fn):
    with open(fn, encoding='utf-8-sig') as f:
        head = f.readline()
    sep = '\t' if '\t' in head else ','
    R = list(csv.DictReader(open(fn, encoding='utf-8-sig'), delimiter=sep))
    if not R:
        return None
    k = R[0].keys()
    tk = 't_ms' if 't_ms' in k else 't_s'
    scale = 0.001 if tk == 't_ms' else 1.0
    t = np.array([float(r[tk]) * scale for r in R])
    if tk == 't_ms':
        t = t - t[0]
    t = t - t[0]
    phi = np.array([float(r['phi']) for r in R]) * LSB_DEG
    enc = np.array([float(r['enc']) for r in R])
    st = np.array([int(r['state']) for r in R])
    cmd = np.array([float(r['cmd']) for r in R]) if 'cmd' in k else np.zeros(len(R))
    d = np.diff(enc)
    d[d > 32768] -= 65536
    d[d < -32768] += 65536
    arm = np.concatenate([[0], np.cumsum(d)]) * CNT2DEG
    return t, phi, arm, st, cmd


def _sine_r2(tt, yy, T=20.0):
    ww = 2 * np.pi / T
    X = np.column_stack([np.ones(len(tt)), np.cos(ww * tt), np.sin(ww * tt), tt])
    c, *_ = np.linalg.lstsq(X, yy, rcond=None)
    pred = X @ c
    den = ((yy - yy.mean()) ** 2).sum()
    if den < 1e-9:
        return -9.0, 0.0
    return 1 - ((yy - pred) ** 2).sum() / den, float(np.hypot(c[1], c[2]))


def auto_best_window(t, phi, arm, st, win=40.0):
    """滑动找【20 秒正弦拟合最好】的窗口（同时要求平衡率 >= 0.9）
       判据换了: 原来按'摆臂行程最大'挑，会挑到整圈旋转的片段，对'圆不圆'毫无意义。"""
    n = len(t)
    step = max(1, int(2.0 / 0.003))          # 每 2 秒滑一次，速度快
    nn = int(win / 0.003)
    if nn >= n:
        return 0, n
    best = None
    for i in range(0, n - nn, step):
        j = i + nn
        if np.mean(st[i:j] == 2) < 0.90:
            continue
        tt = t[i:j] - t[i]
        yy = arm[i:j]
        r2, amp = _sine_r2(tt, yy)
        # 振幅太小说明摆臂没在跟轨迹，也排除
        if amp < 20.0:
            continue
        if best is None or r2 > best[0]:
            best = (r2, i, j)
    if best is None:
        return 0, min(n, nn)
    return best[1], best[2]


def analyze(fn):
    got = load(fn)
    if got is None:
        print('%-24s 无法读取' % fn)
        return None
    t, phi, arm, st, cmd = got
    print('=' * 78)
    print('文件: %s   时长 %.1f s   帧数 %d' % (fn, t[-1], len(t)))
    print('  状态: IDLE %.0f%%  SWING %.0f%%  BALANCE %.0f%%' % (
        100 * np.mean(st == 0), 100 * np.mean(st == 1), 100 * np.mean(st == 2)))
    if np.mean(st == 2) < 0.2:
        print('  ⚠ 平衡占比太低，无法评估'); return None

    i, j = auto_best_window(t, phi, arm, st)
    tt = t[i:j] - t[i]
    pp = phi[i:j]
    yy = arm[i:j]
    cc = cmd[i:j]
    print('  分析窗口: t=%.1f~%.1f s (%.0f s)  BAL %.1f%%' % (
        t[i], t[j - 1], t[j - 1] - t[i], 100 * np.mean(st[i:j] == 2)))

    # ---- 1) 摆杆高频颤振 ----
    w = np.hanning(len(pp))
    F = np.abs(np.fft.rfft((pp - pp.mean()) * w))
    fr = np.fft.rfftfreq(len(pp), 1.0 / FS)
    ac = (F[(fr > 0.05) & (fr < 50)] ** 2).sum()
    chatter = (F[(fr > 4) & (fr < 7)] ** 2).sum() / max(ac, 1e-12)
    lp = np.convolve(pp, np.ones(33) / 33, mode='same')
    hf_rms = float(np.sqrt(np.mean((pp - lp)[40:-40] ** 2)))
    fpk = fr[(fr > 4) & (fr < 7)][np.argmax(F[(fr > 4) & (fr < 7)])] if chatter > 0 else 0
    print('  【摆杆】|phi|均 %.2f°  |phi|max %.1f°' % (np.abs(pp).mean(), np.abs(pp).max()))
    print('         高频颤振: 峰值频 %.2f Hz   4~7Hz 能量占比 %.0f%%   高频 RMS %.2f°' % (
        fpk, 100 * chatter, hf_rms))

    # ---- 2) 摆臂正弦拟合 ----
    best = None
    for T in [20.0, 12.0, 10.0, 8.0, 30.0]:
        ww = 2 * np.pi / T
        X = np.column_stack([np.ones(len(tt)), np.cos(ww * tt), np.sin(ww * tt), tt])
        c, *_ = np.linalg.lstsq(X, yy, rcond=None)
        pred = X @ c
        r2 = 1 - ((yy - pred) ** 2).sum() / max(((yy - yy.mean()) ** 2).sum(), 1e-9)
        if best is None or r2 > best[0]:
            best = (r2, T, c, pred)
    r2, T, c, pred = best
    amp = float(np.hypot(c[1], c[2]))
    res = yy - pred
    print('  【摆臂】拟合周期 %.0f s   振幅 %.1f° (峰峰 %.0f°)   R2 = %.4f   残差RMS %.1f°' % (
        T, amp, 2 * amp, r2, float(res.std())))
    ww = 2 * np.pi / T
    hs = []
    for h in [1, 2, 3, 4]:
        Xh = np.column_stack([np.cos(h * ww * tt), np.sin(h * ww * tt)])
        ch, *_ = np.linalg.lstsq(Xh, yy, rcond=None)
        hs.append(float(np.hypot(*ch)))
    print('         谐波: 1次 %.1f°  2次 %.1f°  3次 %.1f°  4次 %.1f°  (2~4 次和越小越圆)' % tuple(hs))
    dist = 100.0 * np.sqrt(hs[1] ** 2 + hs[2] ** 2 + hs[3] ** 2) / max(hs[0], 1e-9)
    print('         形状畸变(2~4次/1次) = %.1f%%' % dist)
    print('  【指令】cmd std %.0f   sat(|cmd|>1900) %.1f%%' % (cc.std(), 100 * np.mean(np.abs(cc) > 1900)))

    # ---- 3) 评分 ----
    score = 100.0 * r2 - 0.5 * dist - 0.3 * (100 * chatter) - 0.5 * hf_rms * 10
    print('  【平滑度评分】%.1f  (R2 越高/畸变越小/颤振越弱 -> 越高)' % score)
    return dict(file=fn, T=T, amp=amp, r2=r2, dist=dist, chatter=100 * chatter,
                hf_rms=hf_rms, score=score, res=float(res.std()))


if __name__ == '__main__':
    files = sys.argv[1:]
    if not files:
        import glob
        files = sorted(glob.glob('traj_*.csv'))[-3:]
    rows = []
    for f in files:
        try:
            r = analyze(f)
            if r:
                rows.append(r)
        except Exception as e:
            print('%-24s 出错: %s' % (f, e))
    if len(rows) > 1:
        print('=' * 78)
        print('%-24s %7s %7s %7s %7s %7s' % ('文件', 'R2', '畸变%', '4-7Hz%', 'HF_RMS', '评分'))
        for r in rows:
            print('%-24s %7.4f %7.1f %7.0f %7.2f %7.1f' % (
                r['file'], r['r2'], r['dist'], r['chatter'], r['hf_rms'], r['score']))
