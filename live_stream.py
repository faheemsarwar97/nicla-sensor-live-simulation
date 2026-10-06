"""
BLACKBOX CHALLENGE — Live BLE Stream Viewer
FH Kufstein Tirol / SPS.BBM.24
Board: Arduino Nicla Sense ME (ABX00050)

Connects to the Nicla, sends CMD_STREAM (0x02), then shows a rolling
4-second dashboard with:
  • 3D Orientation  — rotating board model driven by BHI260AP quaternion
  • Phase banner    — IDLE / FREE-FALL / IMPACT / SETTLE  (large, colour-coded)
  • Accel X/Y/Z     — real-time waveform
  • Gyro  X/Y/Z     — real-time waveform
  • Resultant |g|   — with free-fall / impact threshold lines
  • Live stats panel

On exit: auto-saves CSV + opens the 7-panel post-analysis dashboard.

Usage:
    pip install bleak matplotlib numpy
    python live_stream.py
"""

import asyncio
import struct
import threading
import time
import sys
import os
import csv
import signal
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D                   # noqa: F401 (registers projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from bleak import BleakClient, BleakScanner

# ──────────────────────────────────────────────────────────────
#  BLE UUIDs
# ──────────────────────────────────────────────────────────────
DEVICE_NAME = "Blackbox-1"
STATUS_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
DATA_UUID   = "19B10003-E8F2-537E-4F6C-D104768A1214"
CTRL_UUID   = "19B10004-E8F2-537E-4F6C-D104768A1214"

CMD_STREAM = b'\x02'

STATUS_NAMES = {
    0x00: "IDLE", 0x01: "FREE-FALL", 0x02: "RECORDING",
    0x03: "READY", 0x04: "SENDING", 0x05: "STREAMING",
}

# ──────────────────────────────────────────────────────────────
#  STREAM PACKET FORMAT — must match firmware StreamPkt (20 bytes)
#  int16: ax, ay, az, gx, gy, gz, qx, qy, qz, qw
# ──────────────────────────────────────────────────────────────
STREAM_FMT  = "<hhhhhhhhhh"                   # 10 × int16
STREAM_SIZE = struct.calcsize(STREAM_FMT)     # 20 bytes

ACCEL_SCALE = 4096.0    # BHI260AP Q12: 1 g = 4096 LSB
GYRO_SCALE  = 16.0      # approx deg/s (raw ÷ 16)
QUAT_SCALE  = 10000.0   # quaternion float×10000 packed as int16

# ──────────────────────────────────────────────────────────────
#  ROLLING WINDOW CONFIG
# ──────────────────────────────────────────────────────────────
WINDOW_MS   = 4000
SAMPLE_RATE = 200
MAX_SAMPLES = int(WINDOW_MS * SAMPLE_RATE / 1000) + 100

FF_THRESH_G  = 0.25
IMP_THRESH_G = 4.00
SMOOTH_N     = 20   # samples to inspect for phase detection (~100 ms at 200 Hz)

# ──────────────────────────────────────────────────────────────
#  THREAD-SAFE BUFFERS
# ──────────────────────────────────────────────────────────────
lock   = threading.Lock()

buf_t  = deque(maxlen=MAX_SAMPLES)
buf_ax = deque(maxlen=MAX_SAMPLES)
buf_ay = deque(maxlen=MAX_SAMPLES)
buf_az = deque(maxlen=MAX_SAMPLES)
buf_R  = deque(maxlen=MAX_SAMPLES)
buf_gx = deque(maxlen=MAX_SAMPLES)
buf_gy = deque(maxlen=MAX_SAMPLES)
buf_gz = deque(maxlen=MAX_SAMPLES)

# Quaternion: keep only latest (we don't need a history for 3D view)
latest_quat = [0.0, 0.0, 0.0, 1.0]   # [qx, qy, qz, qw] — identity

# Full recording for CSV
all_ts = []; all_ax = []; all_ay = []; all_az = []
all_gx = []; all_gy = []; all_gz = []
all_qx = []; all_qy = []; all_qz = []; all_qw = []

ble_status    = {"code": 0xFF, "text": "Connecting…"}
ble_connected = threading.Event()
ble_stop      = threading.Event()

t0_ms = None


# ──────────────────────────────────────────────────────────────
#  BLE CALLBACKS
# ──────────────────────────────────────────────────────────────
def on_sample(sender, data: bytearray):
    global t0_ms
    if len(data) < STREAM_SIZE:
        return
    ax_r, ay_r, az_r, gx_r, gy_r, gz_r, qx_r, qy_r, qz_r, qw_r = \
        struct.unpack(STREAM_FMT, data[:STREAM_SIZE])

    ax = ax_r / ACCEL_SCALE;  ay = ay_r / ACCEL_SCALE;  az = az_r / ACCEL_SCALE
    gx = gx_r / GYRO_SCALE;   gy = gy_r / GYRO_SCALE;   gz = gz_r / GYRO_SCALE
    qx = qx_r / QUAT_SCALE;   qy = qy_r / QUAT_SCALE
    qz = qz_r / QUAT_SCALE;   qw = qw_r / QUAT_SCALE
    R  = np.sqrt(ax*ax + ay*ay + az*az)

    ts = int(time.time() * 1000)
    with lock:
        if t0_ms is None:
            t0_ms = ts
        t_rel = ts - t0_ms
        buf_t.append(t_rel)
        buf_ax.append(ax); buf_ay.append(ay); buf_az.append(az)
        buf_R.append(R)
        buf_gx.append(gx); buf_gy.append(gy); buf_gz.append(gz)
        latest_quat[0] = qx; latest_quat[1] = qy
        latest_quat[2] = qz; latest_quat[3] = qw

        all_ts.append(ts)
        all_ax.append(ax); all_ay.append(ay); all_az.append(az)
        all_gx.append(gx); all_gy.append(gy); all_gz.append(gz)
        all_qx.append(qx); all_qy.append(qy); all_qz.append(qz); all_qw.append(qw)


def on_status(sender, data: bytearray):
    if len(data) >= 1:
        code = data[0]
        ble_status["code"] = code
        ble_status["text"] = STATUS_NAMES.get(code, f"0x{code:02X}")
        print(f"\r[BLE] Status → {ble_status['text']}        ", end="", flush=True)


# ──────────────────────────────────────────────────────────────
#  BLE ASYNC
# ──────────────────────────────────────────────────────────────
async def ble_run():
    print(f"[BLE] Scanning for '{DEVICE_NAME}'…")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print(f"[BLE] '{DEVICE_NAME}' not found — powered on?")
        ble_stop.set(); return

    print(f"[BLE] Found: {device.address}")
    async with BleakClient(device, use_cached=False) as client:
        print("[BLE] Connected ✓ — discovering services…")
        for svc in client.services:
            print(f"  [SVC] {svc.uuid}")
            for ch in svc.characteristics:
                print(f"    [CHR] {ch.uuid}  props={ch.properties}")

        all_chars = {str(ch.uuid).lower(): ch
                     for svc in client.services for ch in svc.characteristics}

        status_uuid = STATUS_UUID.lower()
        data_uuid   = DATA_UUID.lower()
        ctrl_uuid   = CTRL_UUID.lower()

        if status_uuid not in all_chars:
            print("[BLE] STATUS char not found — stale GATT cache?")
            print("[BLE] Windows Bluetooth settings → remove 'Blackbox-1' → re-run.")
            ble_stop.set(); return

        await client.start_notify(status_uuid, on_status)
        await client.start_notify(data_uuid,   on_sample)
        await client.write_gatt_char(ctrl_uuid, CMD_STREAM)
        print("[BLE] CMD_STREAM sent — live data (20-byte packets)")
        ble_connected.set()

        while not ble_stop.is_set():
            if not client.is_connected:
                print("\n[BLE] Disconnected.")
                break
            await asyncio.sleep(0.05)

        try:
            await client.stop_notify(data_uuid)
            await client.stop_notify(status_uuid)
        except Exception:
            pass

    ble_stop.set()
    print("\n[BLE] Session ended.")


def ble_thread_fn():
    asyncio.run(ble_run())


# ──────────────────────────────────────────────────────────────
#  COLOURS
# ──────────────────────────────────────────────────────────────
DARK_BG  = "#0d1117"
CARD_BG  = "#161b22"
GRID_C   = "#21262d"
TEXT_C   = "#c9d1d9"
DIM_C    = "#8b949e"

COL_X    = "#79c0ff"
COL_Y    = "#56d364"
COL_Z    = "#f78166"
COL_R    = "#e3b341"
COL_GX   = "#d2a8ff"
COL_GY   = "#ffa657"
COL_GZ   = "#39d353"

COL_IDLE = "#3fb950"
COL_FF   = "#388bfd"
COL_IMP  = "#f85149"
COL_SET  = "#e3b341"

PHASE_CFG = {
    "IDLE":      (COL_IDLE, "●  IDLE",      "Device at rest"),
    "FREE-FALL": (COL_FF,   "↓  FREE-FALL", "Falling…"),
    "IMPACT":    (COL_IMP,  "✦  IMPACT",    "Impact detected!"),
    "SETTLE":    (COL_SET,  "~  SETTLING",  "Coming to rest"),
}


# Impact latch: once triggered, hold IMPACT badge for this many ms
IMPACT_LATCH_MS  = 1500
_impact_latch_until = 0.0   # absolute time.time()*1000 until latch expires


def phase_of_smart(R_deque):
    """
    Catches brief impact spikes that the 60 ms animation interval misses.

    Logic (priority order):
      IMPACT   — MAX of last SMOOTH_N samples > IMP_THRESH_G → latch 1.5 s
      IMPACT   — latch still active from a recent spike
      FREE-FALL— MIN of last SMOOTH_N samples < FF_THRESH_G
      SETTLE   — median < 0.80 g
      IDLE     — otherwise
    Returns (phase_str, R_median).
    """
    global _impact_latch_until
    now_ms = time.time() * 1000
    recent = list(R_deque)[-SMOOTH_N:]
    if not recent:
        return "IDLE", 1.0

    R_max = max(recent)
    R_min = min(recent)
    R_med = sorted(recent)[len(recent) // 2]

    if R_max > IMP_THRESH_G:
        _impact_latch_until = now_ms + IMPACT_LATCH_MS
        return "IMPACT", R_med

    if now_ms < _impact_latch_until:
        return "IMPACT", R_med

    if R_min < FF_THRESH_G:
        return "FREE-FALL", R_med

    if R_med < 0.80:
        return "SETTLE", R_med

    return "IDLE", R_med


# ──────────────────────────────────────────────────────────────
#  3D BOX GEOMETRY
# ──────────────────────────────────────────────────────────────
# Nicla Sense ME is ~22×22×5mm — render as a slightly flat square board
_W, _H, _D = 0.78, 0.78, 0.18
BOX_VERTS = np.array([
    [-_W, -_H, -_D], [ _W, -_H, -_D], [ _W,  _H, -_D], [-_W,  _H, -_D],
    [-_W, -_H,  _D], [ _W, -_H,  _D], [ _W,  _H,  _D], [-_W,  _H,  _D],
], dtype=float)

FACE_IDX = [
    [0, 1, 2, 3],   # bottom (Z−)
    [4, 5, 6, 7],   # top    (Z+) — component side
    [0, 1, 5, 4],   # front  (Y−)
    [3, 2, 6, 7],   # back   (Y+)
    [0, 3, 7, 4],   # left   (X−)
    [1, 2, 6, 5],   # right  (X+)
]
# Top face brighter (component side), sides darker
FACE_COLORS_BASE = [
    "#102030",  # bottom
    "#1f6feb",  # top — bright blue (the board face)
    "#0d2035",  # front
    "#0d2035",  # back
    "#0f2840",  # left
    "#0f2840",  # right
]
FACE_COLORS_IMP = [
    "#200808",
    "#8b1a1a",
    "#200808",
    "#200808",
    "#200808",
    "#200808",
]


def quat_to_matrix(qx, qy, qz, qw):
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if n < 1e-6:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),      1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)],
    ])


def draw_board_3d(ax3d, qx, qy, qz, qw, phase="IDLE"):
    """Clear and redraw the 3D board model each frame."""
    ax3d.cla()
    ax3d.set_facecolor(DARK_BG)
    ax3d.set_xlim(-1.3, 1.3)
    ax3d.set_ylim(-1.3, 1.3)
    ax3d.set_zlim(-1.3, 1.3)
    ax3d.set_axis_off()
    ax3d.set_title("3D Orientation — BHI260AP Rotation Vector",
                   color=TEXT_C, fontsize=8.5, pad=3)

    R = quat_to_matrix(qx, qy, qz, qw)
    rv = (R @ BOX_VERTS.T).T  # shape (8, 3)

    faces_coords = [[rv[i] for i in idx] for idx in FACE_IDX]
    fc = FACE_COLORS_IMP if phase == "IMPACT" else FACE_COLORS_BASE

    poly = Poly3DCollection(faces_coords, alpha=0.88,
                            linewidths=0.7, edgecolors="#58a6ff")
    poly.set_facecolor(fc)
    ax3d.add_collection3d(poly)

    # Draw board-frame axes (X=red, Y=green, Z=blue)
    scale = 1.05
    for vec, col in [(R[:, 0], '#f85149'), (R[:, 1], '#56d364'), (R[:, 2], '#79c0ff')]:
        ax3d.quiver(0, 0, 0,
                    vec[0]*scale, vec[1]*scale, vec[2]*scale,
                    color=col, linewidth=1.8, arrow_length_ratio=0.22)

    # World-up reference (faint white arrow)
    ax3d.quiver(0, 0, -1.0, 0, 0, 0.5,
                color='#ffffff', linewidth=0.6, alpha=0.3, arrow_length_ratio=0.3)

    # Euler angles from quaternion for label
    sinr = 2*(qw*qx + qy*qz);   cosr = 1 - 2*(qx**2 + qy**2)
    roll  = np.degrees(np.arctan2(sinr, cosr))
    sinp  = 2*(qw*qy - qz*qx)
    pitch = np.degrees(np.arcsin(np.clip(sinp, -1, 1)))
    siny  = 2*(qw*qz + qx*qy);   cosy = 1 - 2*(qy**2 + qz**2)
    yaw   = np.degrees(np.arctan2(siny, cosy))

    ax3d.text2D(0.02, 0.05,
                f"R={roll:+.0f}°  P={pitch:+.0f}°  Y={yaw:+.0f}°",
                transform=ax3d.transAxes,
                color=DIM_C, fontsize=7.5, family="monospace")


# ──────────────────────────────────────────────────────────────
#  FIGURE SETUP
# ──────────────────────────────────────────────────────────────
def setup_figure():
    fig = plt.figure(figsize=(16, 9), facecolor=DARK_BG)
    fig.suptitle("Blackbox Live Stream  ·  Nicla Sense ME  ·  BHI260AP",
                 color=TEXT_C, fontsize=13, fontweight="bold", y=0.98)

    # Layout (3 rows × 2 cols):
    #   Row 0: Phase banner  |  3D Orientation  ← tall row
    #   Row 1: Accel XYZ     |  Gyro XYZ
    #   Row 2: Resultant |g| |  Stats
    gs = gridspec.GridSpec(3, 2,
                           height_ratios=[2.0, 1.5, 1.5],
                           hspace=0.45, wspace=0.28,
                           left=0.06, right=0.97, top=0.93, bottom=0.06)

    ax_phase = fig.add_subplot(gs[0, 0])                    # Phase banner (2D)
    ax_3d    = fig.add_subplot(gs[0, 1], projection='3d')   # 3D orientation
    ax_acc   = fig.add_subplot(gs[1, 0])                    # Accel XYZ
    ax_gyro  = fig.add_subplot(gs[1, 1])                    # Gyro XYZ
    ax_R     = fig.add_subplot(gs[2, 0])                    # Resultant |g|
    ax_stats = fig.add_subplot(gs[2, 1])                    # Stats

    # ── Style data axes ──────────────────────────────────────
    for ax in (ax_acc, ax_gyro, ax_R):
        ax.set_facecolor(DARK_BG)
        ax.tick_params(colors=TEXT_C, labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor(GRID_C)
        ax.grid(True, color=GRID_C, linewidth=0.5, linestyle=":")
        ax.set_xlabel("Time (ms)", color=TEXT_C, fontsize=8)

    ax_acc.set_title  ("Acceleration  X / Y / Z", color=TEXT_C, fontsize=9, pad=4)
    ax_gyro.set_title ("Gyroscope  X / Y / Z",    color=TEXT_C, fontsize=9, pad=4)
    ax_R.set_title    ("Resultant  |g|",           color=TEXT_C, fontsize=9, pad=4)
    ax_acc.set_ylabel ("g",     color=TEXT_C, fontsize=8)
    ax_gyro.set_ylabel("deg/s", color=TEXT_C, fontsize=8)
    ax_R.set_ylabel   ("g",     color=TEXT_C, fontsize=8)

    # ── Phase banner ─────────────────────────────────────────
    ax_phase.set_facecolor(CARD_BG)
    ax_phase.axis("off")

    phase_rect = mpatches.FancyBboxPatch(
        (0.03, 0.05), 0.94, 0.90,
        boxstyle="round,pad=0.02",
        transform=ax_phase.transAxes,
        facecolor=CARD_BG, edgecolor=GRID_C, linewidth=1.5
    )
    ax_phase.add_patch(phase_rect)

    phase_txt = ax_phase.text(0.5, 0.60, "●  IDLE",
        transform=ax_phase.transAxes,
        color=COL_IDLE, fontsize=28, fontweight="bold",
        ha="center", va="center", family="monospace")
    phase_sub = ax_phase.text(0.5, 0.28, "Device at rest",
        transform=ax_phase.transAxes,
        color=DIM_C, fontsize=11, ha="center", va="center")
    R_lbl = ax_phase.text(0.5, 0.88, "|g| = 1.000",
        transform=ax_phase.transAxes,
        color=TEXT_C, fontsize=9.5, ha="center", va="center",
        family="monospace")

    # ── Stats ────────────────────────────────────────────────
    ax_stats.set_facecolor(DARK_BG)
    ax_stats.axis("off")
    stats_txt = ax_stats.text(
        0.04, 0.97, "",
        transform=ax_stats.transAxes,
        color=TEXT_C, fontsize=8.5, va="top", ha="left",
        family="monospace",
        bbox=dict(boxstyle="round,pad=0.4", facecolor=CARD_BG,
                  edgecolor=GRID_C, alpha=0.9)
    )

    # ── Lines ────────────────────────────────────────────────
    lax, = ax_acc.plot([], [], color=COL_X,  lw=1.2, label="X")
    lay, = ax_acc.plot([], [], color=COL_Y,  lw=1.2, label="Y")
    laz, = ax_acc.plot([], [], color=COL_Z,  lw=1.2, label="Z")
    lgx, = ax_gyro.plot([], [], color=COL_GX, lw=1.0, label="Gx")
    lgy, = ax_gyro.plot([], [], color=COL_GY, lw=1.0, label="Gy")
    lgz, = ax_gyro.plot([], [], color=COL_GZ, lw=1.0, label="Gz")
    lr,  = ax_R.plot([], [], color=COL_R, lw=1.5)

    for a, loc in [(ax_acc, "upper left"), (ax_gyro, "upper left")]:
        a.legend(loc=loc, fontsize=7, facecolor=CARD_BG,
                 edgecolor=GRID_C, labelcolor=TEXT_C, framealpha=0.8)

    # Threshold lines on Resultant panel
    ax_R.axhline(FF_THRESH_G,  color=COL_FF,  lw=0.7, linestyle="--", alpha=0.7)
    ax_R.axhline(IMP_THRESH_G, color=COL_IMP, lw=0.7, linestyle="--", alpha=0.7)
    ax_R.text(0.01, FF_THRESH_G  + 0.06, f"FF < {FF_THRESH_G}g",
              color=COL_FF, fontsize=7, transform=ax_R.get_yaxis_transform())
    ax_R.text(0.01, IMP_THRESH_G + 0.06, f"IMP > {IMP_THRESH_G}g",
              color=COL_IMP, fontsize=7, transform=ax_R.get_yaxis_transform())

    banners = (phase_rect, phase_txt, phase_sub, R_lbl)
    lines   = (lax, lay, laz, lgx, lgy, lgz, lr)
    axes    = (ax_phase, ax_3d, ax_acc, ax_gyro, ax_R, ax_stats)
    return fig, axes, lines, banners, stats_txt


# ──────────────────────────────────────────────────────────────
#  ANIMATION UPDATE
# ──────────────────────────────────────────────────────────────
def make_update(fig, axes, lines, banners, stats_txt):
    ax_phase, ax_3d, ax_acc, ax_gyro, ax_R, _ = axes
    lax, lay, laz, lgx, lgy, lgz, lr          = lines
    phase_rect, phase_txt, phase_sub, R_lbl    = banners
    _fc = [0]  # frame counter for 3D throttle

    def update(_frame):
        _fc[0] += 1
        try:
            return _update_inner(_frame)
        except Exception as e:
            print(f"\n[ANIM] frame error (ignored): {e}")
            return lines

    def _update_inner(_frame):
        with lock:
            if len(buf_t) < 2:
                return lines

            t   = np.array(buf_t)
            ax_ = np.array(buf_ax)
            ay_ = np.array(buf_ay)
            az_ = np.array(buf_az)
            R_  = np.array(buf_R)
            gx_ = np.array(buf_gx)
            gy_ = np.array(buf_gy)
            gz_ = np.array(buf_gz)
            qx, qy, qz, qw = latest_quat

        # Window slice
        t_rel = t - t[-1]
        mask  = t_rel >= -WINDOW_MS
        if not mask.any():
            return lines
        sl  = slice(int(np.argmax(mask)), None)
        tw  = t_rel[sl]
        aw  = ax_[sl];  ayw = ay_[sl]; azw = az_[sl]
        Rw  = R_[sl]
        gxw = gx_[sl]; gyw = gy_[sl]; gzw = gz_[sl]

        # Update 2D lines
        lax.set_data(tw, aw);   lay.set_data(tw, ayw); laz.set_data(tw, azw)
        lgx.set_data(tw, gxw);  lgy.set_data(tw, gyw); lgz.set_data(tw, gzw)
        lr.set_data (tw, Rw)

        # Axis limits
        for a in (ax_acc, ax_gyro, ax_R):
            a.set_xlim(tw[0], tw[-1] + 1)

        acc_ext = max(abs(aw).max(), abs(ayw).max(), abs(azw).max())
        ax_acc.set_ylim(-max(1.5, acc_ext + 0.3), max(2.0, acc_ext + 0.3))

        g_ext = max(abs(gxw).max(), abs(gyw).max(), abs(gzw).max())
        ax_gyro.set_ylim(-max(30, g_ext*1.15), max(30, g_ext*1.15))

        ax_R.set_ylim(-0.1, max(2.5, Rw.max()*1.15))

        # Phase — uses max/min/latch so brief spikes are never missed
        phase, R_sm = phase_of_smart(buf_R)
        col, lbl, sub = PHASE_CFG[phase]

        phase_txt.set_text(lbl)
        phase_txt.set_color(col)
        phase_sub.set_text(sub)
        R_lbl.set_text(f"|g| = {Rw[-1]:.3f}  (win-max {max(list(buf_R)[-SMOOTH_N:] or [0]):.2f})")

        bg = "#1a0505" if phase == "IMPACT" else CARD_BG
        ax_phase.set_facecolor(bg)
        phase_rect.set_facecolor(bg)

        # ── 3D board (throttled: every 5th frame ≈ 300 ms) ───
        if _fc[0] % 5 == 0:
            draw_board_3d(ax_3d, qx, qy, qz, qw, phase)

        # Stats
        gyro_mag = np.sqrt(gxw[-1]**2 + gyw[-1]**2 + gzw[-1]**2)
        ff_ms    = (Rw < FF_THRESH_G).sum() * (1000 / max(SAMPLE_RATE, 1))

        # Euler from latest quat
        sinr  = 2*(qw*qx + qy*qz);   cosr = 1 - 2*(qx**2 + qy**2)
        roll  = np.degrees(np.arctan2(sinr, cosr))
        sinp  = np.clip(2*(qw*qy - qz*qx), -1, 1)
        pitch = np.degrees(np.arcsin(sinp))
        siny  = 2*(qw*qz + qx*qy);   cosy = 1 - 2*(qy**2 + qz**2)
        yaw   = np.degrees(np.arctan2(siny, cosy))

        stats = [
            f"Status : {ble_status['text']}",
            f"Phase  : {phase}",
            f"",
            f"Cur |g|: {Rw[-1]:+.3f} g",
            f"Max |g|: {Rw.max():.3f} g",
            f"FF dur : {ff_ms:.0f} ms",
            f"",
            f"Acc X  : {aw[-1]:+.3f} g",
            f"Acc Y  : {ayw[-1]:+.3f} g",
            f"Acc Z  : {azw[-1]:+.3f} g",
            f"",
            f"Gyr X  : {gxw[-1]:+.1f}°/s",
            f"Gyr Y  : {gyw[-1]:+.1f}°/s",
            f"Gyr Z  : {gzw[-1]:+.1f}°/s",
            f"|ω|    : {gyro_mag:.1f}°/s",
            f"",
            f"Roll   : {roll:+.1f}°",
            f"Pitch  : {pitch:+.1f}°",
            f"Yaw    : {yaw:+.1f}°",
            f"",
            f"Samples: {len(all_ts)}",
        ]
        stats_txt.set_text("\n".join(stats))
        return lines

    return update


# ──────────────────────────────────────────────────────────────
#  SAVE + DASHBOARD ON EXIT
# ──────────────────────────────────────────────────────────────
def save_and_dashboard():
    if not all_ts:
        print("[LIVE] No data to save.")
        return

    base    = os.path.dirname(os.path.abspath(__file__))
    ts_str  = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_out = os.path.join(base, f"live_stream_{ts_str}.csv")

    ax_a = np.array(all_ax); ay_a = np.array(all_ay); az_a = np.array(all_az)
    R_a  = np.sqrt(ax_a**2 + ay_a**2 + az_a**2)
    t0   = all_ts[0]
    t_rel_arr = [ts - t0 for ts in all_ts]

    with open(csv_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms_abs", "t_ms_rel",
                    "ax_g", "ay_g", "az_g", "resultant_g",
                    "gx_dps", "gy_dps", "gz_dps",
                    "qx", "qy", "qz", "qw"])
        for i in range(len(all_ts)):
            w.writerow([
                all_ts[i], t_rel_arr[i],
                round(all_ax[i], 5), round(all_ay[i], 5), round(all_az[i], 5),
                round(float(R_a[i]), 5),
                round(all_gx[i], 3), round(all_gy[i], 3), round(all_gz[i], 3),
                round(all_qx[i], 5), round(all_qy[i], 5),
                round(all_qz[i], 5), round(all_qw[i], 5),
            ])
    print(f"\n[LIVE] CSV saved → {csv_out}")

    try:
        sys.path.insert(0, base)
        from visualizer import BlackboxData, plot_drop_data

        R_arr = np.array([np.sqrt(ax**2+ay**2+az**2)
                          for ax,ay,az in zip(all_ax,all_ay,all_az)])
        dur_s = (all_ts[-1]-all_ts[0])/1000.0 if len(all_ts)>1 else 1.
        actual_rate = len(all_ts)/dur_s if dur_s>0 else SAMPLE_RATE

        # ── Detect all distinct impact events ─────────────────
        min_gap = int(actual_rate * 1.5)   # 1.5 s between events
        events  = []
        last_i  = -min_gap
        for i, r in enumerate(R_arr):
            if r > IMP_THRESH_G and (i - last_i) > min_gap:
                events.append((i, r))
                last_i = i
        print(f"\n[LIVE] Drops detected: {len(events)}")
        for k, (pi, pr) in enumerate(events):
            t_s = (all_ts[pi] - all_ts[0]) / 1000.
            print(f"  Drop {k+1}: peak {pr:.1f} g  at t={t_s:.1f} s")

        # ── Extract ±6 s window around the LARGEST impact ─────
        if events:
            best_i = max(events, key=lambda e: e[1])[0]
            win    = int(6 * actual_rate)
            s0     = max(0, best_i - win)
            s1     = min(len(all_ts), best_i + win)
        else:
            s0, s1 = 0, len(all_ts)   # no impact → show all

        bd = BlackboxData()
        bd.ts_ms            = all_ts[s0:s1]
        bd.ax               = all_ax[s0:s1]
        bd.ay               = all_ay[s0:s1]
        bd.az               = all_az[s0:s1]
        bd.pressure         = [0.0] * (s1 - s0)
        bd.total_samples    = s1 - s0
        bd.sample_rate_hz   = int(actual_rate)
        bd.impact_offset_ms = 0

        title_str = (f"Live Stream Post-Analysis — largest of {len(events)} drop(s)"
                     if len(events)>1 else "Live Stream Post-Analysis")
        fig2 = plot_drop_data(bd, title=title_str)
        if fig2:
            png_out = csv_out.replace(".csv", ".png")
            fig2.savefig(png_out, dpi=150, bbox_inches="tight",
                         facecolor=fig2.get_facecolor())
            print(f"[LIVE] Dashboard → {png_out}")
            plt.show()
    except Exception as e:
        print(f"[LIVE] Dashboard failed: {e}")
        print(f"       Run:  python visualizer.py --load {csv_out}")


# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    t = threading.Thread(target=ble_thread_fn, daemon=True)
    t.start()

    print("[LIVE] Waiting for BLE connection…")
    if not ble_connected.wait(timeout=20):
        print("[LIVE] BLE connection timed out.")
        ble_stop.set()
        save_and_dashboard()
        return

    print("[LIVE] Opening dashboard…  (Ctrl+C or close window to stop)")

    fig, axes, lines, banners, stats_txt = setup_figure()
    update_fn = make_update(fig, axes, lines, banners, stats_txt)

    ani = animation.FuncAnimation(
        fig, update_fn,
        interval=100,        # 10 fps — gives event loop breathing room on impact
        blit=False,
        cache_frame_data=False,
    )

    def on_close(_event):
        ble_stop.set()

    fig.canvas.mpl_connect("close_event", on_close)

    def sigint_handler(sig, frame):
        ble_stop.set()
        plt.close("all")

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ble_stop.set()
        t.join(timeout=3)
        save_and_dashboard()


if __name__ == "__main__":
    main()
