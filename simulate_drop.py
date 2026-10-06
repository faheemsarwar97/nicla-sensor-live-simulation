"""
BLACKBOX CHALLENGE — 3-Metre Drop Simulator
FH Kufstein Tirol / SPS.BBM.24

Generates physically realistic accelerometer data for a 3m
free-fall + concrete impact — no hardware required.

Physics:
  - Free-fall from 3 m: t_fall ≈ 0.78 s, v_impact ≈ 7.67 m/s
  - Impact with rigid concrete (stopping Δx ≈ 2–5 mm):
      peak deceleration ≈ v² / (2·Δx) → ~3,000–7,500 m/s² → >300g
      BHI260AP saturates at ±16g → data will clip (realistic)
  - Post-impact: damped oscillation / ringing at ~1g
  - Pressure drop during 3m fall: ≈ 0.036 hPa

Run:
    pip install numpy matplotlib
    python simulate_drop.py

Saves: blackbox_sim_<timestamp>.csv + .png (full 7-panel dashboard)
"""

import numpy as np
import time
import os
import csv
import matplotlib
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────
#  PHYSICS CONSTANTS
# ──────────────────────────────────────────────────────────────
G          = 9.81       # m/s²
DROP_H     = 3.0        # metres
V_IMPACT   = np.sqrt(2 * G * DROP_H)          # 7.67 m/s
T_FALL     = V_IMPACT / G                      # 0.782 s
STOP_DIST  = 0.003      # 3 mm stopping distance (rigid enclosure on concrete)
PEAK_DECEL = V_IMPACT**2 / (2 * STOP_DIST)    # m/s²
PEAK_G     = min(PEAK_DECEL / G, 16.0)        # cap at 16g (sensor limit)

SAMPLE_RATE = 200       # Hz
PRE_MS      = 300       # ms of data before free-fall start
FREEFALL_MS = int(T_FALL * 1000)              # ~782 ms
IMPACT_DUR  = 8         # ms — duration of impact spike
POST_MS     = 1500      # ms of settling data after impact
TOTAL_MS    = PRE_MS + FREEFALL_MS + IMPACT_DUR + POST_MS

print(f"[SIM] 3m drop physics:")
print(f"      Fall time  : {T_FALL*1000:.0f} ms")
print(f"      v at impact: {V_IMPACT:.2f} m/s")
print(f"      Peak decel : {PEAK_DECEL:.0f} m/s²  = {PEAK_DECEL/G:.0f}g")
print(f"      Sensor cap : {PEAK_G:.1f}g (BHI260AP ±16g)")
print(f"      Total log  : {TOTAL_MS} ms")

SEED = 42
rng  = np.random.default_rng(SEED)


# ──────────────────────────────────────────────────────────────
#  SIGNAL GENERATORS
# ──────────────────────────────────────────────────────────────
def noise(n, sigma=0.02):
    return rng.normal(0, sigma, n)

def damped_ring(t_arr, amp, freq_hz, decay, offset):
    """Damped sinusoid starting at t >= offset."""
    out = np.zeros_like(t_arr)
    mask = t_arr >= offset
    dt = t_arr[mask] - offset
    out[mask] = amp * np.exp(-decay * dt) * np.sin(2 * np.pi * freq_hz * dt)
    return out


def generate_drop(orientation_deg: float = 0.0):
    """
    Simulate one drop. orientation_deg tilts the primary impact axis.
    Returns dict of arrays.
    """
    dt_ms = 1000.0 / SAMPLE_RATE
    n_total = int(TOTAL_MS * SAMPLE_RATE / 1000)

    t = np.arange(n_total) * dt_ms          # time in ms, relative to recording start
    ts_ms = (t + 1_000_000).astype(int)     # fake absolute ms timestamp

    # ── Time segment boundaries ──────────────────────────────────
    t_ff_start  = PRE_MS                         # free-fall begins
    t_impact    = PRE_MS + FREEFALL_MS           # impact spike centre
    t_post      = t_impact + IMPACT_DUR          # settling begins

    # ── Orientation: rotate impact axis ──────────────────────────
    theta = np.radians(orientation_deg)
    # Impact primarily along Z-axis, rotated slightly
    impact_z_frac = np.cos(theta)
    impact_x_frac = np.sin(theta) * 0.6
    impact_y_frac = np.sin(theta) * 0.4

    # ── Phase 1: IDLE — stationary, gravity on Z ─────────────────
    idle_mask = t < t_ff_start
    pre_n     = np.sum(idle_mask)

    ax_idle = noise(pre_n, 0.015)
    ay_idle = noise(pre_n, 0.015)
    az_idle = np.ones(pre_n) + noise(pre_n, 0.015)   # 1g on Z

    # ── Phase 2: FREE-FALL — all axes ≈ 0 (weightlessness) ───────
    ff_mask = (t >= t_ff_start) & (t < t_impact)
    ff_n    = np.sum(ff_mask)

    ax_ff = noise(ff_n, 0.03)
    ay_ff = noise(ff_n, 0.03)
    az_ff = noise(ff_n, 0.03)   # near-zero specific force

    # Add tiny tilt from small rotations during drop
    tilt_freq = 2.5   # Hz
    tilt_amp  = 0.04
    ff_t_local = (t[ff_mask] - t_ff_start) / 1000.0  # s
    ax_ff += tilt_amp * np.sin(2*np.pi*tilt_freq * ff_t_local)
    ay_ff += tilt_amp * np.cos(2*np.pi*tilt_freq * ff_t_local * 1.3)

    # ── Phase 3: IMPACT ───────────────────────────────────────────
    imp_mask = (t >= t_impact) & (t < t_post)
    imp_n    = np.sum(imp_mask)
    imp_t_local = (t[imp_mask] - t_impact)   # 0 → IMPACT_DUR ms

    # Half-sine pulse (clipped to ±16g by sensor)
    pulse_t_norm = imp_t_local / IMPACT_DUR  # 0 → 1
    pulse = PEAK_G * np.sin(np.pi * pulse_t_norm)
    pulse = np.clip(pulse, -16.0, 16.0)     # sensor saturation

    ax_imp =  pulse * impact_x_frac + noise(imp_n, 0.3)
    ay_imp =  pulse * impact_y_frac + noise(imp_n, 0.3)
    az_imp = -pulse * impact_z_frac + noise(imp_n, 0.5)  # negative: deceleration

    # ── Phase 4: SETTLING — damped ring back to 1g ────────────────
    post_mask = t >= t_post
    post_n    = np.sum(post_mask)

    # Structural ringing frequencies (enclosure natural modes)
    ring1_freq = 45.0   # Hz (low)
    ring2_freq = 120.0  # Hz (higher)
    post_t_local = (t[post_mask] - t_post) / 1000.0

    ax_settle = (0.3 * np.exp(-8 * post_t_local) *
                 np.sin(2*np.pi*ring1_freq * post_t_local) + noise(post_n, 0.02))
    ay_settle = (0.25 * np.exp(-8 * post_t_local) *
                 np.cos(2*np.pi*ring2_freq * post_t_local * 0.7) + noise(post_n, 0.02))
    az_settle = (1.0  # gravity
                 + 0.15 * np.exp(-10 * post_t_local) *
                   np.sin(2*np.pi*ring1_freq * post_t_local * 1.1)
                 + noise(post_n, 0.02))

    # ── Combine all phases ────────────────────────────────────────
    ax = np.zeros(n_total); ay = np.zeros(n_total); az = np.zeros(n_total)
    ax[idle_mask] = ax_idle;  ay[idle_mask] = ay_idle;  az[idle_mask] = az_idle
    ax[ff_mask]   = ax_ff;    ay[ff_mask]   = ay_ff;    az[ff_mask]   = az_ff
    ax[imp_mask]  = ax_imp;   ay[imp_mask]  = ay_imp;   az[imp_mask]  = az_imp
    ax[post_mask] = ax_settle;ay[post_mask] = ay_settle;az[post_mask] = az_settle

    # ── Gyroscope ─────────────────────────────────────────────────
    # During free-fall: slow rotation ~30 deg/s
    gx_ff_val = 28.0 * np.sin(2*np.pi*1.2 * (t - t_ff_start)/1000.0)
    gy_ff_val = 22.0 * np.cos(2*np.pi*0.9 * (t - t_ff_start)/1000.0)
    gz_ff_val = 15.0 * np.sin(2*np.pi*0.6 * (t - t_ff_start)/1000.0)

    ff_env = np.where((t >= t_ff_start) & (t < t_post), 1.0, 0.0)
    # Smooth transition
    from numpy import convolve, ones
    ker = ones(int(0.04 * SAMPLE_RATE)) / (0.04 * SAMPLE_RATE)
    ff_smooth = np.convolve(ff_env, ker, mode="same")

    gx = gx_ff_val * ff_smooth + noise(n_total, 0.5)
    gy = gy_ff_val * ff_smooth + noise(n_total, 0.5)
    gz = gz_ff_val * ff_smooth + noise(n_total, 0.5)

    # Impact spike in gyro (angular jerk)
    imp_idx = np.where(imp_mask)[0]
    if len(imp_idx) > 0:
        gx[imp_idx] += rng.uniform(-300, 300, len(imp_idx))
        gy[imp_idx] += rng.uniform(-200, 200, len(imp_idx))
        gz[imp_idx] += rng.uniform(-150, 150, len(imp_idx))

    # ── Pressure (BMP390) ─────────────────────────────────────────
    # Δp ≈ -ρg·Δh/1000 hPa = -1.225*9.81*3/100 ≈ -0.036 hPa during fall
    p_base = 1013.25  # hPa sea-level ish
    t_norm = (t - t_ff_start) / FREEFALL_MS
    t_norm = np.clip(t_norm, 0, 1)
    p_fall = p_base - 0.036 * t_norm   # linear drop
    # After impact: recover instantly
    post_t_norm = np.clip((t - t_impact) / 500.0, 0, 1)
    p_data = np.where(t < t_impact, p_fall,
                      p_base - 0.036 * (1 - post_t_norm)) + rng.normal(0, 0.003, n_total)

    return {
        "ts_ms":    ts_ms.tolist(),
        "ax": ax.tolist(), "ay": ay.tolist(), "az": az.tolist(),
        "gx": gx.tolist(), "gy": gy.tolist(), "gz": gz.tolist(),
        "pressure": p_data.tolist(),
        "t_ms":     t.tolist(),
        "impact_offset_ms": float(t_impact),
        "sample_rate_hz":   SAMPLE_RATE,
        "total_samples":    n_total,
    }


# ──────────────────────────────────────────────────────────────
#  SAVE CSV
# ──────────────────────────────────────────────────────────────
def save_csv(d: dict, path: str):
    """Save in the same format as visualizer.py from_csv expects."""
    ax = np.array(d["ax"]); ay = np.array(d["ay"]); az = np.array(d["az"])
    R  = np.sqrt(ax**2 + ay**2 + az**2)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms_abs", "t_ms_rel",
                    "ax_g", "ay_g", "az_g", "resultant_g", "pressure_hpa"])
        for i in range(len(d["ts_ms"])):
            w.writerow([d["ts_ms"][i], d["t_ms"][i],
                        round(d["ax"][i], 5), round(d["ay"][i], 5),
                        round(d["az"][i], 5), round(R[i], 5),
                        round(d["pressure"][i], 4)])
    print(f"[SIM] CSV saved → {path}")


# ──────────────────────────────────────────────────────────────
#  CONVERT DICT → BlackboxData  (reuses visualizer's class)
# ──────────────────────────────────────────────────────────────
def dict_to_blackbox_data(d: dict):
    """
    Convert the simulation dict into a BlackboxData object
    so it can be passed to visualizer.plot_drop_data().
    """
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from visualizer import BlackboxData

    bd = BlackboxData()
    bd.ts_ms             = [int(v) for v in d["ts_ms"]]
    bd.ax                = list(d["ax"])
    bd.ay                = list(d["ay"])
    bd.az                = list(d["az"])
    bd.pressure          = list(d["pressure"])
    bd.total_samples     = len(d["ts_ms"])
    bd.sample_rate_hz    = d["sample_rate_hz"]
    bd.impact_offset_ms  = int(d["impact_offset_ms"])
    return bd


# ──────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("[SIM] Generating 3m drop simulation...")
    d = generate_drop(orientation_deg=12.0)   # 12° tilt for realism

    ts_str  = time.strftime("%Y%m%d_%H%M%S")
    base    = os.path.dirname(os.path.abspath(__file__))
    csv_out = os.path.join(base, f"blackbox_sim_{ts_str}.csv")
    png_out = csv_out.replace(".csv", ".png")

    # Save CSV
    save_csv(d, csv_out)

    # Build BlackboxData and plot with the full 7-panel dashboard
    bd  = dict_to_blackbox_data(d)

    from visualizer import plot_drop_data
    title = (f"3m Drop Simulation — v_impact={V_IMPACT:.2f} m/s  "
             f"peak≥{PEAK_G:.0f}g (clipped)  {SAMPLE_RATE} Hz")
    fig = plot_drop_data(bd, title=title)

    if fig is not None:
        # Add simulation-specific footnote
        fig.text(
            0.01, 0.002,
            f"[SIMULATION]  h={DROP_H}m  t_fall={T_FALL*1000:.0f}ms  "
            f"stop_Δx={STOP_DIST*1000:.0f}mm  peak_decel={PEAK_DECEL/G:.0f}g→clipped {PEAK_G:.0f}g",
            color="#8b949e", fontsize=7, va="bottom", ha="left", family="monospace"
        )
        fig.savefig(png_out, dpi=150, bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        print(f"[SIM] Dashboard saved → {png_out}")
        print("[SIM] Showing plot... (close window to exit)")
        plt.show()
    else:
        print("[SIM] Plot skipped (no data).")

    print(f"\n[SIM] Load in visualizer:  python visualizer.py --load {csv_out}")
