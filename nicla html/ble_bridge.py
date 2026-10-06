"""
BLACKBOX CHALLENGE — BLE-to-WebSocket Bridge + HTTP Server
FH Kufstein Tirol / SPS.BBM.24
Board: Arduino Nicla Sense ME (ABX00050)

Architecture
────────────
  ble_bridge.py          ← THIS FILE
    • Connects to Nicla via BLE, receives 20-byte stream packets
    • Broadcasts JSON over a local WebSocket on port 8765
    • Serves the project directory over HTTP on port 8000
      (so Chrome can load models/niclaSenseME.glb & GLTFLoader.js)

  dashboard.html
    • Open http://localhost:8000/dashboard.html in Chrome
    • Loads the real niclaSenseME.glb model via Three.js
    • Connects to ws://localhost:8765 for live sensor data
    • Rotates the 3D board with BHI260AP quaternion
    • Shows the same live graphs (Accel, Gyro, |g|, Phase, Stats)

Usage:
    pip install bleak websockets
    python ble_bridge.py
    → then open http://localhost:8000/dashboard.html
"""

import asyncio
import struct
import json
import time
import os
import sys
import signal
import functools
import csv
import threading
from datetime import datetime
from http.server import HTTPServer, SimpleHTTPRequestHandler

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from bleak import BleakClient, BleakScanner

try:
    import websockets
except ImportError:
    print("[BRIDGE] Missing 'websockets' package.  Install with:")
    print("         pip install websockets")
    sys.exit(1)

# ──────────────────────────────────────────────────────────────
#  BLE UUIDs  (must match firmware)
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

SAMPLE_RATE  = 200
IMP_THRESH_G = 4.00

# ──────────────────────────────────────────────────────────────
#  RECORDING BUFFERS  (for CSV + post-analysis on exit)
# ──────────────────────────────────────────────────────────────
record_lock = threading.Lock()
all_ts = []; all_ax = []; all_ay = []; all_az = []
all_gx = []; all_gy = []; all_gz = []
all_qx = []; all_qy = []; all_qz = []; all_qw = []

# ──────────────────────────────────────────────────────────────
#  WEBSOCKET CLIENTS
# ──────────────────────────────────────────────────────────────
ws_clients = set()
ble_stop   = asyncio.Event()


# ──────────────────────────────────────────────────────────────
#  HTTP SERVER  (serves project directory on port 8000)
# ──────────────────────────────────────────────────────────────
HTTP_PORT = 8000
SERVE_DIR = os.path.dirname(os.path.abspath(__file__))

class ProjectHTTPHandler(SimpleHTTPRequestHandler):
    """Serve files from the project directory."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SERVE_DIR, **kwargs)

    def log_message(self, format, *args):
        # Show all requests with method + path + status
        print(f"[HTTP]  {self.address_string()} {self.command} {self.path} → {args[1] if len(args) > 1 else '?'}")

    def end_headers(self):
        # Allow CORS for local development
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()


def start_http_server():
    httpd = HTTPServer(("0.0.0.0", HTTP_PORT), ProjectHTTPHandler)
    httpd.daemon_threads = True
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    print(f"[HTTP]  Serving directory: {SERVE_DIR}")
    print(f"[HTTP]  Serving on http://localhost:{HTTP_PORT}/")
    print(f"[HTTP]  Dashboard → http://localhost:{HTTP_PORT}/dashboard.html")
    return httpd


# ──────────────────────────────────────────────────────────────
#  WEBSOCKET BROADCAST
# ──────────────────────────────────────────────────────────────
async def broadcast(msg: str):
    """Send message to all connected WebSocket clients."""
    dead = []
    for ws in ws_clients:
        try:
            await ws.send(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ws_clients.discard(ws)


async def ws_handler(websocket):
    """Handle a new WebSocket connection."""
    ws_clients.add(websocket)
    remote = websocket.remote_address
    print(f"[WS]   Client connected: {remote}")
    try:
        async for _ in websocket:
            pass   # we don't expect messages from the dashboard
    except Exception:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"[WS]   Client disconnected: {remote}")


# ──────────────────────────────────────────────────────────────
#  BLE CALLBACKS
# ──────────────────────────────────────────────────────────────
_loop = None   # will be set in main()

def on_sample(sender, data: bytearray):
    """Parse 20-byte packet, record it, and broadcast as JSON."""
    if len(data) < STREAM_SIZE:
        return
    ax_r, ay_r, az_r, gx_r, gy_r, gz_r, qx_r, qy_r, qz_r, qw_r = \
        struct.unpack(STREAM_FMT, data[:STREAM_SIZE])

    ts  = int(time.time() * 1000)
    ax  = round(ax_r / ACCEL_SCALE, 5)
    ay  = round(ay_r / ACCEL_SCALE, 5)
    az  = round(az_r / ACCEL_SCALE, 5)
    gx  = round(gx_r / GYRO_SCALE, 3)
    gy  = round(gy_r / GYRO_SCALE, 3)
    gz  = round(gz_r / GYRO_SCALE, 3)
    qx  = round(qx_r / QUAT_SCALE, 5)
    qy  = round(qy_r / QUAT_SCALE, 5)
    qz  = round(qz_r / QUAT_SCALE, 5)
    qw  = round(qw_r / QUAT_SCALE, 5)

    # Record for CSV + post-analysis
    with record_lock:
        all_ts.append(ts)
        all_ax.append(ax); all_ay.append(ay); all_az.append(az)
        all_gx.append(gx); all_gy.append(gy); all_gz.append(gz)
        all_qx.append(qx); all_qy.append(qy); all_qz.append(qz); all_qw.append(qw)

    msg = json.dumps({
        "type": "sample", "ts": ts,
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "qx": qx, "qy": qy, "qz": qz, "qw": qw,
    })
    if _loop and _loop.is_running():
        asyncio.run_coroutine_threadsafe(broadcast(msg), _loop)


def on_status(sender, data: bytearray):
    if len(data) >= 1:
        code = data[0]
        name = STATUS_NAMES.get(code, f"0x{code:02X}")
        print(f"\r[BLE]  Status → {name}        ", end="", flush=True)
        msg = json.dumps({"type": "status", "code": code, "text": name})
        if _loop and _loop.is_running():
            asyncio.run_coroutine_threadsafe(broadcast(msg), _loop)


# ──────────────────────────────────────────────────────────────
#  BLE CONNECTION
# ──────────────────────────────────────────────────────────────
async def ble_run():
    print(f"[BLE]  Scanning for '{DEVICE_NAME}'…")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print(f"[BLE]  '{DEVICE_NAME}' not found — is the board powered on?")
        ble_stop.set()
        return

    print(f"[BLE]  Found: {device.address}")
    async with BleakClient(device, use_cached=False) as client:
        print("[BLE]  Connected ✓ — discovering services…")
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
            print("[BLE]  STATUS char not found — stale GATT cache?")
            print("[BLE]  Windows Bluetooth settings → remove 'Blackbox-1' → re-run.")
            ble_stop.set()
            return

        await client.start_notify(status_uuid, on_status)
        await client.start_notify(data_uuid,   on_sample)
        await client.write_gatt_char(ctrl_uuid, CMD_STREAM)
        print("[BLE]  CMD_STREAM sent — live data (20-byte packets)")

        # Broadcast a "connected" event
        await broadcast(json.dumps({"type": "ble_connected"}))

        while not ble_stop.is_set():
            if not client.is_connected:
                print("\n[BLE]  Disconnected.")
                break
            await asyncio.sleep(0.05)

        try:
            await client.stop_notify(data_uuid)
            await client.stop_notify(status_uuid)
        except Exception:
            pass

    ble_stop.set()
    print("\n[BLE]  Session ended.")


# ──────────────────────────────────────────────────────────────
#  SAVE CSV + OPEN VISUALIZER POST-ANALYSIS ON EXIT
#  (same behaviour as live_stream.py)
# ──────────────────────────────────────────────────────────────
def save_and_dashboard():
    if not all_ts:
        print("[BRIDGE] No data to save.")
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
    print(f"\n[BRIDGE] CSV saved → {csv_out}  ({len(all_ts)} samples)")

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
        print(f"\n[BRIDGE] Drops detected: {len(events)}")
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
            print(f"[BRIDGE] Dashboard → {png_out}")
            plt.show()
    except Exception as e:
        print(f"[BRIDGE] Dashboard failed: {e}")
        print(f"         Run:  python visualizer.py --load {csv_out}")


# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
async def main():
    global _loop
    _loop = asyncio.get_running_loop()

    # 1. Start HTTP server (serves files for the dashboard)
    httpd = start_http_server()

    # 2. Start WebSocket server
    ws_server = await websockets.serve(ws_handler, "0.0.0.0", 8765)
    print(f"[WS]   WebSocket server on ws://localhost:8765")

    # 3. Start BLE connection
    ble_task = asyncio.create_task(ble_run())

    print("\n" + "=" * 60)
    print("  Open in Chrome:  http://localhost:8000/dashboard.html")
    print("=" * 60 + "\n")

    # Wait for BLE to finish (Ctrl+C or disconnect)
    try:
        await ble_task
    except asyncio.CancelledError:
        pass
    finally:
        ws_server.close()
        await ws_server.wait_closed()
        httpd.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Stopped.")
    finally:
        save_and_dashboard()
