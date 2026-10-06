# Blackbox Challenge — Arduino Nicla Sense ME
**FH Kufstein Tirol · Rapid Prototyping SPS.BBM.24**

## What is this?

A drop-data-logger built on the **Arduino Nicla Sense ME (ABX00050)**. The device must survive a 3-metre free-fall onto concrete, autonomously detect the drop and impact, record acceleration data, and transmit it wirelessly via BLE for analysis.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Board | Arduino Nicla Sense ME (ABX00050) |
| SoC | Nordic nRF52832 — 64 KB RAM, 512 KB Flash |
| IMU | Bosch BHI260AP (accelerometer + gyroscope + rotation vector) |
| Communication | Bluetooth Low Energy (BLE) |
| Power | LiPo battery |

---

## What I Built

### 1. Firmware — `nicla_blackbox/nicla_blackbox.ino`
State machine running on the Nicla:

- **IDLE** → waits for free-fall (green LED)
- **FREEFALL_DETECT** → resultant acceleration drops below 0.3 g (blue LED)
- **RECORDING** → impact detected > 4 g, records 2000 samples at ~100 Hz into RAM (red LED)
- **DATA_READY** → recording complete, data waiting in RAM (white pulse LED)

Key design decisions:
- BHI260AP Q12 fixed-point format: `1 g = 4096 LSB`
- 20-byte BLE StreamPkt: 10 × int16 (accel XYZ, gyro XYZ, quaternion XYZW)
- **Parallel BLE streaming** — live stream flag runs independently of drop recording pipeline so a connected visualizer never blocks a drop from being captured
- Data survives BLE disconnect (stored in RAM) but is lost on reset/power cycle

### 2. Post-Drop Visualizer — `visualizer.py`
Python script that:
- Scans BLE, connects to `Blackbox-1`, auto-waits for DATA_READY status
- Downloads full drop buffer over BLE notify
- Saves timestamped CSV
- Plots acceleration XYZ, resultant |g|, and detected phases (free-fall / impact / settle)
- Phase detection: backward search from impact peak to find true free-fall start

#### Visualizer Output
![Post-Drop Visualizer](nicla%20html/live_stream_20260624_161857.png)

---

### 3. Live Stream Dashboard — `live_stream.py`
Real-time matplotlib dashboard while the Nicla streams live:
- Scrolling acceleration and gyro plots
- 3D board orientation rendered from quaternion (BHI260AP SENSOR_ID_RV)
- Phase banner (IDLE / FREE-FALL / IMPACT / SETTLE)

---

### 4. 3D BLE Dashboard — `nicla html/`
Browser-based dashboard with real Nicla 3D model:
- `ble_bridge.py` — Python BLE client (bleak) + WebSocket server + built-in HTTP file server
- `dashboard.html` — Three.js r152 with the official `niclaSenseME.glb` model rotating in real time from live quaternion data and dynamic red light highlights representing physical impact direction
- Chart.js plots for accel XYZ, gyro XYZ, resultant |g| matching legacy layouts
- Phase banner and real-time statistics panel matching legacy tools

#### HTML Live Dashboard Preview
![HTML Live Dashboard](nicla%20html/HTML%20live%20dashboard.png)

---

## Project Structure

```
Blackbox_Simulation/
├── nicla_blackbox/
│   └── nicla_blackbox.ino      # Firmware
├── visualizer.py               # Post-drop BLE download + analysis
├── live_stream.py              # Real-time live dashboard
└── nicla html/
    ├── ble_bridge.py           # BLE → WebSocket bridge + HTTP server
    ├── dashboard.html          # Three.js 3D dashboard
    ├── GLTFLoader.js           # Three.js GLTF loader
    ├── HTML live dashboard.png # Preview image
    ├── live_stream_20260624_161857.png # Visualizer preview image
    └── models/
        └── niclaSenseME.glb    # Official Arduino Nicla 3D model
```

---

## How to Run

### Flash firmware
Open `nicla_blackbox.ino` in Arduino IDE, select **Nicla Sense ME**, upload.

### Post-drop analysis
```bash
pip install bleak matplotlib numpy
python visualizer.py
```
Connects automatically, downloads drop data, saves CSV and shows plots.

### Live stream (matplotlib)
```bash
python live_stream.py
```

### 3D browser dashboard
```bash
pip install bleak websockets
python ble_bridge.py
# then open http://localhost:8000/dashboard.html in Chrome
```

---

## BLE Protocol

| UUID suffix | Characteristic | Direction |
|-------------|---------------|-----------|
| `...1214` | STATUS | Nicla → PC (notify) |
| `...1314` | DATA | Nicla → PC (notify) |
| `...1414` | CTRL | PC → Nicla (write) |

Commands: `0x01` = request data, `0x02` = start stream, `0xFF` = reset

---

## LED Colour Reference

| Colour | State |
|--------|-------|
| Green | IDLE — waiting |
| Blue | Free-fall detected |
| Red | Recording impact |
| White pulse | Data ready / sending |

---

## Dependencies

```
Arduino: Arduino_BHY2, ArduinoBLE
Python:  bleak, websockets, matplotlib, numpy, pyserial (optional)
Browser: Chrome (WebBluetooth / WebSocket)
```
