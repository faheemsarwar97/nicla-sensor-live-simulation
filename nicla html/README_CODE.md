# Blackbox Challenge — Code Package
**FH Kufstein Tirol · Rapid Prototyping SPS.BBM.24**  
Board: **Arduino Nicla Sense ME** (ABX00050)

---

## Files

| File | Purpose |
|------|---------|
| `nicla_blackbox/nicla_blackbox.ino` | Arduino firmware — runs on the Nicla |
| `visualizer.py` | Python — BLE connect, download, plot drop data |
| `simulate_drop.py` | Python — offline 3m drop simulator (no hardware needed) |
| `live_stream.py` | **BONUS** — live rolling BLE stream viewer + auto-save |

---

## Quick Start

### 1. Flash the firmware
1. Install **Arduino IDE 2.x**
2. Add Nicla Sense ME board package:  
   `File → Preferences → Board Manager URLs`:  
   `https://downloads.arduino.cc/packages/package_index.json`
3. Install via **Board Manager**: search `Nicla`
4. Install libraries (Library Manager):
   - `Arduino_BHY2` by Arduino
   - `ArduinoBLE` by Arduino
5. Open `nicla_blackbox/nicla_blackbox.ino`, select **Nicla Sense ME**, flash.

### 2. Test with simulation (no hardware)
```bash
pip install numpy matplotlib
python simulate_drop.py
```
Generates a realistic 3m drop dataset and plots all sensor channels.

### 3. Retrieve real data via BLE (drop day)
```bash
pip install numpy matplotlib bleak
python visualizer.py
```
- Scans for `Blackbox-1`
- Device must be in **DATA READY** state (white pulsing LED)
- Downloads all samples, plots X/Y/Z + resultant, saves CSV + PNG

```bash
# Or load a previously saved CSV:
python visualizer.py --load blackbox_drop_20260624_120000.csv
```

### 4. BONUS — Live stream during the drop
```bash
python live_stream.py
```
- Connect **before** dropping — BLE must be active (~80-110 ms display lag)
- Sends `CMD_STREAM (0x02)` to the Nicla → device enters STREAMING mode (blue LED)
- Rolling 4-second window shows: X/Y/Z, resultant with phase colours, pressure, live stats
- On Ctrl+C or window close: auto-saves CSV + opens full 7-panel dashboard

---

## Firmware State Machine

```
IDLE → [resultant < 0.25g for ≥80ms] → FREEFALL_DETECT
                                              │
                                    [confirmed free-fall]
                                              │
                                          RECORDING  ←── captures 100 pre + 400 post = 500 samples
                                              │
                                    [impact settled]
                                              │
                                          DATA_READY  ──→ BLE serves data
```

## LED Status

| LED Colour | State |
|-----------|-------|
| Green (steady) | IDLE — waiting for drop |
| Blue | Free-fall detected |
| Red | Recording impact |
| White (pulsing) | Data ready — connect via BLE |

## BLE Protocol

| Characteristic | UUID suffix | Type | Description |
|---------------|-------------|------|-------------|
| STATUS | `...1214` | Read/Notify | 0=IDLE, 1=FF, 2=REC, 3=READY, 4=SENDING, **5=STREAMING** |
| META | `...1214` | Read | `[u32 total_samples, u32 sample_rate, u32 impact_offset_ms]` |
| DATA | `...1214` | Read/Notify | 6-byte Sample (download) or 12-byte StreamPkt (live stream) |
| CTRL | `...1214` | Write | `0x01` = request data, **`0x02` = live stream**, `0xFF` = reset |

## Packet Formats

### Download packet (visualizer.py) — 6 bytes
```
[int16 ax_raw]  [int16 ay_raw]  [int16 az_raw]
```
Python: `struct.unpack("<hhh", raw)` → 6 bytes
Scale: `ax_g = ax_raw / 4096.0`  (BHI260AP Q12 fixed-point, 1 g = 4096 LSB)

### Live-stream packet (live_stream.py) — 12 bytes
```
[int16 ax]  [int16 ay]  [int16 az]  [int16 gx]  [int16 gy]  [int16 gz]
```
Python: `struct.unpack("<hhhhhh", raw)` → 12 bytes
Accel scale: `ax_g = ax / 4096.0`
Gyro scale:  `gx_dps = gx / 16.0`  (approximate deg/s)

> **Note:** BMP390 barometer removed to free heap for BLE Cordio HCI stack (nRF52832 has only 64 KB RAM).
> **Scale correction:** BHI260AP outputs Q12 fixed-point. The raw accel values from `accel.x()` are 4096 per g, not 1000. Using 1000 caused stationary R ≈ 4.1 g (above the 4 g impact threshold), making the device always show IMPACT when at rest.

## Physics Reference — 3m Drop

| Quantity | Value |
|---------|-------|
| Fall time | ~782 ms |
| Impact velocity | 7.67 m/s |
| Peak g (3mm stop dist) | ~1000g → clipped to 16g by sensor |
| Pressure drop | ~0.036 hPa |
| Free-fall threshold | < 0.25g resultant |

> **Note:** The BHI260AP ±16g range saturates at impact — the clipped signal
> is still valid as proof of impact (flat-top spike with ringing afterwards).
> For higher fidelity, reduce `STOP_DIST` in the simulation to see the full pulse shape.

---

*AI Co-Engineer: Claude (Anthropic) — all prompts documented in AI Logbook*
