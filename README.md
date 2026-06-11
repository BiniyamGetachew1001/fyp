# ESP32-CAM Person Follower — Split Architecture

A two-part robot that follows a person:

- **ESP32-CAM ("The Doer")** — streams video over Wi-Fi and executes motor
  commands. Keeps only low-level motor smoothing (min-speed, kickstart,
  slew) + a network failsafe. No vision, no decision logic.
- **PC ("The Thinker")** — grabs the stream, runs YOLO person detection,
  EMA-smooths the target, runs the follow state machine, and sends per-wheel
  motor commands back over UDP.

```
        MJPEG video  (HTTP :80/stream)
   ESP32-CAM ──────────────────────────────►  PC
   camera + L298N motors                       YOLO + EMA + state machine
        ▲                                          │
        └────────  UDP MotorCmd (:3333)  ◄─────────┘
                   per-wheel duties, ~30 Hz
```

---

## 1. What you need

**Hardware**
- AI-Thinker ESP32-CAM
- FTDI / USB-TTL serial adapter (for flashing)
- L298N motor driver + 2 DC gear motors (forward-only wiring)
- **Separate 5V power supply** for the ESP32-CAM (see the brownout note below)
- A robot chassis + battery for the motors

**Software**
- [PlatformIO Core](https://platformio.org/install/cli) (`pio` on your PATH)
- Python 3.9+ on the PC
- FTDI driver installed (see `FTDI_DRIVER_GUIDE.md` if no COM port appears)

---

## 2. Wiring

### FTDI → ESP32-CAM (flashing only)
| FTDI | ESP32-CAM |
|------|-----------|
| TX   | U0R (RX)  |
| RX   | U0T (TX)  |
| GND  | GND       |
| 5V   | 5V (or use a separate supply — recommended) |

To enter **flash mode**: jumper **IO0 → GND**, then tap **RESET**.
Remove the IO0 jumper after uploading so it boots the program.

### L298N → ESP32-CAM (motors)
Defined in `esp32-firmware/include/config.h`:

| Signal | GPIO | L298N |
|--------|------|-------|
| Left speed (PWM)  | 12 | ENA |
| Left forward      | 13 | IN1 |
| Right forward     | 14 | IN3 |
| Right speed (PWM) | 15 | ENB |

Only one direction pin per motor is wired — the drive is **forward-only**;
turning is done by pivoting (drive one wheel, stop the other).

> ⚠️ **Power / brownout:** the ESP32-CAM + camera draws more current than an
> FTDI 5V pin can reliably supply. Powering it from the FTDI causes a
> **reboot loop** (the serial monitor prints the boot banner over and over).
> Power the ESP32-CAM from a **separate 5V source** (a 5V/1A+ supply or USB
> charger into the 5V pin), and share **GND** between that supply, the FTDI,
> and the L298N.

---

## 3. Configure Wi-Fi

Edit `esp32-firmware/src/main.cpp` (in `setup()`), add your network:

```cpp
wifiMulti.addAP("YOUR_SSID", "YOUR_PASSWORD");
```

The **PC and the ESP32 must be on the same network/subnet**.

---

## 4. Flash the firmware (ESP32 side)

```bash
cd "c:/Users/biniy/Documents/projects/esp32/esp32-firmware"
pio run -t upload
```

The upload port is set to **COM10** in `platformio.ini`. If your port differs,
either edit `upload_port`/`monitor_port` there, or pass it inline:

```bash
pio run -t upload --upload-port COMx
```

Check available ports with:

```bash
pio device list
```

---

## 5. Verify the board

Open the serial monitor:

```bash
cd "c:/Users/biniy/Documents/projects/esp32/esp32-firmware"
pio device monitor --baud 115200
```

You should see a **single** boot, then:

```
[INIT] Camera OK
[INIT] Motors OK
[WIFI] Connected: <ssid>  IP: 192.168.x.x
[WIFI] Stream URL  : http://192.168.x.x/stream
[WIFI] Send cmds to: 192.168.x.x:3333  (UDP)
[NET]  MJPEG server started on :80/stream
[NET]  UDP command listener up on :3333 (core 0)
```

**Note the IP.** Quit the monitor with `Ctrl+C`.

- If the banner repeats endlessly → **brownout** → fix power (section 2).
- Open `http://192.168.x.x/stream` in a browser to confirm live video.

---

## 6. Run the PC "Thinker"

In a new terminal:

```bash
cd "c:/Users/biniy/Documents/projects/esp32/pc"
python follower.py --esp 10.105.3.248 --show
```
python follower.py --esp 192.168.
Replace `192.168.x.x` with the IP from step 5.

- A window opens showing YOLO detections + the current command.
- The first run downloads `yolov8n.pt` automatically.
- Walk into frame: the robot pivots to centre you, then creeps forward; it
  stops when you're too close or leave the frame.
- Drop `--show` for headless / lowest latency.

**Useful flags**

| Flag | Default | Meaning |
|------|---------|---------|
| `--esp` | *(required)* | ESP32-CAM IP for UDP commands |
| `--port` | `3333` | UDP command port |
| `--url` | `http://<esp>/stream` | Override the stream URL |
| `--model` | `yolov8n.pt` | Ultralytics model |
| `--imgsz` | `320` | YOLO inference size |
| `--show` | off | Show the annotated window |

Stop with `Ctrl+C` (or `q` in the window) — it sends a STOP to the robot on exit.

---

## 7. How the state machine behaves

Runs on the PC; sends per-wheel duties to the ESP:

| State | Trigger | Command sent |
|-------|---------|--------------|
| SEARCH | no person | STOP (0,0) |
| WARMUP | just acquired | HOLD (0,0) — EMA settling |
| TURN | person off-centre | PIVOT_L / PIVOT_R (one wheel drives) |
| ADVANCE | person centred | ADVANCE (both forward + small trim) |
| STOPCLS | box too large (too close) | STOP (0,0) |

Tuning lives at the top of `pc/follower.py` (`TURN_PWM`, `ADVANCE_PWM`,
`EMA_ALPHA`, `ALIGNED_ENTER/EXIT`, `AREA_CLOSE_FRAC`, `CONF_THRESHOLD`, …).
Low-level motor limits live in `esp32-firmware/include/config.h`
(`MIN_SPEED`, `KICK_PWM`, `SLEW_MAX_STEP`, `CMD_TIMEOUT_MS`).

---

## 8. Network protocol (why UDP)

Control telemetry is *latest-truth*: command N+1 fully supersedes N. UDP avoids
TCP's retransmission and head-of-line blocking, so a dropped packet is simply
replaced by the next one ~33 ms later — lowest possible control latency. If the
ESP hears nothing for `CMD_TIMEOUT_MS` (250 ms) it **stops the motors**
(failsafe against PC crash / Wi-Fi drop).

Packet — `MotorCmd`, 10 bytes, little-endian (defined in
`esp32-firmware/include/config.h`, mirrored in `pc/follower.py` as `"<HBBBBI"`):

| field | type | meaning |
|-------|------|---------|
| magic | u16 | `0x4332` ('C2') sanity |
| version | u8 | `1` |
| cmd | u8 | intent tag: STOP/HOLD/PIVOT_L/PIVOT_R/ADVANCE |
| left | u8 | target left-wheel duty 0..255 |
| right | u8 | target right-wheel duty 0..255 |
| seq | u32 | PC monotonic counter (debug) |

`left`/`right` are authoritative; `cmd` is for logging on both ends.

---

## 9. Project layout

```
esp32/
├── README.md                    ← this file
├── esp32-firmware/              ← ESP32 "Doer" (PlatformIO)
│   ├── platformio.ini
│   ├── include/config.h         ← pins, motor tuning, shared wire protocol
│   └── src/
│       ├── main.cpp             ← wiring + failsafe motor-executor task
│       ├── camera_stream.*      ← camera + MJPEG HTTP server
│       ├── vision_link.*        ← UDP listener task + staleness failsafe
│       └── motor_control.*      ← forward-only L298N: kickstart/slew/floor
└── pc/                          ← PC "Thinker"
    ├── follower.py              ← YOLO + EMA + state machine + UDP sender
    └── requirements.txt
```

**Core layout (firmware):** Core 0 runs the UDP listener + Wi-Fi/lwIP; Core 1
runs the motor-executor task + `loop()` (MJPEG server). The blocking stream
handler can never starve motor control because the executor is its own task.

---

## 10. Troubleshooting

| Symptom | Fix |
|---------|-----|
| No COM port | Install FTDI driver (`FTDI_DRIVER_GUIDE.md`); use a **data** USB cable, not charge-only |
| Upload won't start | IO0→GND, tap RESET, retry; close any open serial monitor |
| Serial banner repeats forever | Brownout — power ESP32-CAM from a separate 5V supply (section 2) |
| `Connection failed` on boot | SSID/password wrong, or network out of range (section 3) |
| Stream URL blank / no video | Wi-Fi didn't connect; check the monitor for the IP line |
| Window opens but robot doesn't move | Wrong `--esp` IP, PC on a different network, or motor power off |
| Robot moves but won't pivot | Raise `TURN_PWM`; raise `MIN_SPEED` in `config.h` if wheels stall |
| Stops too far / too near | Tune `AREA_CLOSE_FRAC` in `follower.py` |
```
