# ESP32-CAM Streaming System - Setup Status Report

**Date**: June 1, 2026  
**System**: Windows 11 (Python 3.12.10, Git, VS Code)  
**Project**: AI-Integrated Autonomous Robotic Surveillance System

---

## ✅ COMPLETED

### Development Environment
- **Python**: 3.12.10 (with pip 25.0.1)
- **Git**: 2.53.0
- **VS Code**: 1.122.1

### VS Code Extensions (Installed & Active)
- ✅ Python (ms-python.python)
- ✅ Pylance (ms-python.vscode-pylance)
- ✅ PlatformIO IDE (platformio.platformio-ide)
- ✅ C/C++ (ms-vscode.cpptools)
- ✅ Error Lens (usernamehw.errorlens)
- ✅ GitHub Pull Requests (github.vscode-pull-request-github)
- ✅ GitLens (eamodio.gitlens)
- ✅ OpenAI ChatGPT (openai.chatgpt)

### PlatformIO Toolchain
- ✅ PlatformIO CLI (v6.1.19)
- ✅ Espressif 32 Platform (v7.0.1)
- ✅ ESP32-CAM Board Support
- ✅ Toolchain, SDK, and framework installed

### ESP32-CAM Firmware
- ✅ **Firmware compiled successfully** (941.99 seconds)
- ✅ Memory usage: Flash 27.2% (855KB / 3.1MB), RAM 15.2% (49KB / 327KB)
- ✅ No compilation errors
- ✅ Deprecated API warnings fixed (pin_sccb_sda/scl)

**Binary Output**:
- `.pio/build/esp32cam/firmware.bin`
- `.pio/build/esp32cam/firmware.elf`

---

## ⚠️ BLOCKING ISSUE: FTDI USB Serial Driver

### Current Status
- **FTDI Device Detected**: FT232R USB UART (VID_0403:PID_6001, Serial: A5069RR4)
- **Status**: ❌ ERROR (CM_PROB_FAILED_INSTALL)
- **COM Port**: Not assigned (driver failed)
- **Cause**: FTDI driver not installed or installation failed

### Why This Matters
- The FTDI adapter is the bridge between PC and ESP32 via COM port
- Without a valid COM port, PlatformIO cannot upload firmware to ESP32-CAM
- The device is physically connected but Windows cannot assign it a serial port

### Resolution Steps (Manual)

#### Option 1: Automatic Windows Driver Update
1. Open **Device Manager** (Win+X → Device Manager)
2. Expand **Ports (COM & LPT)**
3. Right-click **FT232R USB UART** (should show error icon)
4. Select **Update driver**
5. Choose **Search automatically for updated driver software**
6. Windows will search Windows Update and driver store
7. If found, install and restart

#### Option 2: Manual FTDI Driver Installation
1. Visit: https://ftdichip.com/drivers/d2xx/
2. Download: **CDM (Combined Driver Model)** for Windows 10+
3. Extract the downloaded .zip file
4. Open **Device Manager**
5. Right-click **FT232R USB UART**
6. Select **Update driver**
7. Choose **Browse my computer for drivers**
8. Navigate to the extracted CDM folder and select it
9. Click **Next** and allow installation
10. Restart the PC when prompted

#### Option 3: Disable & Re-enable Device
1. Open **Device Manager**
2. Right-click **FT232R USB UART**
3. Select **Disable device**
4. Wait 3 seconds
5. Right-click again and select **Enable device**
6. Wait for Windows to reassign COM port
7. Verify in Device Manager that COM port now shows without error

#### Option 4: Uninstall & Reconnect
1. Open **Device Manager**
2. Right-click **FT232R USB UART**
3. Select **Uninstall device**
4. Check **"Delete the driver software for this device"**
5. Disconnect FTDI adapter from USB
6. Wait 5 seconds
7. Reconnect FTDI adapter
8. Windows will auto-detect and install built-in driver

---

## 📋 Next Steps (After Driver Resolution)

### 1. Verify COM Port Assignment
```powershell
Get-CimInstance Win32_SerialPort | Format-Table DeviceID, Caption, Description
```
Expected: Should list a COM port (e.g., COM3) with FTDI description

### 2. Update platformio.ini with correct COM port
```ini
[env:esp32cam]
upload_port = COM3  ; Change if different
monitor_port = COM3
```

### 3. Enter ESP32 Flash Mode
- Connect GPIO0 to GND (boot pin)
- Press RESET button
- LED on FTDI should blink
- Release RESET button
- GPIO0 should still be connected to GND

### 4. Upload Firmware
```bash
cd c:\Users\biniy\Documents\projects\esp32\esp32-firmware
pio run --target upload
```

### 5. Monitor Serial Output
```bash
pio device monitor --baud 115200
```

Expected output:
```
Initializing camera...
Camera initialized
Connecting to Wi-Fi...
Connected, IP: 192.168.x.x
Camera server started
```

### 6. Test Video Stream
- Open browser: `http://<ESP32-IP>:80`
- Should display MJPEG stream from camera
- Check FPS and latency

---

## 🏗️ Project Structure

```
c:\Users\biniy\Documents\projects\esp32\
├── esp32-firmware/
│   ├── platformio.ini
│   ├── src/
│   │   └── main.cpp (ESP32-CAM video server code)
│   ├── .pio/
│   │   └── build/esp32cam/
│   │       ├── firmware.bin (ready to upload)
│   │       └── firmware.elf
│   └── build.log
├── Design_and_Development_of_an_AI_Integrated_Autonomous_Robotic_Surveillance_System.pdf
└── README.md (this file)
```

---

## 🔧 Configuration Files

### platformio.ini
```ini
[env:esp32cam]
platform = espressif32
board = esp32cam
framework = arduino
monitor_speed = 115200
upload_speed = 115200
upload_port = COM3
```

### main.cpp Configuration (Edit these)
```cpp
const char* ssid = "YOUR_WIFI_SSID";           // Your network name
const char* password = "YOUR_WIFI_PASSWORD";  // Your network password
```

---

## 📊 Hardware Checklist

- ✅ ESP32-CAM (AI Thinker board)
- ✅ FTDI USB-to-Serial Adapter (FT232R, detected)
- ⚠️ FTDI Driver (needs installation)
- ⏳ COM Port Assignment (pending driver fix)
- ⏳ Wi-Fi Credentials (needs config)
- ⏳ USB Connection Test (pending driver fix)
- ⏳ Serial Monitor Test (pending driver fix)

---

## 🚀 Firmware Features

**Current Implementation**:
- WiFi connectivity (SSID/password configurable)
- OV2640 camera module support
- MJPEG video stream at SVGA resolution (800x600)
- JPEG compression quality 12 (high quality)
- HTTP server on port 80
- Dual frame buffer for smooth streaming
- Serial debug output at 115200 baud

**Stream Endpoints**:
- `/` → HTML page with embedded stream viewer
- `/stream` → Raw MJPEG stream (compatible with browsers, OpenCV, YOLO)

---

## 💡 Troubleshooting

### If COM port is still not visible after driver installation:
1. Check Device Manager for unknown devices
2. Right-click unknown device → **Update driver** → **Have Disk** → select FTDI driver
3. Try a different USB port (avoid USB hubs if possible)
4. Test with `mode` command: `mode COM3` should show configuration

### If upload fails with timeout:
1. Verify GPIO0 is connected to GND during upload
2. Check baud rates match: 115200 for upload and monitor
3. Try lower upload speed in platformio.ini: `upload_speed = 74880`
4. Ensure ESP32 is in bootloader mode (LED should be off or dim)

### If camera shows black screen:
1. Check OV2640 camera module is properly seated on ESP32-CAM
2. Verify camera interface ribbon cable connections
3. Adjust `config.jpeg_quality` (lower = smaller files, better for slow networks)
4. Try lower frame size: change `FRAMESIZE_SVGA` to `FRAMESIZE_VGA` or `FRAMESIZE_QVGA`

---

## 📝 Next Phase: PC-Side YOLO Setup

After video streaming is confirmed working:
1. Set up Python `venv` in `pc-server/` folder
2. Install YOLO dependencies: `pip install ultralytics opencv-python`
3. Download `yolo11n.pt` model file
4. Configure PC server to connect to ESP32 stream
5. Test detection pipeline
6. Implement motor control commands

---

**Status**: ⏸️ PAUSED AT FTDI DRIVER INSTALLATION  
**Action Required**: Install FTDI USB driver (see Options 1-4 above)  
**Next Review**: After COM port assignment confirmed
