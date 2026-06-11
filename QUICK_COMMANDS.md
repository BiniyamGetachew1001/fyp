# Quick Command Reference

This file contains exact commands to run after the FTDI driver is installed and COM port is confirmed.

## Prerequisites Check

```powershell
# Verify Python installation
python --version
# Expected: Python 3.12.10 or similar

# Verify Git
git --version
# Expected: git version 2.xx.x

# Verify PlatformIO CLI
pio --version
# Expected: PlatformIO 6.1.19 or similar
```

## Navigate to Project

```powershell
cd c:\Users\biniy\Documents\projects\esp32\esp32-firmware
```

## Verify COM Port Assignment

```powershell
# List available COM ports (Windows)
Get-CimInstance Win32_SerialPort | Format-Table DeviceID, Caption

# Expected output should include COM3 with FT232R USB UART description
# Example:
# DeviceID  Caption
# --------  -------
# COM3      FT232R USB UART
```

## Update platformio.ini with Correct COM Port

Edit `platformio.ini` and ensure:
```
[env:esp32cam]
upload_port = COM3
monitor_port = COM3
```

(Change COM3 to your actual COM port if different)

## Update WiFi Credentials

Edit `src/main.cpp` around line 16-17:
```cpp
const char* ssid = "YOUR_WIFI_SSID";           // CHANGE THIS
const char* password = "YOUR_WIFI_PASSWORD";   // CHANGE THIS
```

Replace with your actual WiFi network name and password.

## Build Firmware (Recompile with New Settings)

```powershell
pio run
```

Expected output:
```
Checking size .pio\build\esp32cam\firmware.elf
RAM:   [==        ]  15.2% (used XXXX bytes from 327680 bytes)
Flash: [===       ]  27.2% (used XXXX bytes from 3145728 bytes)
======================== [SUCCESS] Took XX.XX seconds ========================
```

## Enter ESP32 Flash Mode (Hardware Steps)

Before running upload command:
1. Connect GPIO0 pin to GND (pull boot pin to ground)
2. Press ESP32 RESET button (or RST pin to GND momentarily)
3. Release RESET button
4. GPIO0 should STILL be connected to GND
5. LED on FTDI adapter should be solid or dim (bootloader mode)

## Upload Firmware to ESP32-CAM

```powershell
pio run --target upload
```

Expected output:
```
esptool.py v4.x.x
COM3 speed = 115200 baud
Detecting chip type...
Detected esp32
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 921600
Uploading binary data
...
Leaving...
Hard resetting via RTS pin...
======================== [SUCCESS] Took XX.XX seconds ========================
```

## Monitor Serial Output

```powershell
pio device monitor --baud 115200
```

Expected output (should see within 10 seconds):
```
ets Jun  8 2016 00:22:57 rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_BOOT)
...
Initializing camera...
Camera initialized
Connecting to Wi-Fi...
Connecting to YOUR_WIFI_SSID
......
Connected, IP: 192.168.X.X
Camera server started on: http://192.168.X.X:80/
```

Note the IP address (e.g., 192.168.1.100)

Press `Ctrl+C` to stop monitoring.

## Test Video Stream in Browser

1. Open your web browser
2. Navigate to: `http://192.168.X.X:80`
   (Replace X.X with the IP address from serial output)
3. You should see:
   - An HTML page with an image viewer
   - A live MJPEG video stream from the camera
   - Stream updates in real-time at ~30 FPS

## Troubleshooting Commands

```powershell
# If upload fails with timeout, check bootloader mode:
# (Repeat: GPIO0 to GND + RESET)
# Then try with lower upload speed:
# Edit platformio.ini and change:
#   upload_speed = 74880

# View complete build output in file:
pio run > build.log 2>&1
Get-Content build.log -Tail 50

# Clean build cache (if needed):
pio run --target clean
pio run
```

## Stream Access via OpenCV (For YOLO Integration)

Once video stream is confirmed working, you can access it from Python:

```python
import cv2
import sys

# ESP32 video stream URL
url = 'http://192.168.X.X/stream'  # Replace X.X with actual IP

# Open stream
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print(f"Failed to open stream: {url}")
    sys.exit(1)

print(f"Connected to {url}")
print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    
    cv2.imshow('ESP32 Camera Stream', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## Summary of Steps

1. ✅ **FTDI Driver**: Install via Device Manager (automatic update or manual)
2. ✅ **Verify COM Port**: Check it appears without error in Device Manager
3. ✅ **Update Config**: Set upload_port in platformio.ini
4. ✅ **WiFi Credentials**: Update SSID/password in main.cpp
5. ✅ **Rebuild**: Run `pio run`
6. ✅ **Flash Mode**: GPIO0→GND, press RESET
7. ✅ **Upload**: Run `pio run --target upload`
8. ✅ **Monitor**: Run `pio device monitor --baud 115200`
9. ✅ **Test**: Open browser to ESP32 IP address

---

**Status**: Ready to proceed once FTDI driver is installed and COM port confirmed
