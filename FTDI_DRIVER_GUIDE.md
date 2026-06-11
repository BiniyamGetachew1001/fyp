# FTDI Driver Installation Guide

## ⚠️ CRITICAL ISSUE
Your FTDI USB adapter is detected by Windows but the driver failed to install. This prevents the PC from assigning a COM port to the ESP32-CAM device.

**Device Details**:
- Name: FT232R USB UART
- USB ID: VID_0403 & PID_6001
- Serial: A5069RR4
- Current Status: ERROR (CM_PROB_FAILED_INSTALL)
- COM Port: NOT ASSIGNED

---

## 🚀 QUICK FIX (Recommended for Windows 11)

### Step 1: Open Device Manager
- Press `Win + X`
- Select **Device Manager**
- (Alternatively: Search for "Device Manager" in Start menu)

### Step 2: Find the FTDI Device
- Look for **Ports (COM & LPT)**
- You should see: **FT232R USB UART** (with a ⚠️ yellow warning icon)

### Step 3: Request Windows Update
- Right-click **FT232R USB UART**
- Select **Update driver**
- Choose **Search automatically for updated driver software**
- Windows will search its driver store and Windows Update

### Step 4: Wait for Installation
- Windows will download and install the driver
- You may see: "Windows has successfully updated your driver software"
- If successful, the warning icon disappears

### Step 5: Verify COM Port
- Check **Ports (COM & LPT)** again
- You should now see: **"COM3 (FT232R USB UART)"** or similar without error icon
- Note the COM port number (COM3 in this example)

### Step 6: Update PlatformIO Configuration
Edit: `c:\Users\biniy\Documents\projects\esp32\esp32-firmware\platformio.ini`

Replace:
```
upload_port = COM3
```
with your actual COM port if different

---

## 🔧 Manual Installation (If Automatic Fails)

### Step 1: Download FTDI Drivers
1. Go to: https://ftdichip.com/drivers/d2xx/
2. Look for **CDM (Combined Driver Model)** → Windows 10+ (or latest available)
3. Click download
4. Extract the ZIP file to a folder like: `C:\FTDI_Drivers`

### Step 2: Install via Device Manager
1. Right-click **FT232R USB UART** (the device with warning)
2. Select **Update driver**
3. Choose **"Browse my computer for driver software"**
4. Click **Browse...**
5. Navigate to: `C:\FTDI_Drivers` (or wherever you extracted)
6. Click **Next**
7. Click **Install**
8. When asked, click **Yes** to trust the driver
9. Restart your PC when prompted

### Step 3: Verify Installation
After restart:
- Open Device Manager again
- Check that **FT232R USB UART** now shows as **COM3** (or similar) without error icon

---

## 🔄 Force Reinstall (If Still Not Working)

### Option A: Disable and Re-enable
1. Right-click **FT232R USB UART**
2. Select **Disable device**
3. Wait 10 seconds
4. Right-click again
5. Select **Enable device**
6. Windows will re-initialize the device

### Option B: Remove and Reconnect
1. Right-click **FT232R USB UART**
2. Select **Uninstall device**
3. Check: **"Delete the driver software for this device"**
4. Click **Uninstall**
5. **Disconnect** FTDI adapter from USB port
6. Wait 10 seconds
7. **Reconnect** FTDI adapter to USB port
8. Windows will auto-detect and reinstall

### Option C: Device Properties Check
1. Right-click **FT232R USB UART**
2. Select **Properties**
3. Go to **Driver** tab
4. Check:
   - Provider: Should be "FTDI"
   - Date: Should be recent
   - Version: Should not be blank
5. If Provider is blank or "Microsoft", click **"Update Driver"** and try Step 2 again

---

## ✅ Verification Commands

After installation, verify via PowerShell:

```powershell
# List all COM ports
Get-CimInstance Win32_SerialPort | Format-Table DeviceID, Caption, Description

# Expected output:
# DeviceID Caption      Description
# ------- -------       -----------
# COM3    FT232R USB UART FT232R USB UART
```

If you see **COM3** with FTDI description, installation is successful!

---

## 🚨 If COM Port Is STILL Missing

This means the driver installation failed or was blocked. Try these:

1. **Check Windows Updates**:
   - Settings → Update & Security → Check for updates
   - Install any pending updates
   - Restart PC
   - Retry driver update

2. **Unblock Driver Files** (if manually downloaded):
   - Extract FTDI driver ZIP
   - In File Explorer, find the extracted folder
   - Right-click **ftdibus.inf** → Properties
   - If there's an "Unblock" button at the bottom, click it
   - Repeat for **ftdiport.inf**
   - Then retry manual installation

3. **Try Alternative Serial Software**:
   - Download: https://www.tera-term.com/ (Tera Term terminal)
   - This can help diagnose if the device is actually present
   - If Tera Term can list COM3, the driver is working

4. **Check Device Manager Details**:
   - Right-click **FT232R USB UART** → Properties → Details tab
   - Take a screenshot of the Hardware IDs
   - Post in forums with the Hardware IDs for specific help

---

## 📞 Next Action

### Once COM Port Is Confirmed:
1. Note the COM port number (COM3, COM4, etc.)
2. Edit `platformio.ini`: set `upload_port = COM3` (your port number)
3. Run: `pio run --target upload` to upload firmware
4. Run: `pio device monitor --baud 115200` to view serial output
5. Configure WiFi in `src/main.cpp` (SSID and password)
6. Recompile and upload again
7. Check ESP32 serial output for WiFi connection confirmation
8. Open browser to `http://<ESP32-IP>:80` to view video stream

---

## 📚 Resources

- FTDI Official: https://ftdichip.com/
- Windows Device Manager Help: https://support.microsoft.com/windows/open-device-manager
- PlatformIO Upload Troubleshooting: https://docs.platformio.org/en/latest/faq.html

---

**Last Updated**: June 1, 2026  
**Status**: ⏸️ BLOCKED - Waiting for FTDI Driver Installation
