# Scuttle AI Voice Assistant Robot
▶️ Watch it in action: https://www.youtube.com/shorts/xofZN5-Q8Ms

Build kit: 👉 https://ScuttleRobot.org (Checkout Code: AVIVMAKES for 10% off!)

A voice-controlled robot built on a Scuttle v3 wheeled base running on a Raspberry Pi 5, powered by OpenAI Whisper, GPT-4o, and Viam robotics platform.

---
<img width="2931" height="4700" alt="AI_Assistant_Robot-System_Architecture" src="https://github.com/user-attachments/assets/61aaa4fd-c475-419d-8de0-61cf0cc57c73" />


## 🛠️ Hardware

> (As an Amazon Associate, I earn from qualifying purchases #ad)

| Component | Link |
|---|---|
| Raspberry Pi 5 | https://amzn.to/4p3ncNY |
| High-Torque DC Drive Motors (120 RPM) | https://amzn.to/4b6iHMQ |
| Motor Controller / Driver Shield | https://amzn.to/3SXU2nA |
| Raspberry Pi HQ Camera (M12 / CSI) | https://amzn.to/44OXX8I |
| HDMI to CSI Camera Adapter Board | https://amzn.to/44Mue09 |
| AirHug USB Speaker & Microphone | https://amzn.to/4f1tLMm |
| MPU-6050 6-DoF Accelerometer/Gyro IMU | https://amzn.to/4gvAznX |
| Anker 87W 20,000mAh Power Bank | https://amzn.to/4eILc5D |
| RPLidar (any model) | https://amzn.to/4fCIQFF |
| 3030 Vertical Extrusion | https://amzn.to/44utr41 |
| SO-101 Robotic Arm Kit | https://amzn.to/3QWlG3v |
| Upgraded Servos (optional) | https://amzn.to/4eG8hpt |
| Adafruit Qualia ESP32-S3 for TTL RGB-666 Displays | https://www.adafruit.com/product/5800 |
| Round RGB TTL TFT Display - 4" 720x720 (NV3052C) | https://www.adafruit.com/product/5793 |
| 16x NeoPixel strip | https://www.adafruit.com/product/3919 |
| AS5048B Magnetic Encoders (x2) |AS5048B-TS_EK_AB|

---

## 🔑 Credentials Needed

Get these before starting — you'll need them during setup:

- **OpenAI API key** — https://platform.openai.com/api-keys
- **Viam API key, API key ID, and robot address** — app.viam.com → your machine → Connect tab

---

## 📁 Files

Download these files and save them to `/home/user/AI-ASSIST/` on your Pi:

| File | Location | Description |
|---|---|---|
| `main.py` | `/home/user/AI-ASSIST/main.py` | Main voice assistant code (runs on Pi) |
| `eyes_viam.py` | `/home/user/AI-ASSIST/eyes_viam.py` | Eye tracking — sends face gaze coordinates to the Qualia display |
| `code.py` | Qualia CIRCUITPY drive | CircuitPython code for the round display and NeoPixels (auto-runs on boot) |

---

## 🎭 Step 1 — Qualia Display Setup

1. Flash **CircuitPython 10.2.0** for the Qualia ESP32-S3 from [circuitpython.org](https://circuitpython.org)
2. Install required Adafruit libraries onto the CIRCUITPY drive:
   - `adafruit_qualia`
   - `neopixel`
   - `vectorio`
   - `displayio`
3. Copy `code.py` to the root of the CIRCUITPY drive — it will run automatically on boot
4. The NeoPixel strip connects to pin **A0** on the Qualia board (16 LEDs)
5. Connect the Qualia to the Pi via USB — it will appear as `/dev/ttyACM0`

---

## ⚙️ Step 2 — Raspberry Pi Setup

### Enable I2C and reboot
```bash
sudo raspi-config nonint do_i2c 0 && sudo reboot
```

### After reboot — install dependencies and fix LIDAR permissions
```bash
pip install viam-sdk openai pyserial pydub numpy --break-system-packages && sudo apt install ffmpeg -y && sudo sed -i 's/exit 0/chmod 666 \/dev\/ttyUSB0\nexit 0/' /etc/rc.local
```

---

## 🤖 Step 3 — Viam Setup

Go to [app.viam.com](https://app.viam.com), create a free account, add a new machine, and follow the on-screen install instructions to install viam-agent on the Pi.

Then go to **Configure → JSON** and paste the contents of `viam_config.json` from this repository. Update `/dev/ttyUSB0` if your LIDAR is on a different port.

---

## 📝 Step 4 — Fill in Credentials

Open `main.py` and fill in:
- **Line ~100**: Your OpenAI API key
- **Bottom of file**: Your Viam API key, API key ID, and robot address

Open `eyes_viam.py` and fill in:
- `ROBOT_ADDRESS`, `API_KEY_ID`, `API_KEY` at the top of the file

---

## 🚀 Step 5 — Run the Assistant

Make sure Thonny is **closed**, then run both scripts in two separate terminals:

**Terminal 1:**
```bash
python3 /home/user/AI-ASSIST/main.py
```

**Terminal 2:**
```bash
python3 /home/user/AI-ASSIST/eyes_viam.py
```

---

## 🗣️ Voice Commands

Say **"robot"** to wake the assistant, then speak your command:

| Command | Action |
|---|---|
| "move forward" | Moves forward 500mm |
| "move backward" | Moves backward 500mm |
| "turn left" | Turns left 45 degrees |
| "turn right" | Turns right 45 degrees |
| "stop" | Stops all movement |
| "dance" | Performs a square dance routine |
| "follow me" | Starts person-following mode using camera |
| "stop following" | Stops person-following mode |
| "get out of the way" | Moves aside |
| "what do you see?" | Captures camera image and describes it |
| "how far is the wall?" | Reads LIDAR distance |
| Anything else | General conversation via GPT-4o |

---

## 🛟 Recovery (if SD card fails)

If your Pi SD card dies, you need to restore:
1. Re-flash Pi OS and reinstall viam-agent (your Viam config is safe in the cloud)
2. Reinstall Python dependencies (Step 2 above)
3. Restore these files from backup:
   - `main.py`
   - `eyes_viam.py`
4. Re-enter your credentials in both files
5. `code.py` is safe — it lives on the Qualia's own flash memory

---

## 🦾 SO-101 Robotic Arm Setup (Optional)

Official module reference: https://github.com/viam-devrel/so-101

These are the steps and fixes for getting the SO-101 arm identified, powered, and connected to Viam on a Raspberry Pi 5.

> **Note:** We initially tried setting up the arm through LeRobot's Python/conda tooling instead of Viam's native workflow. We abandoned that in favor of doing motor setup and calibration through the Viam module directly (step 5) — no separate Python environment needed, and it's the simpler path if your goal is Viam integration rather than using LeRobot directly.

### 0. Add the module and discovery service in Viam

On your machine's **CONFIGURE** tab in the Viam app:

1. **+ Create component or service** → search the registry for `devrel:so101-arm` → add it.
2. Add a service using this module with the `devrel:so101:discovery` model (e.g. named `so101-discovery`). Its Test panel/Control tab can then suggest configs for the arm, gripper, and calibration sensor components.
3. Save. (See step 3 if the module fails to start.)

### 1. Identify the arm's serial port

```bash
ls /dev/ttyACM* /dev/ttyUSB*
udevadm info -a -n /dev/ttyACM0 | grep -E "SERIAL|PRODUCT|MODEL|idVendor|idProduct"
```

Vendor ID `1a86` (QinHeng/CH340) = the Waveshare bus servo adapter / arm. In our case, the arm was confirmed on **`/dev/ttyACM0`**.

### 2. Power requirements

**USB alone does not power the servos.** USB only powers the Waveshare board's logic/serial chip; the servos need a separate power feed (in our case, 12V from a buck converter off a 4S LiPo).

If the bus finds zero motors even with the correct port, check:
1. Buck converter output voltage matches your servos' rating
2. Power connector into the Waveshare board is seated (not just USB)
3. Daisy-chain cables are fully seated

### 3. Fixing the Viam module's `nlopt` dependency install failure

The module's first-run script installs `nlopt` via a Viam-hosted apt repo that sometimes returns a 404. This causes the module to fail on a loop, producing errors like:

```
ConnectError: [unknown] no node found with api rdk:service:discovery and name <name>
```

**Fix:** install `nlopt` from the standard Debian repo so the script detects it's already present and skips the broken step:

```bash
sudo apt install libnlopt-dev libnlopt0
```

### 4. Configuring the arm component

Requires at minimum a `port`:

```json
{
  "port": "/dev/ttyACM0"
}
```

### 5. Motor ID setup and calibration (Viam-native)

Done through the `devrel:so101:calibration` sensor component's `DoCommand` interface — no LeRobot/Python needed.

**Motor setup**, one servo at a time, in reverse order (gripper → wrist_roll → wrist_flex → elbow_flex → shoulder_lift → shoulder_pan):

```json
{"command": "motor_setup_discover", "motor_name": "gripper"}
{"command": "motor_setup_assign_id", "motor_name": "gripper", "current_id": 1, "target_id": 6, "current_baudrate": 57600}
```

Then verify:
```json
{"command": "motor_setup_verify"}
```

**Calibration:**

```json
{"command": "start"}
{"command": "set_homing"}
{"command": "start_range_recording"}
{"command": "stop_range_recording"}
{"command": "save_calibration"}
```

Copy the calibration file path into both the `arm` and `gripper` components:

```json
{
  "port": "/dev/ttyACM0",
  "calibration_file": "so101_calibration.json"
}
```

**Verify:** run `ping` or `diagnose` on the arm component to confirm all 6 servos respond, then use the arm's Control tab in Viam to command joint positions.

---

## 📺 YouTube

Follow the build process on YouTube: https://www.youtube.com/@AvivMakesRobots
