"""
arm_server.py — runs on Pi 5
Direct tick control — no IK, no placo.
Arm hand position maps directly to servo ticks using calibrated ranges.
Base hand commands drive the SCUTTLE mobile base via Viam.

Run with:
    conda activate lerobot
    sudo kill $(sudo lsof -t /dev/ttyACM0) 2>/dev/null
    python3 ~/Physical-AI-Companion/pi_scripts/arm_server.py

Fill in VIAM_ADDRESS, VIAM_API_KEY, VIAM_API_KEY_ID below
(Viam app -> your machine -> CONNECT tab -> Python).
"""

import asyncio
import socket
import json
import time
import threading
from scservo_sdk import PortHandler, PacketHandler

# ── Viam credentials ────────────────────────────────────────────────────────
VIAM_ADDRESS    = ""   # e.g. robot-abc123.viam.cloud
VIAM_API_KEY    = ""
VIAM_API_KEY_ID = ""
BASE_NAME       = "base"            # must match the base component name in Viam exactly
# ────────────────────────────────────────────────────────────────────────────

PORT_PATH   = "/dev/ttyACM0"
BAUD_RATE   = 1000000
SERVER_PORT = 5556
HOST        = "0.0.0.0"

# Calibrated tick ranges from LeRobot calibration
SERVO_RANGES = {
    2: (1703, 3090),   # shoulder_lift  — UP=1700, DOWN=3082
    3: (0,    4094),   # elbow_flex
    4: (1124, 3359),   # wrist_flex
    5: (0,    4095),   # wrist_roll
    6: (1735, 3068),   # gripper
}

J1_FIXED_TICK = 2047
DEFAULT_SPEED = 50
DEFAULT_ACCEL = 10

# Per-servo overrides (speed in steps/s, accel 0-254).
# J2 (shoulder_lift) is an STS3250; J1, J3-J6 are STS3215. Tune each one
# separately until they feel matched. Servos missing here use the defaults.
SERVO_SPEED = {1: 600, 2: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1500}
SERVO_ACCEL = {1: 40,  2: 50,   3: 50,   4: 50,   5: 50,   6: 50}

NEUTRAL_TICKS = {
    2: 3082,
    3: 2048,
    4: 2241,
    5: 2046,
    6: 3068,
}

# Base speed scaling — 30% power. Viam set_power takes -1.0 to 1.0.
BASE_MAX_LINEAR  = 0.30
BASE_MAX_ANGULAR = 0.30

# Safety: if no base command arrives for this long while the base is moving,
# stop it (Wi-Fi drop, PC script crash, etc.). The PC sends at 20 Hz.
BASE_TIMEOUT_S = 0.5

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def pct_to_ticks(servo_id, pct):
    lo, hi = SERVO_RANGES[servo_id]
    return int(round(lo + pct * (hi - lo)))

# ── Shared base command state (written by TCP thread, read by Viam loop) ────
class BaseState:
    def __init__(self):
        self._lock   = threading.Lock()
        self.linear  = 0.0
        self.angular = 0.0
        self.updated = False
        self.stamp   = time.monotonic()

    def set(self, linear, angular):
        with self._lock:
            self.linear  = linear
            self.angular = angular
            self.updated = True
            self.stamp   = time.monotonic()

    def snapshot(self):
        """Returns (linear, angular, updated, seconds_since_last_set) and clears 'updated'."""
        with self._lock:
            l, a, u = self.linear, self.angular, self.updated
            age = time.monotonic() - self.stamp
            self.updated = False
            return l, a, u, age

base_state = BaseState()

# ── Viam base control loop ────────────────────────────────────────────────
async def viam_base_loop():
    """Connects to Viam and drives the base whenever base_state has a command."""
    try:
        from viam.robot.client import RobotClient
        from viam.components.base import Base
        from viam.proto.common import Vector3
    except ImportError:
        print("WARNING: viam-sdk not installed — base control disabled.")
        print("Install with: pip install viam-sdk --break-system-packages")
        return

    if not (VIAM_ADDRESS and VIAM_API_KEY and VIAM_API_KEY_ID):
        print("WARNING: Viam credentials are empty — base control disabled.")
        return

    print("Connecting to Viam for base control...")
    robot = None
    try:
        robot = await RobotClient.at_address(
            VIAM_ADDRESS,
            RobotClient.Options.with_api_key(
                api_key=VIAM_API_KEY,
                api_key_id=VIAM_API_KEY_ID,
            ),
        )
        base = Base.from_robot(robot, BASE_NAME)
        print("Viam base connected.")
    except Exception as e:
        print(f"WARNING: Could not connect to Viam base: {e!r}")
        print("Base control will be unavailable.")
        if robot is not None:
            try:
                await robot.close()
            except Exception:
                pass
        return

    moving = False
    try:
        while True:
            linear, angular, updated, age = base_state.snapshot()

            if updated:
                try:
                    if abs(linear) < 0.01 and abs(angular) < 0.01:
                        if moving:
                            await base.stop()
                            moving = False
                    else:
                        moving = True
                        # set_power: linear forward (y) = positive, angular left (z) = positive
                        await base.set_power(
                            linear=Vector3(y=clamp(linear,  -BASE_MAX_LINEAR,  BASE_MAX_LINEAR)),
                            angular=Vector3(z=clamp(angular, -BASE_MAX_ANGULAR, BASE_MAX_ANGULAR)),
                        )
                except Exception as e:
                    print(f"Base command error: {e!r}")

            elif moving and age > BASE_TIMEOUT_S:
                # No fresh command while driving — fail safe.
                try:
                    await base.stop()
                    print("Base command timeout — stopped.")
                except Exception as e:
                    print(f"Base stop error: {e!r}")
                moving = False

            await asyncio.sleep(0.05)  # 20Hz

    finally:
        try:
            await base.stop()
            await robot.close()
        except Exception:
            pass

def run_viam_loop():
    """Run the Viam async loop in a background thread."""
    asyncio.run(viam_base_loop())

# ── Main TCP server ───────────────────────────────────────────────────────
def main():
    port = PortHandler(PORT_PATH)
    ph   = PacketHandler(0)

    if not port.openPort():
        print(f"ERROR: Could not open {PORT_PATH}")
        return
    port.setBaudRate(BAUD_RATE)
    print(f"Servo bus open on {PORT_PATH}")

    # Enable torque and set speed/accel
    for sid in [1, 2, 3, 4, 5, 6]:
        ph.write1ByteTxRx(port, sid, 40, 1)
        ph.write2ByteTxRx(port, sid, 46, SERVO_SPEED.get(sid, DEFAULT_SPEED))
        ph.write1ByteTxRx(port, sid, 41, SERVO_ACCEL.get(sid, DEFAULT_ACCEL))

    ph.write2ByteTxRx(port, 1, 42, J1_FIXED_TICK)
    for sid, tick in NEUTRAL_TICKS.items():
        ph.write2ByteTxRx(port, sid, 42, tick)
    print("Moved to neutral. Torque enabled.")

    # Start Viam base loop in background thread
    viam_thread = threading.Thread(target=run_viam_loop, daemon=True)
    viam_thread.start()

    # TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, SERVER_PORT))
    server.listen(1)
    print(f"Listening on port {SERVER_PORT}...")

    conn = None
    try:
        conn, addr = server.accept()
        print(f"Connected from {addr}")
        conn.settimeout(5.0)
        buffer = ""

        while True:
            try:
                data = conn.recv(4096).decode("utf-8")
                if not data:
                    print("Client disconnected")
                    break

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        cmd = msg.get("command", "")

                        if cmd == "ping":
                            conn.sendall((json.dumps({"ok": True}) + "\n").encode())

                        elif cmd == "pcts":
                            # Arm joints
                            pcts = msg.get("pcts", [0.0, 0.5, 0.5, 0.5, 1.0])
                            sids = [2, 3, 4, 5, 6]
                            ticks = {}
                            for sid, pct in zip(sids, pcts):
                                tick = pct_to_ticks(sid, pct)
                                ph.write2ByteTxRx(port, sid, 42, tick)
                                ticks[str(sid)] = tick

                            # Base command (optional)
                            base = msg.get("base")
                            if base:
                                base_state.set(
                                    float(base.get("linear",  0.0)),
                                    float(base.get("angular", 0.0))
                                )

                            response = {"ok": True, "ticks": ticks}
                            conn.sendall((json.dumps(response) + "\n").encode())

                        elif cmd == "stop":
                            print("Stop received")
                            base_state.set(0.0, 0.0)
                            raise StopIteration

                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue
            except StopIteration:
                break

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        base_state.set(0.0, 0.0)
        time.sleep(0.2)   # let the Viam loop send the final stop
        print("Disabling torque...")
        for sid in [1, 2, 3, 4, 5, 6]:
            ph.write1ByteTxRx(port, sid, 40, 0)
        port.closePort()
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass
        server.close()
        print("Done.")

if __name__ == "__main__":
    main()
