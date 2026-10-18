"""
arm_server.py — runs on Pi 5
Direct tick control — no IK, no placo.
Hand position maps directly to servo ticks using calibrated ranges.

Run with:
    conda activate lerobot
    sudo kill $(sudo lsof -t /dev/ttyACM0) 2>/dev/null
    python3 ~/arm_server.py
"""

import socket
import json
import time
from scservo_sdk import PortHandler, PacketHandler

PORT_PATH   = "/dev/ttyACM0"
BAUD_RATE   = 1000000
SERVER_PORT = 5556
HOST        = "0.0.0.0"

# Calibrated tick ranges from LeRobot calibration
# shoulder_pan (J1) fixed — not controlled
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

# Neutral positions (arm in safe folded position)
NEUTRAL_TICKS = {
    2: 3082,   # shoulder down (folded)
    3: 2048,   # elbow center
    4: 2241,   # wrist center
    5: 2046,   # wrist roll center
    6: 3068,   # gripper open
}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def pct_to_ticks(servo_id, pct):
    """Map 0.0-1.0 directly to calibrated tick range."""
    lo, hi = SERVO_RANGES[servo_id]
    return int(round(lo + pct * (hi - lo)))

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
        ph.write2ByteTxRx(port, sid, 46, DEFAULT_SPEED)
        ph.write1ByteTxRx(port, sid, 41, DEFAULT_ACCEL)

    # Fix J1, move others to neutral
    ph.write2ByteTxRx(port, 1, 42, J1_FIXED_TICK)
    for sid, tick in NEUTRAL_TICKS.items():
        ph.write2ByteTxRx(port, sid, 42, tick)
    print("Moved to neutral position. Torque enabled.")

    # TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, SERVER_PORT))
    server.listen(1)
    print(f"Listening on port {SERVER_PORT}...")

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
                            # Receive percentages per joint
                            # pcts = [j2, j3, j4, j5, gripper] — all 0.0 to 1.0
                            pcts = msg.get("pcts", [0.0, 0.5, 0.5, 0.5, 1.0])
                            sids = [2, 3, 4, 5, 6]

                            ticks = {}
                            for sid, pct in zip(sids, pcts):
                                tick = pct_to_ticks(sid, pct)
                                ph.write2ByteTxRx(port, sid, 42, tick)
                                ticks[str(sid)] = tick

                            response = {"ok": True, "ticks": ticks}
                            conn.sendall((json.dumps(response) + "\n").encode())

                        elif cmd == "stop":
                            print("Stop received")
                            break

                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        print("Disabling torque...")
        for sid in [1, 2, 3, 4, 5, 6]:
            ph.write1ByteTxRx(port, sid, 40, 0)
        port.closePort()
        try:
            conn.close()
        except Exception:
            pass
        server.close()
        print("Done.")

if __name__ == "__main__":
    main()