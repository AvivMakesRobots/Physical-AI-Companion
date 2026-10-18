#!/usr/bin/env python3
"""
SO-101 arm server
Raspberry Pi side.

Architecture:
    Windows C920 + MediaPipe
            |
            | TCP JSON
            v
    Raspberry Pi + Placo
            |
            v
    Robify servos

J1 is fixed.
J2 is mechanically restricted to 1827..2519 ticks.
"""

import json
import math
import socket
import threading
import time
from pathlib import Path

import numpy as np
from lerobot.model.kinematics import RobotKinematics
import scservo_sdk

# ------------------------------------------------------------
# CONNECTION
# ------------------------------------------------------------

HOST = "0.0.0.0"
PORT = 5555

SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1_000_000

# ------------------------------------------------------------
# SERVO ADDRESSES
# ------------------------------------------------------------

ADDR_ACCEL = 41
ADDR_GOAL_POSITION = 42
ADDR_SPEED = 46
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

# ------------------------------------------------------------
# SERVO LIMITS
# ------------------------------------------------------------

SERVO_LIMITS = {
    1: (1684, 2052),
    2: (1827, 2519),  # HARD J2 LIMIT
    3: (176, 2164),
    4: (1298, 3592),
    5: (0, 4095),
    6: (1855, 3154),
}

J1_FIXED_TICK = 1868

# Current provisional physical calibration.
# These are the encoder ticks corresponding to 0 degrees
# in the official SO-101 new-calibration URDF.
TICKS_AT_URDF_ZERO = {
    2: 1830,
    3: 1170,
    4: 2443,
    5: 2048,
}

TICK_DIRECTION = {
    2: +1,
    3: +1,
    4: +1,
    5: +1,
}

TICK_DEG = 360.0 / 4096.0

# ------------------------------------------------------------
# MOTION PARAMETERS
# ------------------------------------------------------------

SERVO_SPEED = 30
SERVO_ACCEL = 5

MAX_JOINT_STEP_DEG = {
    2: 1.5,
    3: 2.0,
    4: 2.5,
    5: 4.0,
}

# ------------------------------------------------------------
# SO-101 KINEMATICS
# ------------------------------------------------------------

URDF_PATH = Path.home() / "SO-ARM100/Simulation/SO101/so101_new_calib.urdf"
EE_FRAME = "gripper_frame_link"

IK_JOINTS = [
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# ------------------------------------------------------------
# STATE
# ------------------------------------------------------------

port_handler = None
packet_handler = None

arm_enabled = False
state_lock = threading.Lock()

# q_deg is always the latest commanded/model joint state.
q_deg = np.zeros(4, dtype=float)

kin = None
last_target_xyz = None


def swap_bytes(value):
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


def clamp(value, low, high):
    return max(low, min(high, value))


def ticks_to_urdf_deg(joint_id, ticks):
    return (
        (ticks - TICKS_AT_URDF_ZERO[joint_id])
        * TICK_DEG
        * TICK_DIRECTION[joint_id]
    )


def urdf_deg_to_ticks(joint_id, angle_deg):
    ticks = (
        TICKS_AT_URDF_ZERO[joint_id]
        + TICK_DIRECTION[joint_id] * angle_deg / TICK_DEG
    )

    low, high = SERVO_LIMITS[joint_id]
    return int(round(clamp(ticks, low, high)))


def read_position(joint_id):
    raw, result, error = packet_handler.read2ByteTxRx(
        port_handler, joint_id, ADDR_PRESENT_POSITION
    )

    if result != scservo_sdk.COMM_SUCCESS:
        raise RuntimeError(
            f"Read J{joint_id} failed: "
            f"{packet_handler.getTxRxResult(result)}"
        )

    if error != 0:
        raise RuntimeError(
            f"Read J{joint_id} error: "
            f"{packet_handler.getRxPacketError(error)}"
        )

    # Robify servo reports these 16-bit values byte-swapped.
    return swap_bytes(raw)


def read_positions():
    return {jid: read_position(jid) for jid in range(1, 7)}


def write_position(joint_id, ticks):
    ticks = int(clamp(ticks, *SERVO_LIMITS[joint_id]))

    result, error = packet_handler.write2ByteTxRx(
        port_handler,
        joint_id,
        ADDR_GOAL_POSITION,
        ticks,
    )

    if result != scservo_sdk.COMM_SUCCESS:
        raise RuntimeError(
            f"Write J{joint_id} failed: "
            f"{packet_handler.getTxRxResult(result)}"
        )

    if error != 0:
        raise RuntimeError(
            f"Write J{joint_id} error: "
            f"{packet_handler.getRxPacketError(error)}"
        )


def write_motion(joint_id, ticks, speed=SERVO_SPEED, accel=SERVO_ACCEL):
    ticks = int(clamp(ticks, *SERVO_LIMITS[joint_id]))

    packet_handler.write1ByteTxRx(
        port_handler, joint_id, ADDR_ACCEL, int(accel)
    )
    packet_handler.write2ByteTxRx(
        port_handler, joint_id, ADDR_SPEED, int(speed)
    )
    write_position(joint_id, ticks)


def torque(enable):
    value = 1 if enable else 0
    for jid in range(1, 7):
        result, error = packet_handler.write1ByteTxRx(
            port_handler, jid, ADDR_TORQUE_ENABLE, value
        )
        if result != scservo_sdk.COMM_SUCCESS:
            raise RuntimeError(
                f"Torque J{jid} failed: "
                f"{packet_handler.getTxRxResult(result)}"
            )
        if error != 0:
            raise RuntimeError(
                f"Torque J{jid} error: "
                f"{packet_handler.getRxPacketError(error)}"
            )


def ticks_to_q(ticks):
    return np.array(
        [
            ticks_to_urdf_deg(2, ticks[2]),
            ticks_to_urdf_deg(3, ticks[3]),
            ticks_to_urdf_deg(4, ticks[4]),
            ticks_to_urdf_deg(5, ticks[5]),
        ],
        dtype=float,
    )


def initialize_kinematics():
    global kin, q_deg, last_target_xyz

    if not URDF_PATH.exists():
        raise FileNotFoundError(f"Missing URDF: {URDF_PATH}")

    kin = RobotKinematics(
        urdf_path=str(URDF_PATH),
        target_frame_name=EE_FRAME,
        joint_names=IK_JOINTS,
    )

    ticks = read_positions()
    q_deg = ticks_to_q(ticks)

    T = kin.forward_kinematics(q_deg)
    last_target_xyz = np.array(T[:3, 3], dtype=float)

    return ticks, q_deg.copy(), last_target_xyz.copy()


def fk_xyz():
    T = kin.forward_kinematics(q_deg)
    return np.array(T[:3, 3], dtype=float)


def solve_ik(target_xyz):
    global q_deg

    target_xyz = np.asarray(target_xyz, dtype=float)

    # Start from the current modeled pose.
    current_T = kin.forward_kinematics(q_deg)

    # Preserve the current orientation. We are doing position-only control,
    # but RobotKinematics expects a full 4x4 transform.
    target_T = current_T.copy()
    target_T[:3, 3] = target_xyz

    solved = kin.inverse_kinematics(
        current_joint_pos=q_deg.copy(),
        desired_ee_pose=target_T,
        position_weight=1.0,
        orientation_weight=0.0,
        max_iters=8,
    )

    solved = np.asarray(solved[:4], dtype=float)

    # Official SO-101 URDF joint limits.
    urdf_limits = {
        2: (-100.0, 100.0),
        3: (-math.degrees(1.69), math.degrees(1.69)),
        4: (-math.degrees(1.65806), math.degrees(1.65806)),
        5: (-math.degrees(2.74385), math.degrees(2.84121)),
    }

    for i, jid in enumerate([2, 3, 4, 5]):
        solved[i] = clamp(
            solved[i],
            urdf_limits[jid][0],
            urdf_limits[jid][1],
        )

        step = solved[i] - q_deg[i]
        solved[i] = q_deg[i] + clamp(
            step,
            -MAX_JOINT_STEP_DEG[jid],
            MAX_JOINT_STEP_DEG[jid],
        )

    # Convert to encoder ticks and back once so the internal state exactly
    # matches the physical values that will be commanded.
    tick_dict = {
        2: urdf_deg_to_ticks(2, solved[0]),
        3: urdf_deg_to_ticks(3, solved[1]),
        4: urdf_deg_to_ticks(4, solved[2]),
        5: urdf_deg_to_ticks(5, solved[3]),
    }

    q_deg = ticks_to_q(tick_dict)

    return q_deg.copy(), tick_dict

def command_xyz(target_xyz):
    global last_target_xyz

    target_xyz = np.asarray(target_xyz, dtype=float)

    # Conservative Cartesian workspace.
    target_xyz = np.clip(
        target_xyz,
        np.array([0.09, -0.03, -0.08]),
        np.array([0.13, 0.03, -0.04]),
    )

    current = fk_xyz()

    # Limit Cartesian movement per command.
    delta = target_xyz - current
    distance = float(np.linalg.norm(delta))

    if distance > 0.002:
        target_xyz = current + delta * (0.002 / distance)

    q, ticks = solve_ik(target_xyz)

    if not arm_enabled:
        return {
            "ok": True,
            "enabled": False,
            "q_deg": q.tolist(),
            "ticks": {str(k): int(v) for k, v in ticks.items()},
            "xyz": fk_xyz().tolist(),
        }

    # J1 is fixed. J6 is deliberately not commanded here.
    for jid in [2, 3, 4, 5]:
        write_motion(jid, ticks[jid])

    last_target_xyz = target_xyz.copy()

    return {
        "ok": True,
        "enabled": True,
        "q_deg": q.tolist(),
        "ticks": {str(k): int(v) for k, v in ticks.items()},
        "xyz": fk_xyz().tolist(),
    }


def handle_command(message):
    global arm_enabled

    command = message.get("command")

    if command == "ping":
        return {"ok": True, "enabled": arm_enabled}

    if command == "read":
        ticks = read_positions()
        return {
            "ok": True,
            "ticks": {str(k): int(v) for k, v in ticks.items()},
            "q_deg": ticks_to_q(ticks).tolist(),
            "xyz": fk_xyz().tolist(),
        }

    if command == "enable":
        arm_enabled = True
        return {"ok": True, "enabled": True}

    if command == "disable":
        arm_enabled = False
        return {"ok": True, "enabled": False}

    if command == "torque":
        enable = bool(message.get("enable", False))
        torque(enable)
        arm_enabled = enable
        return {"ok": True, "enabled": arm_enabled}

    if command == "xyz":
        target = message.get("xyz")
        if target is None or len(target) != 3:
            return {"ok": False, "error": "xyz must contain 3 values"}
        return command_xyz(target)

    if command == "ticks":
        # Manual low-level testing only. Still clamps all limits.
        ticks = message.get("ticks", {})
        for jid in [2, 3, 4, 5]:
            key = str(jid)
            if key in ticks and arm_enabled:
                write_motion(jid, int(ticks[key]))
        return {"ok": True}

    if command == "stop":
        arm_enabled = False
        torque(False)
        return {"ok": True, "enabled": False}

    return {"ok": False, "error": f"unknown command: {command}"}


def client_thread(conn, addr):
    print("Client connected:", addr)

    with conn:
        buffer = b""

        while True:
            data = conn.recv(4096)
            if not data:
                break

            buffer += data

            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)

                if not raw.strip():
                    continue

                try:
                    message = json.loads(raw.decode("utf-8"))
                    response = handle_command(message)
                except Exception as exc:
                    response = {
                        "ok": False,
                        "error": str(exc),
                    }

                conn.sendall(
                    (json.dumps(response) + "\n").encode("utf-8")
                )

    print("Client disconnected:", addr)


def main():
    global port_handler, packet_handler

    print("Opening servo bus:", SERVO_PORT)

    port_handler = scservo_sdk.PortHandler(SERVO_PORT)
    packet_handler = scservo_sdk.PacketHandler(1.0)

    if not port_handler.openPort():
        raise RuntimeError("Could not open servo port.")

    if not port_handler.setBaudRate(SERVO_BAUD):
        raise RuntimeError("Could not set servo baud rate.")

    ticks, q, xyz = initialize_kinematics()

    print("INITIAL SERVO TICKS:", ticks)
    print("INITIAL J2-J5 DEG:", q)
    print("INITIAL EE XYZ:", xyz)
    print("J2 HARD LIMIT: 1827..2519")
    print("ARM STARTS DISABLED.")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"Listening on {HOST}:{PORT}")

    try:
        while True:
            conn, addr = server.accept()
            client_thread(conn, addr)
    finally:
        arm_enabled = False
        try:
            torque(False)
        except Exception:
            pass

        server.close()
        port_handler.closePort()


if __name__ == "__main__":
    main()
