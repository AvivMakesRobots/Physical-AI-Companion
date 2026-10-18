"""
ik_server.py — runs on Raspberry Pi 5

Hybrid teleoperation controller for the custom Robify-servo SO-101.

Controls:
    J2 = direct hand-depth control
    J3 = direct hand-vertical control
    J4 = direct hand-horizontal control
    J5 = kinematic orientation control
    J6 = pinch/gripper control
    J1 = fixed

The Windows client sends:
    palm = [image_x, image_y, hand_size]
    pinch = thumb/index distance

The controller intentionally does NOT use Cartesian position IK to
choose J2/J3/J4. Those three joints are deliberately commanded
directly from the hand inputs.

J5 is selected with kinematics so the gripper tries to maintain the
orientation captured when SPACE is pressed.

Run with:
    cd /home/aviv/SO-ARM100/Simulation/SO101
    source ~/.venv-so101/bin/activate
    source ~/smoke_env.sh
    sudo kill $(sudo lsof -t /dev/ttyACM0) 2>/dev/null
    python3 ~/ik_server.py
"""

import socket
import json
from pathlib import Path

import numpy as np
from scservo_sdk import PortHandler, PacketHandler


# ============================================================
# CONNECTION / HARDWARE
# ============================================================

URDF_PATH = Path(
    "/home/aviv/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"
)

PORT_PATH = "/dev/ttyACM0"
BAUD_RATE = 1_000_000

SERVER_PORT = 5556
HOST = "0.0.0.0"


# ============================================================
# SERVO CALIBRATION
# ============================================================

TICK_DEG = 360.0 / 4096.0

J1_FIXED_TICK = 1868

TICKS_AT_URDF_ZERO = {
    2: 1700,
    3: 4042,
    4: 2334,
    5: 2046,
}

TICK_DIRECTION = {
    2: +1,
    3: +1,
    4: +1,
    5: +1,
}

# Physical/safe limits based on your measured robot.
SERVO_LIMITS = {
    1: (1684, 2052),
    2: (1703, 3090),
    3: (0, 4094),
    4: (1124, 3359),
    5: (0, 4095),
    6: (1735, 3068),
}


# ============================================================
# REGISTERS
# ============================================================

TORQUE_ENABLE_ADDR = 40
GOAL_POSITION_ADDR = 42
RUNNING_SPEED_ADDR = 46
ACCELERATION_ADDR = 41
PRESENT_POSITION_ADDR = 56


# ============================================================
# GRIPPER
# ============================================================

GRIPPER_OPEN_TICK = 3068
GRIPPER_CLOSED_TICK = 1735

PINCH_OPEN = 0.16
PINCH_CLOSED = 0.030

GRIPPER_ALPHA = 0.35


# ============================================================
# DIRECT HAND CONTROL
# ============================================================

# Hand image-size range used for direct J2 control.
MIN_HAND_IMAGE_SIZE = 0.15
MAX_HAND_IMAGE_SIZE = 0.85

# Direct joint targets:
#
# J2:
#   small hand = low end
#   large hand = high end
#
# J3:
#   top of frame    = + extreme
#   bottom of frame = - extreme
#
# J4:
#   left side  = minimum
#   right side = maximum
#
# These deliberately use the full measured servo ranges, with
# J2 retaining its existing safe mechanical limit.
J2_MIN_TICK = 1827
J2_MAX_TICK = 2519

J3_TOP_TICK = 2164
J3_BOTTOM_TICK = 176

J4_LEFT_TICK = 1298
J4_RIGHT_TICK = 3592


# ============================================================
# FILTERING / MOTION
# ============================================================

HAND_ALPHA = 0.40

# Maximum joint change per incoming control frame.
# At 20 Hz these still permit reasonably fast movement while
# reducing jumps caused by MediaPipe noise.
MAX_JOINT_STEP_DEG = {
    2: 4.0,
    3: 5.0,
    4: 5.0,
    5: 8.0,
}

DEFAULT_SPEED = 50
DEFAULT_ACCEL = 10

IK_JOINT_IDS = [2, 3, 4, 5]

IK_JOINTS = [
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


# ============================================================
# HELPERS
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def swap_bytes(value):
    value = int(value) & 0xFFFF
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


def lerp(a, b, t):
    return a + (b - a) * t


def urdf_deg_to_ticks(joint_id, angle_deg):
    zero = TICKS_AT_URDF_ZERO[joint_id]
    direction = TICK_DIRECTION[joint_id]

    ticks = zero + direction * angle_deg / TICK_DEG

    lo, hi = SERVO_LIMITS[joint_id]

    return int(round(clamp(ticks, lo, hi)))


def ticks_to_urdf_deg(joint_id, ticks):
    zero = TICKS_AT_URDF_ZERO[joint_id]
    direction = TICK_DIRECTION[joint_id]

    return (ticks - zero) * TICK_DEG * direction


def make_gripper_tick(pinch):
    t = (
        (pinch - PINCH_CLOSED)
        / (PINCH_OPEN - PINCH_CLOSED)
    )

    t = clamp(t, 0.0, 1.0)

    return int(round(
        GRIPPER_OPEN_TICK * t
        + GRIPPER_CLOSED_TICK * (1.0 - t)
    ))


def read_present_ticks(port, packet, joint_id):
    raw, comm_result, error = packet.read2ByteTxRx(
        port,
        joint_id,
        PRESENT_POSITION_ADDR,
    )

    if comm_result != 0:
        raise RuntimeError(
            f"J{joint_id} read communication error: {comm_result}"
        )

    if error != 0:
        raise RuntimeError(
            f"J{joint_id} read servo error: {error}"
        )

    return swap_bytes(raw)


def read_actual_ik_ticks(port, packet):
    return {
        sid: read_present_ticks(port, packet, sid)
        for sid in IK_JOINT_IDS
    }


def limit_joint_step(joint_id, target_tick, previous_tick):
    max_step = int(
        MAX_JOINT_STEP_DEG[joint_id] / TICK_DEG
    )

    diff = target_tick - previous_tick

    if abs(diff) > max_step:
        target_tick = (
            previous_tick
            + max_step * (1 if diff > 0 else -1)
        )

    lo, hi = SERVO_LIMITS[joint_id]

    return clamp(target_tick, lo, hi)


# ============================================================
# MAIN
# ============================================================

def main():

    print(f"Loading URDF from {URDF_PATH}...")

    if not URDF_PATH.exists():
        raise FileNotFoundError(
            f"URDF not found at {URDF_PATH}"
        )

    from lerobot.model.kinematics import RobotKinematics

    kin = RobotKinematics(
        urdf_path=str(URDF_PATH),
        target_frame_name="gripper_frame_link",
        joint_names=IK_JOINTS,
    )

    print("IK ready.")

    # --------------------------------------------------------
    # Servo bus
    # --------------------------------------------------------

    port = PortHandler(PORT_PATH)
    packet = PacketHandler(0)

    if not port.openPort():
        raise RuntimeError(
            f"Could not open {PORT_PATH}"
        )

    if not port.setBaudRate(BAUD_RATE):
        raise RuntimeError(
            f"Could not set baud rate to {BAUD_RATE}"
        )

    print(
        f"Servo bus open on {PORT_PATH} "
        f"at {BAUD_RATE} baud."
    )

    # --------------------------------------------------------
    # Enable servos
    # --------------------------------------------------------

    for sid in [1, 2, 3, 4, 5, 6]:

        packet.write1ByteTxRx(
            port,
            sid,
            TORQUE_ENABLE_ADDR,
            1,
        )

        packet.write2ByteTxRx(
            port,
            sid,
            RUNNING_SPEED_ADDR,
            DEFAULT_SPEED,
        )

        packet.write1ByteTxRx(
            port,
            sid,
            ACCELERATION_ADDR,
            DEFAULT_ACCEL,
        )

    # Keep J1 fixed.
    packet.write2ByteTxRx(
        port,
        1,
        GOAL_POSITION_ADDR,
        J1_FIXED_TICK,
    )

    print("Servos ready.")

    # --------------------------------------------------------
    # Initial actual positions
    # --------------------------------------------------------

    try:

        prev_ticks = read_actual_ik_ticks(
            port,
            packet,
        )

        print(
            "Actual starting ticks:",
            prev_ticks,
        )

    except Exception as exc:

        print(
            "Could not read starting positions:",
            exc,
        )

        prev_ticks = {
            sid: TICKS_AT_URDF_ZERO[sid]
            for sid in IK_JOINT_IDS
        }

        print(
            "Using calibrated URDF-zero ticks."
        )

    # --------------------------------------------------------
    # Controller state
    # --------------------------------------------------------

    hand_ref = None
    filt_hand = None

    # Orientation captured when SPACE is pressed.
    center_orientation = None

    last_gripper = GRIPPER_OPEN_TICK

    # --------------------------------------------------------
    # TCP server
    # --------------------------------------------------------

    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM,
    )

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1,
    )

    server.bind(
        (HOST, SERVER_PORT)
    )

    server.listen(1)

    print(
        f"Listening on port {SERVER_PORT}..."
    )

    conn = None

    try:

        conn, addr = server.accept()

        print(
            f"Connected from {addr}"
        )

        conn.settimeout(5.0)

        buffer = ""

        while True:

            try:

                data = conn.recv(4096).decode(
                    "utf-8"
                )

                if not data:
                    print("Client disconnected.")
                    break

                buffer += data

                while "\n" in buffer:

                    line, buffer = buffer.split(
                        "\n",
                        1,
                    )

                    line = line.strip()

                    if not line:
                        continue

                    try:

                        msg = json.loads(line)
                        cmd = msg.get(
                            "command",
                            "",
                        )

                        # ====================================
                        # PING
                        # ====================================

                        if cmd == "ping":

                            conn.sendall(
                                (
                                    json.dumps({
                                        "ok": True
                                    })
                                    + "\n"
                                ).encode()
                            )

                        # ====================================
                        # CENTER
                        # ====================================

                        elif cmd == "center":

                            palm = np.array(
                                msg["palm"],
                                dtype=float,
                            )

                            try:

                                actual_ticks = (
                                    read_actual_ik_ticks(
                                        port,
                                        packet,
                                    )
                                )

                                prev_ticks = (
                                    actual_ticks.copy()
                                )

                            except Exception as exc:

                                print(
                                    "Center read failed:",
                                    exc,
                                )

                                actual_ticks = (
                                    prev_ticks.copy()
                                )

                            current_q = np.array([
                                ticks_to_urdf_deg(
                                    2,
                                    actual_ticks[2],
                                ),
                                ticks_to_urdf_deg(
                                    3,
                                    actual_ticks[3],
                                ),
                                ticks_to_urdf_deg(
                                    4,
                                    actual_ticks[4],
                                ),
                                ticks_to_urdf_deg(
                                    5,
                                    actual_ticks[5],
                                ),
                            ])

                            # Save the actual current tool
                            # orientation as the desired
                            # orientation for J5 compensation.
                            fk = kin.forward_kinematics(
                                current_q
                            )

                            center_orientation = (
                                fk[:3, :3].copy()
                            )

                            hand_ref = palm.copy()
                            filt_hand = palm.copy()

                            print(
                                "Hybrid controller centered."
                            )

                            print(
                                "  Actual ticks:",
                                actual_ticks,
                            )

                            print(
                                "  J2/J3/J4 = direct hand control"
                            )

                            print(
                                "  J5 = orientation IK"
                            )

                            conn.sendall(
                                (
                                    json.dumps({
                                        "ok": True
                                    })
                                    + "\n"
                                ).encode()
                            )

                        # ====================================
                        # HAND CONTROL
                        # ====================================

                        elif cmd == "xyz":

                            if hand_ref is None:
                                continue

                            palm = np.array(
                                msg["palm"],
                                dtype=float,
                            )

                            pinch = float(
                                msg["pinch"]
                            )

                            # Smooth MediaPipe noise.
                            if filt_hand is None:
                                filt_hand = palm.copy()

                            filt_hand = (
                                HAND_ALPHA * palm
                                + (1.0 - HAND_ALPHA)
                                * filt_hand
                            )

                            image_x = clamp(
                                float(filt_hand[0]),
                                0.0,
                                1.0,
                            )

                            image_y = clamp(
                                float(filt_hand[1]),
                                0.0,
                                1.0,
                            )

                            hand_size = clamp(
                                float(filt_hand[2]),
                                MIN_HAND_IMAGE_SIZE,
                                MAX_HAND_IMAGE_SIZE,
                            )

                            # --------------------------------
                            # DIRECT JOINT CONTROL
                            #
                            # J2 <- hand size
                            # J3 <- vertical hand position
                            # J4 <- horizontal hand position
                            # --------------------------------

                            size_t = (
                                hand_size
                                - MIN_HAND_IMAGE_SIZE
                            ) / (
                                MAX_HAND_IMAGE_SIZE
                                - MIN_HAND_IMAGE_SIZE
                            )

                            desired_j2 = int(round(
                                lerp(
                                    J2_MIN_TICK,
                                    J2_MAX_TICK,
                                    size_t,
                                )
                            ))

                            # Top of frame = +J3
                            # Bottom of frame = -J3
                            desired_j3 = int(round(
                                lerp(
                                    J3_TOP_TICK,
                                    J3_BOTTOM_TICK,
                                    image_y,
                                )
                            ))

                            # Left = minimum J4
                            # Right = maximum J4
                            desired_j4 = int(round(
                                lerp(
                                    J4_LEFT_TICK,
                                    J4_RIGHT_TICK,
                                    image_x,
                                )
                            ))

                            desired_j2 = clamp(
                                desired_j2,
                                J2_MIN_TICK,
                                J2_MAX_TICK,
                            )

                            desired_j3 = clamp(
                                desired_j3,
                                J3_BOTTOM_TICK,
                                J3_TOP_TICK,
                            )

                            desired_j4 = clamp(
                                desired_j4,
                                J4_LEFT_TICK,
                                J4_RIGHT_TICK,
                            )

                            # Apply movement limits so a single
                            # MediaPipe jump cannot command a huge
                            # physical jump.
                            j2_tick = limit_joint_step(
                                2,
                                desired_j2,
                                prev_ticks[2],
                            )

                            j3_tick = limit_joint_step(
                                3,
                                desired_j3,
                                prev_ticks[3],
                            )

                            j4_tick = limit_joint_step(
                                4,
                                desired_j4,
                                prev_ticks[4],
                            )

                            # --------------------------------
                            # J5 ORIENTATION IK
                            #
                            # J2/J3/J4 are now fixed at their direct
                            # hand-controlled targets.
                            #
                            # Build that pose, then let the
                            # kinematics solver determine J5 needed
                            # to reproduce the orientation captured
                            # at SPACE.
                            # --------------------------------

                            if center_orientation is None:
                                center_orientation = (
                                    kin.forward_kinematics(
                                        np.array([
                                            ticks_to_urdf_deg(
                                                2,
                                                j2_tick,
                                            ),
                                            ticks_to_urdf_deg(
                                                3,
                                                j3_tick,
                                            ),
                                            ticks_to_urdf_deg(
                                                4,
                                                j4_tick,
                                            ),
                                            ticks_to_urdf_deg(
                                                5,
                                                prev_ticks[5],
                                            ),
                                        ])
                                    )[:3, :3].copy()
                                )

                            direct_q = np.array([
                                ticks_to_urdf_deg(
                                    2,
                                    j2_tick,
                                ),
                                ticks_to_urdf_deg(
                                    3,
                                    j3_tick,
                                ),
                                ticks_to_urdf_deg(
                                    4,
                                    j4_tick,
                                ),
                                ticks_to_urdf_deg(
                                    5,
                                    prev_ticks[5],
                                ),
                            ])

                            direct_fk = (
                                kin.forward_kinematics(
                                    direct_q
                                )
                            )

                            desired_pose = np.eye(4)

                            # Preserve the orientation recorded at
                            # center while keeping the XYZ position
                            # generated by the direct J2/J3/J4 pose.
                            desired_pose[:3, :3] = (
                                center_orientation
                            )

                            desired_pose[:3, 3] = (
                                direct_fk[:3, 3]
                            )

                            try:

                                ik_solution = (
                                    kin.inverse_kinematics(
                                        direct_q,
                                        desired_pose,
                                        position_weight=1.0,
                                        orientation_weight=1.0,
                                    )
                                )

                                desired_j5 = (
                                    urdf_deg_to_ticks(
                                        5,
                                        ik_solution[3],
                                    )
                                )

                            except Exception as exc:

                                print(
                                    "J5 IK failed:",
                                    exc,
                                )

                                desired_j5 = (
                                    prev_ticks[5]
                                )

                            j5_tick = limit_joint_step(
                                5,
                                desired_j5,
                                prev_ticks[5],
                            )

                            # --------------------------------
                            # Final ticks
                            # --------------------------------

                            ticks = {
                                2: j2_tick,
                                3: j3_tick,
                                4: j4_tick,
                                5: j5_tick,
                            }

                            # --------------------------------
                            # Gripper
                            # --------------------------------

                            gripper_target = (
                                make_gripper_tick(
                                    pinch
                                )
                            )

                            last_gripper = int(
                                round(
                                    GRIPPER_ALPHA
                                    * gripper_target
                                    + (
                                        1.0
                                        - GRIPPER_ALPHA
                                    )
                                    * last_gripper
                                )
                            )

                            last_gripper = clamp(
                                last_gripper,
                                SERVO_LIMITS[6][0],
                                SERVO_LIMITS[6][1],
                            )

                            ticks[6] = last_gripper

                            # --------------------------------
                            # Send commands
                            # --------------------------------

                            for sid, tick in ticks.items():

                                packet.write2ByteTxRx(
                                    port,
                                    sid,
                                    GOAL_POSITION_ADDR,
                                    tick,
                                )

                            prev_ticks[2] = j2_tick
                            prev_ticks[3] = j3_tick
                            prev_ticks[4] = j4_tick
                            prev_ticks[5] = j5_tick

                            # --------------------------------
                            # Compute displayed FK
                            # --------------------------------

                            final_q = np.array([
                                ticks_to_urdf_deg(
                                    2,
                                    j2_tick,
                                ),
                                ticks_to_urdf_deg(
                                    3,
                                    j3_tick,
                                ),
                                ticks_to_urdf_deg(
                                    4,
                                    j4_tick,
                                ),
                                ticks_to_urdf_deg(
                                    5,
                                    j5_tick,
                                ),
                            ])

                            final_fk = (
                                kin.forward_kinematics(
                                    final_q
                                )
                            )

                            joint_degs = {
                                str(sid): round(
                                    ticks_to_urdf_deg(
                                        sid,
                                        tick,
                                    ),
                                    1,
                                )
                                for sid, tick
                                in ticks.items()
                                if sid != 6
                            }

                            response = {
                                "ok": True,

                                "xyz": [
                                    round(
                                        float(v),
                                        3,
                                    )
                                    for v
                                    in final_fk[:3, 3]
                                ],

                                "ticks": {
                                    str(k): int(v)
                                    for k, v
                                    in ticks.items()
                                },

                                "degs": joint_degs,
                            }

                            conn.sendall(
                                (
                                    json.dumps(
                                        response
                                    )
                                    + "\n"
                                ).encode()
                            )

                        # ====================================
                        # STOP
                        # ====================================

                        elif cmd == "stop":

                            print("Stop received.")
                            break

                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue

    except KeyboardInterrupt:

        print(
            "\nShutting down..."
        )

    finally:

        print(
            "Disabling torque..."
        )

        for sid in [1, 2, 3, 4, 5, 6]:

            try:
                packet.write1ByteTxRx(
                    port,
                    sid,
                    TORQUE_ENABLE_ADDR,
                    0,
                )
            except Exception:
                pass

        try:
            port.closePort()
        except Exception:
            pass

        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass

        try:
            server.close()
        except Exception:
            pass

        print("Done.")


if __name__ == "__main__":
    main()
