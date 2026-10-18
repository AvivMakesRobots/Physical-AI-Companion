from scservo_sdk import *
import time

# ============================================================
# SETTINGS
# ============================================================

PORT = "/dev/ttyACM0"
BAUDRATE = 1000000

J2_ID = 2

# Registers
TORQUE_ENABLE_ADDR = 40
PRESENT_POSITION_ADDR = 56


# ============================================================
# CONNECT
# ============================================================

port = PortHandler(PORT)
packet = PacketHandler(0)

if not port.openPort():
    print("ERROR: Could not open port")
    quit()

if not port.setBaudRate(BAUDRATE):
    print("ERROR: Could not set baud rate")
    port.closePort()
    quit()

print("Connected.")


# ============================================================
# DISABLE J2 TORQUE
# ============================================================

result, error = packet.write1ByteTxRx(
    port,
    J2_ID,
    TORQUE_ENABLE_ADDR,
    0
)

if result != COMM_SUCCESS:
    print(
        "Communication error disabling torque:",
        packet.getTxRxResult(result)
    )

elif error != 0:
    print(
        "Servo error disabling torque:",
        packet.getRxPacketError(error)
    )

else:
    print("J2 torque disabled.")


print()
print("Move J2 slowly by hand.")
print("Put the shoulder at the HIGHEST/BACKMOST position")
print("you would actually want gesture control to reach.")
print()
print("Press Ctrl+C when finished.")
print()


# ============================================================
# READ POSITION
# ============================================================

try:

    while True:

        position, result, error = packet.read2ByteTxRx(
            port,
            J2_ID,
            PRESENT_POSITION_ADDR
        )

        if result == COMM_SUCCESS and error == 0:

            print(
                f"\rJ2 position: {position:4d} ticks",
                end="",
                flush=True
            )

        else:

            print(
                "\rREAD ERROR                    ",
                end="",
                flush=True
            )

        time.sleep(0.10)


except KeyboardInterrupt:

    print()
    print("Stopped.")


finally:

    port.closePort()