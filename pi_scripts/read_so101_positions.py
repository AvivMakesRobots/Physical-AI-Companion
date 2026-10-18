import time
import scservo_sdk

SERVO_PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000

ADDR_PRESENT_POSITION = 56


def swap_bytes(value):
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


port_handler = scservo_sdk.PortHandler(SERVO_PORT)
packet_handler = scservo_sdk.PacketHandler(1.0)

if not port_handler.openPort():
    raise RuntimeError("Could not open servo port")

if not port_handler.setBaudRate(BAUDRATE):
    raise RuntimeError("Could not set baud rate")

print()
print("SO-101 servo positions")
print("Press Ctrl+C to stop.")
print()

try:

    while True:

        positions = []

        for servo_id in range(1, 7):

            raw, result, error = packet_handler.read2ByteTxRx(
                port_handler,
                servo_id,
                ADDR_PRESENT_POSITION
            )

            if result != scservo_sdk.COMM_SUCCESS:
                positions.append(
                    f"J{servo_id}=ERROR"
                )
                continue

            actual = swap_bytes(raw)

            positions.append(
                f"J{servo_id}={actual:4d}"
            )

        print("  ".join(positions))

        time.sleep(0.5)

except KeyboardInterrupt:

    print("\nStopped.")

finally:

    port_handler.closePort()
