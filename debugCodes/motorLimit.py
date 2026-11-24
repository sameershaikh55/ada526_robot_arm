
#!/usr/bin/env python3

from dynamixel_sdk import (
    COMM_SUCCESS,
    PacketHandler,
    PortHandler,
)

# ============================================================
# CONFIGURATION
# ============================================================

PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

# Dynamixel Control Table Addresses (X-Series)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52

# ============================================================
# MOTOR CONFIGURATION
# ============================================================

MOTOR_IDS = [1, 2, 3, 4, 5]

MOTOR_NAMES = [
    "Joint 1",
    "Joint 2",
    "Joint 3",
    "Joint 4",
    "Joint 5"
]

# Gear ratios: motor angle / joint angle
GEAR_RATIO = [-2, 1, 1, 1, 1]

# Joint limits in degrees (from DHRobot)
MOTOR_LIMITS_DEG = [
    [-110, 110],
    [-90, 90],
    [-90, 90],
    [-100, 80],
    [-360, 360]
]

# Dynamixel position configuration
TICKS_PER_REVOLUTION = 4096
CENTER_TICK = 2048

# Operating modes
POSITION_CONTROL_MODE = 3
EXTENDED_POSITION_MODE = 4


# ============================================================
# CONVERSION
# ============================================================

def degrees_to_ticks(angle_deg, gear_ratio=1):

    motor_angle_deg = angle_deg * gear_ratio

    ticks = CENTER_TICK + (
        motor_angle_deg / 360.0
    ) * TICKS_PER_REVOLUTION

    return int(round(ticks))


def calculate_motor_limits():

    motor_limits = {}

    print("\n========== MOTOR LIMITS ==========\n")

    for motor_id, name, limits, ratio in zip(
        MOTOR_IDS,
        MOTOR_NAMES,
        MOTOR_LIMITS_DEG,
        GEAR_RATIO
    ):

        min_deg, max_deg = limits

        tick1 = degrees_to_ticks(min_deg, ratio)
        tick2 = degrees_to_ticks(max_deg, ratio)

        min_ticks, max_ticks = sorted([tick1, tick2])

        motor_limits[motor_id] = (min_ticks, max_ticks)

        print(f"{name} (ID {motor_id})")
        print(f"  Joint limits: {min_deg}° to {max_deg}°")
        print(f"  Gear ratio:   {ratio}")
        print(f"  Motor ticks:  {min_ticks} to {max_ticks}")
        print()

    return motor_limits


# ============================================================
# DYNAMIXEL COMMUNICATION
# ============================================================

packetHandler = PacketHandler(PROTOCOL_VERSION)
portHandler = PortHandler(PORT_NAME)


def check_result(result, error, motor_id, operation):

    if result != COMM_SUCCESS:

        print(
            f"Motor {motor_id}: {operation} failed: "
            f"{packetHandler.getTxRxResult(result)}"
        )

        return False

    if error != 0:

        print(
            f"Motor {motor_id}: {operation} error: "
            f"{packetHandler.getRxPacketError(error)}"
        )

        return False

    return True


# ============================================================
# CONFIGURE MOTORS
# ============================================================

def configure_motors():

    motor_limits = calculate_motor_limits()

    print("Connecting to Dynamixel...")

    if not portHandler.openPort():
        print("ERROR: Could not open port!")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print("ERROR: Could not set baudrate!")
        portHandler.closePort()
        return

    print("Connected successfully!\n")

    try:

        for motor_id, name in zip(MOTOR_IDS, MOTOR_NAMES):

            min_limit, max_limit = motor_limits[motor_id]

            print(f"Configuring {name} (ID {motor_id})")

            # ------------------------------------------------
            # Check operating mode
            # ------------------------------------------------

            mode, result, error = packetHandler.read1ByteTxRx(
                portHandler,
                motor_id,
                ADDR_OPERATING_MODE
            )

            if not check_result(
                result, error, motor_id, "Read operating mode"
            ):
                continue

            if mode not in [
                POSITION_CONTROL_MODE,
                EXTENDED_POSITION_MODE
            ]:

                print(
                    f"Unsupported operating mode: {mode}. "
                    "Skipping motor."
                )
                continue

            # ------------------------------------------------
            # Check whether limits fit the selected mode
            # ------------------------------------------------

            if mode == POSITION_CONTROL_MODE:

                if min_limit < 0 or max_limit > 4095:
                    print(
                        "WARNING: Limits exceed single-turn range. "
                        "Skipping motor."
                    )
                    continue

            # ------------------------------------------------
            # Disable torque
            # ------------------------------------------------

            result, error = packetHandler.write1ByteTxRx(
                portHandler,
                motor_id,
                ADDR_TORQUE_ENABLE,
                0
            )

            if not check_result(
                result, error, motor_id, "Disable torque"
            ):
                continue

            # ------------------------------------------------
            # Write minimum position limit
            # ------------------------------------------------

            result, error = packetHandler.write4ByteTxRx(
                portHandler,
                motor_id,
                ADDR_MIN_POSITION_LIMIT,
                min_limit
            )

            if not check_result(
                result, error, motor_id, "Write minimum limit"
            ):
                continue

            # ------------------------------------------------
            # Write maximum position limit
            # ------------------------------------------------

            result, error = packetHandler.write4ByteTxRx(
                portHandler,
                motor_id,
                ADDR_MAX_POSITION_LIMIT,
                max_limit
            )

            if not check_result(
                result, error, motor_id, "Write maximum limit"
            ):
                continue

            # ------------------------------------------------
            # Read back limits
            # ------------------------------------------------

            actual_min, result, error = packetHandler.read4ByteTxRx(
                portHandler,
                motor_id,
                ADDR_MIN_POSITION_LIMIT
            )

            if not check_result(
                result, error, motor_id, "Read minimum limit"
            ):
                continue

            actual_max, result, error = packetHandler.read4ByteTxRx(
                portHandler,
                motor_id,
                ADDR_MAX_POSITION_LIMIT
            )

            if not check_result(
                result, error, motor_id, "Read maximum limit"
            ):
                continue

            # ------------------------------------------------
            # Verify
            # ------------------------------------------------

            if actual_min == min_limit and actual_max == max_limit:

                print("SUCCESS!")
                print(f"  Min: {actual_min} ticks")
                print(f"  Max: {actual_max} ticks")

            else:

                print("WARNING: Position limits do not match!")

            print()

    finally:

        portHandler.closePort()

        print("Configuration completed.")
        print("Port closed.")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    configure_motors()
