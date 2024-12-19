import time
import keyboard
from lerobot.common.robot_devices.motors.hiwonder import HiWonderMotorsBus

# Define servo position limits
MIN_POS = 0
MAX_POS = 1000
STEP = 10  # Position increment step size

# Initialize motor positions (starting positions)
motor_positions = {
    "base_rotation": 590,  # Default "straight" position
    "shoulder": 590,
    "elbow": 590,
    "wrist_pitch": 590,
    "wrist_roll": 590,
    "gripper": 590,
}

# Motor mapping and USB connection setup
motors_bus = HiWonderMotorsBus(
    port="/dev/ttyUSB0",
    motors={
        "base_rotation": (1, "hiwonder"),
        "shoulder": (2, "hiwonder"),
        "elbow": (3, "hiwonder"),
        "wrist_pitch": (4, "hiwonder"),
        "wrist_roll": (5, "hiwonder"),
        "gripper": (6, "hiwonder"),
    }
)

# Functions to safely increment/decrement positions
def move_motor(motor_name, step):
    motor_positions[motor_name] = max(MIN_POS, min(MAX_POS, motor_positions[motor_name] + step))
    #print(f"{motor_name} moved to {motor_positions[motor_name]}")

def send_positions():
    motors_bus.write("Goal_Position", list(motor_positions.values()), motor_names=list(motor_positions.keys()))

# Key control mappings
def control_servos():
    print("Use keys to control the robot arm:\n"
          "W/S: Shoulder Up/Down | A/D: Base Rotate Left/Right\n"
          "I/K: Elbow Up/Down   | J/L: Wrist Pitch Up/Down\n"
          "U/O: Wrist Roll Left/Right | T/Y: Gripper Open/Close\n"
          "ESC to exit.")

    motors_bus.connect()
    motors_bus.write("Torque_Enable", [1] * len(motor_positions), motor_names=list(motor_positions.keys()))
    try:
        prev_positions = motor_positions.copy()  # Store previous positions

        while True:
            key_pressed = False  # Flag to track key presses

            if keyboard.is_pressed("w"):
                move_motor("shoulder", STEP)
                key_pressed = True
            elif keyboard.is_pressed("s"):
                move_motor("shoulder", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("a"):
                move_motor("base_rotation", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("d"):
                move_motor("base_rotation", STEP)
                key_pressed = True
            elif keyboard.is_pressed("i"):
                move_motor("elbow", STEP)
                key_pressed = True
            elif keyboard.is_pressed("k"):
                move_motor("elbow", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("j"):
                move_motor("wrist_pitch", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("l"):
                move_motor("wrist_pitch", STEP)
                key_pressed = True
            elif keyboard.is_pressed("u"):
                move_motor("wrist_roll", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("o"):
                move_motor("wrist_roll", STEP)
                key_pressed = True
            elif keyboard.is_pressed("t"):
                move_motor("gripper", STEP)
                key_pressed = True
            elif keyboard.is_pressed("y"):
                move_motor("gripper", -STEP)
                key_pressed = True
            elif keyboard.is_pressed("esc"):
                print("Exiting control...")
                break

            # Send updated positions **only if a key was pressed or positions changed**
            if key_pressed or motor_positions != prev_positions:
                send_positions()
                prev_positions = motor_positions.copy()

            time.sleep(0.05)  # Small delay to avoid overloading the bus

    finally:
        #motors_bus.write("Torque_Enable", [0] * len(motor_positions), motor_names=list(motor_positions.keys()))
        motors_bus.disconnect()
        print("Robot arm disconnected. Goodbye!")

if __name__ == "__main__":
    control_servos()

