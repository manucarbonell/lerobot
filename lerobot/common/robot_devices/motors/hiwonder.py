import enum
import logging
import math
import time
from copy import deepcopy

import numpy as np

from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.utils.utils import capture_timestamp_utc

# Control table structure for HiWonder motors (based on protocol)
HIWONDER_CONTROL_TABLE = {
    "ID": (1, 1),
    "Baud_Rate": (2, 1),
    "Goal_Position": (3, 2),
    "Present_Position": (5, 2),
    "Torque_Enable": (40, 1),
    "LED": (41, 1),
    "Moving_Speed": (42, 2),
    "Present_Speed": (46, 2),
    "Present_Temperature": (50, 1),
}

# Conversion constants
MODEL_RESOLUTION = {
    "hiwonder": 1024,  # Assuming 10-bit resolution for HiWonder servos
}

class HiWonderMotorsBus:
    """
    The HiWonderMotorsBus class allows for efficient reading and writing to the HiWonder motors.

    Example usage:

    ```python
    motors_bus = HiWonderMotorsBus(
        port="/dev/ttyUSB0",
        motors={"base": (1, "hiwonder"), "elbow": (2, "hiwonder")}
    )
    motors_bus.connect()

    # Move the base motor to position 512
    motors_bus.write("Goal_Position", [512], motor_names=["base"])

    # Read the current position of the elbow motor
    position = motors_bus.read("Present_Position", motor_names=["elbow"])

    motors_bus.disconnect()
    ```
    """

    def __init__(self, port, motors, mock=False):
        self.port = port
        self.motors = motors
        self.mock = mock

        self.model_ctrl_table = deepcopy(HIWONDER_CONTROL_TABLE)
        self.model_resolution = deepcopy(MODEL_RESOLUTION)

        self.device = None
        self.is_connected = False

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("HiWonderMotorsBus is already connected.")

        if self.mock:
            logging.info("Mock mode enabled. Skipping hardware connection.")
            self.is_connected = True
            return

        try:
            import easyhid
            en = easyhid.Enumeration()
            devices = en.find()
            for device in devices:
                if "hiwonder" in device.product_string.lower():
                    self.device = device
                    break

            if not self.device:
                raise RuntimeError("HiWonder device not found.")

            self.device.open()
            self.is_connected = True
            logging.info(f"Connected to HiWonder device: {self.device.product_string}")

        except Exception as e:
            raise ConnectionError(f"Failed to connect to HiWonder device: {e}")

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("HiWonderMotorsBus is not connected.")

        if self.device:
            self.device.close()

        self.is_connected = False
        logging.info("Disconnected from HiWonder robot.")

    def write(self, data_name, values, motor_names=None, duration=600):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("HiWonderMotorsBus is not connected.")

        if motor_names is None:
            motor_names = list(self.motors.keys())

        for motor_name, value in zip(motor_names, values):
            motor_id, model = self.motors[motor_name]
            self._send_command(motor_id, data_name, value, duration)



    def read(self, data_name, motor_names=None):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("HiWonderMotorsBus is not connected.")

        if motor_names is None:
            motor_names = list(self.motors.keys())

        results = []
        for motor_name in motor_names:
            motor_id, model = self.motors[motor_name]
            result = self._receive_command(motor_id, data_name)
            results.append(result)

        return np.array(results)

    def _send_command(self, motor_id, data_name, value, duration=600):
        """Send a write command to the motor with time duration for smooth movement."""
        address, size = self.model_ctrl_table[data_name]

        # Split time and position into LSB and MSB
        t_lsb, t_msb = duration & 0xFF, (duration >> 8) & 0xFF
        v_lsb, v_msb = value & 0xFF, (value >> 8) & 0xFF

        # Construct the movement command
        command = [0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, motor_id, v_lsb, v_msb]
        self.device.write(command)

    def _receive_command(self, motor_id, data_name):
        """Send a read command and parse the response."""
        address, size = self.model_ctrl_table[data_name]
        read_command = [0x55, 0x55, motor_id, 0x02, address, size]
        self.device.write(read_command)

        response = self.device.read()
        if len(response) < size:
            raise RuntimeError("Invalid response size.")

        value = int.from_bytes(response[5:5 + size], byteorder="little")
        return value

    def __del__(self):
        if self.is_connected:
            self.disconnect()

