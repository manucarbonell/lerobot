import time
import cv2
import numpy as np
from dataclasses import dataclass, replace
import platform
from pathlib import Path
import math
import threading
from threading import Thread

from lerobot.common.robot_devices.utils import (
    RobotDeviceAlreadyConnectedError,
    RobotDeviceNotConnectedError,
    busy_wait,
)
from lerobot.common.utils.utils import capture_timestamp_utc


@dataclass
class OpenCVCameraConfig:
    fps: int | None = None
    width: int | None = None
    height: int | None = None
    color_mode: str = "rgb"
    channels: int | None = None
    rotation: int | None = None
    mock: bool = False

    def __post_init__(self):
        if self.color_mode not in ["rgb", "bgr"]:
            raise ValueError(
                f"`color_mode` is expected to be 'rgb' or 'bgr', but {self.color_mode} is provided."
            )
        self.channels = 3
        if self.rotation not in [-90, None, 90, 180]:
            raise ValueError(f"`rotation` must be in [-90, None, 90, 180] (got {self.rotation})")


def is_valid_unix_path(path: str) -> bool:
    p = Path(path)
    return p.is_absolute() and p.exists()


def get_camera_index_from_unix_port(port: Path) -> int:
    return int(str(port.resolve()).removeprefix("/dev/video"))


class OpenCVCamera:
    def __init__(self, camera_index: int | str, config: OpenCVCameraConfig | None = None, **kwargs):
        if config is None:
            config = OpenCVCameraConfig()

        config = replace(config, **kwargs)

        self.camera_index = camera_index
        self.port = None

        if platform.system() == "Linux":
            if isinstance(self.camera_index, int):
                self.port = Path(f"/dev/video{self.camera_index}")
            elif isinstance(self.camera_index, str) and is_valid_unix_path(self.camera_index):
                self.port = Path(self.camera_index)
                self.camera_index = get_camera_index_from_unix_port(self.port)
            else:
                raise ValueError(f"Please check the provided camera_index: {camera_index}")

        self.fps = config.fps
        self.width = config.width
        self.height = config.height
        self.channels = config.channels
        self.color_mode = config.color_mode
        self.mock = config.mock

        self.camera = None
        self.is_connected = False
        self.thread = None
        self.stop_event = None
        self.color_image = None
        self.logs = {}

        if self.mock:
            import tests.mock_cv2 as cv2
        else:
            import cv2

        self.rotation = None
        if config.rotation == -90:
            self.rotation = cv2.ROTATE_90_COUNTERCLOCKWISE
        elif config.rotation == 90:
            self.rotation = cv2.ROTATE_90_CLOCKWISE
        elif config.rotation == 180:
            self.rotation = cv2.ROTATE_180

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(f"OpenCVCamera({self.camera_index}) is already connected.")

        if self.mock:
            import tests.mock_cv2 as cv2
        else:
            import cv2
            #cv2.setNumThreads(1)

        camera_idx = f"/dev/video{self.camera_index}" if platform.system() == "Linux" else self.camera_index
        tmp_camera = cv2.VideoCapture(camera_idx)
        is_camera_open = tmp_camera.isOpened()
        tmp_camera.release()
        del tmp_camera

        if not is_camera_open:
            raise OSError(f"Can't access OpenCVCamera({camera_idx}).")

        self.camera = cv2.VideoCapture(camera_idx)

        if self.fps is not None:
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        if self.width is not None:
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height is not None:
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

        if self.fps is not None and not math.isclose(self.fps, actual_fps, rel_tol=1e-3):
            raise OSError(
                f"Can't set {self.fps=} for OpenCVCamera({self.camera_index}). Actual value is {actual_fps}."
            )
        if self.width is not None and not math.isclose(self.width, actual_width, rel_tol=1e-3):
            raise OSError(
                f"Can't set {self.width=} for OpenCVCamera({self.camera_index}). Actual value is {actual_width}."
            )
        if self.height is not None and not math.isclose(self.height, actual_height, rel_tol=1e-3):
            raise OSError(
                f"Can't set {self.height=} for OpenCVCamera({self.camera_index}). Actual value is {actual_height}."
            )

        self.fps = round(actual_fps)
        self.width = round(actual_width)
        self.height = round(actual_height)
        self.is_connected = True

    def read(self, temporary_color_mode: str | None = None) -> np.ndarray:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"OpenCVCamera({self.camera_index}) is not connected. Try running `camera.connect()` first."
            )

        start_time = time.perf_counter()

        ret, color_image = self.camera.read()
        if not ret:
            raise OSError(f"Can't capture color image from camera {self.camera_index}.")

        requested_color_mode = self.color_mode if temporary_color_mode is None else temporary_color_mode

        if requested_color_mode not in ["rgb", "bgr"]:
            raise ValueError(
                f"Expected color values are 'rgb' or 'bgr', but {requested_color_mode} is provided."
            )

        if self.mock:
            import tests.mock_cv2 as cv2
        else:
            import cv2

        if requested_color_mode == "rgb":
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        h, w, _ = color_image.shape
        if h != self.height or w != self.width:
            raise OSError(
                f"Expected {self.height}x{self.width}, got {h}x{w}."
            )

        if self.rotation is not None:
            color_image = cv2.rotate(color_image, self.rotation)

        self.logs["delta_timestamp_s"] = time.perf_counter() - start_time
        self.logs["timestamp_utc"] = capture_timestamp_utc()

        self.color_image = color_image
        return color_image

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"OpenCVCamera({self.camera_index}) is not connected. Try running `camera.connect()` first."
            )

        if self.thread is not None:
            self.stop_event.set()
            self.thread.join()
            self.thread = None
            self.stop_event = None

        self.camera.release()
        self.camera = None
        self.is_connected = False


def detect_red_cube(frame_rgb):
    # frame_rgb is in RGB format as per camera class
    # Convert to HSV for better color segmentation
    frame_hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

    # Define red color range (this may need adjustment depending on lighting and cube color)
    # Red can be tricky since it wraps the hue range in HSV. Often split into two ranges.
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(frame_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(frame_hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours of red areas
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    # Find largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < 100:  # Arbitrary threshold to filter noise
        return None

    x, y, w, h = cv2.boundingRect(largest_contour)
    return (x, y, w, h)


if __name__ == "__main__":
    import cv2

    # Instantiate camera (change index if needed)
    camera = OpenCVCamera(camera_index=0, fps=30, width=640, height=480, color_mode="rgb")
    camera.connect()

    cv2.namedWindow("Red Cube Detection", cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = camera.read()  # frame in RGB
            bbox = detect_red_cube(frame)
            
            # Convert back to BGR for display in OpenCV window
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            if bbox is not None:
                x, y, w, h = bbox
                cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame_bgr, "Red Cube", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Red Cube Detection", frame_bgr)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # If you want exactly one update per second, uncomment the next line:
            # time.sleep(1)

    finally:
        camera.disconnect()
        cv2.destroyAllWindows()

