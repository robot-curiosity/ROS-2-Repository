# Imports:
# JSON:
import json

# Queue:
import queue

# Threading:
import threading

# Time:
import time

# CV2:
import cv2

# Requests:
import requests

# ROS:
import rclpy

# > Node:
from rclpy.node import Node

# Standard:
from std_msgs.msg import String

# Ultralytics:
from ultralytics import YOLO

# Target logic (ROS-free, unit tested):
from trash_vision.target_logic import build_target, get_direction


# Detector:
class TrashDetector(Node):
    # Initialization:
    def __init__(self) -> None:
        # Initialization:
        super().__init__("trash_detector")

        # Parameters:
        self.declare_parameters(
            # Namespace:
            namespace="",

            # Parameters:
            parameters=[
                # Model:
                (
                    "model_path",
                    "/home/christopherg/best_ncnn_model",
                ),

                # Camera:
                (
                    "camera_source",
                    "tcp://127.0.0.1:8888",
                ),

                # Confidence:
                ("confidence", 0.40),

                # Deadband:
                ("deadband", 0.12),

                # Mapping:
                (
                    "mapping_upload_url",
                    "http://192.168.0.102:5000/frame",
                ),

                # Mapping:
                ("mapping_enabled", True),

                # Interval:
                ("mapping_interval_seconds", 2.0),

                # Quality:
                ("mapping_jpeg_quality", 80),
            ],
        )

        # Variables (Assignment):
        # Path:
        model_path: str = str(
            self.get_parameter("model_path").value
        )

        # Camera:
        camera_source: str = str(
            self.get_parameter("camera_source").value
        )

        # Confidence:
        self.confidence: float = float(
            self.get_parameter("confidence").value
        )

        # Deadband:
        self.deadband: float = float(
            self.get_parameter("deadband").value
        )

        # Mapping:
        self.mapping_upload_url: str = str(
            self.get_parameter("mapping_upload_url").value
        )

        # Mapping:
        self.mapping_enabled: bool = bool(
            self.get_parameter("mapping_enabled").value
        )

        # Interval:
        self.mapping_interval_seconds: float = float(
            self.get_parameter(
                "mapping_interval_seconds"
            ).value
        )

        # Quality:
        self.mapping_jpeg_quality: int = int(
            self.get_parameter(
                "mapping_jpeg_quality"
            ).value
        )

        # Publisher:
        self.target_publisher = self.create_publisher(
            # Message:
            String,

            # URL:
            "/vision/target",

            # Frequency:
            10,
        )

        # 
        self.direction_publisher = self.create_publisher(
            # Message:
            String,

            # URL:
            "/vision/direction",

            # Frequency:
            10,
        )

        # Logging:
        self.get_logger().info(
            f"Loading YOLO model: {model_path}"
        )

        # Variables (Assignment):
        # Model:
        self.model = YOLO(model_path)

        # Logger:
        self.get_logger().info(
            f"Opening camera source: {camera_source}"
        )

        # Camera:
        self.camera = cv2.VideoCapture(camera_source)

        # Validation:
        if not self.camera.isOpened():
            raise RuntimeError(
                f"Could not open camera source: {camera_source}"
            )

        # Variables (Assignment):
        # Frame:
        self.frame_number = 0

        # Count:
        self.fps_frame_count = 0

        # Time:
        self.fps_start_time = time.monotonic()

        # Time:
        self.last_mapping_upload_time = 0.0

        # Queue:
        self.mapping_queue: queue.Queue = queue.Queue(maxsize=1)

        # Event:
        self.mapping_stop_event = threading.Event()

        # Thread:
        self.mapping_thread = threading.Thread(
            # Target:
            target=self.mapping_upload_worker,

            # Name:
            name="lingbot-frame-uploader",

            # Daemon:
            daemon=True,
        )

        # Thread:
        self.mapping_thread.start()

        # Timer:
        self.timer = self.create_timer(
            # Frequency:
            0.001,

            # Function:
            self.process_frame,
        )

        # Logging:
        self.get_logger().info(
            "Trash detector node started"
        )

        # Mapping:
        if self.mapping_enabled:
            # Logging:
            self.get_logger().info(
                "LingBot frame uploading enabled: "
                f"{self.mapping_upload_url}"
            )
        else:
            # Logging:
            self.get_logger().info(
                "LingBot frame uploading disabled"
            )

    # Methods:
    def process_frame(self) -> None:
        # Variables (Assignment):
        # Success, Frame:
        success, frame = self.camera.read()

        # Validation:
        if not success:
            # Logger:
            self.get_logger().warning(
                # Message:
                "Camera frame unavailable",

                # Throttle:
                throttle_duration_sec=2.0,
            )

            # Logic:
            return

        # Variables (Assignment):
        # Frame:
        self.frame_number += 1

        # Count:
        self.fps_frame_count += 1

        # Queue:
        self.queue_mapping_frame(frame)

        # Height, Width:
        height, width = frame.shape[:2]

        # Results:
        results = self.model.predict(
            # Source:
            source=frame,

            # Size:
            imgsz=640,

            # Confidence:
            conf=self.confidence,

            # Verbose:
            verbose=False,
        )

        # Boxes:
        boxes = results[0].boxes

        # Validation:
        if boxes is None or len(boxes) == 0:
            # Publish:
            self.publish_no_target()

            # Report:
            self.report_processing_rate()

            # Logic:
            return

        # Variables (Assignment):
        # Confidences:
        confidences = boxes.conf.cpu().numpy()

        # Index:
        best_index = int(confidences.argmax())

        # Coordinates:
        x1, y1, x2, y2 = (
            boxes.xyxy[best_index]
            .cpu()
            .numpy()
            .tolist()
        )

        # Confidence:
        confidence: float = float(
            confidences[best_index]
        )

        # Direction and target message:
        target = build_target(
            bbox=(x1, y1, x2, y2),
            image_size=(width, height),
            confidence=confidence,
            frame_number=self.frame_number,
            deadband=self.deadband,
        )

        # Direction:
        direction = target["direction"]

        # Target:
        self.publish_target(target, direction)

        # Rate:
        self.report_processing_rate(
            # Direction:
            direction=direction,

            # Confidence:
            confidence=confidence,
        )

    def get_direction(self, horizontal_error: float) -> str:
        # Direction:
        return get_direction(horizontal_error, self.deadband)

    def publish_target(self, target: dict, direction: str) -> None:
        # Variables (Assignment):
        # Message:
        target_message = String()

        # Data:
        target_message.data = json.dumps(target)

        # Publish:
        self.target_publisher.publish(target_message)

        # Message:
        direction_message = String()

        # > Data:
        direction_message.data = direction

        # Publish:
        self.direction_publisher.publish(direction_message)

    def publish_no_target(self) -> None:
        # Variables (Assignment);
        # Message:
        target_message = String()

        # > Data:
        target_message.data = json.dumps(
            {
                # Detected:
                "detected": False,

                # Frame:
                "frame_number": self.frame_number,
            }
        )

        # Publish:
        self.target_publisher.publish(target_message)

        # Message:
        direction_message = String()

        # > Data:
        direction_message.data = "NO_TARGET"

        # Publish:
        self.direction_publisher.publish(direction_message)

    def report_processing_rate(
        self,

        # Direction:
        direction: str | None = None,

        # Confidence:
        confidence: float | None = None,
    ) -> None:
        # Validation:
        if self.fps_frame_count < 30:
            return

        # Variables (Assignment):
        # Time:
        current_time = time.monotonic()

        # Elapsed:
        elapsed_time = (
            current_time - self.fps_start_time
        )

        # Validation:
        if elapsed_time <= 0:
            return

        # Rate:
        processing_rate = (
            self.fps_frame_count / elapsed_time
        )

        # Message:
        message = (
            f"processing_rate={processing_rate:.1f} FPS"
        )

        # Direction:
        if direction is not None:
            # Message:
            message += f" target={direction}"

        # Confidence:
        if confidence is not None:
            # Message:
            message += (
                f" confidence={confidence:.2f}"
            )

        # Logging:
        self.get_logger().info(message)

        # Count:
        self.fps_frame_count = 0

        # Time:
        self.fps_start_time = current_time

    def queue_mapping_frame(self, frame) -> None:
        # Validation:
        if not self.mapping_enabled:
            return

        # Variables (Assignment):
        # Time:
        current_time = time.monotonic()

        # Elapsed:
        elapsed_time = (
            current_time
            - self.last_mapping_upload_time
        )

        # Validation:
        if (
            elapsed_time
            < self.mapping_interval_seconds
        ):
            return

        # Upload:
        self.last_mapping_upload_time = current_time

        # Frame:
        frame_copy = frame.copy()

        # Item:
        queued_item = (self.frame_number, time.time(), frame_copy)

        # Logic:
        try:
            # Queue:
            self.mapping_queue.put_nowait(
                queued_item
            )

            # Logic:
            return
        except queue.Full:
            pass

        # Logic:
        try:
            # Queue:
            self.mapping_queue.get_nowait()

            # Task:
            self.mapping_queue.task_done()
        except queue.Empty:
            pass

        # Logic:
        try:
            # Queue:
            self.mapping_queue.put_nowait(
                queued_item
            )
        except queue.Full:
            # Logging:
            self.get_logger().warning(
                # Mapping Queue:
                "Mapping queue remained full; "

                # Frame:
                "dropping frame",

                # Throttle:
                throttle_duration_sec=5.0,
            )

    def mapping_upload_worker(self) -> None:
        # Variables (Assignment):
        # Session:
        session = requests.Session()

        # Logic:
        while not self.mapping_stop_event.is_set():
            try:
                # Variables (Assignment):
                # Unpacking:
                (
                    frame_number,
                    timestamp,
                    frame,
                ) = self.mapping_queue.get(
                    timeout=0.5
                )
            except queue.Empty:
                continue

            # Logic:
            try:
                # Variables (Assignment):
                # Success, Frame:
                encode_success, encoded_frame = (
                    cv2.imencode(
                        # Format:
                        ".jpg",

                        # Frame:
                        frame,
                        [
                            # JPG:
                            cv2.IMWRITE_JPEG_QUALITY,

                            # Quality:
                            self.mapping_jpeg_quality,
                        ],
                    )
                )

                # Validation:
                if not encode_success:
                    # Logging:
                    self.get_logger().warning(
                        "Could not encode mapping frame"
                    )

                    # Logic:
                    continue

                # Timestamp:
                timestamp_ms: int = int(
                    timestamp * 1000
                )

                # Filename:
                filename: str = (
                    f"frame_{timestamp_ms}_"
                    f"{frame_number:08d}.jpg"
                )

                # Response:
                response = session.post(
                    # URL:
                    self.mapping_upload_url,

                    # Files:
                    files={
                        # Frame:
                        "frame": (
                            # File:
                            filename,

                            # Bytes:
                            encoded_frame.tobytes(),

                            # Format:
                            "image/jpeg",
                        )
                    },

                    # Data:
                    data={
                        # Filename:
                        "filename": filename,

                        # Number:
                        "frame_number": str(
                            frame_number
                        ),

                        # Timestamp:
                        "timestamp": str(timestamp),
                    },

                    # Timeout:
                    timeout=(2.0, 10.0),
                )

                # Status:
                response.raise_for_status()

                # Logger:
                self.get_logger().info(
                    f"Uploaded mapping frame: "
                    f"{filename}"
                )

            except requests.RequestException as error:
                # LOgging:
                self.get_logger().warning(
                    "Mapping upload failed: "
                    f"{error}",
                    throttle_duration_sec=5.0,
                )

            except Exception as error:
                # Logging:
                self.get_logger().error(
                    "Unexpected mapping uploader "
                    f"error: {error}"
                )

            finally:
                # Task:
                self.mapping_queue.task_done()

        # Session:
        session.close()

    def destroy_node(self) -> None:
        # Logger:
        self.get_logger().info(
            "Shutting down trash detector"
        )

        # Mapping:
        self.mapping_stop_event.set()

        # Threading:
        if self.mapping_thread.is_alive():
            self.mapping_thread.join(
                timeout=2.0
            )

        # Camera:
        if self.camera.isOpened():
            self.camera.release()

        # Node:
        super().destroy_node()


def main(arguments=None) -> None:
    # Initialization:
    rclpy.init(args=arguments)

    # Variables (Assignment):
    # Node:
    node = None


    # Logic:
    try:
        # Variables (Assignment):
        # Node:
        node = TrashDetector()

        # Spin:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        # Validation:
        if node is not None:
            node.get_logger().fatal(
                f"Fatal node error: {error}"
            )
        else:
            print(
                f"Failed to initialize node: {error}"
            )

        raise
    finally:
        # Destruction:
        if node is not None:
            node.destroy_node()

        # Shutdown:
        if rclpy.ok():
            rclpy.shutdown()


# Node:
if __name__ == "__main__":
    main()
