#!/usr/bin/env python3
"""
Detect FRC robots by their colored bumpers (red/blue) and report 3D positions.

Pipeline stages:
1. Acquire aligned depth + color frames from RealSense
2. Apply depth filtering (spatial/temporal) to reduce noise
3. Segment by depth range (default 0.3-5.0m)
4. Segment by color (red AND blue bumpers simultaneously)
5. Combine masks and apply morphological cleanup
6. Find contours and validate bumper shapes
7. Extract centroid and 3D position for each robot
8. Publish arrays to NetworkTables (up to 6 robots)
9. Stream debug images via cscore (toggleable)
"""

import pyrealsense2 as rs
import ntcore
import cscore as cs
import numpy as np
import cv2
import argparse
import time
from dataclasses import dataclass, field


@dataclass
class RobotDetection:
    """Represents a detected robot bumper."""
    # 2D pixel coordinates of centroid
    pixel: tuple[int, int]
    # 3D coordinates in camera frame (meters)
    point_3d: tuple[float, float, float]
    # Bounding box for debug visualization (x, y, w, h)
    bbox: tuple[int, int, int, int]
    # Alliance color: "red" or "blue"
    alliance: str
    # Confidence score (0.0 - 1.0)
    confidence: float
    # Contour for debug visualization (may be None)
    contour: np.ndarray | None = None


class RobotDetector:
    """Detect FRC robots by their bumper colors and track their 3D positions."""

    # Maximum number of robots to track
    MAX_ROBOTS = 6

    def __init__(self, args):
        self.args = args

        # Detection parameters (can be tuned via NetworkTables)
        self.min_depth = args.min_depth
        self.max_depth = args.max_depth
        self.min_contour_area = args.min_area
        self.morph_kernel_size = 5

        # Red HSV parameters (red wraps around hue spectrum)
        self.red_hue_low1 = 0
        self.red_hue_high1 = 10
        self.red_hue_low2 = 170
        self.red_hue_high2 = 179
        self.red_sat_low = 100
        self.red_sat_high = 255
        self.red_val_low = 50
        self.red_val_high = 255

        # Blue HSV parameters
        self.blue_hue_low = 100
        self.blue_hue_high = 130
        self.blue_sat_low = 100
        self.blue_sat_high = 255
        self.blue_val_low = 50
        self.blue_val_high = 255

        # Shape validation parameters
        # Bumpers are wider than tall, but not as extreme as a bar
        self.min_aspect_ratio = 1.5
        self.max_aspect_ratio = 6.0
        # Solidity: contour area / convex hull area
        self.min_solidity = 0.6

        # Debug stream toggles
        self.stream_color = True
        self.stream_depth = True
        self.stream_color_mask = True
        self.stream_depth_mask = True
        self.stream_combined_mask = True
        self.stream_result = True

        # Temporal smoothing using exponential moving average (EMA)
        self.smoothing_alpha = args.smoothing
        # Dictionary to store smoothed positions by approximate location
        # Key: (grid_x, grid_y) based on pixel position, Value: RobotDetection
        self.smoothed_robots: dict[tuple[int, int], RobotDetection] = {}
        self.frames_since_detection = 0

        # Camera health tracking
        self.camera_connected = False
        self.last_frame_time = 0.0
        self.frame_timeout_sec = 2.0
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay_sec = 2.0
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10
        self.robots_detected = False

        # Initialize subsystems
        self._setup_networktables()
        self._setup_cscore()
        self._setup_realsense()

    def _setup_realsense(self) -> bool:
        """
        Initialize RealSense pipeline with aligned depth and color.
        Returns True if successful, False if camera not available.
        """
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()

            # Enable depth and color streams
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            print("Starting RealSense pipeline...")
            self.profile = self.pipeline.start(config)

            # Create align object to align depth to color frame
            self.align = rs.align(rs.stream.color)

            # Set up depth filtering for noise reduction
            self.spatial_filter = rs.spatial_filter()
            self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
            self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
            self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)

            self.temporal_filter = rs.temporal_filter()
            self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
            self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20)

            self.hole_filling = rs.hole_filling_filter()

            # Get intrinsics for 3D projection
            depth_stream = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()
            self.intrinsics = depth_stream.get_intrinsics()

            # Get depth scale
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()

            print(f"Depth scale: {self.depth_scale}")
            print(f"Resolution: {self.intrinsics.width}x{self.intrinsics.height}")

            # Update status
            self.camera_connected = True
            self.last_frame_time = time.time()
            self.consecutive_errors = 0
            self._update_camera_status("running", "")

            return True

        except Exception as e:
            error_msg = f"Failed to initialize camera: {e}"
            print(f"ERROR: {error_msg}")
            self.camera_connected = False
            self._update_camera_status("failed", error_msg)
            return False

    def _stop_realsense(self):
        """Stop the RealSense pipeline safely."""
        try:
            if hasattr(self, 'pipeline') and self.pipeline:
                self.pipeline.stop()
                print("RealSense pipeline stopped.")
        except Exception as e:
            print(f"Error stopping pipeline: {e}")
        self.camera_connected = False

    def _reconnect_camera(self) -> bool:
        """
        Attempt to reconnect to the camera after a failure.
        Returns True if reconnection successful.
        """
        self._update_camera_status("reconnecting", "Attempting to reconnect...")
        self.reconnect_attempts += 1
        self.reconnect_attempts_pub.set(self.reconnect_attempts)

        print(f"Reconnection attempt {self.reconnect_attempts}/{self.max_reconnect_attempts}...")

        # Stop existing pipeline if any
        self._stop_realsense()

        # Wait before attempting reconnect
        time.sleep(self.reconnect_delay_sec)

        # Try to reinitialize
        if self._setup_realsense():
            print("Camera reconnected successfully!")
            self.reconnect_attempts = 0
            return True

        if self.reconnect_attempts >= self.max_reconnect_attempts:
            error_msg = f"Max reconnection attempts ({self.max_reconnect_attempts}) reached"
            print(f"ERROR: {error_msg}")
            self._update_camera_status("failed", error_msg)

        return False

    def _update_camera_status(self, state: str, error_message: str):
        """Update camera status in NetworkTables."""
        self.camera_connected_pub.set(self.camera_connected)
        self.camera_state_pub.set(state)
        self.error_message_pub.set(error_message)
        time_since = time.time() - self.last_frame_time if self.last_frame_time > 0 else 0
        self.time_since_frame_pub.set(time_since)
        self.reconnect_attempts_pub.set(self.reconnect_attempts)

        # Healthy = connected + running + receiving frames + few errors
        is_healthy = (
            self.camera_connected and
            state == "running" and
            time_since < self.frame_timeout_sec and
            self.consecutive_errors < self.max_consecutive_errors
        )
        self.healthy_pub.set(is_healthy)

    def _check_camera_health(self) -> bool:
        """
        Check if camera is healthy based on frame timing.
        Returns True if healthy, False if timeout detected.
        """
        if not self.camera_connected:
            return False

        time_since_frame = time.time() - self.last_frame_time
        self.time_since_frame_pub.set(time_since_frame)

        if time_since_frame > self.frame_timeout_sec:
            print(f"WARNING: No frames for {time_since_frame:.1f}s (timeout: {self.frame_timeout_sec}s)")
            return False

        return True

    def _setup_networktables(self):
        """Initialize NetworkTables publishers and subscribers."""
        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_inst.startClient4("robot-detector")
        self.nt_inst.setServer(self.args.server, self.args.port)

        print(f"Connecting to NetworkTables at {self.args.server}:{self.args.port}...")
        while not self.nt_inst.isConnected():
            time.sleep(0.5)
        print("NetworkTables connected!")

        table = self.nt_inst.getTable("RobotDetector")

        # Robot detection arrays
        robots_table = table.getSubTable("robots")

        # Number of robots detected (0-6)
        self.count_pub = robots_table.getIntegerTopic("count").publish()

        # Arrays of robot data
        self.x_pub = robots_table.getDoubleArrayTopic("x_m").publish()
        self.y_pub = robots_table.getDoubleArrayTopic("y_m").publish()
        self.z_pub = robots_table.getDoubleArrayTopic("z_m").publish()
        self.pixel_x_pub = robots_table.getIntegerArrayTopic("pixel_x").publish()
        self.pixel_y_pub = robots_table.getIntegerArrayTopic("pixel_y").publish()
        self.alliance_pub = robots_table.getStringArrayTopic("alliance").publish()
        self.confidence_pub = robots_table.getDoubleArrayTopic("confidence").publish()

        # True if any robots detected
        self.valid_pub = robots_table.getBooleanTopic("valid").publish()

        # FPS counter
        self.fps_pub = table.getDoubleTopic("fps").publish()

        # Camera status for fault detection
        status_table = table.getSubTable("status")
        self.camera_connected_pub = status_table.getBooleanTopic("camera_connected").publish()
        self.camera_state_pub = status_table.getStringTopic("camera_state").publish()
        self.time_since_frame_pub = status_table.getDoubleTopic("time_since_frame_sec").publish()
        self.reconnect_attempts_pub = status_table.getIntegerTopic("reconnect_attempts").publish()
        self.error_message_pub = status_table.getStringTopic("error_message").publish()
        self.healthy_pub = status_table.getBooleanTopic("healthy").publish()

        # Latency/timing information
        timing_table = table.getSubTable("timing")
        self.capture_timestamp_pub = timing_table.getDoubleTopic("capture_timestamp_ms").publish()
        self.processing_time_pub = timing_table.getDoubleTopic("processing_time_ms").publish()
        self.total_latency_pub = timing_table.getDoubleTopic("total_latency_ms").publish()
        self.publish_timestamp_pub = timing_table.getDoubleTopic("publish_timestamp_ms").publish()

        # Debug stream toggles
        debug_table = table.getSubTable("debug")
        self.stream_color_sub = debug_table.getBooleanTopic("stream_color").subscribe(True)
        self.stream_depth_sub = debug_table.getBooleanTopic("stream_depth").subscribe(True)
        self.stream_color_mask_sub = debug_table.getBooleanTopic("stream_color_mask").subscribe(True)
        self.stream_depth_mask_sub = debug_table.getBooleanTopic("stream_depth_mask").subscribe(True)
        self.stream_combined_mask_sub = debug_table.getBooleanTopic("stream_combined_mask").subscribe(True)
        self.stream_result_sub = debug_table.getBooleanTopic("stream_result").subscribe(True)

        # Publish defaults so they appear in Glass
        debug_table.getBooleanTopic("stream_color").publish().set(True)
        debug_table.getBooleanTopic("stream_depth").publish().set(True)
        debug_table.getBooleanTopic("stream_color_mask").publish().set(True)
        debug_table.getBooleanTopic("stream_depth_mask").publish().set(True)
        debug_table.getBooleanTopic("stream_combined_mask").publish().set(True)
        debug_table.getBooleanTopic("stream_result").publish().set(True)

        # Tuning parameters
        tuning = table.getSubTable("tuning")

        # Depth range
        self.min_depth_sub = tuning.getDoubleTopic("min_depth_m").subscribe(self.min_depth)
        self.max_depth_sub = tuning.getDoubleTopic("max_depth_m").subscribe(self.max_depth)
        tuning.getDoubleTopic("min_depth_m").publish().set(self.min_depth)
        tuning.getDoubleTopic("max_depth_m").publish().set(self.max_depth)

        # Red HSV parameters
        self.red_hue_low1_sub = tuning.getIntegerTopic("red_hue_low1").subscribe(self.red_hue_low1)
        self.red_hue_high1_sub = tuning.getIntegerTopic("red_hue_high1").subscribe(self.red_hue_high1)
        self.red_hue_low2_sub = tuning.getIntegerTopic("red_hue_low2").subscribe(self.red_hue_low2)
        self.red_hue_high2_sub = tuning.getIntegerTopic("red_hue_high2").subscribe(self.red_hue_high2)
        self.red_sat_low_sub = tuning.getIntegerTopic("red_sat_low").subscribe(self.red_sat_low)
        self.red_sat_high_sub = tuning.getIntegerTopic("red_sat_high").subscribe(self.red_sat_high)
        self.red_val_low_sub = tuning.getIntegerTopic("red_val_low").subscribe(self.red_val_low)
        self.red_val_high_sub = tuning.getIntegerTopic("red_val_high").subscribe(self.red_val_high)

        tuning.getIntegerTopic("red_hue_low1").publish().set(self.red_hue_low1)
        tuning.getIntegerTopic("red_hue_high1").publish().set(self.red_hue_high1)
        tuning.getIntegerTopic("red_hue_low2").publish().set(self.red_hue_low2)
        tuning.getIntegerTopic("red_hue_high2").publish().set(self.red_hue_high2)
        tuning.getIntegerTopic("red_sat_low").publish().set(self.red_sat_low)
        tuning.getIntegerTopic("red_sat_high").publish().set(self.red_sat_high)
        tuning.getIntegerTopic("red_val_low").publish().set(self.red_val_low)
        tuning.getIntegerTopic("red_val_high").publish().set(self.red_val_high)

        # Blue HSV parameters
        self.blue_hue_low_sub = tuning.getIntegerTopic("blue_hue_low").subscribe(self.blue_hue_low)
        self.blue_hue_high_sub = tuning.getIntegerTopic("blue_hue_high").subscribe(self.blue_hue_high)
        self.blue_sat_low_sub = tuning.getIntegerTopic("blue_sat_low").subscribe(self.blue_sat_low)
        self.blue_sat_high_sub = tuning.getIntegerTopic("blue_sat_high").subscribe(self.blue_sat_high)
        self.blue_val_low_sub = tuning.getIntegerTopic("blue_val_low").subscribe(self.blue_val_low)
        self.blue_val_high_sub = tuning.getIntegerTopic("blue_val_high").subscribe(self.blue_val_high)

        tuning.getIntegerTopic("blue_hue_low").publish().set(self.blue_hue_low)
        tuning.getIntegerTopic("blue_hue_high").publish().set(self.blue_hue_high)
        tuning.getIntegerTopic("blue_sat_low").publish().set(self.blue_sat_low)
        tuning.getIntegerTopic("blue_sat_high").publish().set(self.blue_sat_high)
        tuning.getIntegerTopic("blue_val_low").publish().set(self.blue_val_low)
        tuning.getIntegerTopic("blue_val_high").publish().set(self.blue_val_high)

        # Shape validation parameters
        self.min_aspect_ratio_sub = tuning.getDoubleTopic("min_aspect_ratio").subscribe(self.min_aspect_ratio)
        self.max_aspect_ratio_sub = tuning.getDoubleTopic("max_aspect_ratio").subscribe(self.max_aspect_ratio)
        self.min_solidity_sub = tuning.getDoubleTopic("min_solidity").subscribe(self.min_solidity)
        self.min_area_sub = tuning.getIntegerTopic("min_area").subscribe(self.min_contour_area)

        tuning.getDoubleTopic("min_aspect_ratio").publish().set(self.min_aspect_ratio)
        tuning.getDoubleTopic("max_aspect_ratio").publish().set(self.max_aspect_ratio)
        tuning.getDoubleTopic("min_solidity").publish().set(self.min_solidity)
        tuning.getIntegerTopic("min_area").publish().set(self.min_contour_area)

        # Smoothing factor
        self.smoothing_sub = tuning.getDoubleTopic("smoothing").subscribe(self.smoothing_alpha)
        tuning.getDoubleTopic("smoothing").publish().set(self.smoothing_alpha)

    def _setup_cscore(self):
        """Initialize cscore camera server for debug streams."""
        # Create CvSource and MjpegServer manually on separate ports
        self.color_output = cs.CvSource("Color", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.color_server = cs.MjpegServer("ColorServer", 1181)
        self.color_server.setSource(self.color_output)

        self.depth_output = cs.CvSource("Depth", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.depth_server = cs.MjpegServer("DepthServer", 1182)
        self.depth_server.setSource(self.depth_output)

        self.color_mask_output = cs.CvSource("ColorMask", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.color_mask_server = cs.MjpegServer("ColorMaskServer", 1183)
        self.color_mask_server.setSource(self.color_mask_output)

        self.depth_mask_output = cs.CvSource("DepthMask", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.depth_mask_server = cs.MjpegServer("DepthMaskServer", 1184)
        self.depth_mask_server.setSource(self.depth_mask_output)

        self.combined_mask_output = cs.CvSource("CombinedMask", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.combined_mask_server = cs.MjpegServer("CombinedMaskServer", 1185)
        self.combined_mask_server.setSource(self.combined_mask_output)

        self.result_output = cs.CvSource("Result", cs.VideoMode.PixelFormat.kBGR, 640, 480, 30)
        self.result_server = cs.MjpegServer("ResultServer", 1186)
        self.result_server.setSource(self.result_output)

        # Pre-allocate frame buffers
        self.color_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.depth_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.color_mask_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.depth_mask_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.combined_mask_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.result_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        print("CameraServer streams available:")
        print("  Color:         http://localhost:1181")
        print("  Depth:         http://localhost:1182")
        print("  Color Mask:    http://localhost:1183")
        print("  Depth Mask:    http://localhost:1184")
        print("  Combined Mask: http://localhost:1185")
        print("  Result:        http://localhost:1186")

    def _update_parameters(self):
        """Update detection parameters from NetworkTables."""
        self.min_depth = self.min_depth_sub.get()
        self.max_depth = self.max_depth_sub.get()

        # Red HSV parameters
        self.red_hue_low1 = int(self.red_hue_low1_sub.get())
        self.red_hue_high1 = int(self.red_hue_high1_sub.get())
        self.red_hue_low2 = int(self.red_hue_low2_sub.get())
        self.red_hue_high2 = int(self.red_hue_high2_sub.get())
        self.red_sat_low = int(self.red_sat_low_sub.get())
        self.red_sat_high = int(self.red_sat_high_sub.get())
        self.red_val_low = int(self.red_val_low_sub.get())
        self.red_val_high = int(self.red_val_high_sub.get())

        # Blue HSV parameters
        self.blue_hue_low = int(self.blue_hue_low_sub.get())
        self.blue_hue_high = int(self.blue_hue_high_sub.get())
        self.blue_sat_low = int(self.blue_sat_low_sub.get())
        self.blue_sat_high = int(self.blue_sat_high_sub.get())
        self.blue_val_low = int(self.blue_val_low_sub.get())
        self.blue_val_high = int(self.blue_val_high_sub.get())

        # Shape validation parameters
        self.min_aspect_ratio = max(1.0, self.min_aspect_ratio_sub.get())
        self.max_aspect_ratio = max(self.min_aspect_ratio, self.max_aspect_ratio_sub.get())
        self.min_solidity = max(0.0, min(1.0, self.min_solidity_sub.get()))
        self.min_contour_area = max(100, int(self.min_area_sub.get()))

        # Update stream toggles
        self.stream_color = self.stream_color_sub.get()
        self.stream_depth = self.stream_depth_sub.get()
        self.stream_color_mask = self.stream_color_mask_sub.get()
        self.stream_depth_mask = self.stream_depth_mask_sub.get()
        self.stream_combined_mask = self.stream_combined_mask_sub.get()
        self.stream_result = self.stream_result_sub.get()

        # Update smoothing factor (clamp to valid range)
        self.smoothing_alpha = max(0.0, min(0.95, self.smoothing_sub.get()))

    def _filter_depth(self, depth_frame):
        """Apply spatial and temporal filtering to depth frame."""
        filtered = self.spatial_filter.process(depth_frame)
        filtered = self.temporal_filter.process(filtered)
        filtered = self.hole_filling.process(filtered)
        return filtered

    def _create_depth_mask(self, depth_image):
        """Create mask for pixels within depth range."""
        depth_meters = depth_image * self.depth_scale
        mask = (depth_meters > self.min_depth) & (depth_meters < self.max_depth)
        return mask.astype(np.uint8) * 255

    def _create_red_mask(self, hsv_image):
        """Create mask for red pixels (wraps around hue spectrum)."""
        # Range 1: hue 0-10 (red-orange side)
        low1 = np.array([self.red_hue_low1, self.red_sat_low, self.red_val_low])
        high1 = np.array([self.red_hue_high1, self.red_sat_high, self.red_val_high])
        mask1 = cv2.inRange(hsv_image, low1, high1)

        # Range 2: hue 170-179 (red-magenta side)
        low2 = np.array([self.red_hue_low2, self.red_sat_low, self.red_val_low])
        high2 = np.array([self.red_hue_high2, self.red_sat_high, self.red_val_high])
        mask2 = cv2.inRange(hsv_image, low2, high2)

        return cv2.bitwise_or(mask1, mask2)

    def _create_blue_mask(self, hsv_image):
        """Create mask for blue pixels."""
        low = np.array([self.blue_hue_low, self.blue_sat_low, self.blue_val_low])
        high = np.array([self.blue_hue_high, self.blue_sat_high, self.blue_val_high])
        return cv2.inRange(hsv_image, low, high)

    def _cleanup_mask(self, mask):
        """Apply morphological operations to clean up mask."""
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morph_kernel_size, self.morph_kernel_size)
        )
        # Remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Fill small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _validate_bumper_shape(self, contour, contour_area) -> tuple[bool, float, float]:
        """
        Validate that the contour has a bumper-like shape.

        Bumpers are rectangular, wider than tall, with aspect ratio typically 1.5-6.0.

        Returns (valid, aspect_ratio, solidity) tuple.
        """
        # Get minimum area bounding rectangle
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]

        # Avoid division by zero
        if width == 0 or height == 0:
            return (False, 0.0, 0.0)

        # Calculate aspect ratio (always length/width, so >= 1)
        aspect_ratio = max(width, height) / min(width, height)

        # Calculate solidity (how "filled" the contour is)
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)

        if hull_area == 0:
            return (False, aspect_ratio, 0.0)

        solidity = contour_area / hull_area

        # Check thresholds
        if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
            return (False, aspect_ratio, solidity)

        if solidity < self.min_solidity:
            return (False, aspect_ratio, solidity)

        return (True, aspect_ratio, solidity)

    def _calculate_confidence(
        self,
        aspect_ratio: float,
        solidity: float,
        contour_area: float
    ) -> float:
        """
        Calculate confidence score (0.0 - 1.0) based on detection quality.

        Factors:
        1. Aspect ratio score: How well it matches expected bumper ratio
        2. Solidity score: How close to 1.0
        3. Size score: How large relative to minimum area

        Returns confidence value between 0.0 and 1.0.
        """
        # Aspect ratio score (0.0 - 0.3)
        # Best score when aspect ratio is in the middle of the valid range
        ideal_aspect = (self.min_aspect_ratio + self.max_aspect_ratio) / 2
        aspect_range = self.max_aspect_ratio - self.min_aspect_ratio
        aspect_distance = abs(aspect_ratio - ideal_aspect) / (aspect_range / 2)
        aspect_score = max(0.0, 0.3 * (1.0 - aspect_distance))

        # Solidity score (0.0 - 0.3)
        solidity_score = min(0.3, max(0.0, (solidity - self.min_solidity) / (1.0 - self.min_solidity) * 0.3))

        # Size score (0.0 - 0.25)
        size_ratio = contour_area / self.min_contour_area
        size_score = min(0.25, max(0.0, (size_ratio - 1.0) / 9.0 * 0.25))

        # Base confidence (detection passed all thresholds)
        base_confidence = 0.5

        # Calculate total confidence
        confidence = base_confidence + aspect_score + solidity_score + size_score

        return max(0.0, min(1.0, confidence))

    def _detect_robots_in_mask(
        self,
        mask: np.ndarray,
        depth_image: np.ndarray,
        alliance: str
    ) -> list[RobotDetection]:
        """
        Detect robots in a color mask and return list of detections.

        Args:
            mask: Binary mask of color-matched pixels
            depth_image: Depth image for 3D projection
            alliance: "red" or "blue"

        Returns:
            List of RobotDetection objects
        """
        detections = []

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return detections

        # Sort by area (largest first) and check up to 10
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]

        for contour in sorted_contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue

            valid, aspect_ratio, solidity = self._validate_bumper_shape(contour, area)
            if not valid:
                continue

            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate centroid
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Get 3D coordinates
            point_3d = self._pixel_to_3d(cx, cy, depth_image)

            # Skip if no valid depth
            if point_3d[2] <= 0:
                continue

            # Calculate confidence
            confidence = self._calculate_confidence(aspect_ratio, solidity, area)

            detections.append(RobotDetection(
                pixel=(cx, cy),
                point_3d=point_3d,
                bbox=(x, y, w, h),
                alliance=alliance,
                confidence=confidence,
                contour=contour
            ))

        return detections

    def _pixel_to_3d(self, px, py, depth_image):
        """Convert pixel coordinates to 3D point using depth."""
        depth_meters = depth_image[py, px] * self.depth_scale

        if depth_meters <= 0:
            # Try to find nearby valid depth
            depth_meters = self._get_nearby_depth(px, py, depth_image)

        if depth_meters <= 0:
            return (0.0, 0.0, 0.0)

        # Calculate 3D coordinates using intrinsics
        x = (px - self.intrinsics.ppx) * depth_meters / self.intrinsics.fx
        y = (py - self.intrinsics.ppy) * depth_meters / self.intrinsics.fy
        z = depth_meters

        return (x, y, z)

    def _get_nearby_depth(self, px, py, depth_image, search_radius=10):
        """Search nearby pixels for valid depth value."""
        h, w = depth_image.shape

        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = px + dx, py + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        d = depth_image[ny, nx] * self.depth_scale
                        if d > 0:
                            return d
        return 0.0

    def _apply_smoothing(self, detections: list[RobotDetection]) -> list[RobotDetection]:
        """
        Apply temporal smoothing to robot detections.

        Uses a grid-based approach to match detections across frames.
        """
        if not detections:
            self.frames_since_detection += 1
            if self.frames_since_detection > 10:
                self.smoothed_robots.clear()
            return []

        self.frames_since_detection = 0

        if self.smoothing_alpha <= 0:
            return detections

        # Grid size for matching (pixels)
        grid_size = 50

        smoothed = []
        used_keys = set()

        for det in detections:
            # Find grid cell for this detection
            grid_x = det.pixel[0] // grid_size
            grid_y = det.pixel[1] // grid_size
            key = (grid_x, grid_y, det.alliance)

            # Check nearby grid cells for existing smoothed detection
            best_match = None
            best_dist = float('inf')

            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    check_key = (grid_x + dx, grid_y + dy, det.alliance)
                    if check_key in self.smoothed_robots and check_key not in used_keys:
                        prev = self.smoothed_robots[check_key]
                        dist = ((det.pixel[0] - prev.pixel[0]) ** 2 +
                               (det.pixel[1] - prev.pixel[1]) ** 2) ** 0.5
                        if dist < best_dist and dist < grid_size * 2:
                            best_dist = dist
                            best_match = check_key

            if best_match and best_match in self.smoothed_robots:
                # Apply EMA smoothing
                prev = self.smoothed_robots[best_match]
                alpha = self.smoothing_alpha

                smoothed_pixel = (
                    int(alpha * prev.pixel[0] + (1 - alpha) * det.pixel[0]),
                    int(alpha * prev.pixel[1] + (1 - alpha) * det.pixel[1])
                )

                smoothed_3d = (
                    alpha * prev.point_3d[0] + (1 - alpha) * det.point_3d[0],
                    alpha * prev.point_3d[1] + (1 - alpha) * det.point_3d[1],
                    alpha * prev.point_3d[2] + (1 - alpha) * det.point_3d[2]
                )

                smoothed_det = RobotDetection(
                    pixel=smoothed_pixel,
                    point_3d=smoothed_3d,
                    bbox=det.bbox,
                    alliance=det.alliance,
                    confidence=det.confidence,
                    contour=det.contour
                )

                used_keys.add(best_match)
                # Update stored position with new grid cell
                del self.smoothed_robots[best_match]
                self.smoothed_robots[key] = smoothed_det
                smoothed.append(smoothed_det)
            else:
                # New detection, no smoothing
                self.smoothed_robots[key] = det
                smoothed.append(det)

        # Clean up old entries
        for key in list(self.smoothed_robots.keys()):
            if key not in used_keys and key not in [(d.pixel[0] // grid_size, d.pixel[1] // grid_size, d.alliance) for d in detections]:
                del self.smoothed_robots[key]

        return smoothed

    def _publish_detections(self, detections: list[RobotDetection]):
        """Publish robot detection data to NetworkTables."""
        count = len(detections)
        self.count_pub.set(count)
        self.valid_pub.set(count > 0)

        if count == 0:
            # Publish empty arrays
            self.x_pub.set([])
            self.y_pub.set([])
            self.z_pub.set([])
            self.pixel_x_pub.set([])
            self.pixel_y_pub.set([])
            self.alliance_pub.set([])
            self.confidence_pub.set([])
        else:
            self.x_pub.set([d.point_3d[0] for d in detections])
            self.y_pub.set([d.point_3d[1] for d in detections])
            self.z_pub.set([d.point_3d[2] for d in detections])
            self.pixel_x_pub.set([d.pixel[0] for d in detections])
            self.pixel_y_pub.set([d.pixel[1] for d in detections])
            self.alliance_pub.set([d.alliance for d in detections])
            self.confidence_pub.set([d.confidence for d in detections])

    def _publish_debug_streams(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        red_mask: np.ndarray,
        blue_mask: np.ndarray,
        depth_mask: np.ndarray,
        combined_mask: np.ndarray,
        detections: list[RobotDetection]
    ):
        """Publish debug images to cscore streams."""
        if self.stream_color:
            np.copyto(self.color_frame, color_image)
            cv2.putText(self.color_frame, "COLOR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.color_output.putFrame(self.color_frame)

        if self.stream_depth:
            depth_vis = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            np.copyto(self.depth_frame, depth_colormap)
            cv2.putText(self.depth_frame, "DEPTH", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            self.depth_output.putFrame(self.depth_frame)

        if self.stream_color_mask:
            # Show red and blue masks combined with color coding
            color_mask_vis = np.zeros((480, 640, 3), dtype=np.uint8)
            # Red mask in red channel
            color_mask_vis[:, :, 2] = red_mask
            # Blue mask in blue channel
            color_mask_vis[:, :, 0] = blue_mask
            np.copyto(self.color_mask_frame, color_mask_vis)
            cv2.putText(self.color_mask_frame, "COLOR MASK (R/B)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            self.color_mask_output.putFrame(self.color_mask_frame)

        if self.stream_depth_mask:
            depth_mask_bgr = cv2.cvtColor(depth_mask, cv2.COLOR_GRAY2BGR)
            np.copyto(self.depth_mask_frame, depth_mask_bgr)
            cv2.putText(self.depth_mask_frame, "DEPTH MASK", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.depth_mask_output.putFrame(self.depth_mask_frame)

        if self.stream_combined_mask:
            # Show combined mask with color coding
            combined_vis = np.zeros((480, 640, 3), dtype=np.uint8)
            combined_vis[:, :, 2] = cv2.bitwise_and(red_mask, depth_mask)
            combined_vis[:, :, 0] = cv2.bitwise_and(blue_mask, depth_mask)
            np.copyto(self.combined_mask_frame, combined_vis)
            cv2.putText(self.combined_mask_frame, "COMBINED (R/B)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            self.combined_mask_output.putFrame(self.combined_mask_frame)

        if self.stream_result:
            np.copyto(self.result_frame, color_image)

            for det in detections:
                # Choose color based on alliance
                if det.alliance == "red":
                    box_color = (0, 0, 255)  # BGR red
                    text_color = (255, 255, 255)
                else:
                    box_color = (255, 0, 0)  # BGR blue
                    text_color = (255, 255, 255)

                # Draw bounding box
                x, y, w, h = det.bbox
                cv2.rectangle(self.result_frame, (x, y), (x + w, y + h), box_color, 2)

                # Draw centroid
                cv2.circle(self.result_frame, det.pixel, 6, box_color, -1)
                cv2.circle(self.result_frame, det.pixel, 8, (255, 255, 255), 2)

                # Draw 3D coordinate label
                label = f"{det.point_3d[2]:.2f}m"
                cv2.putText(self.result_frame, label, (det.pixel[0] + 10, det.pixel[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
                cv2.putText(self.result_frame, label, (det.pixel[0] + 10, det.pixel[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)

                # Draw contour outline
                if det.contour is not None:
                    cv2.drawContours(self.result_frame, [det.contour], 0, box_color, 1)

            # Show count
            count_text = f"Robots: {len(detections)}"
            cv2.putText(self.result_frame, count_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            self.result_output.putFrame(self.result_frame)

    def _process_frame(self) -> bool:
        """
        Process a single frame from the camera.
        Returns True if frame processed successfully, False on error.
        """
        try:
            # Get frames with timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)

            # Record processing start time
            process_start = time.perf_counter()

            # Get frame capture timestamp from camera (milliseconds)
            capture_timestamp_ms = frames.get_timestamp()

            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return True  # No frame but no error

            # Update frame time for health monitoring
            self.last_frame_time = time.time()
            self.consecutive_errors = 0

            # Apply depth filtering
            filtered_depth = self._filter_depth(depth_frame)

            # Convert to numpy arrays
            depth_image = np.asanyarray(filtered_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Convert to HSV for color detection
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Create masks
            depth_mask = self._create_depth_mask(depth_image)
            red_mask = self._create_red_mask(hsv_image)
            blue_mask = self._create_blue_mask(hsv_image)

            # Combine with depth mask
            red_combined = cv2.bitwise_and(red_mask, depth_mask)
            blue_combined = cv2.bitwise_and(blue_mask, depth_mask)

            # Clean up masks
            red_clean = self._cleanup_mask(red_combined)
            blue_clean = self._cleanup_mask(blue_combined)

            # Detect robots in each mask
            red_detections = self._detect_robots_in_mask(red_clean, depth_image, "red")
            blue_detections = self._detect_robots_in_mask(blue_clean, depth_image, "blue")

            # Merge and sort by distance (closest first)
            all_detections = red_detections + blue_detections
            all_detections.sort(key=lambda d: d.point_3d[2])

            # Limit to MAX_ROBOTS
            all_detections = all_detections[:self.MAX_ROBOTS]

            # Apply temporal smoothing
            smoothed_detections = self._apply_smoothing(all_detections)

            # Track detection status
            self.robots_detected = len(smoothed_detections) > 0

            # Publish results
            self._publish_detections(smoothed_detections)

            # Calculate and publish timing information
            process_end = time.perf_counter()
            processing_time_ms = (process_end - process_start) * 1000

            publish_timestamp_ms = time.time() * 1000
            total_latency_ms = publish_timestamp_ms - capture_timestamp_ms

            self.capture_timestamp_pub.set(capture_timestamp_ms)
            self.processing_time_pub.set(processing_time_ms)
            self.total_latency_pub.set(total_latency_ms)
            self.publish_timestamp_pub.set(publish_timestamp_ms)

            # Publish debug streams
            combined_mask = cv2.bitwise_or(red_clean, blue_clean)
            self._publish_debug_streams(
                color_image, depth_image,
                red_mask, blue_mask, depth_mask, combined_mask,
                smoothed_detections
            )

            return True

        except RuntimeError as e:
            self.consecutive_errors += 1
            error_msg = str(e)
            if "timeout" in error_msg.lower():
                print(f"Frame timeout ({self.consecutive_errors} consecutive errors)")
            else:
                print(f"RealSense error: {error_msg}")
            return False

        except Exception as e:
            self.consecutive_errors += 1
            print(f"Frame processing error: {e}")
            return False

    def run(self):
        """Main processing loop with fault detection and recovery."""
        print("Running robot detection... Press Ctrl+C to stop.")

        frame_count = 0
        fps_start_time = time.time()

        try:
            while True:
                # Check if camera needs reconnection
                if not self.camera_connected:
                    if self.reconnect_attempts < self.max_reconnect_attempts:
                        if not self._reconnect_camera():
                            time.sleep(1)
                            continue
                    else:
                        print("Waiting 10s before resetting reconnection attempts...")
                        time.sleep(10)
                        self.reconnect_attempts = 0
                        continue

                # Check camera health
                if not self._check_camera_health():
                    print("Camera health check failed, triggering reconnect...")
                    self.camera_connected = False
                    continue

                # Check for too many consecutive errors
                if self.consecutive_errors >= self.max_consecutive_errors:
                    print(f"Too many consecutive errors ({self.consecutive_errors}), triggering reconnect...")
                    self.camera_connected = False
                    continue

                # Update parameters from NetworkTables
                self._update_parameters()

                # Process frame
                if self._process_frame():
                    frame_count += 1

                # Update status
                self._update_camera_status("running", "")

                # Calculate and publish FPS
                elapsed = time.time() - fps_start_time
                if elapsed >= 1.0:
                    fps = frame_count / elapsed
                    self.fps_pub.set(fps)
                    robot_count = len(self.smoothed_robots)
                    print(f"FPS: {fps:.1f} | Robots: {robot_count} | Camera: OK | Errors: {self.consecutive_errors}")
                    frame_count = 0
                    fps_start_time = time.time()

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self._update_camera_status("stopped", "User requested shutdown")
            self._stop_realsense()
            self.nt_inst.stopClient()


def main():
    parser = argparse.ArgumentParser(
        description="Detect FRC robots by bumper color and report 3D positions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Network settings
    parser.add_argument("server", help="NetworkTables server IP")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port")

    # Depth range (meters)
    parser.add_argument("--min-depth", type=float, default=0.3, help="Minimum depth (m)")
    parser.add_argument("--max-depth", type=float, default=5.0, help="Maximum depth (m)")

    # Detection settings
    parser.add_argument("--min-area", type=int, default=2000, help="Minimum contour area (pixels)")

    # Temporal smoothing (EMA)
    parser.add_argument("--smoothing", type=float, default=0.3,
                        help="Temporal smoothing factor 0.0-0.9 (0=none, higher=smoother)")

    args = parser.parse_args()

    detector = RobotDetector(args)
    detector.run()


if __name__ == "__main__":
    main()
