#!/usr/bin/env python3
"""Detect and passively visualize the future position of a green marker.

This node publishes an annotated image only. Prediction is a constant-velocity
baseline used for visual comparison; no target coordinates or actuator commands
are published.
"""

from collections import deque
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


class VisionMarkerTracker:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/vision/annotated_image")
        self.offset_topic = rospy.get_param("~offset_topic", "/vision/marker_offset")
        self.predicted_pixel_topic = rospy.get_param(
            "~predicted_pixel_topic", "/vision/predicted_pixel"
        )
        self.min_area = float(rospy.get_param("~min_area", 100.0))
        self.prediction_horizon = max(0.0, float(rospy.get_param("~prediction_horizon", 0.25)))
        self.history_length = max(2, int(rospy.get_param("~history_length", 8)))
        self.predictor_mode = rospy.get_param("~predictor_mode", "mlp").lower()
        self.history = deque(maxlen=self.history_length)
        self.missed_frames = 0
        self.model_weights = None
        self.model_feature_scale = 1.0
        self._load_model(rospy.get_param("~model_path", ""))

        lower = rospy.get_param("~hsv_lower", [40, 70, 70])
        upper = rospy.get_param("~hsv_upper", [85, 255, 255])
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError("hsv_lower and hsv_upper must each contain 3 HSV values")
        self.hsv_lower = np.array(lower, dtype=np.uint8)
        self.hsv_upper = np.array(upper, dtype=np.uint8)

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.offset_publisher = rospy.Publisher(self.offset_topic, PointStamped, queue_size=1)
        self.predicted_pixel_publisher = rospy.Publisher(
            self.predicted_pixel_topic, PointStamped, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.kernel = np.ones((3, 3), dtype=np.uint8)

        rospy.loginfo("Visual marker demo listening on %s", self.image_topic)
        rospy.loginfo("Annotated images will be published on %s", self.output_topic)

    def image_callback(self, image_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Could not convert camera image: %s", exc)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated = frame.copy()
        detected = False
        predicted_msg = PointStamped()
        predicted_msg.header = image_msg.header
        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area >= self.min_area:
                x, y, width, height = cv2.boundingRect(contour)
                center = (x + width // 2, y + height // 2)
                offset_msg = PointStamped()
                offset_msg.header = image_msg.header
                offset_msg.point.x = float((center[0] - frame.shape[1] / 2.0) / (frame.shape[1] / 2.0))
                offset_msg.point.y = float((center[1] - frame.shape[0] / 2.0) / (frame.shape[0] / 2.0))
                offset_msg.point.z = float(area / (frame.shape[0] * frame.shape[1]))
                self.offset_publisher.publish(offset_msg)
                stamp = image_msg.header.stamp.to_sec()
                if stamp <= 0.0:
                    stamp = rospy.get_time()
                future = self._predict(center, stamp, frame.shape)
                self.missed_frames = 0

                if future is not None:
                    predicted_msg.point.x = float(future[0])
                    predicted_msg.point.y = float(future[1])
                    predicted_msg.point.z = float(width)

                cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 255, 0), 2)
                cv2.drawMarker(
                    annotated,
                    center,
                    (0, 255, 255),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=18,
                    thickness=2,
                )
                cv2.putText(
                    annotated,
                    "Current position",
                    (max(8, x), max(24, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                if future is not None:
                    future_x = int(round(np.clip(future[0], 0, frame.shape[1] - 1)))
                    future_y = int(round(np.clip(future[1], 0, frame.shape[0] - 1)))
                    cv2.line(annotated, center, (future_x, future_y), (255, 120, 0), 2)
                    cv2.circle(annotated, (future_x, future_y), 8, (255, 120, 0), 2)
                    cv2.putText(
                        annotated,
                        "Prediction +{:.2f}s".format(self.prediction_horizon),
                        (max(8, min(future_x, frame.shape[1] - 250)), max(24, future_y - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 150, 0),
                        2,
                        cv2.LINE_AA,
                    )
                detected = True

        self.predicted_pixel_publisher.publish(predicted_msg)

        if not detected:
            self.missed_frames += 1
            if self.missed_frames > 4:
                self.history.clear()
            cv2.putText(
                annotated,
                "Waiting for green marker",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            "Visual prediction only - no actuator output",
            (12, annotated.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        try:
            output_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            output_msg.header = image_msg.header
            self.publisher.publish(output_msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Could not publish annotated image: %s", exc)

    def _load_model(self, model_path):
        if self.predictor_mode != "mlp":
            return
        if not model_path or not os.path.isfile(model_path):
            rospy.logwarn("Pretrained model not found; using linear prediction")
            self.predictor_mode = "linear"
            return

        try:
            with np.load(model_path, allow_pickle=False) as data:
                model_history_length = int(data["history_length"])
                model_horizon = float(data["horizon_seconds"])
                if model_history_length != self.history_length:
                    raise ValueError("model history_length does not match node configuration")
                if abs(model_horizon - self.prediction_horizon) > 1e-6:
                    raise ValueError("model horizon_seconds does not match node configuration")
                self.model_feature_scale = float(data["feature_scale"])
                self.model_weights = {
                    name: np.asarray(data[name], dtype=np.float32)
                    for name in ("w1", "b1", "w2", "b2", "w3", "b3")
                }
            rospy.loginfo("Loaded pretrained trajectory model from %s", model_path)
        except (OSError, KeyError, ValueError) as exc:
            rospy.logwarn("Could not load pretrained model (%s); using linear prediction", exc)
            self.predictor_mode = "linear"
            self.model_weights = None

    def _predict(self, center, stamp, frame_shape):
        """Predict a short-horizon image position using MLP or linear baseline."""
        if self.history and stamp <= self.history[-1][0]:
            self.history.clear()
        self.history.append((stamp, float(center[0]), float(center[1])))
        if len(self.history) < 3 or self.prediction_horizon <= 0.0:
            return None

        samples = np.asarray(self.history, dtype=np.float64)
        relative_times = samples[:, 0] - samples[-1, 0]

        if (
            self.predictor_mode == "mlp"
            and self.model_weights is not None
            and len(samples) >= self.history_length
        ):
            model_history = samples[-self.history_length :, 1:3]
            current = model_history[-1]
            image_scale = np.asarray([frame_shape[1], frame_shape[0]], dtype=np.float32)
            offsets = (model_history - current) / image_scale
            features = (offsets.reshape(1, -1) * self.model_feature_scale).astype(np.float32)
            hidden1 = np.tanh(features @ self.model_weights["w1"] + self.model_weights["b1"])
            hidden2 = np.tanh(hidden1 @ self.model_weights["w2"] + self.model_weights["b2"])
            predicted_delta = (
                hidden2 @ self.model_weights["w3"] + self.model_weights["b3"]
            )[0] / self.model_feature_scale
            predicted = np.asarray(center, dtype=np.float64) + predicted_delta * image_scale
            if np.all(np.isfinite(predicted)):
                return float(predicted[0]), float(predicted[1])

        velocity_x = np.polyfit(relative_times, samples[:, 1], 1)[0]
        velocity_y = np.polyfit(relative_times, samples[:, 2], 1)[0]
        return (
            center[0] + velocity_x * self.prediction_horizon,
            center[1] + velocity_y * self.prediction_horizon,
        )


def main():
    rospy.init_node("vision_marker_tracker", anonymous=False)
    try:
        VisionMarkerTracker()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exc:
        rospy.logerr("Vision marker demo stopped: %s", exc)


if __name__ == "__main__":
    main()
