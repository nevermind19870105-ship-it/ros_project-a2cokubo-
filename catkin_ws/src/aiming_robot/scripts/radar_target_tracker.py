#!/usr/bin/env python3
"""Convert one or two FMCW polar images into a target position and prediction.

The demo robot is fixed at the Gazebo world origin. Each radar reports a 2D
range/bearing image, so the output height is the radar scan plane height.
"""

from collections import deque
import math
import threading

import numpy as np
import rospy
from pathlib import Path
import rospkg
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


class RadarTargetTracker:
    def __init__(self):
        self.resolution = float(rospy.get_param("~range_resolution", 0.0438))
        self.min_range = float(rospy.get_param("~min_range", 0.6))
        self.max_range = float(rospy.get_param("~max_range", 8.0))
        self.min_intensity = int(rospy.get_param("~min_intensity", 20))
        self.sensor_y = float(rospy.get_param("~sensor_y_offset", 0.14))
        self.scan_height = float(rospy.get_param("~scan_height", 1.50))
        self.max_bearing = math.radians(float(rospy.get_param("~max_bearing_degrees", 70.0)))
        self.max_pair_separation = float(rospy.get_param("~max_pair_separation", 1.0))
        self.sensor_side = rospy.get_param("~sensor_side", "").lower()
        if self.sensor_side not in ("", "left", "right"):
            raise ValueError("~sensor_side must be empty, 'left', or 'right'")
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 0.25))
        self.predictor_mode = rospy.get_param("~predictor_mode", "mlp").lower()
        default_model = Path(rospkg.RosPack().get_path("aiming_robot")) / "models" / "radar_trajectory_mlp.npz"
        model_path = rospy.get_param("~model_path", str(default_model))
        self.model = None
        self.model_scale = 1.0
        self.model_interval = 0.25
        self.model_horizon = 0.25
        if self.predictor_mode == "mlp":
            try:
                with np.load(model_path) as archive:
                    self.model = {key: archive[key].copy() for key in ("w1", "b1", "w2", "b2", "w3", "b3")}
                    history_length = int(archive["history_length"])
                    self.model_interval = float(archive["sample_interval"])
                    self.model_horizon = float(archive["horizon_seconds"])
                    self.model_scale = float(archive["feature_scale"])
                if history_length != 8 or self.model["w1"].shape[0] != 16:
                    raise ValueError("radar model must use 8 XY observations")
                if abs(self.model_horizon - self.prediction_horizon) > 1e-4:
                    raise ValueError("model horizon does not match prediction_horizon")
                rospy.loginfo("Loaded pretrained radar MLP from %s", model_path)
            except (OSError, KeyError, ValueError) as exc:
                rospy.logwarn("Radar MLP unavailable (%s); using linear prediction", exc)
                self.predictor_mode = "linear"
        elif self.predictor_mode != "linear":
            rospy.logwarn("Unknown radar predictor_mode=%s; using linear prediction", self.predictor_mode)
            self.predictor_mode = "linear"
        self.latest = {"left": None, "right": None}
        self.last_pair = None
        self.history = deque(maxlen=40)
        self.lock = threading.Lock()

        self.position_pub = rospy.Publisher("/radar/target_position", PointStamped, queue_size=1)
        self.prediction_pub = rospy.Publisher(
            "/radar/predicted_target_position", PointStamped, queue_size=1
        )
        sides = (self.sensor_side,) if self.sensor_side else ("left", "right")
        for side in sides:
            topic = rospy.get_param("~{}_image_topic".format(side), "/shooter/radar_{}/image".format(side))
            rospy.Subscriber(topic, Image, self._image_callback, callback_args=side, queue_size=1)
        if self.sensor_side:
            rospy.loginfo("Tracking from the %s FMCW radar; predictor=%s", self.sensor_side, self.predictor_mode)
        else:
            rospy.loginfo("Fusing left/right FMCW radar images; predictor=%s", self.predictor_mode)

    def _image_callback(self, message, side):
        try:
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5.0, "Could not decode %s radar image: %s", side, exc)
            return

        if self.sensor_side:
            detection = self._detect(image, self.sensor_y if side == "left" else -self.sensor_y)
            if detection is None:
                rospy.logwarn_throttle(3.0, "No forward target return above the radar intensity threshold")
                return
            self._publish_position(detection[0], message.header.stamp)
            return

        with self.lock:
            self.latest[side] = (message.header.stamp, image)
            left = self.latest["left"]
            right = self.latest["right"]
            if left is None or right is None:
                return
            pair_key = (left[0].to_nsec(), right[0].to_nsec())
            if pair_key == self.last_pair:
                return
            # Gazebo updates the two sensors on separate simulation ticks; allow
            # one scan interval while still rejecting clearly stale pairs.
            if abs((left[0] - right[0]).to_sec()) > 1.5:
                return
            self.last_pair = pair_key

        detections = [
            self._detect(left[1], self.sensor_y),
            self._detect(right[1], -self.sensor_y),
        ]
        detections = [item for item in detections if item is not None]
        if not detections:
            rospy.logwarn_throttle(3.0, "No target return above the radar intensity threshold")
            return

        if len(detections) == 2:
            separation = float(np.linalg.norm(detections[0][0] - detections[1][0]))
            if separation > self.max_pair_separation:
                rospy.logwarn_throttle(
                    3.0, "Rejecting radar scan: left/right estimates disagree by %.2f m", separation
                )
                return
        elif len(detections) == 1:
            rospy.logwarn_throttle(5.0, "Rejecting radar scan: a matching return from both sensors is required")
            return

        weights = np.asarray([max(1.0, item[1]) for item in detections], dtype=np.float64)
        xy = np.average(np.stack([item[0] for item in detections]), axis=0, weights=weights)
        stamp = max(left[0], right[0])
        self._publish_position(xy, stamp)

    def _publish_position(self, xy, stamp):
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = "world"
        point.point.x, point.point.y, point.point.z = float(xy[0]), float(xy[1]), self.scan_height
        self.position_pub.publish(point)
        self._publish_prediction(point)

    def _detect(self, image, sensor_y):
        if image.shape[1] < 2:
            return None
        start = max(0, int(self.min_range / self.resolution))
        stop = min(image.shape[0], int(self.max_range / self.resolution) + 1)
        if stop <= start:
            return None
        roi = image[start:stop]
        # Restrict detections to the forward sector. Back-facing returns are
        # commonly reflections from the robot, ground masts, or the other radar.
        columns = np.arange(image.shape[1])
        bearings = -math.pi + 2.0 * math.pi * (columns + 0.5) / image.shape[1]
        forward_columns = np.flatnonzero(np.abs(bearings) <= self.max_bearing)
        if forward_columns.size == 0:
            return None
        sector = roi[:, forward_columns]
        row, sector_column = np.unravel_index(int(np.argmax(sector)), sector.shape)
        column = int(forward_columns[sector_column])
        intensity = int(sector[row, sector_column])
        if intensity < self.min_intensity:
            return None

        range_m = (start + row + 0.5) * self.resolution
        # The sensor image starts at -pi (its SDF min_angle) and progresses
        # toward +pi. The previous zero-based conversion rotated returns by pi.
        bearing = bearings[column]
        xy = np.asarray(
            [range_m * math.cos(bearing), sensor_y + range_m * math.sin(bearing)],
            dtype=np.float64,
        )
        return xy, intensity

    def _publish_prediction(self, point):
        stamp_seconds = point.header.stamp.to_sec()
        if self.history and stamp_seconds <= self.history[-1][0]:
            if stamp_seconds < self.history[-1][0]:
                self.history.clear()
            else:
                return
        self.history.append((stamp_seconds, point.point.x, point.point.y))
        if len(self.history) < 3 or self.prediction_horizon <= 0.0:
            return

        samples = self._resample_history()
        if samples is None:
            return
        current = samples[-1, 1:3]
        if self.predictor_mode == "mlp" and self.model is not None:
            offsets = ((samples[:, 1:3] - current) / self.model_scale).reshape(1, -1)
            delta = self._forward(offsets, self.model)[0] * self.model_scale
            future_xy = current + delta
        else:
            future_xy = self._linear_prediction(samples)
        predicted = PointStamped()
        predicted.header = point.header
        predicted.point.x, predicted.point.y = float(future_xy[0]), float(future_xy[1])
        predicted.point.z = self.scan_height
        self.prediction_pub.publish(predicted)

    def _resample_history(self):
        if len(self.history) < 8:
            return None
        samples = np.asarray(self.history, dtype=np.float64)
        interval = self.model_interval if self.predictor_mode == "mlp" else 0.25
        query_times = samples[-1, 0] - interval * np.arange(7, -1, -1)
        if query_times[0] < samples[0, 0] or samples[-1, 0] - samples[-2, 0] > 1.0:
            return None
        regular = np.column_stack((query_times, np.interp(query_times, samples[:, 0], samples[:, 1]),
                                   np.interp(query_times, samples[:, 0], samples[:, 2])))
        return regular

    @staticmethod
    def _forward(features, weights):
        hidden1 = np.tanh(features @ weights["w1"] + weights["b1"])
        hidden2 = np.tanh(hidden1 @ weights["w2"] + weights["b2"])
        return hidden2 @ weights["w3"] + weights["b3"]

    def _linear_prediction(self, samples):
        relative_time = samples[:, 0] - samples[-1, 0]
        design = np.column_stack((relative_time, np.ones_like(relative_time)))
        velocity_x = np.linalg.lstsq(design, samples[:, 1], rcond=None)[0][0]
        velocity_y = np.linalg.lstsq(design, samples[:, 2], rcond=None)[0][0]
        return np.asarray([samples[-1, 1] + velocity_x * self.prediction_horizon,
                           samples[-1, 2] + velocity_y * self.prediction_horizon])


def main():
    rospy.init_node("radar_target_tracker", anonymous=False)
    RadarTargetTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
