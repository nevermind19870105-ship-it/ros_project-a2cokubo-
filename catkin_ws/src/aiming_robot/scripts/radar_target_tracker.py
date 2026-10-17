#!/usr/bin/env python3
"""Fuse two FMCW polar images into a target position and short-term prediction.

The demo robot is fixed at the Gazebo world origin. Each radar reports a 2D
range/bearing image, so the output height is the radar scan plane height.
"""

from collections import deque
import math
import threading

import numpy as np
import rospy
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
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 0.25))
        self.latest = {"left": None, "right": None}
        self.last_pair = None
        self.history = deque(maxlen=8)
        self.lock = threading.Lock()

        self.position_pub = rospy.Publisher("/radar/target_position", PointStamped, queue_size=1)
        self.prediction_pub = rospy.Publisher(
            "/radar/predicted_target_position", PointStamped, queue_size=1
        )
        for side in ("left", "right"):
            topic = rospy.get_param("~{}_image_topic".format(side), "/shooter/radar_{}/image".format(side))
            rospy.Subscriber(topic, Image, self._image_callback, callback_args=side, queue_size=1)
        rospy.loginfo("Fusing left/right FMCW radar images into world-coordinate target estimates")

    def _image_callback(self, message, side):
        try:
            image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5.0, "Could not decode %s radar image: %s", side, exc)
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
            if separation > 1.5:
                rospy.logwarn_throttle(
                    3.0, "Radar estimates disagree by %.2f m; using the stronger return", separation
                )
                detections = [max(detections, key=lambda item: item[1])]

        weights = np.asarray([max(1.0, item[1]) for item in detections], dtype=np.float64)
        xy = np.average(np.stack([item[0] for item in detections]), axis=0, weights=weights)
        stamp = max(left[0], right[0])
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
        row, column = np.unravel_index(int(np.argmax(roi)), roi.shape)
        intensity = int(roi[row, column])
        if intensity < self.min_intensity:
            return None

        range_m = (start + row + 0.5) * self.resolution
        bearing = -2.0 * math.pi * (column + 0.5) / image.shape[1]
        xy = np.asarray(
            [range_m * math.cos(bearing), sensor_y + range_m * math.sin(bearing)],
            dtype=np.float64,
        )
        return xy, intensity

    def _publish_prediction(self, point):
        stamp_seconds = point.header.stamp.to_sec()
        if self.history and stamp_seconds <= self.history[-1][0]:
            self.history.clear()
        self.history.append((stamp_seconds, point.point.x, point.point.y))
        if len(self.history) < 3 or self.prediction_horizon <= 0.0:
            return

        samples = np.asarray(self.history, dtype=np.float64)
        relative_time = samples[:, 0] - samples[-1, 0]
        design = np.column_stack((relative_time, np.ones_like(relative_time)))
        velocity_x = np.linalg.lstsq(design, samples[:, 1], rcond=None)[0][0]
        velocity_y = np.linalg.lstsq(design, samples[:, 2], rcond=None)[0][0]
        predicted = PointStamped()
        predicted.header = point.header
        predicted.point.x = point.point.x + velocity_x * self.prediction_horizon
        predicted.point.y = point.point.y + velocity_y * self.prediction_horizon
        predicted.point.z = self.scan_height
        self.prediction_pub.publish(predicted)


def main():
    rospy.init_node("radar_target_tracker", anonymous=False)
    RadarTargetTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
