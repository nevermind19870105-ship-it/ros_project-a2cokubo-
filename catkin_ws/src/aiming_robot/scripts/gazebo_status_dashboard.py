#!/usr/bin/env python3
"""Show Gazebo gimbal angles, its camera image, and marker coordinates."""

import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, JointState


def rotate_vector(quaternion, vector):
    """Rotate a 3D vector by a ROS-order quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    qv = np.asarray([x, y, z], dtype=np.float64)
    v = np.asarray(vector, dtype=np.float64)
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + w * v)


def rotate_y(angle, vector):
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = vector
    return np.asarray([c * x + s * z, y, -s * x + c * z])


def rotate_z(angle, vector):
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = vector
    return np.asarray([c * x - s * y, s * x + c * y, z])


class GazeboStatusDashboard:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/vision/annotated_image")
        self.joint_topic = rospy.get_param("~joint_state_topic", "/shooter/joint_states")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.predicted_pixel_topic = rospy.get_param(
            "~predicted_pixel_topic", "/vision/predicted_pixel"
        )
        self.marker_model = rospy.get_param("~marker_model", "demo_marker")
        self.gimbal_model = rospy.get_param("~gimbal_model", "camera_gimbal")
        self.horizontal_fov = math.radians(float(rospy.get_param("~horizontal_fov_degrees", 60.0)))
        self.marker_diameter = float(rospy.get_param("~marker_diameter", 0.30))
        self.prediction_horizon = float(rospy.get_param("~radar_prediction_horizon", 0.25))
        self.radar_predictor_mode = rospy.get_param("~radar_predictor_mode", "mlp").upper()
        self.radar_sides = rospy.get_param("~radar_sides", ["left", "right"])
        self.radar_range_resolution = float(rospy.get_param("~radar_range_resolution", 0.0438))
        self.radar_min_range = float(rospy.get_param("~radar_min_range", 1.0))
        self.radar_max_range = float(rospy.get_param("~radar_max_range", 8.0))

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image = None
        self.joint_angles = {}
        self.marker_position = None
        self.gimbal_pose = None
        self.predicted_pixel = None
        self.radar_position = None
        self.radar_prediction = None
        self.image_stamp = None
        self.radar_images = {side: None for side in self.radar_sides}

        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(self.joint_topic, JointState, self._joint_callback, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self._models_callback, queue_size=1)
        rospy.Subscriber(
            self.predicted_pixel_topic, PointStamped, self._prediction_callback, queue_size=1
        )
        rospy.Subscriber("/radar/target_position", PointStamped, self._radar_position_callback, queue_size=1)
        rospy.Subscriber(
            "/radar/predicted_target_position", PointStamped,
            self._radar_prediction_callback, queue_size=1
        )
        for side in self.radar_sides:
            topic = rospy.get_param("~radar_{}_topic".format(side), "/shooter/radar_{}/image".format(side))
            rospy.Subscriber(topic, Image, self._radar_callback, callback_args=side, queue_size=1)
        rospy.loginfo("Gazebo status dashboard listening for camera, joints, and model states")

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Dashboard image conversion failed: %s", exc)
            return
        with self.lock:
            self.image = image
            self.image_stamp = message.header.stamp

    def _radar_callback(self, message, side):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Radar %s image conversion failed: %s", side, exc)
            return
        with self.lock:
            self.radar_images[side] = image

    def _joint_callback(self, message):
        angles = {}
        for name in ("pan_joint", "tilt_joint"):
            try:
                angles[name] = float(message.position[message.name.index(name)])
            except (ValueError, IndexError):
                continue
        with self.lock:
            self.joint_angles.update(angles)

    def _models_callback(self, message):
        try:
            marker_index = message.name.index(self.marker_model)
            gimbal_index = message.name.index(self.gimbal_model)
        except ValueError:
            return
        marker = message.pose[marker_index].position
        gimbal = message.pose[gimbal_index]
        marker_position = np.asarray([marker.x, marker.y, marker.z], dtype=np.float64)
        gimbal_pose = (
            np.asarray([gimbal.position.x, gimbal.position.y, gimbal.position.z]),
            np.asarray(
                [gimbal.orientation.x, gimbal.orientation.y, gimbal.orientation.z, gimbal.orientation.w]
            ),
        )
        with self.lock:
            self.marker_position = marker_position
            self.gimbal_pose = gimbal_pose

    def _prediction_callback(self, message):
        with self.lock:
            self.predicted_pixel = (float(message.point.x), float(message.point.y), float(message.point.z))

    def _radar_position_callback(self, message):
        with self.lock:
            self.radar_position = np.asarray(
                [message.point.x, message.point.y, message.point.z], dtype=np.float64
            )

    def _radar_prediction_callback(self, message):
        with self.lock:
            self.radar_prediction = np.asarray(
                [message.point.x, message.point.y, message.point.z], dtype=np.float64
            )

    def _predicted_world_position(self, image_shape, angles, gimbal_pose, prediction):
        """Back-project predicted pixels using apparent size of the known sphere."""
        if prediction is None or gimbal_pose is None or not all(
            name in angles for name in ("pan_joint", "tilt_joint")
        ):
            return None
        u, v, apparent_width = prediction
        height, width = image_shape[:2]
        if apparent_width < 2.0 or width < 2 or height < 2:
            return None

        fx = width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        fy = fx
        depth = fx * self.marker_diameter / apparent_width
        x_optical = (u - width / 2.0) * depth / fx
        y_optical = (v - height / 2.0) * depth / fy

        # Camera image axes (right, down, forward) to tilt_link (forward, left, up).
        ray_in_link = np.asarray([depth, -x_optical, -y_optical])
        pan = angles["pan_joint"]
        tilt = angles["tilt_joint"]
        ray_world = rotate_vector(gimbal_pose[1], rotate_z(pan, rotate_y(tilt, ray_in_link)))

        # Joint origins are 0.15 m and 0.10 m above their respective parents;
        # the camera is mounted 0.30 m forward from tilt_link.
        camera_offset = np.asarray([0.0, 0.0, 0.25]) + rotate_z(
            pan, rotate_y(tilt, np.asarray([self.camera_forward_offset, 0.0, 0.0]))
        )
        camera_world = gimbal_pose[0] + rotate_vector(gimbal_pose[1], camera_offset)
        return camera_world + ray_world

    def run(self):
        self.camera_forward_offset = float(rospy.get_param("~camera_forward_offset", 0.30))
        window = "Gazebo Gimbal Monitor"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            with self.lock:
                image = None if self.image is None else self.image.copy()
                angles = dict(self.joint_angles)
                marker = None if self.marker_position is None else self.marker_position.copy()
                gimbal = self.gimbal_pose
                prediction = self.predicted_pixel
                radar_images = dict(self.radar_images)
                radar_position = None if self.radar_position is None else self.radar_position.copy()
                radar_prediction = None if self.radar_prediction is None else self.radar_prediction.copy()

            if image is None:
                image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(image, "Waiting for camera image...", (25, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 220, 255), 2, cv2.LINE_AA)
            predicted_world = self._predicted_world_position(image.shape, angles, gimbal, prediction)

            panel_width = 440
            radar_panel_height = 320
            canvas = np.zeros((max(image.shape[0], 560) + radar_panel_height, image.shape[1] + panel_width, 3), dtype=np.uint8)
            canvas[:image.shape[0], :image.shape[1]] = image
            x0 = image.shape[1] + 24
            lines = [
                ("GIMBAL JOINT ANGLES", (255, 230, 120)),
                ("Pan:  {}".format(self._angle_text(angles.get("pan_joint"))), (240, 240, 240)),
                ("Tilt: {}".format(self._angle_text(angles.get("tilt_joint"))), (240, 240, 240)),
                ("", (240, 240, 240)),
                ("MARKER WORLD POSITION (Gazebo)", (100, 255, 140)),
                (self._point_text(marker), (240, 240, 240)),
                ("", (240, 240, 240)),
                ("PREDICTED WORLD POSITION (vision)", (80, 180, 255)),
                (self._point_text(predicted_world), (240, 240, 240)),
                ("", (240, 240, 240)),
                ("Estimate depth from known 0.30 m marker size.", (180, 180, 180)),
                ("Predicted pixel: {}".format(self._pixel_text(prediction)), (180, 180, 180)),
                ("World frame: Gazebo world [m]", (180, 180, 180)),
                ("", (240, 240, 240)),
                ("RADAR FUSED POSITION", (100, 255, 140)),
                (self._point_text(radar_position), (240, 240, 240)),
                ("RADAR {} PREDICTION (+{:.2f}s)".format(
                    self.radar_predictor_mode, self.prediction_horizon
                ), (80, 180, 255)),
                (self._point_text(radar_prediction), (240, 240, 240)),
            ]
            canvas_height = max(560, 42 + len(lines) * 38) + radar_panel_height
            if canvas_height > canvas.shape[0]:
                expanded = np.zeros((canvas_height, canvas.shape[1], 3), dtype=np.uint8)
                expanded[:canvas.shape[0], :, :] = canvas
                canvas = expanded
            y = 42
            for text, color in lines:
                if text:
                    scale = 0.62 if text.isupper() else 0.48
                    cv2.putText(canvas, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                                color, 1 if scale < 0.6 else 2, cv2.LINE_AA)
                y += 38
            for index, side in enumerate(self.radar_sides):
                x = index * 520 + 10
                radar = radar_images[side]
                if radar is None:
                    radar_bgr = np.zeros((120, 500, 3), dtype=np.uint8)
                    cv2.putText(radar_bgr, "Waiting for FMCW radar...", (15, 65), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 220, 255), 1, cv2.LINE_AA)
                else:
                    peak = int(np.max(radar))
                    if peak > 0:
                        # Scale each scan to its own peak before shrinking so
                        # sparse, weak returns remain visible in the monitor.
                        scaled = np.clip(radar.astype(np.float32) * (255.0 / peak), 0, 255).astype(np.uint8)
                        scaled = cv2.dilate(scaled, np.ones((3, 3), dtype=np.uint8), iterations=1)
                        resized = cv2.resize(scaled, (500, 120), interpolation=cv2.INTER_AREA)
                        radar_bgr = cv2.applyColorMap(resized, cv2.COLORMAP_TURBO)
                        radar_bgr[resized == 0] = (0, 0, 0)
                    else:
                        radar_bgr = np.zeros((120, 500, 3), dtype=np.uint8)
                cv2.putText(canvas, "{} FMCW radar  (angle x range, relative strength)".format(side.upper()),
                            (x, image.shape[0] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                            (200, 230, 255), 1, cv2.LINE_AA)
                canvas[image.shape[0] + 28:image.shape[0] + 148, x:x + 500] = radar_bgr
                cv2.putText(canvas, "Range profile: strongest angle at each distance",
                            (x, image.shape[0] + 170), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (200, 230, 255), 1, cv2.LINE_AA)
                profile = self._radar_range_profile(radar)
                chart_y = image.shape[0] + 178
                canvas[chart_y:chart_y + profile.shape[0], x:x + profile.shape[1]] = profile
            cv2.imshow(window, canvas)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                rospy.signal_shutdown("Dashboard window closed")
                break
            rate.sleep()
        cv2.destroyAllWindows()

    @staticmethod
    def _angle_text(angle):
        return "--" if angle is None else "{:.3f} rad  ({:.1f} deg)".format(angle, math.degrees(angle))

    @staticmethod
    def _point_text(point):
        return "--" if point is None else "x={:+.3f}  y={:+.3f}  z={:+.3f} m".format(*point)

    @staticmethod
    def _pixel_text(prediction):
        return "--" if prediction is None else "u={:.0f}  v={:.0f} px".format(*prediction[:2])

    def _radar_range_profile(self, radar):
        """Plot max return across azimuth versus range, with raw-value axes."""
        panel = np.zeros((140, 500, 3), dtype=np.uint8)
        if radar is None:
            cv2.putText(panel, "Waiting for FMCW radar...", (125, 75), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 220, 255), 1, cv2.LINE_AA)
            return panel
        start = max(0, int(self.radar_min_range / self.radar_range_resolution))
        stop = min(radar.shape[0], int(self.radar_max_range / self.radar_range_resolution) + 1)
        if stop <= start or radar.shape[1] == 0:
            return panel

        profile = radar[start:stop, :].max(axis=1).astype(np.float64)
        raw_peak = float(profile.max()) if profile.size else 0.0
        if raw_peak <= 0.0:
            cv2.putText(panel, "No return in selected range", (95, 75), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (160, 160, 160), 1, cv2.LINE_AA)
            return panel

        left, right, top, bottom = 54, 492, 8, 112
        y_step = max(1, int(math.ceil(raw_peak / 4.0)))
        y_max = y_step * max(1, int(math.ceil(raw_peak * 1.15 / y_step)))
        cv2.putText(panel, "strength", (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32,
                    (180, 180, 180), 1, cv2.LINE_AA)

        for value in range(0, y_max + 1, y_step):
            y = bottom - int((value / float(y_max)) * (bottom - top))
            cv2.line(panel, (left, y), (right, y), (45, 45, 45), 1, cv2.LINE_AA)
            cv2.putText(panel, str(value), (left - 35, y + 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.34, (180, 180, 180), 1, cv2.LINE_AA)

        range_span = max(1e-6, self.radar_max_range - self.radar_min_range)
        first_tick = int(math.ceil(self.radar_min_range))
        last_tick = int(math.floor(self.radar_max_range))
        for value in range(first_tick, last_tick + 1):
            x = left + int(((value - self.radar_min_range) / range_span) * (right - left))
            cv2.line(panel, (x, top), (x, bottom), (40, 40, 40), 1, cv2.LINE_AA)
            cv2.putText(panel, str(value), (x - 5, 128), cv2.FONT_HERSHEY_SIMPLEX,
                        0.34, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(panel, "range (m)", (225, 138), cv2.FONT_HERSHEY_SIMPLEX,
                    0.34, (180, 180, 180), 1, cv2.LINE_AA)

        ranges = (np.arange(start, stop, dtype=np.float64) + 0.5) * self.radar_range_resolution
        points = []
        for range_m, strength in zip(ranges, profile):
            x = left + int(((range_m - self.radar_min_range) / range_span) * (right - left))
            y = bottom - int((strength / float(y_max)) * (bottom - top))
            points.append((min(right, max(left, x)), min(bottom, max(top, y))))
        if len(points) > 1:
            cv2.polylines(panel, [np.asarray(points, dtype=np.int32)], False,
                          (40, 100, 255), 2, cv2.LINE_AA)
        peak_index = int(np.argmax(profile))
        cv2.circle(panel, points[peak_index], 3, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(panel, "peak={:.0f}".format(raw_peak), (left + 5, top + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (220, 220, 220), 1, cv2.LINE_AA)
        return panel


def main():
    rospy.init_node("gazebo_status_dashboard", anonymous=False)
    try:
        GazeboStatusDashboard().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
