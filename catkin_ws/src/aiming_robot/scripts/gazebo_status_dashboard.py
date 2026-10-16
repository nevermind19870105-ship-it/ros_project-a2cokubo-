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

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image = None
        self.joint_angles = {}
        self.marker_position = None
        self.gimbal_pose = None
        self.predicted_pixel = None
        self.image_stamp = None

        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(self.joint_topic, JointState, self._joint_callback, queue_size=1)
        rospy.Subscriber(self.model_states_topic, ModelStates, self._models_callback, queue_size=1)
        rospy.Subscriber(
            self.predicted_pixel_topic, PointStamped, self._prediction_callback, queue_size=1
        )
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
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            with self.lock:
                image = None if self.image is None else self.image.copy()
                angles = dict(self.joint_angles)
                marker = None if self.marker_position is None else self.marker_position.copy()
                gimbal = self.gimbal_pose
                prediction = self.predicted_pixel

            if image is None:
                image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(image, "Waiting for camera image...", (25, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 220, 255), 2, cv2.LINE_AA)
            predicted_world = self._predicted_world_position(image.shape, angles, gimbal, prediction)

            panel_width = 440
            canvas = np.zeros((max(image.shape[0], 560), image.shape[1] + panel_width, 3), dtype=np.uint8)
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
            ]
            y = 42
            for text, color in lines:
                if text:
                    scale = 0.62 if text.isupper() else 0.48
                    cv2.putText(canvas, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                                color, 1 if scale < 0.6 else 2, cv2.LINE_AA)
                y += 38
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


def main():
    rospy.init_node("gazebo_status_dashboard", anonymous=False)
    try:
        GazeboStatusDashboard().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
