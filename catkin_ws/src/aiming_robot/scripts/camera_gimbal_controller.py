#!/usr/bin/env python3
"""Aim a Gazebo camera gimbal using vision offsets or world-coordinate radar data.

This controller only operates the simulated camera pan/tilt joints. It does not
use Gazebo model-state ground truth, future-position estimates, or any firing
or projectile interface.
"""

import math

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class CameraGimbalController:
    def __init__(self):
        self.target_source = rospy.get_param("~target_source", "vision").lower()
        self.offset_topic = rospy.get_param("~offset_topic", "/vision/marker_offset")
        self.radar_topic = rospy.get_param("~radar_topic", "/radar/predicted_target_position")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/shooter/joint_states")
        self.pan_command_topic = rospy.get_param(
            "~pan_command_topic", "/shooter/pan_position_controller/command"
        )
        self.tilt_command_topic = rospy.get_param(
            "~tilt_command_topic", "/shooter/tilt_position_controller/command"
        )

        self.horizontal_fov = math.radians(float(rospy.get_param("~horizontal_fov_degrees", 60.0)))
        self.vertical_fov = math.radians(float(rospy.get_param("~vertical_fov_degrees", 46.8)))
        self.gain = max(0.0, float(rospy.get_param("~gain", 0.6)))
        self.aim_origin_z = float(rospy.get_param("~aim_origin_z", 0.25))
        self.deadband = max(0.0, float(rospy.get_param("~deadband", 0.01)))
        self.pan_limits = (
            float(rospy.get_param("~pan_min", -3.14)),
            float(rospy.get_param("~pan_max", 3.14)),
        )
        self.tilt_limits = (
            float(rospy.get_param("~tilt_min", -1.57)),
            float(rospy.get_param("~tilt_max", 1.57)),
        )

        self.joint_positions = {}
        self.pan_publisher = rospy.Publisher(self.pan_command_topic, Float64, queue_size=1)
        self.tilt_publisher = rospy.Publisher(self.tilt_command_topic, Float64, queue_size=1)
        self.joint_subscriber = rospy.Subscriber(
            self.joint_state_topic, JointState, self._joint_state_callback, queue_size=1
        )
        if self.target_source == "radar":
            self.offset_subscriber = rospy.Subscriber(
                self.radar_topic, PointStamped, self._radar_callback, queue_size=1
            )
        elif self.target_source == "vision":
            self.offset_subscriber = rospy.Subscriber(
                self.offset_topic, PointStamped, self._offset_callback, queue_size=1
            )
        else:
            raise ValueError("~target_source must be 'vision' or 'radar'")

        rospy.loginfo("Camera gimbal target source=%s", self.target_source)

    def _radar_callback(self, message):
        if "pan_joint" not in self.joint_positions or "tilt_joint" not in self.joint_positions:
            return
        x, y, z = message.point.x, message.point.y, message.point.z
        if not all(math.isfinite(value) for value in (x, y, z)) or x <= 0.0:
            return

        # The turret points along +X at zero pan/tilt. Positive tilt rotates
        # the barrel down around +Y, so upward targets require a negative pitch.
        desired_pan = math.atan2(y, x)
        horizontal_range = math.hypot(x, y)
        desired_tilt = -math.atan2(z - self.aim_origin_z, horizontal_range)
        current_pan = self.joint_positions["pan_joint"]
        current_tilt = self.joint_positions["tilt_joint"]
        pan_error = math.atan2(math.sin(desired_pan - current_pan), math.cos(desired_pan - current_pan))
        tilt_error = desired_tilt - current_tilt
        if abs(pan_error) > self.deadband:
            self.pan_publisher.publish(Float64(data=self._clamp(
                current_pan + self.gain * pan_error, self.pan_limits
            )))
        if abs(tilt_error) > self.deadband:
            self.tilt_publisher.publish(Float64(data=self._clamp(
                current_tilt + self.gain * tilt_error, self.tilt_limits
            )))

    def _joint_state_callback(self, message):
        for joint_name in ("pan_joint", "tilt_joint"):
            try:
                index = message.name.index(joint_name)
                self.joint_positions[joint_name] = float(message.position[index])
            except (ValueError, IndexError):
                continue

    def _offset_callback(self, message):
        if "pan_joint" not in self.joint_positions or "tilt_joint" not in self.joint_positions:
            return

        error_x = float(message.point.x)
        error_y = float(message.point.y)
        if not math.isfinite(error_x) or not math.isfinite(error_y):
            return
        if abs(error_x) > 1.0 or abs(error_y) > 1.0:
            return

        pan = self.joint_positions["pan_joint"]
        tilt = self.joint_positions["tilt_joint"]
        pan_error = math.atan(error_x * math.tan(self.horizontal_fov / 2.0))
        tilt_error = math.atan(error_y * math.tan(self.vertical_fov / 2.0))

        if abs(error_x) > self.deadband:
            # Positive image-x is to the right; positive pan rotates left.
            pan = self._clamp(pan - self.gain * pan_error, self.pan_limits)
            self.pan_publisher.publish(Float64(data=pan))
        if abs(error_y) > self.deadband:
            # Positive image-y is down; positive tilt rotates the camera down.
            tilt = self._clamp(tilt + self.gain * tilt_error, self.tilt_limits)
            self.tilt_publisher.publish(Float64(data=tilt))

    @staticmethod
    def _clamp(value, limits):
        return min(max(value, limits[0]), limits[1])


def main():
    rospy.init_node("camera_gimbal_controller", anonymous=False)
    CameraGimbalController()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
