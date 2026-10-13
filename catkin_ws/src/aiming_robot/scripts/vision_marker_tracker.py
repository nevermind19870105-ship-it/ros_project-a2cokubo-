#!/usr/bin/env python3
"""Visualize a green marker in a ROS image stream.

This node is a passive computer-vision demonstration: it publishes an annotated
image only. It does not estimate a future position, publish machine-actionable
coordinates, or command any robot actuator.
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class VisionMarkerTracker:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/vision/annotated_image")
        self.min_area = float(rospy.get_param("~min_area", 100.0))

        lower = rospy.get_param("~hsv_lower", [40, 70, 70])
        upper = rospy.get_param("~hsv_upper", [85, 255, 255])
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError("hsv_lower and hsv_upper must each contain 3 HSV values")
        self.hsv_lower = np.array(lower, dtype=np.uint8)
        self.hsv_upper = np.array(upper, dtype=np.uint8)

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
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
        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area >= self.min_area:
                x, y, width, height = cv2.boundingRect(contour)
                center = (x + width // 2, y + height // 2)
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
                    "Green marker detected",
                    (max(8, x), max(24, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                detected = True

        if not detected:
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
            "Visual demo only - no prediction or actuator output",
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


def main():
    rospy.init_node("vision_marker_tracker", anonymous=False)
    try:
        VisionMarkerTracker()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exc:
        rospy.logerr("Vision marker demo stopped: %s", exc)


if __name__ == "__main__":
    main()
