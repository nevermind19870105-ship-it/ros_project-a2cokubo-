#!/usr/bin/env python3
"""Move a harmless green marker in Gazebo for a vision demo."""

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest


def main():
    rospy.init_node("move_demo_marker", anonymous=False)
    rospy.wait_for_service("/gazebo/set_model_state")
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # Give Gazebo a moment to spawn the demo marker before updating it.
    rospy.sleep(1.0)
    rate = rospy.Rate(15)
    start_time = rospy.get_time()
    rospy.loginfo("Moving the green demo marker in a slow, repeatable loop")

    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        state = ModelState()
        state.model_name = "demo_marker"
        state.reference_frame = "world"
        state.pose.position.x = 2.0 + 0.8 * math.cos(0.35 * t)
        state.pose.position.y = 0.2 + 0.8 * math.sin(0.35 * t)
        state.pose.position.z = 1.5 + 0.15 * math.sin(0.7 * t)
        state.pose.orientation.w = 1.0

        request = SetModelStateRequest()
        request.model_state = state
        try:
            response = set_model_state(request)
            if not response.success:
                rospy.logwarn_throttle(5.0, "Gazebo did not update demo_marker: %s", response.status_message)
        except rospy.ServiceException as exc:
            rospy.logerr_throttle(5.0, "Gazebo model-state service failed: %s", exc)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
