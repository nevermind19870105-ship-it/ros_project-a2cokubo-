#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

class DroneTracker:
    def __init__(self):
        rospy.init_node('drone_tracker_node', anonymous=True)

        self.pan_pub = rospy.Publisher('/shooter/pan_position_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/shooter/tilt_position_controller/command', Float64, queue_size=10)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # 砲台の設置位置と高さ
        self.shooter_x = 0.0
        self.shooter_y = 0.0
        self.shooter_z = 0.25

        rospy.loginfo("Drone Tracker Node Initialized.")

    def model_states_callback(self, msg):
        try:
            if 'drone' in msg.name:
                idx = msg.name.index('drone')
                drone_pose = msg.pose[idx]

                dx = drone_pose.position.x - self.shooter_x
                dy = drone_pose.position.y - self.shooter_y
                dz = drone_pose.position.z - self.shooter_z

                # 水平角 (Pan)
                target_pan = math.atan2(dy, dx)

                # 仰角 (Tilt) : 右手系Y軸回転（Pitch）のため、見上げる場合はマイナスまたは正負の向きに注意
                horizontal_dist = math.sqrt(dx**2 + dy**2)
                target_tilt = -math.atan2(dz, horizontal_dist)

                self.pan_pub.publish(Float64(target_pan))
                self.tilt_pub.publish(Float64(target_tilt))

        except Exception as e:
            rospy.logerr(f"Error in tracker: {e}")

if __name__ == '__main__':
    try:
        tracker = DroneTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
