#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def move_drone():
    rospy.init_node('drone_mover', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    rate = rospy.Rate(30) # 30Hz
    start_time = rospy.get_time()
    
    rospy.loginfo("Drone flight node started.")
    
    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        
        # 円軌道 + 上下運動（半径1.5m, 高さ1.5mを中心に揺れ）
        x = 2.0 + 1.2 * math.cos(0.5 * t)
        y = 1.2 * math.sin(0.5 * t)
        z = 1.5 + 0.3 * math.sin(1.0 * t)
        
        state = ModelState()
        state.model_name = 'drone'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0
        state.reference_frame = 'world'
        
        try:
            set_state(state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
        rate.sleep()

if __name__ == '__main__':
    try:
        move_drone()
    except rospy.ROSInterruptException:
        pass
