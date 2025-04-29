#!/usr/bin/env python3

import rospy
from locobot_learning.srv import PrimitiveBase, PrimitiveBaseRequest

def call_primitive_base(action_type, value):
    rospy.wait_for_service('primitive_base_service')
    try:
        primitive_base = rospy.ServiceProxy('primitive_base_service', PrimitiveBase)
        req = PrimitiveBaseRequest(action_type=action_type, value=value)
        resp = primitive_base(req)
        if resp.success:
            rospy.loginfo(f"[Test Client] Action {action_type} successful!")
        else:
            rospy.logwarn(f"[Test Client] Action {action_type} failed!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('primitive_base_test_client')

    rospy.sleep(1.0)  # Give some time to connect
    call_primitive_base("move_forward", 0.4)  # Move forward
    rospy.sleep(2.0)

    call_primitive_base("turn_left", 0.5)  # Turn left
    rospy.sleep(2.0)

    call_primitive_base("turn_right", 0.5)  # Turn right
