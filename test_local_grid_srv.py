import rospy
import numpy as np
from locobot_learning.srv import LocalGrid, LocalGridRequest
import time

def test_service():
    # Call the service to clear costmaps
    rospy.wait_for_service('/locobot/move_base/clear_costmaps')
    try:
        get_local_grid = rospy.ServiceProxy('/local_grid', LocalGrid)
        res = get_local_grid(LocalGridRequest(
            size=15
        ))

        width = res.grid.info.width

        height = res.grid.info.height

        data = np.array(res.grid.data).reshape(height, width)
        print(data)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    while True:
        test_service()
        print("\n\n\n")
        time.sleep(2)