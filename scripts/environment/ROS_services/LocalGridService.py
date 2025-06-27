#!/usr/bin/env python3

from typing import Union, Tuple
import sys
import os
import rospy
import tf
import numpy as np
from locobot_learning.srv import LocalGrid, LocalGridRequest, LocalGridResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from std_srvs.srv import Empty

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from occupancy_grid import OccupancyGridMerger
from local_view_generator import LocalViewGenerator

class LocalGridService:
    def __init__(self):
        """
        Initializes the LocalGrid service. This service provides a local costmap around the robot, with a
        configurable size.
        """
        rospy.init_node('LocalGridService', anonymous=True)

        self.local_grid_srv = rospy.Service("/local_grid", LocalGrid, self.get_grid_callback)
        rospy.loginfo("[LocalGridService] Local Grid service is initialized")
        rospy.loginfo("[LocalGridService] Local Grid service is ready")


        self.tf_listener = tf.TransformListener()  # Listener for transform data
        self.merger = OccupancyGridMerger()  # Merger for occupancy grid and costmap data
        self.generator = LocalViewGenerator()  # Generator for local views

    def occupancy_grid_callback(self, data: OccupancyGrid):
        # Callback function for processing occupancy grid messagesgrid_info
        # Reshape the flat occupancy grid data into a 2D NumPy array
        self.occupancy_grid = np.array(data.data).reshape((data.info.height, data.info.width))
        self.grid_info = data.info  # Store the metadata of the occupancy grid

    def clear_costmaps(self):
        """
        Clears costmaps for the planner. This is used to eliminate the buildup of costmap errors over time.
        """
        rospy.wait_for_service('/locobot/move_base/clear_costmaps')
        try:
            clear_costmaps = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
            clear_costmaps()
            rospy.loginfo("[LocalGridService] Cleared the cost maps")
        except rospy.ServiceException as e:
            rospy.logerr("[LocalGridService] Service call failed: %s", e)


    def get_grid_callback(self, req: LocalGridRequest) -> LocalGridResponse:
        """
        Callback to get the current local occupancy grid.
        """
        rospy.loginfo("[LocalGridService] Obtaining local occupancy grid")

        size = req.size

        # res = self.get_local_view(size=size)
        occupancy_grid, grid_info = self.merger.get_occupancy_grid()
        if occupancy_grid is None:
            rospy.logerr("[LocalGridService] Failed to get occupancy grid")
            return LocalGridResponse(
                grid=None,
            )
        
        local_view = self.generator.get_local_view(occupancy_grid, grid_info, view_size=size)


        if local_view is None:
            rospy.logerr("[LocalGridService] Failed to get local occupancy grid")
            return LocalGridResponse(
                grid=None,
            )

        return LocalGridResponse(
            grid=local_view,
        )


if __name__ == "__main__":
    LocalGridService()
    rospy.spin()