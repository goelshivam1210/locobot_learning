#!/usr/bin/env python3

import rospy 
import tf 
import numpy as np 
from nav_msgs.msg import OccupancyGrid  
import tf2_geometry_msgs

class LocalViewExtractor:
    def __init__(self, local_view_size=8):
        # Initialize the LocalViewExtractor with a specified size for the local view
        self.local_view_size = local_view_size  # The size of the local view square
        self.occupancy_grid = None  # To store the occupancy grid data
        self.tf_listener = tf.TransformListener()  # Listener for transform data

        # Subscribe to the occupancy grid topic to receive grid maps published by the robot
        self.grid_subscriber = rospy.Subscriber("/locobot/rtabmap/grid_map", OccupancyGrid, self.occupancy_grid_callback)

    def occupancy_grid_callback(self, data):
        # Callback function for processing occupancy grid messages
        # Reshape the flat occupancy grid data into a 2D NumPy array
        self.occupancy_grid = np.array(data.data).reshape((data.info.height, data.info.width))
        self.grid_info = data.info  # Store the metadata of the occupancy grid

    def get_local_view(self):
        # Function to extract and return a local view of the occupancy grid centered around the robot's current position
        if self.occupancy_grid is None:
            # If the occupancy grid hasn't been received yet, log a message and return None
            rospy.loginfo("Occupancy grid not yet received.")
            return None

        try:
            # Get the robot's current position in the map frame
            (trans, _) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))
            x, y = trans[0], trans[1]  # Extract the x, y coordinates of the robot's position
            print(f"Robot's position in map: x={x}, y={y}")

            # Convert the robot's world coordinates to occupancy grid coordinates
            grid_x = int((x - self.grid_info.origin.position.x) / self.grid_info.resolution)
            grid_y = int((y - self.grid_info.origin.position.y) / self.grid_info.resolution)

            # Perform boundary checks and adjustments to ensure the extracted local view remains within the grid bounds
            half_size = self.local_view_size // 2
            start_x = max(grid_x - half_size, 0)
            end_x = min(grid_x + half_size + 1, self.grid_info.width)
            start_y = max(grid_y - half_size, 0)
            end_y = min(grid_y + half_size + 1, self.grid_info.height)

            # Extract the local view from the occupancy grid, centered around the robot's current position
            local_view = self.occupancy_grid[start_y:end_y, start_x:end_x]

            # Determine whether the robot is centered within the extracted local view
            view_center_x = (end_x - start_x) // 2
            view_center_y = (end_y - start_y) // 2
            robot_centered = (view_center_x == half_size) and (view_center_y == half_size)
            print(f"Is robot centered in local view? {robot_centered}")

            return local_view  # Return the extracted local view
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            return None

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('local_view_extractor')
    # Create an instance of the LocalViewExtractor with a specified local view size
    extractor = LocalViewExtractor(local_view_size=10)

    # Define the loop rate (in Hz)
    rate = rospy.Rate(1)  # Adjust the rate as needed for your application
    while not rospy.is_shutdown():
        local_view = extractor.get_local_view()
        if local_view is not None:
            print("Local view: \n", local_view)
        rate.sleep() 
