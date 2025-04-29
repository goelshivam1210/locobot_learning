from typing import Union
import rospy
import tf
import numpy as np
from locobot_learning.srv import LocalGrid, LocalGridRequest, LocalGridResponse
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from std_srvs.srv import Empty


class LocalGridService:
    def __init__(self):
        """
        Initializes the LocalGrid service. This service provides a local costmap around the robot, with a
        configurable size.
        """
        rospy.init_node('LocalGridService', anonymous=True)

        self.local_grid_srv = rospy.Service("/local_grid", LocalGrid, self.get_grid_callback)
        rospy.loginfo("Local Grid service is initialized")
        rospy.loginfo("Local Grid service is ready")


        # Initialize the LocalViewExtractor with a specified size for the local view
        self.occupancy_grid: Union[np.ndarray, None] = None  # To store the occupancy grid data
        self.grid_info: Union[MapMetaData, None] = None
        self.tf_listener = tf.TransformListener()  # Listener for transform data

        self.grid_subscriber: Union[rospy.Subscriber, None] = None
        self.subscribe()


    def subscribe(self):
        """
        Subscribes to the costmap topic that we use to construct the local costmap.
        """

        if self.grid_subscriber is not None:
            # If we're already subscribed, we need to resubscribe or else we'll have a stale reference
            # to the costmap.
            self.grid_subscriber.unregister()
        # Subscribe to the occupancy grid topic to receive grid maps published by the robot
        self.grid_subscriber = rospy.Subscriber(
            "/locobot/move_base/global_costmap/costmap",
            OccupancyGrid,
            self.occupancy_grid_callback
        )

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
            rospy.loginfo("Cleared the cost maps")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def get_local_view(self, size: int):
        # Function to extract and return a local view of the occupancy grid centered around the robot's current position
        if self.occupancy_grid is None:
            # If the occupancy grid hasn't been received yet, log a message and return None
            rospy.loginfo("Occupancy grid not yet received.")

            return None

        try:
            # Get the robot's current position in the map frame
            (trans, _) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))
            x, y = trans[0], trans[1]  # Extract the x, y coordinates of the robot's position

            # Convert the robot's world coordinates to occupancy grid coordinates
            grid_x = int((x - self.grid_info.origin.position.x) / self.grid_info.resolution)
            grid_y = int((y - self.grid_info.origin.position.y) / self.grid_info.resolution)

            # Perform boundary checks and adjustments to ensure the extracted local view remains within the grid bounds
            half_size = size // 2
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

            (height, width) = local_view.shape

            msg = OccupancyGrid(
                header=Header(
                    seq=0,
                    stamp=rospy.Time.now(),
                    frame_id="map"
                ),
                info=MapMetaData(
                    map_load_time=self.grid_info.map_load_time,
                    resolution=self.grid_info.resolution,
                    width=width,
                    height=height,
                    origin=self.grid_info.origin, #TODO: Adjust the origin to account for our changes
                ),
                # OccupancyGrid data is defined as a one dimensional array (tuple)
                data=local_view.flatten()
            )

            return msg  # Return the extracted local view
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        finally:
            self.subscribe()
            self.clear_costmaps()


    def get_grid_callback(self, req: LocalGridRequest):
        """
        Callback to get the current local occupancy grid.
        """
        rospy.loginfo("Obtaining local occupancy grid")

        size = req.size

        local_view_msg = self.get_local_view(size=size)
        return LocalGridResponse(
            grid=local_view_msg
        )


if __name__ == "__main__":
    LocalGridService()
    rospy.spin()