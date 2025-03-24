import rospy
import tf
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs


class SubSymbolicState:
    def __init__(self, local_view_size=10):
        self.local_view_size = local_view_size
        self.occupancy_grid = None
        self.grid_info = None

        self.tf_listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.grid_subscriber = rospy.Subscriber(
            "/locobot/rtabmap/grid_map", OccupancyGrid, self.occupancy_grid_callback
        )

        # Relative pose target objects
        self.target_objects = ["bin_1", "generic_object", "table", "doorway_1"]

    def occupancy_grid_callback(self, data):
        self.occupancy_grid = np.array(data.data).reshape(
            (data.info.height, data.info.width)
        )
        self.grid_info = data.info

    def get_robot_position(self):
        try:
            (trans, _) = self.tf_listener.lookupTransform(
                "/map", "locobot/base_link", rospy.Time(0)
            )
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup failed for robot position.")
            return None

    def get_local_view(self):
        if self.occupancy_grid is None or self.grid_info is None:
            rospy.logwarn("Occupancy grid not ready.")
            return np.zeros((self.local_view_size, self.local_view_size))

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return np.zeros((self.local_view_size, self.local_view_size))

        x, y = robot_pos
        grid_x = int((x - self.grid_info.origin.position.x) / self.grid_info.resolution)
        grid_y = int((y - self.grid_info.origin.position.y) / self.grid_info.resolution)

        half = self.local_view_size // 2
        start_x = max(grid_x - half, 0)
        end_x = min(grid_x + half + 1, self.grid_info.width)
        start_y = max(grid_y - half, 0)
        end_y = min(grid_y + half + 1, self.grid_info.height)

        local_view = self.occupancy_grid[start_y:end_y, start_x:end_x]
        padded = np.zeros((self.local_view_size, self.local_view_size))
        padded[: local_view.shape[0], : local_view.shape[1]] = local_view

        return padded / 100.0  # normalize to [0,1]

    def get_relative_position(self, object_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                "locobot/base_link", object_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            trans = transform.transform.translation
            return [trans.x, trans.y]
        except Exception as e:
            rospy.logwarn(f"TF lookup failed for {object_frame}: {e}")
            return [0.0, 0.0]

    def get_relative_poses(self):
        return [self.get_relative_position(obj) for obj in self.target_objects]

    def get_symbolic_predicates(self, at_room, holding, facing):
        # at[room_1, room_2] = 2 dims
        # holding[nothing, object] = 1 dim
        # facing[generic_object, table, bin, doorway, nothing] = 5 dims

        at_encoding = [1.0 if at_room == "room_1" else 0.0, 1.0 if at_room == "room_2" else 0.0]
        hold_encoding = [1.0 if holding else 0.0]
        facing_options = ["generic_object", "table", "bin", "doorway", "nothing"]
        facing_encoding = [1.0 if facing == opt else 0.0 for opt in facing_options]

        return at_encoding + hold_encoding + facing_encoding

    def get_observation(self, at_room, holding, facing):
        local_view = self.get_local_view().flatten()  # 1D array
        relative_poses = np.array(self.get_relative_poses()).flatten()
        predicates = np.array(self.get_symbolic_predicates(at_room, holding, facing))
        return np.concatenate([local_view, relative_poses, predicates])
