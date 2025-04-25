#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from locobot_learning.srv import LocalGrid, LocalGridRequest
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


class SubSymbolicState:
    """
    SubSymbolicState handles low-level continuous state representation for the RecycleBot agent.
    It retrieves:
    - Local occupancy grid around the robot (via /local_grid service)
    - Relative pose (x, y) of known objects w.r.t. the robot
    """

    def __init__(self, local_view_size=10):
        """
        Initializes the service client and TF system for relative position queries.
        """
        rospy.wait_for_service('/local_grid')
        self.local_grid_client = rospy.ServiceProxy('/local_grid', LocalGrid)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.local_view_size = local_view_size
        self.target_objects = ["bin_1", "generic_object", "table", "doorway_1"]

    def get_local_grid(self) -> np.ndarray:
        """
        Queries the /local_grid service to get a normalized (0.0 to 1.0) flattened local occupancy grid.
        """
        try:
            req = LocalGridRequest(size=self.local_view_size)
            res = self.local_grid_client(req)
            grid_flat = np.array(res.grid.data, dtype=np.int8)
            normalized = grid_flat.astype(np.float32) / 100.0
            return normalized
        except rospy.ServiceException as e:
            rospy.logerr(f"[SubSymbolicState] /local_grid service call failed: {e}")
            return np.zeros(self.local_view_size * self.local_view_size, dtype=np.float32)

    def get_relative_pose(self, target_frame: str) -> list:
        """
        Returns the (x, y) relative position of the given object frame w.r.t the robot.
        """
        try:
            transform = self.tf_buffer.lookup_transform("locobot/base_link", target_frame, rospy.Time(0), rospy.Duration(1.0))
            trans = transform.transform.translation
            return [trans.x, trans.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[SubSymbolicState] TF lookup failed for {target_frame}: {e}")
            return [0.0, 0.0]

    def get_all_relative_poses(self) -> np.ndarray:
        """
        Returns a flattened array of relative (x, y) positions for all target objects.
        """
        rel_poses = [self.get_relative_pose(obj) for obj in self.target_objects]
        return np.array(rel_poses, dtype=np.float32).flatten()

    def get_subsymbolic_observation(self) -> np.ndarray:
        """
        Concatenates and returns the full subsymbolic state as a NumPy array:
        [ flattened occupancy grid | relative poses ]
        """
        grid = self.get_local_grid()
        poses = self.get_all_relative_poses()
        return np.concatenate([grid, poses])


if __name__ == "__main__":
    rospy.init_node("test_subsymbolic_state", anonymous=True)
    rospy.loginfo("Testing SubSymbolicState")

    state_extractor = SubSymbolicState(local_view_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        obs = state_extractor.get_subsymbolic_observation()
        rospy.loginfo(f"Observation shape: {obs.shape}")
        print(obs)
        rate.sleep()
