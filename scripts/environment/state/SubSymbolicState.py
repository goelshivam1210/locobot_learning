#!/usr/bin/env python3

from typing import Tuple, Union
import rospy
import tf
import numpy as np
from collections import namedtuple
from locobot_learning.srv import LocalGrid, LocalGridRequest

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from check_for_collision import would_collide
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from defaults import DEFAULT_LOCAL_VIEW_SIZE
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'agent', 'core')))
from PDDLActions import PDDLActions


StructuredObservation = namedtuple('StructuredObservation', [
    'occupancy_grid',
    'is_obstructed',
    'relative_poses',
])


class SubSymbolicState:
    """
    SubSymbolicState handles low-level continuous state representation for the RecycleBot agent.
    It retrieves:
    - Local occupancy grid around the robot (via /local_grid service). Omitted if include_grid=False.
    - A bit indicating if the robot is obstructed 
    - Relative pose (x, y) of known objects w.r.t. the robot
    """

    def __init__(self, local_view_size: Union[int, None]=None, include_grid=True):
        """
        Initializes the service client and TF system for relative position queries.
        """
        if local_view_size is None:
            local_view_size = DEFAULT_LOCAL_VIEW_SIZE
        rospy.wait_for_service('/local_grid')
        self.local_grid_client = rospy.ServiceProxy('/local_grid', LocalGrid)

        self.tf_listener = tf.TransformListener()

        self.local_view_size = local_view_size
        self.include_grid = include_grid
        self.target_objects = ["bin_1", "generic_object", "table", "atdoor", "postdoor"]

        try:
            self.real_nav_goals = rospy.get_param("real_nav_goals")
        except rospy.ROSException as e:
            rospy.logwarn("real_nav_goals not found on param server.")
            self.real_nav_goals = {}

    def get_local_grid(self) -> Union[Tuple[np.ndarray, np.shape], None]:
        """
        Queries the /local_grid service to get a normalized (0.0 to 1.0) flattened local occupancy grid.
        """
        if not self.include_grid:
            return None
        try:
            req = LocalGridRequest(size=self.local_view_size)
            res = self.local_grid_client(req)
            grid_flat = np.array(res.grid.data, dtype=np.int8)
            height, width = res.grid.info.height, res.grid.info.width
            normalized = grid_flat.astype(np.float32) / 100.0
            return normalized, (height, width)
        except rospy.ServiceException as e:
            rospy.logerr(f"[SubSymbolicState] /local_grid service call failed: {e}")
            return np.zeros(self.local_view_size * self.local_view_size, dtype=np.float32), (self.local_view_size, self.local_view_size)


    def get_robot_position(self):
        try:
            self.tf_listener.waitForTransform('/map', 'locobot/base_link', rospy.Time(0), rospy.Duration(3.0))
            (trans, _) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))
        
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed for robot position: {e}.")
            return None

    def get_relative_pose(self, target_name: str) -> list:
        """
        Computes (x, y) relative position between robot and given object (from param server).
        """
        # Get robot position
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            rospy.logwarn(f"[SubSymbolicState] Robot pose not available, returning zeros.")
            return [0.0, 0.0]

        # Some code maps "bin" in the nav goals to "bin_1"
        processed_target_name = PDDLActions.map_to_generic_object(target_name)
            
        # Get object absolute position from params
        obj_coords = self.real_nav_goals.get(processed_target_name)
        if obj_coords is None:
            rospy.logwarn(f"[SubSymbolicState] Object {target_name} not found in real_nav_goals, returning zeros.")
            return [0.0, 0.0]
        
        obj_x, obj_y = obj_coords['position']['x'], obj_coords['position']['y']

        # Relative position = object - robot
        rel_x = obj_x - robot_pos[0]
        rel_y = obj_y - robot_pos[1]

        return [rel_x, rel_y]


    def get_all_relative_poses(self) -> np.ndarray:
        """
        Returns a flattened array of relative (x, y) positions for all target objects.
        """
        rel_poses = [self.get_relative_pose(obj) for obj in self.target_objects]
        return np.array(rel_poses, dtype=np.float32).flatten()

    def get_subsymbolic_observation(self) -> np.ndarray:
        """
        Concatenates and returns the full subsymbolic state as a NumPy array:
        [ flattened occupancy grid | is_obstructed | relative poses ]
        """
        grid = None
        grid_reshaped = None
        grid_result = self.get_local_grid()
        if grid_result is not None:
            grid, shape = grid_result
            grid_reshaped = grid.reshape(shape)
        obstructed = 1 if grid_reshaped is not None and would_collide(grid_reshaped) else 0
        poses = self.get_all_relative_poses()
        obs = np.concatenate([[obstructed], poses])
        if grid is not None:
            obs = np.concatenate([grid, obs])
        
        return obs
    
    def parse_observation(self, obs: np.ndarray) -> StructuredObservation:
        """
        Parses the observation array into a structured format.
        """
        occupancy_grid = None
        index = 0
        if self.include_grid:
            grid_size = self.local_view_size * self.local_view_size
            occupancy_grid = obs[index:grid_size].reshape((self.local_view_size, self.local_view_size))
            index += grid_size
        is_obstructed = obs[index]
        index += 1
        relative_poses_arr = obs[index:]
        pose_index = 0
        relative_poses = {}
        for obj in self.target_objects:
            if pose_index >= len(relative_poses_arr):
                raise ValueError(f"Observation does not contain relative pose for object: {obj}")

            relative_poses[obj] = relative_poses_arr[pose_index:pose_index+2].tolist()            
            pose_index += 2

        return StructuredObservation(
            occupancy_grid=occupancy_grid,
            is_obstructed=bool(is_obstructed),
            relative_poses=relative_poses
        )


if __name__ == "__main__":
    rospy.init_node("test_subsymbolic_state", anonymous=True)
    rospy.loginfo("Testing SubSymbolicState")

    state_extractor = SubSymbolicState()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        obs = state_extractor.get_subsymbolic_observation()
        rospy.loginfo(f"Observation shape: {obs.shape}")
        print(obs)
        rate.sleep()
