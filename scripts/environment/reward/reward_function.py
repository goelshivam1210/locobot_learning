from typing import Set
import numpy as np
import rospy
import sys
import os
from shapely.geometry import Point, Polygon


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'state')))
from observation_space import ObservationSpace
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'agent', 'core')))
from planner.planner import Action
from PDDLActions import PDDLActions




class RewardFunction:
    def __init__(self, plannable_state: Set, failed_operator: Action, use_sparse: bool = False):
        """
        plannable_state: set of predicates that defines recovery state
        """
        self.plannable_state = plannable_state
        self.failed_operator = failed_operator
        self.facing_boundaries = rospy.get_param('facing_boundaries', None)
        if self.facing_boundaries is None:
            raise ValueError("Facing boundaries not found in parameter server. Please set 'facing_boundaries' parameter.")
        self.at_boundaries = rospy.get_param('at_boundaries', None)
        if self.at_boundaries is None:
            raise ValueError("At boundaries not found in parameter server. Please set 'at_boundaries' parameter.")
        self.nav_goals = rospy.get_param('real_nav_goals', None)
        if self.nav_goals is None:
            raise ValueError("Navigation goals not found in parameter server. Please set 'real_nav_goals' parameter.")
    
    def _boundary_contains_waypoint(self, boundary: Polygon, waypoint: str) -> bool:
        waypoint = PDDLActions.map_to_generic_object(waypoint)
        waypoint_coords = self.nav_goals.get(waypoint, None)
        if waypoint_coords is None:
            rospy.logwarn(f"[RewardFunction] No coordinates found for waypoint: {waypoint}")
            return False
        waypoint_coords = waypoint_coords["position"]
        point = Point(waypoint_coords['x'], waypoint_coords['y'])
        return point.within(boundary)

    def get_distance_to_goal(self, observation_space: ObservationSpace):
        """
        Computes the distance to the goal based on the failed operator
        """
        targets = []
        for effect in self.failed_operator.add_effects:
            effect = list(effect)
            effect_name = effect.pop(0)
            if effect_name == "facing":
                facing_waypoint = effect[0]
                if facing_waypoint == 'nothing':
                    continue
                targets = [facing_waypoint]
            elif effect_name == "at":
                location = effect[0]
                if location.startswith('bin_'):
                    #TODO: Handle at bin case
                    pass
                elif location.startswith('room_'):
                    room_boundary = self.at_boundaries.get(location, None)
                    if room_boundary is None:
                        rospy.logwarn(f"[RewardFunction] No boundary found for room: {location}")
                        return None
                    room_boundary = Polygon(room_boundary)
                    waypoints = observation_space.subsymbolic_state.target_objects
                    targets = [
                        waypoint for waypoint in waypoints
                        if self._boundary_contains_waypoint(room_boundary, waypoint)
                    ]
                else:
                    rospy.logwarn(f"[RewardFunction] Unrecognized location: {location}")
                    return None

        
        if len(targets) == 0:
            return None
        
        obs = observation_space.subsymbolic_state.get_subsymbolic_observation()
        if obs is None:
            rospy.logwarn("[RewardFunction] Subsymbolic observation is None, cannot compute distance.")
            return None
        
        obs = observation_space.subsymbolic_state.parse_observation(obs)
        distances = {target: np.sqrt(x**2 + y**2) for target, (x,y) in obs.relative_poses.items() if target in targets}  

        for target in targets:
            x, y = obs.relative_poses.get(target, (None, None))
            if x is None or y is None:
                rospy.logwarn(f"[RewardFunction] No relative pose found for target: {target}")
                continue
            distance = np.sqrt(x**2 + y**2)
            rospy.loginfo(f"Distance to {target}: {distance}")

        if len(distances) == 0:
            rospy.logwarn("[RewardFunction] No distances computed, returning None.")
            return None

        min_distance = min(distances.values())
        
        return min_distance



    def compute_reward(self, observation_space: ObservationSpace):
        """
        observation_space: an observation space object from which we can get current symbolic and subsymbolic states
        Returns: reward (float), done (bool)
        """
        current_symbolic_state = observation_space.symbolic_state.get_current_predicates()

        rospy.loginfo(f"[RewardFunction] Current symbolic state: {current_symbolic_state}")
        rospy.loginfo(f"[RewardFunction] Plannable state: {self.plannable_state}")
        rospy.loginfo(f"[RewardFunction] Is current state a subset: {len(current_symbolic_state) > 0 and current_symbolic_state.issubset(self.plannable_state)}")

        reward = 0.0
        done = False
        distance = self.get_distance_to_goal(observation_space)
        if distance is not None:
            reward += 1/distance  # reward inversely proportional to distance to goal
        if len(current_symbolic_state) > 0 and current_symbolic_state.issubset(self.plannable_state):
            reward += 1.0
            done = True  # success achieved
        rospy.loginfo(f"[RewardFunction] Giving a reward of {reward}")
        if done:
            rospy.loginfo("[RewardFunction] Episode done, success achieved.")
        return reward, done



if __name__ == "__main__":
    from planner.planner import Planner
    from Agent import Agent
    from PDDLActions import PDDLActions
    from PDDLPredicates import PDDLPredicates
    from argparse import ArgumentParser

    parser = ArgumentParser()

    parser.add_argument(
        "-c", "--current",
        action="store_true",
        help="If passed, this makes the plan use the current state, rather than a hard-coded state representing the state when the curtain novelty fails.",
    )

    args = parser.parse_args()

    # Path to PDDL domain file
    domain_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'knowledge', 'PDDL', 'recycle_bot', 'domain.pddl'))

    # Define objects for the problem
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }


    rospy.init_node('test_reward_function', anonymous=True)
    planner = Planner(domain_file)
    obs_space = ObservationSpace()
    actions = PDDLActions()
    predicates = PDDLPredicates()
    agent = Agent(planner, actions, predicates)

    if args.current:
        initial_state = agent.fetch_current_state(objects)
    else:
        initial_state = {
            'object_locations': {'ball_1': 'room_1', 'bin_1': 'room_2'},
            'robot_facing': 'doorway_1',
            'robot_holding': 'ball_1',
            'robot_location': 'room_1'
        }


    planner.new_problem(objects, initial_state)

    plan = planner.plan
    failed_operator_name = 'pass_through_door'
    failed_operator_params = None

    failed_operator = None
    for op in plan:
        if op.name == failed_operator_name:
            failed_operator = op
            break
    
    if failed_operator is None:
        raise ValueError(f"[RewardFunction test] Could not find operator for failed operator name {failed_operator_name}")

    plannable_states = planner.compute_plannable_states(plan, failed_operator)

    rf = RewardFunction(plannable_state=plannable_states, failed_operator=failed_operator)
    reward, done = rf.compute_reward(obs_space)
    rospy.loginfo(f"[RewardFunction] Reward: {reward}, Done: {done}")
