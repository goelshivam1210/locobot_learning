#!/usr/bin/env python3

'''
This class implements the observation generator for the environment.
'''

import rospy
import sys
import os
import itertools
import time

sys.path.append("../knowledge/PDDL/recycle_bot")
sys.path.append("../knowledge/pddl-parser")

import numpy as np
from pddl_parser.PDDL import PDDL_Parser

from locobot_custom.srv import GazeboObservation
from std_srvs.srv import Empty


# from ObservationUtils import GazeboObservationGenerator


class ObservationGenerator:
    def __init__(self, domain_file, problem_file):

         # Initialize ROS node (if not already initialized)
        if not rospy.get_node_uri():
            rospy.init_node('observation_generator_client', anonymous=True)

        # Reference to the environment
        # self.env = env
        self.parser = PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)

        # self.observation_size = self.get_observation_size()
        # self.observation = self.get_observation()

    # def _get_problem_file(self):
    #     generate_new_problem_file
    #     return self.parser.problem_file

    def reset(self):
        try:
            # Create a service proxy for the reset service
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

            # Call the service
            reset_world()
            rospy.loginfo("Gazebo world reset successfully!")
            self.clear_costmaps()
            rospy.loginfo("Costmaps cleared successfully!")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def clear_costmaps(self):
        try:
            # Assuming you're using move_base and the default service names
            clear_costmaps_service_name = "/locobot/move_base/clear_costmaps"
            rospy.wait_for_service(clear_costmaps_service_name, timeout=5)
            clear_costmaps = rospy.ServiceProxy(clear_costmaps_service_name, Empty)
            clear_costmaps()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


    def get_observation(self):
        # Call the gazebo_observation service
        try:
            gazebo_observation_service = rospy.ServiceProxy('GazeboObservation', GazeboObservation)
            response = gazebo_observation_service()
            
            flattened_occupancy_grid = np.array(response.occupancy_grid)
            relative_locations = np.column_stack((response.relative_locations_x, response.relative_locations_y)).flatten()
            relative_orientations = np.array(response.relative_orientations)

            observation = np.concatenate([flattened_occupancy_grid, relative_locations, relative_orientations, self.get_predicates_vectors()])
            # print ("observation = {}".format(observation))
            return observation

        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return []
    
    def get_predicates_vectors(self):

        facable_objects = self.parser.types['facable'] #get the types
        facable_object_instances = [instance for faceable in facable_objects for instance in self.parser.objects.get(faceable, [])]

        holdable_objects = self.parser.types['holdable']
        holdable_object_instances = [instance for holdable in holdable_objects for instance in self.parser.objects.get(holdable, [])]

        robots = self.parser.objects['robot']
        
        rooms = self.parser.objects['room']

        facing_vector = [0] * len(facable_object_instances)
        holding_vector = [0] * len(holdable_object_instances)
        at_vector = [0] * len(rooms)

        for predicate in self.parser.state:
            if predicate[0] == 'facing':
                object_index = facable_object_instances.index(predicate[1])
                facing_vector[object_index] = 1
            elif predicate[0] == 'hold':
                object_index = holdable_object_instances.index(predicate[1])
                holding_vector[object_index] = 1
            elif predicate[0] == 'at' and predicate[2] in robots:
                room_index = rooms.index(predicate[1])
                at_vector[room_index] = 1
        # print ("facing_vector: ", facing_vector)
        # print ("holding_vector: ", holding_vector)
        # print ("at_vector: ", at_vector)

        return facing_vector + holding_vector + at_vector
    
    def get_observation_space_size(self):
        self.observation_size = len(self.get_observation())
        return self.observation_size
    
# testing
def main():
    # Specify the paths to your domain and problem PDDL files
    print (os.getcwd())
    domain_file = "../knowledge/PDDL/recycle_bot/domain.pddl"
    problem_file = "../knowledge/PDDL/recycle_bot/problem.pddl"  # Replace with your actual problem PDDL file path

    # Create an ObservationGenerator instance
    observation_generator = ObservationGenerator(domain_file, problem_file)

    # Generate an observation
    observation = observation_generator.get_observation()

    # Print the observation
    print(observation)

if __name__ == "__main__":
    main()
