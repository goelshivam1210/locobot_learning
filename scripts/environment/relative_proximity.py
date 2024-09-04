#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Point
import math

class DistanceOrientationCalculator:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        rospy.init_node('distance_orientation_calculator', anonymous=True)
        
        # Define the locations of the objects
        self.objects = {
            "table": Point(-1.0656546354293823, 0.5288128852844238, 0.0),
            "toy": Point(-1.5815556049346924, 0.04497265815734863, 0.0),
            "bin": Point(0.06352591514587402, -1.584059476852417, 0.0)
        }

    def compute_distance_and_orientation(self, robot_position, robot_orientation, object_position):
        # Compute relative distance
        distance = math.sqrt((object_position.x - robot_position[0])**2 + (object_position.y - robot_position[1])**2)
        
        # Compute relative orientation
        orientation_to_object = math.atan2(object_position.y - robot_position[1], object_position.x - robot_position[0])
        
        # Robot's current orientation (yaw)
        _, _, robot_yaw = tf.transformations.euler_from_quaternion(robot_orientation)
        
        # Relative orientation
        relative_orientation = orientation_to_object - robot_yaw
        relative_orientation = math.atan2(math.sin(relative_orientation), math.cos(relative_orientation))  # Normalize
        
        return distance, relative_orientation

    def update_and_print_distances_orientations(self):
        try:
            # Get the robot's current position and orientation in the map frame
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            print(f"Robot's position in map: x={trans[0]}, y={trans[1]}")

            for object_name, object_position in self.objects.items():
                distance, orientation = self.compute_distance_and_orientation(trans, rot, object_position)
                print(f"Distance to {object_name}: {distance:.2f}, Relative orientation: {orientation:.2f} rad")
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)

if __name__ == '__main__':
    calculator = DistanceOrientationCalculator()
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        calculator.update_and_print_distances_orientations()
        rate.sleep()
