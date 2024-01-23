import rospy
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_geometry_msgs #pleasthis helps in the tf2 transform error and exception


def marker_callback(marker_msg):
    # Create a PoseStamped message to hold the transformed pose
    pose_stamped = PoseStamped()
    pose_stamped.header = marker_msg.header
    pose_stamped.pose = marker_msg.pose
    pose_stamped.header.frame_id = "locobot/camera_link"
    
    # Transform the pose from camera_color_optical_frame to arm_base_link
    try:
        transformed_pose = tf_buffer.transform(pose_stamped, "locobot/arm_base_link")
        
        # Create a new Marker message with the transformed pose
        transformed_marker = Marker()
        transformed_marker.header = transformed_pose.header
        transformed_marker.id = marker_msg.id
        transformed_marker.type = marker_msg.type
        transformed_marker.action = marker_msg.action
        transformed_marker.pose = transformed_pose.pose
        transformed_marker.scale = marker_msg.scale
        transformed_marker.color = marker_msg.color
        transformed_marker.lifetime = marker_msg.lifetime
        transformed_marker.frame_locked = marker_msg.frame_locked
        transformed_marker.points = marker_msg.points
        transformed_marker.colors = marker_msg.colors
        transformed_marker.text = marker_msg.text
        transformed_marker.mesh_resource = marker_msg.mesh_resource
        transformed_marker.mesh_use_embedded_materials = marker_msg.mesh_use_embedded_materials
        
        # Publish the transformed marker message
        pub.publish(transformed_marker)
    except Exception as e:
        rospy.logwarn("Transformation failed for ID %d: %s", marker_msg.id, str(e))

if __name__ == "__main__":
    rospy.init_node("marker_transformer")
    
    # Initialize a TF buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    
    # Subscribe to the original Marker topic
    rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, marker_callback)
    
    # Create a Publisher for the transformed Marker messages
    pub = rospy.Publisher("/locobot/transformed_markers", Marker, queue_size=10)
    
    # Keep the node running
    rospy.spin()


