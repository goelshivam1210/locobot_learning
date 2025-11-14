import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import image_geometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class BinVisibilityChecker:
    def __init__(self, debug: bool = False):
        self.debug = debug

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_info = None
        self.camera_model = None

        self.depth_tolerance = 0.15
        self.bin_height = 0.145
        at_boundaries = rospy.get_param('at_boundaries', None)
        if at_boundaries is None:
            raise ValueError("At boundaries not found in parameter server. Please set 'at_boundaries' parameter.")
        bin_boundary = at_boundaries.get('bin_1', None)
        if bin_boundary is None:
            raise ValueError("No boundary found for bin_1. Please check the at_boundaries parameter.")
        
        self.bin_polygon = bin_boundary

        self.camera_frame = "locobot/camera_color_optical_frame"
        self.base_frame = "map"

        self.debug_bin_pub = rospy.Publisher("~debug_bin", Marker, queue_size=1)
        self.projected_face_marker_pub = rospy.Publisher("~projected_face_marker", Marker, queue_size=1)
        self.debug_image_pub = rospy.Publisher("~debug_overlay", Image, queue_size=1)

        self.rgb_sub = rospy.Subscriber("locobot/camera/color/image_raw", Image, self.rgb_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("locobot/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1)
        self.info_sub = rospy.Subscriber("locobot/camera/color/camera_info", CameraInfo, self.info_callback, queue_size=1)

        self.floor_z = self.get_floor_z_from_base_link() + 0.016 # Floor height is actually slightly above base_link, at least when facing the bin

        self.rgb_image = None
        self.depth_image = None

    def get_floor_z_from_base_link(self) -> float:
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'locobot/base_link', rospy.Time(0), rospy.Duration(1.0)
            )
            return trans.transform.translation.z
        except Exception as e:
            rospy.logwarn(f"[BinVisibilityChecker] Failed to get base_link Z: {e}")
            return 0.0  # Fallback if transform fails


    def get_visibility(self) -> float:
        while self.rgb_image is None or self.depth_image is None:
            rospy.loginfo("Waiting for RGB and depth images...")
            rospy.sleep(0.1)
            
        selected_faces, _ = self.project_bin_to_image()

        if not selected_faces:
            rospy.logwarn("[BinVisibilityChecker] No face selected")
            return 0.0
        if self.debug:
            self.publish_debug_bin()

        red_mask = self.get_red_mask(self.rgb_image)

        visible_mask = np.zeros_like(red_mask, dtype=np.uint8)
        total_area = 0
        total_visible = 0

        for _, poly_2d, _ in selected_faces:
            poly_np = np.array(poly_2d, dtype=np.int32)
            if len(poly_np) < 3:
                continue

            face_mask = np.zeros_like(red_mask, dtype=np.uint8)
            cv2.fillPoly(face_mask, [poly_np], 255)
            expected_depth = self.compute_expected_face_depth(poly_np)
            proximity_mask = np.abs(self.depth_image.astype(np.float32) / 1000.0 - expected_depth) < self.depth_tolerance
            proximity_mask = proximity_mask.astype(np.uint8) * 255

            visible = cv2.bitwise_and(red_mask, face_mask)
            visible = cv2.bitwise_and(visible, proximity_mask)

            face_visible = cv2.countNonZero(visible)
            face_area = cv2.countNonZero(face_mask)

            total_area += face_area
            total_visible += face_visible

            visible_mask = cv2.bitwise_or(visible_mask, visible)

            # rospy.loginfo(f"Face {i}: visible = {face_visible}, total = {face_area}, ratio = {(face_visible / face_area) * 100 if face_area else 0:.2f}%")

        rospy.loginfo(f"[BinVisibilityChecker] Total visible = {total_visible}, total area = {total_area}")
        visible_ratio = 0.0
        if total_area > 0:
            visible_ratio = total_visible / total_area
            rospy.loginfo(f"[BinVisibilityChecker] Bin visible: {(visible_ratio * 100.0):.2f}%")

        if self.debug:
            self.publish_debug_overlay(self.rgb_image, selected_faces[0][1], red_mask)

            for _, _, face3d in selected_faces:
                self.publish_projected_face_marker(face3d)


        return visible_ratio
        

    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)

    def get_red_mask(self, rgb_image: np.ndarray):
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        return red_mask

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def compute_expected_face_depth(self, polygon):
        mask = np.zeros_like(self.depth_image, dtype=np.uint8)
        cv2.fillPoly(mask, [polygon], 255)
        depths = self.depth_image[mask > 0].astype(np.float32) / 1000.0
        if len(depths) == 0:
            return 0
        return np.median(depths)

    def depth_callback(self, msg):
        if self.camera_info is None:
            return
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def project_bin_to_image(self):
        faces = []
        bin_poly = self.bin_polygon
        h = self.bin_height

        base = [np.array([x, y, self.floor_z]) for x, y in bin_poly]
        top = [np.array([x, y, self.floor_z + h]) for x, y in bin_poly]

        num_pts = len(bin_poly)
        for i in range(num_pts):
            next_i = (i + 1) % num_pts
            faces.append([base[i], base[next_i], top[next_i], top[i]])

        projected_faces = []
        face_scores = []

        for idx, face in enumerate(faces):
            image_points = []
            face_3d_camera = []
            z_positive = False

            for pt in face:
                pt_msg = PointStamped()
                pt_msg.header.frame_id = self.base_frame
                pt_msg.header.stamp = rospy.Time(0)
                pt_msg.point.x, pt_msg.point.y, pt_msg.point.z = pt

                try:
                    trans = self.tf_buffer.transform(pt_msg, self.camera_frame, rospy.Duration(0.5))
                    pt_cam = np.array([trans.point.x, trans.point.y, trans.point.z])
                    if pt_cam[2] > 0:
                        z_positive = True
                        uv = self.camera_model.project3dToPixel(pt_cam)
                        image_points.append(uv)
                        face_3d_camera.append(pt_cam)
                except Exception as e:
                    rospy.logwarn(f"Transform failed: {e}")

            if len(image_points) >= 3 and z_positive:
                projected_faces.append((idx, image_points, face_3d_camera))
                score = -np.mean([pt[2] for pt in face])  # Sort by proximity
                face_scores.append((idx, score))

        if not face_scores:
            return [], projected_faces

        best_idx = max(face_scores, key=lambda x: x[1])[0]
        selected = [(idx, pts2d, pts3d) for idx, pts2d, pts3d in projected_faces if idx == best_idx]
        return selected, projected_faces

    def publish_debug_bin(self):
        """
        Publishes a Marker with all 8 corners of the bin: 4 base, 4 top.
        :param bin_polygon: list of (x, y) tuples in map frame (base of bin)
        """
        if not self.bin_polygon or len(self.bin_polygon) < 4:
            rospy.logwarn("Not enough points in bin polygon to form a box.")
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame  # e.g., "map"
        marker.ns = "debug_bin"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)  # bright magenta

        from pprint import pformat
        # Add 4 base corners and 4 top corners
        for (x, y) in self.bin_polygon:
            marker.points.append(Point(x=x, y=y, z=self.floor_z))          # base
            marker.points.append(Point(x=x, y=y, z=self.floor_z + self.bin_height))   # top

        self.debug_bin_pub.publish(marker)

    def publish_debug_overlay(self, cv_image, projected_face, red_mask):
        overlay = cv_image.copy()

        # Draw the projected face in green
        face_pts = np.array(projected_face, dtype=np.int32)
        cv2.polylines(overlay, [face_pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Draw the red mask in red
        red_mask_bgr = np.zeros_like(cv_image)
        red_mask_bgr[red_mask > 0] = (0, 0, 255)  # Red overlay

        # Highlight overlap in yellow (where red_mask and projected face intersect)
        overlap_mask = np.zeros_like(red_mask, dtype=np.uint8)
        cv2.fillPoly(overlap_mask, [face_pts], 255)
        overlap_region = cv2.bitwise_and(overlap_mask, red_mask)

        yellow_overlay = np.zeros_like(cv_image)
        yellow_overlay[overlap_region > 0] = (0, 255, 255)

        # Combine all overlays
        combined = cv2.addWeighted(overlay, 1.0, red_mask_bgr, 0.5, 0)
        combined = cv2.addWeighted(combined, 1.0, yellow_overlay, 0.8, 0)

        # Publish to RViz
        debug_msg = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        self.debug_image_pub.publish(debug_msg)


    def publish_projected_face_marker(self, polygon_3d):
        if len(polygon_3d) < 3:
            rospy.logwarn("Not enough points to form a polygon")
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame
        marker.ns = "projected_face"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # no rotation

        marker.scale.x = 0.002  # line width

        # Bright yellow for visibility
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pt in polygon_3d:
            pt_cam = PointStamped()
            pt_cam.header.stamp = rospy.Time.now()
            pt_cam.header.frame_id = self.camera_frame
            pt_cam.point.x = pt[0]
            pt_cam.point.y = pt[1]
            pt_cam.point.z = pt[2]

            try:
                pt_map = self.tf_buffer.transform(
                    pt_cam, self.base_frame, timeout=rospy.Duration(0.2)
                )
                marker.points.append(pt_map.point)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"[BinVisibilityChecker] TF transform failed for projected face marker point: {e}")

        # Close the polygon if needed
        if marker.points and marker.points[0] != marker.points[-1]:
            marker.points.append(marker.points[0])

        self.projected_face_marker_pub.publish(marker)


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description="Bin Visibility Checker")

    parser.add_argument("-d", "--debug", action="store_true", help="Enable debug mode for additional RViz visualization.")

    args = parser.parse_args()

    try:
        # rospy.init_node('bin_visibility_checker', anonymous=True)
        rospy.init_node('bin_visibility_node')
        if args.debug:
            rospy.loginfo("[BinVisibilityChecker] Debug mode enabled. Additional visualizations will be published to RViz.")
        checker = BinVisibilityChecker(debug=args.debug)

        while not rospy.is_shutdown():
            visibility = checker.get_visibility()

            rospy.loginfo(f"[BinVisibilityChecker] Bin visibility: {(visibility * 100):.2f}%")
            rospy.sleep(1)



    except rospy.ROSInterruptException:
        pass
