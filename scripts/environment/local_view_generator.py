import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.ndimage import affine_transform
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from occupancy_grid import OccupancyGridMerger
from defaults import DEFAULT_LOCAL_VIEW_SIZE


class LocalViewGenerator:
    def __init__(self, visualize: bool = False):
        self.tf_listener = tf.TransformListener()
        self.visualize = visualize

    def _crop_and_pad(self, grid_array: np.ndarray, cx: int, cy: int, view_size: int):
        height, width = grid_array.shape
        half = view_size // 2
        xmin = cx - half
        xmax = xmin + view_size
        ymin = cy - half
        ymax = ymin + view_size

        src_xmin = max(xmin, 0)
        src_xmax = min(xmax, width)
        src_ymin = max(ymin, 0)
        src_ymax = min(ymax, height)

        dst_xmin = src_xmin - xmin
        dst_ymin = src_ymin - ymin

        cropped = np.full((view_size, view_size), -1, dtype=np.int8)
        cropped_sub = grid_array[src_ymin:src_ymax, src_xmin:src_xmax]
        cropped[dst_ymin:dst_ymin + cropped_sub.shape[0],
                dst_xmin:dst_xmin + cropped_sub.shape[1]] = cropped_sub

        return cropped, (dst_xmin, dst_ymin)

    def get_local_view(self, occupancy_grid: np.ndarray, info: MapMetaData, view_size: int, local_view_resolution: float = 0.1) -> OccupancyGrid:
        resolution = info.resolution
        origin = info.origin
        self.tf_listener.waitForTransform('/map', 'locobot/base_link', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))

        height = occupancy_grid.shape[0]
        grid_cx, grid_cy = self._grid_to_world_indices(trans, origin=origin, height=height, resolution=resolution)
        flipped_grid = np.flipud(occupancy_grid)
        flipped_cy = height - 1 - grid_cy

        yaw_rad = euler_from_quaternion(rot)[2]
        angle_deg = np.degrees(yaw_rad) - 90.0

        rotated_full_grid = self.rotate_about_point(
            grid=flipped_grid,
            angle_deg=angle_deg,
            center=(flipped_cy, grid_cx),
            cval=-1
        )

        # Compute crop region dimensions in high-res grid
        scale = round(local_view_resolution / resolution)
        highres_view_size = view_size * scale

        # Crop so robot is at bottom center of view
        xmin = grid_cx - (highres_view_size // 2)
        xmax = xmin + highres_view_size
        ymax = flipped_cy + 1
        ymin = ymax - highres_view_size

        # Ensure within grid bounds
        xmin_clip = max(xmin, 0)
        xmax_clip = min(xmax, rotated_full_grid.shape[1])
        ymin_clip = max(ymin, 0)
        ymax_clip = min(ymax, rotated_full_grid.shape[0])

        # Create empty region and insert overlapping patch
        cropped = np.full((highres_view_size, highres_view_size), -1, dtype=np.int8)
        sub = rotated_full_grid[ymin_clip:ymax_clip, xmin_clip:xmax_clip]
        dst_xmin = xmin_clip - xmin
        dst_ymin = ymin_clip - ymin
        cropped[dst_ymin:dst_ymin + sub.shape[0], dst_xmin:dst_xmin + sub.shape[1]] = sub


        # === Downsample ===
        downsampled = self.max_pool_downsample(cropped, scale=scale)
        assert downsampled.shape == (view_size, view_size)

        # === Compute local origin (bottom center of view) ===
        dx = (view_size // 2) * local_view_resolution
        dy = 0.0
        cos_t, sin_t = np.cos(yaw_rad), np.sin(yaw_rad)
        origin_x = trans[0] - cos_t * dx + sin_t * dy
        origin_y = trans[1] - sin_t * dx - cos_t * dy

        local_origin = Pose()
        local_origin.position.x = origin_x
        local_origin.position.y = origin_y
        local_origin.position.z = 0.0
        local_origin.orientation.x = rot[0]
        local_origin.orientation.y = rot[1]
        local_origin.orientation.z = rot[2]
        local_origin.orientation.w = rot[3]

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = local_view_resolution
        msg.info.width = view_size
        msg.info.height = view_size
        msg.info.origin = local_origin
        msg.data = downsampled.flatten().tolist()

        if self.visualize:
            self.visualize_original_and_local_view(
                cropped,
                downsampled,
                # original_resolution=resolution,
                # downsampled_resolution=local_view_resolution,
                output_path="comparison.png"
            )

        return msg
    
    def max_pool_downsample(self, grid: np.ndarray, scale: int) -> np.ndarray:
        """
        Downsamples a 2D occupancy grid using max-pooling.

        Each coarse cell takes the maximum of the fine-resolution cells it covers.

        Parameters:
        - grid: 2D numpy array
        - scale: Integer factor by which resolution is reduced (e.g., 2 for 0.05 → 0.1m)

        Returns:
        - 2D numpy array downsampled by `scale`
        """
        h, w = grid.shape
        h_trim, w_trim = h - (h % scale), w - (w % scale)
        trimmed = grid[:h_trim, :w_trim]
        reshaped = trimmed.reshape(h_trim // scale, scale, w_trim // scale, scale)
        downsampled = reshaped.max(axis=(1, 3))
        return downsampled.astype(np.int8)



    def get_local_view_old(self, occupancy_grid: np.ndarray, info: MapMetaData, view_size: int) -> OccupancyGrid:
        resolution = info.resolution
        origin = info.origin
        self.tf_listener.waitForTransform('/map', 'locobot/base_link', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))
        height = occupancy_grid.shape[0]
        grid_cx, grid_cy = self._grid_to_world_indices(trans, origin=origin, height=height, resolution=resolution)

        flipped_grid = np.flipud(occupancy_grid)
        flipped_cy = height - 1 - grid_cy


        yaw_rad = euler_from_quaternion(rot)[2]
        angle_deg = np.degrees(yaw_rad) - 90.0

        rotated_full_grid = self.rotate_about_point(
            grid=flipped_grid,
            angle_deg=angle_deg,
            center=(flipped_cy, grid_cx),
            cval=-1
        )

        rotated_cropped, offset = self._crop_and_pad(rotated_full_grid, grid_cx, flipped_cy, view_size)

        local_origin = Pose()
        local_origin.position.x = trans[0] - (view_size // 2) * resolution
        local_origin.position.y = trans[1] - (view_size // 2) * resolution
        local_origin.position.z = 0.0
        local_origin.orientation.x = 0.0
        local_origin.orientation.y = 0.0
        local_origin.orientation.z = rot[2]
        local_origin.orientation.w = rot[3]

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = resolution
        msg.info.width = view_size
        msg.info.height = view_size
        msg.info.origin = local_origin
        msg.data = rotated_cropped.flatten().tolist()

        return msg

    def rotate_about_point(self, grid: np.ndarray, angle_deg: float, center: tuple, cval=-1):
        angle_rad = np.radians(angle_deg)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)

        rot_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a,  cos_a]
        ])

        center = np.asarray(center)
        offset = center - rot_matrix @ center

        rotated = affine_transform(
            input=grid,
            matrix=rot_matrix,
            offset=offset,
            output_shape=grid.shape,
            order=0,
            mode='constant',
            cval=cval
        )
        return rotated

    def _grid_to_world_indices(self, trans, origin: tuple, height: int, resolution: float):
        grid_cx = int((trans[0] - origin.position.x) / resolution)
        grid_cy = int((trans[1] - origin.position.y) / resolution)
        return grid_cx, grid_cy

    def visualize_original_and_local_view(self, original_grid: np.ndarray, downsampled_grid: np.ndarray,
                                          output_path: str = "comparison.png"):
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(1, 2, figsize=(14, 7))
        titles = ['Original Grid (High Res)', 'Downsampled Local View']
        grids = [original_grid, downsampled_grid]

        for ax, grid, title in zip(axs, grids, titles):
            height, width = grid.shape
            cax = ax.imshow(grid, cmap='gray_r', origin='upper', vmin=-1, vmax=100)
            ax.set_title(title)
            ax.set_xticks(np.arange(-0.5, width, 1), minor=True)
            ax.set_yticks(np.arange(-0.5, height, 1), minor=True)
            ax.grid(which='minor', color='black', linewidth=0.5)
            ax.tick_params(which='both', bottom=False, left=False, labelbottom=False, labelleft=False)

        fig.colorbar(cax, ax=axs.ravel().tolist(), shrink=0.7, label='Occupancy')
        plt.tight_layout()
        plt.savefig(output_path)
        plt.close()
        print(f"Saved side-by-side visualization to {output_path}")

class LocalViewGeneratorOld:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.resolution = rospy.get_param('/locobot/move_base/global_costmap/resolution', 0.05)

    def _get_robot_pose(self):
        self.tf_listener.waitForTransform('/map', 'locobot/base_link', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = self.tf_listener.lookupTransform('/map', 'locobot/base_link', rospy.Time(0))
        return trans, rot

    def _prepare_grid(self, grid_array, map_info):
        height, width = map_info.height, map_info.width
        grid_array = np.array(grid_array, dtype=np.int8).reshape((height, width))
        grid_array = np.flipud(grid_array)  # Flip vertically for top-down view
        return grid_array

    def _grid_to_world_indices(self, trans, map_info):
        origin = map_info.origin
        height = map_info.height
        grid_cx = int((trans[0] - origin.position.x) / self.resolution)
        grid_cy = height - 1 - int((trans[1] - origin.position.y) / self.resolution)
        return grid_cx, grid_cy

    def _crop_and_pad(self, grid_array, cx, cy, view_size):
        height, width = grid_array.shape
        half = view_size // 2
        xmin = cx - half
        xmax = xmin + view_size
        ymin = cy - half
        ymax = ymin + view_size

        src_xmin = max(xmin, 0)
        src_xmax = min(xmax, width)
        src_ymin = max(ymin, 0)
        src_ymax = min(ymax, height)

        dst_xmin = src_xmin - xmin
        dst_ymin = src_ymin - ymin

        cropped = np.full((view_size, view_size), -1, dtype=np.int8)
        cropped_sub = grid_array[src_ymin:src_ymax, src_xmin:src_xmax]
        cropped[dst_ymin:dst_ymin + cropped_sub.shape[0],
                dst_xmin:dst_xmin + cropped_sub.shape[1]] = cropped_sub
        
        return cropped, (dst_xmin, dst_ymin)

    def _compute_origin(self, robot_x, robot_y, local_pos, theta_rad):
        dx = (local_pos[0] + 0.5) * self.resolution
        dy = (local_pos[1] + 0.5) * self.resolution
        cos_t, sin_t = np.cos(theta_rad), np.sin(theta_rad)
        dx_rot = cos_t * dx - sin_t * dy
        dy_rot = sin_t * dx + cos_t * dy
        return robot_x - dx_rot, robot_y - dy_rot

    def _build_msg(self, grid, view_size, origin_x, origin_y, theta_rad):
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta_rad)
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = view_size
        msg.info.height = view_size
        msg.info.origin = Pose(Point(origin_x, origin_y, 0.0), Quaternion(qx, qy, qz, qw))
        msg.data = grid.flatten().tolist()
        return msg

    def get_local_observation(self, grid_array, map_info, view_size):
        bin_names = [
            "East - Northeast",
            "Northeast - North",
            "North - Northwest",
            "Northwest - West",
            "West - Southwest",
            "Southwest - South",
            "South - Southeast",
            "Southeast - East"
        ]
        bin_names_help = "\n\t".join([f"{i}: {bin_names[i]}" for i in range(len(bin_names))])
        print(f"""
Orientation bins:
    {bin_names_help}
              """)
        trans, rot = self._get_robot_pose()
        grid_array = self._prepare_grid(grid_array, map_info)
        import sys
        np.savetxt("./prepared_grid.out.txt", grid_array, fmt='%3d')
        grid_cx, grid_cy = self._grid_to_world_indices(trans, map_info)
        cropped, offset = self._crop_and_pad(grid_array, grid_cx, grid_cy, view_size)
        print("Cropped grid:\n")
        np.savetxt(sys.stdout, cropped, fmt='%3d')
        print("==========================\n")
        robot_local = (view_size // 2, view_size // 2)

        _, _, yaw = euler_from_quaternion(rot)
        yaw_adj = yaw
        yaw_deg = np.degrees(yaw_adj)
        orientation_bin = int((yaw_deg + 22.5) // 45) % 8
        num_90_rot = ((orientation_bin - 2) % 8) // 2
        rotated = np.rot90(cropped, k=num_90_rot, axes=(1, 0))
        for _ in range(num_90_rot):
            robot_local = (view_size - 1 - robot_local[1], robot_local[0])
        theta_rad = np.deg2rad(90 * num_90_rot)

        print("Rotated:\n")
        np.savetxt(sys.stdout, rotated, fmt='%3d')
        print("==========================\n")

        origin_x, origin_y = self._compute_origin(trans[0], trans[1], robot_local, theta_rad)
        msg = self._build_msg(rotated, view_size, origin_x, origin_y, theta_rad)

        return msg, yaw_deg, robot_local

if __name__ == "__main__":
    rospy.init_node('test_local_view_generator', anonymous=True)

    from argparse import ArgumentParser

    parser = ArgumentParser()

    # parser.add_argument("-s", "--size", default=DEFAULT_LOCAL_VIEW_SIZE, type=int)
    parser.add_argument("-s", "--size", default=14, type=int)

    args = parser.parse_args()

    merger = OccupancyGridMerger()
    generator = LocalViewGenerator(visualize=True)

    while not rospy.is_shutdown():
        grid, info = merger.get_occupancy_grid()
        if grid is not None:
            break
    
    if grid is None:
        rospy.logwarn("[LocalViewGenerator] rospy shut down before it could deliver the merged occupancy grid")
        exit

    width, height = info.width, info.height
    shaped_grid = np.array(grid).reshape((height, width))
    np.savetxt("occupancy_grid.txt", shaped_grid, fmt='%3d')
    obs = generator.get_local_view(shaped_grid, info, view_size=args.size, local_view_resolution=0.1)
    # obs, _,  _ = generator.get_local_observation(
    #     grid_array=grid,
    #     map_info=info,
    #     view_size=args.size,
    # )

    height, width = obs.info.height, obs.info.width
    local_grid = np.array(obs.data).reshape((height, width))

    import sys

    np.savetxt(sys.stdout, local_grid, fmt='%3d')

