import rospy
import numpy as np

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'state')))
from defaults import DEFAULT_LINEAR_VELOCITY, DEFAULT_LOCAL_VIEW_RESOLUTION


LETHAL_THRESHOLD = 1.0

def get_coordinates(local_grid: np.ndarray, velocity: float, resolution: float):
    N = local_grid.shape[0]
    assert local_grid.shape[1] == N, "Grid must be square"

    cx = N // 2
    cy = N - 1  # bottom center

    max_dist = velocity * 1.0
    max_steps = int(np.ceil(max_dist / resolution))
    end_y = cy - max_steps

    return N, cx, cy, end_y


def get_collision_cells(
    local_grid: np.ndarray,
    robot_radius: float,
    resolution: float,
    velocity: float,
    return_after_one: bool,
):
    N, cx, _, end_y = get_coordinates(local_grid, velocity, resolution)

    # Conservative safety margin: half a cell
    # The problem here is that cells are treated as points for collision detection--if the
    # center of a cell is not within the robot's footprint, it will not be detected as a
    # collision, even if its value is the lethal obstacle threshold. Adding a small buffer
    # around the footprint lets us catch such cases without needing a more complex geometry
    # overlap check. It might inaccurately mark some cells as collisions that aren't, but that's
    # preferable to missing a collision.
    safety_margin = 0.5 * resolution
    effective_radius = robot_radius + safety_margin
    rad_cells = int(np.ceil(effective_radius / resolution))

    collision_cells = []
    for dx in range(-rad_cells, rad_cells + 1):
        for dy in range(-rad_cells, rad_cells + 1):
            x = cx + dx
            y = end_y + dy

            # Conservative circular mask check
            if dx**2 + dy**2 > (effective_radius / resolution)**2:
                continue

            if 0 <= x < N and 0 <= y < N:
                if local_grid[y, x] >= LETHAL_THRESHOLD:
                    collision_cells.append((x, y))
                    if return_after_one:
                        return collision_cells
    return collision_cells

def would_collide(
    local_grid: np.ndarray,
) -> bool:
    """
    Returns True if moving forward (upward in the grid) for 1 second at the given velocity 
    would result in a collision. Assumes robot is located at the bottom center of the grid.
    """
    velocity = DEFAULT_LINEAR_VELOCITY
    resolution = DEFAULT_LOCAL_VIEW_RESOLUTION

    robot_radius = rospy.get_param('/locobot/move_base/global_costmap/robot_radius', None)

    if robot_radius is None:
        raise ValueError("Robot radius not found in parameter server.")
    
    collision_cells = get_collision_cells(
        local_grid=local_grid,
        robot_radius=robot_radius,
        resolution=resolution,
        velocity=velocity,
        return_after_one=True
    )

    return len(collision_cells) > 0

def visualize_forward_collision(
    local_grid: np.ndarray,
    save_path: str = None
):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    velocity = DEFAULT_LINEAR_VELOCITY
    resolution = DEFAULT_LOCAL_VIEW_RESOLUTION
    N, cx, cy, end_y = get_coordinates(local_grid, velocity, resolution)

    print(f"N={N}, cx={cx}, cy={cy}, end_y={end_y}")

    def flip_y(y): return N - 1 - y

    robot_radius = rospy.get_param('/locobot/move_base/global_costmap/robot_radius', None)

    if robot_radius is None:
        raise ValueError("Robot radius not found in parameter server.")
    
    collision_cells = get_collision_cells(
        local_grid=local_grid,
        robot_radius=robot_radius,
        resolution=resolution,
        velocity=velocity,
        return_after_one=False
    )

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(local_grid, cmap="gray_r", origin="upper", extent=[0, N, 0, N])

    ax.set_xlim(-0.5, N)
    ax.set_ylim(-0.5, N)
    ax.set_xticks(np.arange(0, N + 1, 1))
    ax.set_yticks(np.arange(0, N + 1, 1))
    ax.grid(which='both', color='lightgray', linestyle='-', linewidth=0.5)
    ax.set_xlim(0, N)
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    radius = robot_radius / resolution

    start_circle = Circle((cx + 0.5, flip_y(cy - 0.5)), radius,
                      edgecolor="blue", facecolor="none", linewidth=2, label="Start")

    # End: still use flipped y-coordinate
    end_circle = Circle((cx + 0.5, flip_y(end_y - 0.5)), radius,
                        edgecolor="green", facecolor="none", linewidth=2, label="End")

    ax.add_patch(start_circle)
    ax.add_patch(end_circle)

    added_label = False
    for x, y in collision_cells:
        label = "Collision" if not added_label else ""
        ax.plot(x + 0.5, flip_y(y) + 0.5, marker="x", color="red", markersize=8, label=label)
        added_label = True

    ax.set_title("Forward Motion Collision Check")
    ax.legend(loc="upper right")

    if save_path:
        plt.savefig(save_path, bbox_inches="tight")
        plt.close()
    else:
        plt.show()


def would_collide_old(
    local_grid: np.ndarray,
) -> bool:
    """
    Returns True if moving forward for 1s at the given velocity results in collision.
    """
    resolution = DEFAULT_LOCAL_VIEW_RESOLUTION
    # resolution = rospy.get_param('/locobot/move_base/global_costmap/resolution', None)

    # if resolution is None:
    #     raise ValueError("Costmap resolution not found in parameter server.")
    
    robot_radius = rospy.get_param('/locobot/move_base/global_costmap/robot_radius', None)

    if robot_radius is None:
        raise ValueError("Robot radius not found in parameter server.")
    
    N = local_grid.shape[0]
    assert local_grid.shape[0] == local_grid.shape[1], "Grid must be square"
    
    cx, cy = N // 2, N // 2  # robot center
    max_dist = DEFAULT_LINEAR_VELOCITY  # distance covered in 1s
    max_steps = int(np.ceil(max_dist / resolution))

    # Iterate over each step in the forward direction (first row is "ahead")
    for step in range(1, max_steps + 1):
        y = cy - step  # moving "up" in grid
        if y < 0:
            break

        # Determine radius in grid cells
        rad_cells = int(np.ceil(robot_radius / resolution))

        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if dx**2 + dy**2 > (robot_radius / resolution)**2:
                    continue  # outside of circular footprint

                nx = cx + dx
                ny = y + dy

                if 0 <= nx < N and 0 <= ny < N:
                    if local_grid[ny, nx] == 1.0:
                        rospy.loginfo(f"Collision detected at ({nx}, {ny}) with step {step} and dx, dy ({dx}, {dy})")
                        np.savetxt(sys.stdout, local_grid, fmt="%3d")
                        return True  # collision
    return False  # no collision


def visualize_forward_collision_old(
    local_grid: np.ndarray,
    velocity: float,
    robot_radius: float,
    resolution: float,
    lethal_threshold: float = 1.0,
    save_path: str = None
):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    N = local_grid.shape[0]
    assert N == local_grid.shape[1], "Grid must be square"
    cx, cy = N // 2, N // 2

    # Forward motion: decreasing row index
    max_dist = velocity * 1.0
    max_steps = int(np.ceil(max_dist / resolution))
    end_y = cy - max_steps

    # Define helper to flip y-coordinates for plotting
    def flip_y(y):
        return N - 1 - y

    # Collect collision cells
    rad_cells = int(np.ceil(robot_radius / resolution))
    collision_cells = []

    for dx in range(-rad_cells, rad_cells + 1):
        for dy in range(-rad_cells, rad_cells + 1):
            if dx**2 + dy**2 > (robot_radius / resolution)**2:
                continue

            end_x = cx + dx
            end_y_cell = end_y + dy

            if 0 <= end_x < N and 0 <= end_y_cell < N:
                if local_grid[end_y_cell, end_x] >= lethal_threshold:
                    collision_cells.append((end_x, end_y_cell))

    # Plot setup
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(local_grid, cmap="gray_r", origin="upper", extent=[0, N, 0, N])

    ax.set_xticks(np.arange(0, N + 1, 1))
    ax.set_yticks(np.arange(0, N + 1, 1))
    ax.grid(which='both', color='lightgray', linestyle='-', linewidth=0.5)

    ax.set_xlim(0, N)
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Draw robot positions (flip Y for correct visual placement)
    start_circle = Circle((cx + 0.5, flip_y(cy) + 0.5), robot_radius / resolution,
                          edgecolor="blue", facecolor="none", linewidth=2, label="Start")
    end_circle = Circle((cx + 0.5, flip_y(end_y) + 0.5), robot_radius / resolution,
                        edgecolor="green", facecolor="none", linewidth=2, label="End")
    ax.add_patch(start_circle)
    ax.add_patch(end_circle)

    # Draw collisions
    added_collision_label = False
    for (x, y) in collision_cells:
        label = "Collision" if not added_collision_label else ""
        ax.plot(x + 0.5, flip_y(y) + 0.5, marker="x", color="red", markersize=8, label=label)
        added_collision_label = True

    ax.set_title("Forward Motion Collision Check")
    ax.legend(loc="upper right")

    if save_path:
        plt.savefig(save_path, bbox_inches="tight")
        plt.close()
    else:
        plt.show()



if __name__ == "__main__":
    rospy.init_node('collision_checker', anonymous=True)

    from observation_space import ObservationSpace

    obs_space = ObservationSpace()

    grid, shape = obs_space.subsymbolic_state.get_local_grid()
    if grid is None:
        rospy.logerr("No local grid available, cannot check for collision.")
        sys.exit(1)
    grid = grid.reshape(shape)

    def custom_fmt(x):
        return "0.5" if x == 0.5 else str(int(x))
    rospy.loginfo("Local view grid:\n")
    for row in grid:
        formatted_row = ' '.join(f"{custom_fmt(val):^4}" for val in row)
        print(formatted_row)


    collision = would_collide(grid)
    rospy.loginfo(f"Collision check result: {collision}")

    visualize_forward_collision(
        local_grid=grid,
        save_path="./collision.png",
    )