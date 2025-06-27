#TODO: Move these into ROS parameters?

# Default length of a side of the local view (it's always a square),
# in cells (cell size is in local view resolution, which is usually
# different from the occupancy grid resolution)
DEFAULT_LOCAL_VIEW_SIZE = 8

# Default resolution of the local view, in meters per cell of the grid
DEFAULT_LOCAL_VIEW_RESOLUTION = 0.1

# Default velocity for turning primitive actions
DEFAULT_ANGULAR_VELOCITY = 0.2

# Defauly velocity for linear primitive actions
DEFAULT_LINEAR_VELOCITY = 0.5
