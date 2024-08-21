
# Locobot Learning Project

This repository provides instructions for operating Locobot with ROS for various tasks such as navigation, localization, and manipulation. The following steps guide you through connecting to Locobot, running essential ROS launch files, and troubleshooting common issues.

## Prerequisites

- Ensure you have access to Shivam's account with the password: `turtlebot112`.
- Install ROS and relevant Locobot packages on your machine or the robot.

## Running Locobot

### Connecting to Locobot

1. **SSH into Locobot**:  
   Open a terminal and run:
   ```bash
   ssh -X locobot@10.0.60.2
   ```
   Password: `locobot`

2. **Launching Locobot**:  
   Navigate to the Locobot workspace:
   ```bash
   cd ~/interbotix_ws/src/locobot/launch
   ```
   Then, launch the robot's launch file:
   ```bash
   roslaunch nav_moveit_perception.launch use_rviz:=false localization:=true
   ```
   **Note**: You must be in the `~/interbotix_ws/src/locobot/launch` folder to launch this successfully.

3. **Localization**:  
   Locobot should localize on the map correctly. If not, use the *pose estimate* feature in RViz.


4. **Launching Remote Viewer (on your computer)**:  
   On your local computer, run:
   ```bash
   roslaunch interbotix_xslocobot_descriptions remote_view.launch
   ```
   (This can be run on Locobot as well, but it will be slower due to GUI performance.)

## Resetting Locobot

1. Turn off the middle button.
2. Turn off the side button.
3. Place Locobot on the charger.

## Approach Service

1. **Load navigation goals**:
   ```bash
   rosparam load real_nav_goals.yaml
   ```
For this you will need to go to the config directory in `locobot_learning/config`

2. **Run the service**
Go to the directory where the script (executor)
```bash 
cd locobot_learning/scripts/executor
```
Launch the approach server
```bash
python approach_server_real.py
```

3. **List available services**:
   ```bash
   rosservice list | grep app
   ```

4. **Call the approach service**:
   ```bash
   rosservice call /approach "target: '<Target_Name>'"
   ```
   Check for the arguments in the Yaml file in the config directory.

## Arm Service

1. **Run the arm server**
Go to the directory where the script (executor)
```bash 
cd locobot_learning/scripts/executor
```
Launch the approach server
```bash
python arm_server.py
```
2. **List available services**:
   ```bash
   rosservice list | grep grasp/drop/drop_aside
   ```

3. **Call the approach service**:
   ```bash
   rosservice call /grasp_object 
   ```
   Please use tabs to autocomplete!
   Check for the arguments in the Yaml file in the config directory.

## General Notes

- **VS Code Remote Explorer**:  
  Use the VS Code remote explorer feature to connect to Locobot (same IP as SSH).


- **Useful ROS Commands**:
  - List all topics:
    ```bash
    rostopic list
    ```
  - Get info on a specific topic:
    ```bash
    rostopic info <topic_name>
    ```
  - Get point cloud data:
    ```bash
    rostopic echo /locobot/pc_filter/pointcloud/objects -n 1
    ```

- **Navigation and Mapping**:
  - Launch navigation:
    ```bash
    roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx200 use_lidar:=true rtabmap_args:=-d
    ```
    **Note**: This deletes the current map for remapping the environment.
  - Clear costmaps:
    ```bash
    rosservice call /locobot/move_base/clear_costmaps "{}"
    ```

- **RViz Remote Viewer**:
  ```bash
  roslaunch interbotix_xslocobot_descriptions remote_view.launch
  ```

- **Teleoperation**:
  Run the following to teleoperate Locobot:
  ```bash
  rosrun locobot_custom locobot_teleop.py
  ```
  (Make sure you are in the `~/interbotix_ws/src/locobot_custom/scripts` folder.)

- **Get Locobot’s coordinates**:
  ```bash
  rosrun tf tf_echo /map locobot/base_link
  ```

## Troubleshooting

- **Base Reset**:  
  When starting the robot each time, you may need to restart the base and do a chronyc restart.

## Git Notes

- SSH has been configured on Locobot, allowing you to push and pull directly from the SpheroGames repository.

## Resources

- [Converting ROS Images to OpenCV in Python](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
- [Interbotix XS Arms Perception Pipeline Configuration](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/perception_pipeline_configuration.html)




```

.
├── CMakeLists.txt
├── config
│   ├── at_boundaries.yaml
│   ├── facing_boundaries.yaml
│   ├── facing_thresh.yaml
│   ├── hold_thresh.yaml
│   ├── map.txt
│   ├── real_nav_goals.yaml
│   └── sim_nav_goals.yaml
├── maps
├── package.xml
├── README.md
├── readme_real.md
├── rostopics.txt
├── scripts
│   ├── agent
│   │   ├── Agent.py
│   │   ├── dummy_planner.py
│   │   ├── __init__.py
│   │   ├── learner
│   │   │   ├── __init__.py
│   │   │   ├── learner.py
│   │   │   ├── random_learner.py
│   │   │   └── test_discrete.py
│   │   ├── PDDLActions.py
│   │   ├── PDDLPredicates.py
│   │   ├── planner
│   │   │   ├── __init__.py
│   │   │   ├── Planner.py
│   │   │   └── __pycache__
│   │   │       ├── __init__.cpython-38.pyc
│   │   │       └── Planner.cpython-38.pyc
│   │   ├── ppo_learner.py
│   │   ├── __pycache__
│   │   │   ├── Agent.cpython-38.pyc
│   │   │   ├── PDDLActions.cpython-38.pyc
│   │   │   ├── PDDLPredicates.cpython-38.pyc
│   │   │   ├── ppo_learner.cpython-38.pyc
│   │   │   ├── RecycleBotAgent.cpython-38.pyc
│   │   │   └── RecycleBotPlanner.cpython-38.pyc
│   │   ├── RecycleBotAgent.py
│   │   ├── RecycleBotMain.py
│   │   └── RecycleBotPlanner.py
│   ├── environment
│   │   ├── ActionGenerator.py
│   │   ├── __init__.py
│   │   ├── NovelGym.py
│   │   ├── ObservationGenerator.py
│   │   ├── observation_server_gazebo.py
│   │   ├── ObservationUtils.py
│   │   ├── occupancy_grid.py
│   │   ├── predicate_services
│   │   │   ├── at.py
│   │   │   ├── contain.py
│   │   │   ├── facing.py
│   │   │   ├── hold_real.py
│   │   │   └── __init__.py
│   │   ├── __pycache__
│   │   │   ├── ActionGenerator.cpython-38.pyc
│   │   │   ├── NovelGym.cpython-38.pyc
│   │   │   ├── ObservationGenerator.cpython-38.pyc
│   │   │   ├── RecycleBot.cpython-38.pyc
│   │   │   ├── RewardFunction.cpython-38.pyc
│   │   │   └── RewardFunctionGenerator.cpython-38.pyc
│   │   ├── RecycleBot.py
│   │   ├── relative_proximity.py
│   │   ├── RewardFunction.py
│   │   ├── test.py
│   │   └── tests
│   │       ├── __init__.py
│   │       └── test_observation_gazebo_client.py
│   ├── executor
│   │   ├── approach_server_real.py
│   │   ├── arm_server.py
│   │   ├── cmd_vel_publisher.py
│   │   ├── __init__.py
│   │   ├── test.py
│   │   ├── tests
│   │   │   ├── __init__.py
│   │   │   ├── test_approach.py
│   │   │   ├── test_grasp.py
│   │   │   ├── test_place.py
│   │   │   └── test_primitive.py
│   │   └── utils
│   │       └── __init__.py
│   ├── knowledge
│   │   ├── PDDL
│   │   │   ├── __init__.py
│   │   │   └── recycle_bot
│   │   │       ├── domain.pddl
│   │   │       ├── __init__.py
│   │   │       ├── problem.pddl
│   │   │       └── problem_plan.pddl
│   │   └── pddl-parser
│   │       ├── build
│   │       │   ├── lib
│   │       │   │   └── pddl_parser
│   │       │   │       ├── action.py
│   │       │   │       ├── __init__.py
│   │       │   │       ├── PDDL.py
│   │       │   │       └── planner.py
│   │       │   └── lib.linux-x86_64-2.7
│   │       │       └── pddl_parser
│   │       │           ├── action.py
│   │       │           ├── __init__.py
│   │       │           ├── PDDL.py
│   │       │           └── planner.py
│   │       ├── examples
│   │       │   ├── blocksworld
│   │       │   │   ├── blocksworld.pddl
│   │       │   │   ├── pb1.pddl
│   │       │   │   ├── pb2.pddl
│   │       │   │   ├── pb3.pddl
│   │       │   │   ├── pb4.pddl
│   │       │   │   ├── pb5.pddl
│   │       │   │   └── pb6.pddl
│   │       │   ├── dinner
│   │       │   │   ├── dinner.pddl
│   │       │   │   └── pb1.pddl
│   │       │   ├── dwr
│   │       │   │   ├── dwr.pddl
│   │       │   │   ├── pb1.pddl
│   │       │   │   └── pb2.pddl
│   │       │   └── tsp
│   │       │       ├── pb1.pddl
│   │       │       └── tsp.pddl
│   │       ├── LICENSE
│   │       ├── pddl_parser
│   │       │   ├── action.py
│   │       │   ├── __init__.py
│   │       │   ├── PDDL.py
│   │       │   ├── planner.py
│   │       │   └── __pycache__
│   │       │       ├── action.cpython-38.pyc
│   │       │       ├── __init__.cpython-38.pyc
│   │       │       ├── PDDL.cpython-38.pyc
│   │       │       └── planner.cpython-38.pyc
│   │       ├── README.md
│   │       ├── setup.py
│   │       ├── test_PDDL.py
│   │       └── test_planner.py
│   └── perception
│       ├── ARTracker.py
│       ├── find_object_coordinates.py
│       ├── ObjectOrientation.py
│       └── transform_markers.py
├── srv
│   ├── AlignRobot.srv
│   ├── Approach.srv
│   ├── At.srv
│   ├── Contain.srv
│   ├── DropObjectAside.srv
│   ├── DropObject.srv
│   ├── Facing.srv
│   ├── GazeboObservation.srv
│   ├── GraspObject.srv
│   ├── GraspPose.srv
│   ├── Grasp.srv
│   ├── Hold.srv
│   ├── Place.srv
│   └── PrimitiveBase.srv
└── tree.txt

```