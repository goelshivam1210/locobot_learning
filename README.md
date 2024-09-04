
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


### Notes on virtual environment (ROS-python interfacing)

Make your vitual environment 

```bash
python -env venv <.venv>
```
`.venv` is the name and the location and the name of the virtual environment. 

Activate the environment

```bash 
source .venv/bin/activate
```
You can deactivate the environment using:

```bash
deactivate
```

Installing requirements inside the virtual environment

```bash
pip install -r requirements.txt
```

Sourcing ROS

```bash
source ~/devel/setup.bash
```

## Directory structure

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
├── rostopics.txt
├── scripts
│   ├── agent
│   │   ├── core
│   │   │   ├── Agent.py
│   │   │   ├── HybridAgent.py
│   │   │   ├── __init__.py
│   │   │   ├── learner
│   │   │   │   ├── BaseLearner.py
│   │   │   │   ├── __init__.py
│   │   │   │   ├── LearningAgent.py
│   │   │   │   └── PPOLearner.py
│   │   │   ├── PDDLActions.py
│   │   │   ├── PDDLPredicates.py
│   │   │   └── planner
│   │   │       ├── __init__.py
│   │   │       ├── planner.py
│   │   ├── README.md
│   │   ├── tests
│   ├── environment
│   │   ├── action
│   │   │   ├── action_space.py
│   │   │   └── __init__.py
│   │   ├── Environment.py
│   │   ├── __init__.py
│   │   ├── README.md
│   │   ├── RecycleBotSMDP.py
│   │   ├── reward
│   │   │   ├── __init__.py
│   │   │   └── reward_function.py
│   │   ├── ROS_services
│   │   │   ├── at.py
│   │   │   ├── contain.py
│   │   │   ├── facing.py
│   │   │   ├── hold_real.py
│   │   │   └── __init__.py
│   │   ├── state
│   │   │   ├── __init__.py
│   │   │   ├── observation_space.py
│   │   │   ├── SubSymbolicState.py
│   │   │   └── SymbolicState.py
│   │   ├── tests
│   ├── executor
│   │   ├── approach_server_real.py
│   │   ├── arm_server.py
│   │   ├── cmd_vel_publisher.py
│   │   ├── __init__.py
│   │   ├── tests
│   │   └── utils
│   ├── knowledge
│   │   ├── PDDL
│   │   │   ├── __init__.py
│   │   │   └── recycle_bot
│   │   │       ├── domain.pddl
│   │   │       ├── __init__.py
│   │   │       ├── problem.pddl
│   │   │       └── problem_plan.pddl
│   │   └── pddl-parser
│   ├── perception
│   │   ├── find_object_coordinates.py
│   │   ├── ObjectOrientation.py
│   │   └── transform_markers.py
├── srv
│   ├── Approach.srv
│   ├── At.srv
│   ├── Contain.srv
│   ├── DropObjectAside.srv
│   ├── DropObject.srv
│   ├── Facing.srv
│   ├── GraspObject.srv
│   ├── Grasp.srv
│   ├── Hold.srv
│   ├── Place.srv
│   └── PrimitiveBase.srv

```