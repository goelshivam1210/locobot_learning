# Environment (Architecture)
## Overview

The `environment` directory is a critical component of the overall agent system, responsible for managing the interaction between the agent and its surroundings. This directory includes modules for generating the [action space](./action/action_space.py), observing the environment's [state](./state), computing [rewards](./reward/reward_function.py), and interfacing with [ROS services](./ROS_services) for executing actions. The environment effectively bridges the gap between high-level planning and low-level execution, enabling the agent to operate effectively in both simulated and real-world scenarios.

## Directory Structure


```
.
├── environment
│   ├── Environment.py
│   ├── ROS_services
│   │   ├── __init__.py
│   │   ├── at.py
│   │   ├── contain.py
│   │   ├── facing.py
│   │   └── hold_real.py
│   ├── RecycleBotSMDP.py
│   ├── __init__.py
│   ├── action
│   │   ├── __init__.py
│   │   └── action_space.py
│   ├── reward
│   │   ├── __init__.py
│   │   └── reward_function.py
│   ├── state
│   │   ├── SubSymbolicState.py
│   │   ├── SymbolicState.py
│   │   ├── __init__.py
│   │   └── observation_space.py
│   ├── tests
│   │   └── __init__.py
├── agent
│   ├── README.md
│   ├── core
│   │   ├── Agent.py
│   │   ├── HybridAgent.py
│   │   ├── PDDLActions.py
│   │   ├── PDDLPredicates.py
│   │   ├── __init__.py
│   │   ├── learner
│   │   ├── planner
│   ├── tests
├── executor
│   ├── __init__.py
├── knowledge
│   ├── PDDL
│   ├── pddl-parser
├── perception
```

## Core components

### 1. **[Environment.py](./Environment.py)**
The `Environment` class serves as the base class for managing the interaction between different modules in the environment, such as the action space, state observation, and reward generation. It interfaces with the agent and executor components, providing a unified API for interacting with the environment.

**Key Responsibilities:**
- **Module Integration:** Combines the functionality of action space generation, state observation, and reward computation.
- **Agent Interface:** Provides methods for the agent to interact with the environment.
- **Executor Communication:** Manages communication with the executor to perform actions.

### 2. **[RecycleBotSMDP.py](./RecycleBotSMDP.py)**
The `RecycleBotSMDP` class extends the `Environment` class, implementing a Semi-Markov Decision Process (SMDP) environment specifically for the RecycleBot. It handles the high-level operations such as step execution, state transitions, and reward assignment.

**Key Features:**
- **SMDP Handling:** Manages extended time-step actions that involve sequences of primitive actions.
- **State Transitions:** Handles both symbolic and sub-symbolic state transitions.
- **Action Execution:** Executes actions and manages their outcomes, integrating with ROS services when necessary.

### 3. **[state/](./state/)**
This directory contains the classes and methods responsible for state management, including both symbolic and sub-symbolic states.

- **[SymbolicState.py](./state/SymbolicState.py):** Manages symbolic state representations, which include high-level, discrete representations of the environment.
- **[SubSymbolicState.py](./state/SubSymbolicState.py):** Handles sub-symbolic state representations, such as continuous sensor data and low-level environmental features.
- **[observation_generator.py](./state/observation_space.py):** Generates observations from both symbolic and sub-symbolic states to provide a complete view of the environment.

**Key Responsibilities:**
- **State Representation:** Provides both high-level and low-level representations of the environment.
- **Observation Generation:** Combines symbolic and sub-symbolic data to generate comprehensive observations for the agent.

### 4. **[action/action_space_generator.py](./action/action_space.py)**
This module is responsible for generating the action space based on the PDDL domain and problem files. It parses the PDDL files to generate both grounded and non-grounded actions that the agent can execute.

**Core Functions:**
- **Action Generation:** Creates a list of all possible actions that can be taken in the environment.
- **Grounding Actions:** Generates grounded actions by combining action templates with the objects available in the environment.
- **Integration with PDDL:** Parses PDDL files to ensure that the action space is consistent with the domain and problem definitions.

### 5. **[reward/reward_function.py](./reward/reward_function.py)**
The `RewardFunction` class handles the computation of rewards based on the outcomes of actions. It evaluates the current state and assigns rewards to guide the agent towards achieving the goal.

**Key Features:**
- **Reward Calculation:** Computes rewards based on the success or failure of actions.
- **Goal Evaluation:** Assesses whether the agent's actions are moving towards the defined goal.
- **Integration with SMDP:** Works within the SMDP framework to provide rewards that reflect extended time-step actions.

### 6. **[ROS_services/](./ROS_services/)**
This directory contains the ROS service scripts that handle interactions with the robot hardware or simulation. These services manage low-level tasks such as checking if an object is held or if the robot is facing a particular direction.

- **[at.py](./ROS_services/at.py):** Service to check if an object is at a specific location.
- **[contain.py](./ROS_services/contain.py):** Service to check if an object is contained within another object (e.g., if a ball is in a bin).
- **[facing.py](./ROS_services/facing.py):** Service to determine if the robot is facing a specific object.
- **[hold_real.py](./ROS_services/hold_real.py):** Service to check if the robot is currently holding an object.

**Core Functions:**
- **Low-Level Interactions:** Provides the necessary services for low-level interactions between the agent and the environment.
- **Integration with Actions:** Ensures that high-level actions have the necessary low-level support to be executed effectively.