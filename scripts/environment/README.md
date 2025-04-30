# Environment (Architecture)
## Overview

The `environment` directory is a component that is responsible for managing the interaction between the agent and its surroundings. This directory includes modules for generating the [action space](./action/action_space.py), observing the environment's [state](./state), computing [rewards](./reward/reward_function.py), and interfacing with [ROS services](./ROS_services) for executing actions. The environment effectively bridges the gap between high-level planning and low-level execution, enabling the agent to perform learning and reasoning.

## Directory Structure


```
.
├── environment
│   ├── RecycleBotSMDP.py
│   ├── ROS_services
│   │   ├── __init__.py
│   │   ├── at.py
│   │   ├── contain.py
│   │   ├── facing.py
│   │   └── LocalGridService.py
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


### 1. **[RecycleBotSMDP.py](./RecycleBotSMDP.py)**
This is the main environment class that implements a Semi-Markov Decision Process (SMDP) interface.
It is responsible for managing the interaction between the agent and the environment, including state transitions, action execution, and reward computation.

**Key Features:**

- **SMDP Framework:** Supports temporally extended actions like symbolic `approach`, `pick`, `place`, and `pass_through_door`, which abstract over multiple primitive steps.
- **Symbolic and Sub-Symbolic Coordination:** Integrates with both symbolic planning modules and reinforcement learning modules.
- **Action Execution:** Delegates action execution to the `ActionSpace`, which in turn interacts with primitive ROS services or symbolic action handlers.
- **Precondition Checking:** Symbolic actions are only executed if their symbolic preconditions are satisfied at runtime using the `PDDLPredicates` module.


### 3. **[state/](./state/)**
This directory includes components to compute symbolic and sub-symbolic state representations and combine them into observations.

- **[SymbolicState.py](./state/SymbolicState.py):** Queries symbolic predicates (e.g., `at`, `hold`, `facing`) via ROS services.
  - Encodes these into a one-hot symbolic state vector.
- **[SubSymbolicState.py](./state/SubSymbolicState.py):**  Computes spatial features such as the local occupancy grid and relative object poses using services like `LocalGridService`.
  - Generates continuous numerical encodings of the robot's perceptual field.
- **[observation_space.py](./state/observation_space.py):** - Fuses symbolic and sub-symbolic features into a single observation vector for the RL policy.
  - Defines the observation dimensionality and composition.


**Key Responsibilities:**
- **State Representation:** Provides both high-level and low-level representations of the environment.
- **Observation Generation:** Combines symbolic and sub-symbolic data to generate comprehensive observations for the agent.

### 4. **[action/action_space.py](./action/action_space.py)**
This module defines the unified action space for the RL agent, combining primitive and symbolic actions into a flat discrete space.

**Core Functions:**
- **Action Enumeration:** Generates both primitive (e.g., `move_forward`, `turn_left`) and grounded symbolic actions from the PDDL domain/problem.
- **Filtering:** Excludes invalid grounded actions based on prior domain knowledge.
- **Precondition Checking:** Rejects symbolic actions at runtime if their preconditions (checked via `PDDLPredicates`) are not met.
- **Execution Routing:** Routes symbolic actions to `PDDLActions.execute()` and primitive actions to a ROS velocity service.

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
- **[LocalGridService.py](./ROS_services/LocalGridService.py):** Produces occupancy grid and spatial info for sub-symbolic state.

**Integration:** These services are invoked by `SymbolicState`, `PDDLPredicates`, and `PDDLActions` to compute state and execute symbolic effects reliably.


**Core Functions:**
- **Low-Level Interactions:** Provides the necessary services for low-level interactions between the agent and the environment.
- **Integration with Actions:** Ensures that high-level actions have the necessary low-level support to be executed effectively.