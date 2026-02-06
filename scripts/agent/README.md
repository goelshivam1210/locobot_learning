# Agent (Architecture)

This level contains the code for a hybrid agent system integrating PDDL planning and reinforcement learning. Below is an overview of the key components, their relationships, and how they fit into the overall system.

## Tests

The test script `test_hybrid_agent.py` runs the planning and learning process in `HybridAgent.py`. To run it, use the following command:

```bash
python test_hybrid_agent.py
```

### Arguments

The following arguments can be passed to the test script:

- `--demonstrations`: Number of human demonstrations to run before the robot starts learning. Optional, default is 0.
- `--test_only`: If set, the script will only run test episodes using a saved policy without performing any learning. Optional, default is False.
- `--run_dir`: Directory to save logs and models. Optional, defaults to the current date, in YYYY-MM-DD format. You can pass an existing run ID here to continue training from a previous run. In that case, the script will look for the latest checkpoint in that directory and load it.
- `--no-local-view`: If set, the agent will not include local view observations in the learning process. Optional, default is False. This is currently untested and should be considered unsupported.


### Outputs

The run directory will contain various artifacts from the training:

- `learning_agent_params.json`: A JSON file containing the parameters used for the learning agent.
- `stats.csv`: A CSV file logging episode statistics such as rewards, success rates, and episode durations.
- `policies/`: A directory containing saved policy checkpoints. Each checkpoint is saved every `policy_checkpoint_frequency` episodes. It will also contain the latest incomplete policy, which is updated after every episode, allowing you to resume training from the most recent state in case of interruptions.


### Test mode

If you run the script with the `--test_only` flag, it will skip the learning process if there is a saved policy for the failed operator in the `policies/` directory. Instead, it will directly run the saved policy for a maximum of `max_steps` steps. If the saved policy succeeds, it will log the success and move on to the next episode. If it fails or if there is no saved policy, it will log the failure and skip to the next episode without attempting to learn a new policy. This allows you to evaluate the performance of previously learned policies without performing any new learning. The number of test episodes is hard coded in `test_hybrid_agent.py` if `--test_only` is set.


### Using saved policies

There is no automated process for putting a saved policy into the `policies/` directory. Instead, you will have to manually copy the policy checkpoint file you want to use from the run directory's `policies/` directory and rename it to change the `_episode_XX.pth` to just `.pth`. For example, if you have a checkpoint file named `room_1_room_2_doorway_1_episode_10.pth`, you would copy it and rename the copy to `room_1_room_2_doorway_1.pth`. The script will look for `room_1_room_2_doorway_1.pth` when running in test mode. Be sure to place the policy file into the directory corresponding to the failed operator (e.g. `pass_through_door/`). The run directory has the proper directory structure for this.

For example:

runs/2026-01-01/policies/pass_through_door/room_1_room_2_doorway_1_episode_10.pth  --> copy and rename to --> locobot_learning/policies/pass_through_door/room_1_room_2_doorway_1.pth

Note that the script will start learning if it encounters a failed operator for which there is no saved policy.

## Directory Structure

```
.
├── agent
│   ├── README.md
│   ├── core
│   │   ├── Agent.py
│   │   ├── HybridAgent.py
│   │   ├── PDDLActions.py
│   │   ├── PDDLPredicates.py
│   │   ├── __init__.py
│   │   ├── learner
│   │   │   ├── BaseLearner.py
│   │   │   ├── LearningAgent.py
│   │   │   ├── PPOLearner.py
│   │   │   ├── __init__.py
│   │   └── planner
│   │       ├── RecycleBotPlanner.py
│   │       ├── __init__.py
│   │       └── planner.py
│   ├── tests
│   │   ├── test_hybrid_agent.py
│   │   └── test_planner.py
.
├── environment
│   ├── predicate_services
│   ├── ActionGenerator.py
│   ├── RecycleBot.py
│   ├── RewardFunction.py
│   ├── __init__.py
├── executor
├── knowledge
├── perception

```

## Key directories and files

- **[`core/`](core/)**: Contains the core logic for the agent, including the main agent classes, planners, and learning components.
- **[`core/learner/`](core/learner/)**: Houses the learning components such as the base learner and PPO-based learner.
- **[`core/planner/`](core/planner/)**: Includes the PDDL-based planner and its specialized extensions for the RecycleBot domain.
- **[`../../environment/`](../../environment/)**: Implements the environment in which the agent operates, including action space generation, observation, and reward functions.
- **[`../../knowledge/PDDL/recycle_bot/`](../../knowledge/PDDL/recycle_bot/)**: Contains PDDL files that define the domain and problem space for the RecycleBot.

## Core components

### 1. [`Agent.py`](core/Agent.py)

The `Agent` class is the core class responsible for executing the plan generated by the planner. It verifies the preconditions and effects of each action, ensuring that the plan can be executed successfully. It interacts closely with the `PDDLActions` and `PDDLPredicates` classes to perform these checks.

**Key responsibilities**:
- **Plan Execution**: Executes the sequence of actions generated by the planner.
- **Verification**: Checks if the environment meets the necessary conditions before and after executing an action.

### 2. [`HybridAgent.py`](core/HybridAgent.py)

The `HybridAgent` class extends the `Agent` class by adding failure recovery and learning capabilities. If a plan fails during execution, the `HybridAgent` can enter a recovery mode, where it attempts to retry the failed action or invoke a learning mechanism (like PPO) to adapt and retry the plan.

**Key features**:
- **Recovery Mode**: Handles failures by retrying actions and invoking the learning mechanism if retries are exhausted.
- **Learning Integration**: Utilizes reinforcement learning to adapt and improve the agent’s policy based on observed failures.

### 3. [`PDDLActions.py`](core/PDDLActions.py)

This class contains the implementations for executing the actions specified in the PDDL plan. Each method corresponds to a specific action in the domain, such as `approach`, `pick`, or `place`.

**Core functions**:
- **Converts symbolic actions** from the PDDL plan into executable procedures.
- **Ensures that each action** is performed in the correct sequence with the necessary parameters.

### 4. [`PDDLPredicates.py`](core/PDDLPredicates.py)

The `PDDLPredicates` class is responsible for checking the preconditions and effects of actions. It interacts with the environment (or in a ROS context, with the ROS services) to validate whether certain conditions hold true.

**Responsibilities**:
- **Precondition Checking**: Verifies that the environment meets the necessary conditions before an action is executed.
- **Effect Validation**: Confirms that the outcomes of an action match the expected results.

### 5. [`learner/LearningAgent.py`](core/learner/LearningAgent.py)

The `LearningAgent` class integrates the reinforcement learning component with the agent system. It manages the learning process, interacting with the environment to refine the agent’s policy when failures occur.

**Key features**:
- **Environment Interaction**: Interfaces with the `RecycleBot` environment for state observations and rewards.
- **Learning Process**: Utilizes PPO (Proximal Policy Optimization) to improve the agent’s policy, making it more capable of handling similar situations in the future.
- **Model Management**: Includes methods to save and load trained models for reuse.

### 6. [`PPOLearner.py`](core/learner/PPOLearner.py)

The `PPOLearner` class implements the PPO algorithm, a state-of-the-art method for policy gradient reinforcement learning. It is responsible for the core RL logic used by the `LearningAgent`.

**Core functions**:
- **Implements the PPO algorithm** to optimize the agent’s policy based on rewards received from the environment.
- **Provides methods to save and load models**, facilitating the reuse and refinement of learned policies.

### 7. [`planner/planner.py`](core/planner/planner.py)

The `Planner` class is the core planning module that interfaces with the PDDL domain and problem files. It parses the domain, generates problem strings, and interacts with the PDDL planner to create executable plans.

**Capabilities**:
- **PDDL Parsing**: Parses PDDL files to understand the domain and problem context.
- **Plan Generation**: Generates a feasible plan of actions that the agent can execute to reach its goals.

### 8. [`RecycleBotPlanner.py`](core/planner/RecycleBotPlanner.py)

The `RecycleBotPlanner` class extends the base `Planner` to handle domain-specific logic for the `RecycleBot`, a robot designed to operate in environments with defined rooms, objects, and actions.

**Specialized functions**:
- **Tailors planning logic** to suit the specific needs of the `RecycleBot` domain, such as handling room transitions and object interactions.
- **Ensures that the generated plans** are optimized for the constraints and requirements of the `RecycleBot` environment.

## Recovery Mode and Learning

### Recovery Mode

If an action fails during execution, the `HybridAgent` ([`HybridAgent.py`](core/HybridAgent.py)) triggers recovery mode. It will:

1. **Retry the Failed Action**: Attempt to execute the action up to a specified number of times, leveraging any available information to increase the chance of success.
2. **Invoke the Learning Agent**: If retries are exhausted, the agent attempts to execute a learned policy. If no policy exists or it also fails, the agent triggers the learning process to adapt its policy for future attempts.

### Learning Process

When invoked, the `LearningAgent` ([`LearningAgent.py`](core/learner/LearningAgent.py)) interacts with the `RecycleBot` environment to collect observations and rewards. It uses these to train a new policy via PPO. The agent can then use this learned policy to retry failed actions in future plans.

## Testing and Usage

### Testing the Components

- **Testing the Planner**: Use [`tests/test_planner.py`](tests/test_planner.py) to verify the planner’s ability to generate valid plans from the PDDL domain and problem files.
- **Testing the Hybrid Agent**: Use [`tests/test_hybrid_agent.py`](tests/test_hybrid_agent.py) to simulate plan execution, recovery mode, and the integration of learning into the agent’s workflow.

### Running Tests

To run the tests, navigate to the `tests` directory and use the following commands:

```bash
python test_planner.py
```
```bash
python test_hybrid_agent.py
```