#!/usr/bin/env python3

import rospy
from typing import List, Tuple, Union, Dict, Any
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from defaults import DEFAULT_LINEAR_VELOCITY, DEFAULT_ANGULAR_VELOCITY
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'agent', 'core')))
from planner.planner import Action

class ActionSpace:
    def __init__(
        self,
        grounded_symbolic_actions: List[Tuple[str, List[str]]],
        skip_action: Union[Action, None] = None,
    ):
        """
        Initializes the ActionSpace.

        Args:
            grounded_symbolic_actions: List of tuples (action_name, [parameters])
        """
        self.action_list = []
        self._load_actions(grounded_symbolic_actions, skip_action)

    def _load_actions(
        self,
        grounded_symbolic_actions: List[Tuple[str, List[str]]],
        skip_action: Union[Action, None]
    ):
        """
        Loads primitive and symbolic grounded actions into a flat action list.
        """
        # Add primitive actions
        primitive_actions = ["move_forward", "turn_left", "turn_right"]
        for action_name in primitive_actions:
            params = None
            if action_name == "move_forward":
                params = {
                    "velocity": DEFAULT_LINEAR_VELOCITY,
                }
            else:
                params = {
                    "velocity": DEFAULT_ANGULAR_VELOCITY,
                }
            self.action_list.append({
                "type": "primitive",
                "name": action_name,
                "params": params,
            })

        # Add grounded symbolic actions
        for action_name, params in grounded_symbolic_actions:
            self.action_list.append({
                "type": "symbolic",
                "name": action_name,
                "params": params
            })
        if skip_action is not None:
            self.action_list = [
                action for action in self.action_list
                if not (skip_action.name == action["name"] and skip_action.parameters == tuple(action["params"]))
            ]

    @staticmethod
    def _contains_action(action_list: List[Action], action: Dict[str, Any]) -> bool:
        for a in action_list:
            if a.name == action["name"] and a.parameters == tuple(action["params"]):
                return True
        return False

    def get_action_id(self, action_name: str) -> Union[int, None]:
        """
        Returns the ID of the action identified by the specified name.

        Args:
            action_name: Name of the action.
        Returns:
            dict with keys: 'type', 'name', 'params'
            None if action_name is not a valid action.
        """
        for id, action in enumerate(self.action_list):
            if action["name"] == action_name:
                return id
        return None

    @property
    def size(self) -> int:
        """
        Returns the total size of the action space.
        """
        return len(self.action_list)

    def get_action(self, action_id: int) -> dict:
        """
        Given an action index, returns the action dictionary.

        Args:
            action_id: Integer index of the action.
        Returns:
            dict with keys: 'type', 'name', 'params'
        """
        return self.action_list[action_id]

    def execute_action(self, action_id, primitive_client, pddl_actions_client, predicate_checker=None):
        """
        Executes the action based on its type.

        Args:
            action_id: Integer index of the action.
            primitive_client: Callable client for primitive actions.
            pddl_actions_client: Instance of PDDLActions class.
        """
    
        action = self.get_action(action_id)

        if action["type"] == "primitive":
            primitive_client(action["name"], action["params"])
            return True

        elif action["type"] == "symbolic":
            if predicate_checker is not None:
                if not predicate_checker.check_preconditions(action["name"], action["params"]):
                    rospy.logwarn(f"[ActionSpace] Skipping symbolic action due to failed preconditions: {action}")
                    return False
            pddl_actions_client.execute(action["name"], action["params"])
            return True

        else:
            rospy.logwarn(f"[ActionSpace] Unknown action type: {action['type']}")
            return False
