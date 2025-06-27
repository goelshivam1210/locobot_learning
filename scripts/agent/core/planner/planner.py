import re
import sys
import os
from os import mkdir
from os.path import exists, join

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'knowledge', 'pddl-parser')))

from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.planner import Planner as PDDL_Planner
from pddl_parser.action import Action as PDDL_Action

#Reexport the PDDL_Action class
Action = PDDL_Action

class Planner:
    def __init__(self, domain_path: str):
        self._domain_path = domain_path
        self._predicates = {}
        self._preconditions = {}
        self._effects = {}
        self._actions = []  # Initialize _actions here

        self._parser = PDDL_Parser()
        self._parser.parse_domain(self._domain_path)

        self.__parse_predicates()
        self.__parse_actions()

        self.pddl_planner = PDDL_Planner()  # Initialize the PDDL planner    

    def verify_preconditions(self, action: str, *args):
        for pred in self._preconditions[action]:
            if not pred['func'](pred['name'], pred['arg_indecies'], pred['consts'], *args):
                return False
        return True
    
    def verify_effects(self, action: str, *args):
        for pred in self._effects[action]:
            if not pred['func'](pred['name'], pred['arg_indecies'], pred['consts'], *args):
                return False
        return True

    def generate_problem_str(self, objects: dict, current_state: dict) -> str:
        """
        Generate a PDDL problem string based on the current state of the world.
        Args:
            objects (dict): Dictionary containing objects and their states.
            current_state (dict): Dictionary containing the dynamic state of the world.
        Returns:
            str: The problem file content as a string.
        """
        problem_str = f"(define (problem recycle) (:domain recycle_bot)\n"
        problem_str += "(:objects \n"
        
        for obj_type, obj_list in objects.items():
            obj_str = " ".join(obj_list)
            problem_str += f"    {obj_str} - {obj_type}\n"
        
        problem_str += ")\n\n(:init\n"

        # Add fixed connections
        problem_str += "    (connect room_1 room_2 doorway_1)\n"
        problem_str += "    (connect room_2 room_1 doorway_1)\n"

        # Add always-true states
        problem_str += "    (at room_2 doorway_1)\n"
        problem_str += "    (at room_1 doorway_1)\n"

        # Add dynamic initial states from current_state
        robot_location = current_state['robot_location']
        problem_str += f"    (at {robot_location} robot_1)\n"

        robot_facing = current_state['robot_facing']
        problem_str += f"    (facing {robot_facing})\n"

        robot_holding = current_state['robot_holding']
        problem_str += f"    (hold {robot_holding})\n"

        for obj, location in current_state['object_locations'].items():
            problem_str += f"    (at {location} {obj})\n"
        
        problem_str += ")\n\n(:goal (and\n"
        problem_str += "    (contain ball_1 bin_1)\n"
        problem_str += "))\n)"

        return problem_str


    def generate_plan_str(self, objects, current_state) -> str:
        """
        Generate a plan string by invoking the PDDL planner and applying post-processing.
        """
        problem_file = "temp_problem.pddl"  # Temporary filename for the problem
        with open(problem_file, 'w') as file:
            file.write(self.generate_problem_str(objects, current_state))  # Pass current_state

        # Call the PDDL planner to generate a plan
        plan = self.pddl_planner.solve(self._domain_path, problem_file)

        print(f"Plan from PDDLParser = {plan}")

        if plan is None:
            raise Exception("No plan found")

        # Post-process the plan to fix inconsistencies
        corrected_plan = self.post_process_plan(plan)
        self.plan = corrected_plan

        # Convert the corrected plan to a string
        plan_str = ""
        for act in corrected_plan:
            plan_str += f"{act.name} {' '.join(act.parameters)}\n"

        return plan_str

    def new_problem(self, objects: dict, current_state: dict):
        """
        Create a new planning problem based on the current state.
        """
        problem_str = self.generate_problem_str(objects, current_state)
        print("Generated problem file:")
        print(problem_str)

        plan_str = self.generate_plan_str(objects, current_state)  # Pass current_state here
        print("Generated plan file:")
        print(plan_str)

        self.__create_action_generator(plan_str)

    def post_process_plan(self, plan):
        """
        Adjust the generated plan to account for runtime state changes like
        the robot's facing direction.
        """
        corrected_plan = []
        current_facing = "nothing"  # Start with the initial facing direction
        
        for act in plan:
            action_name = act.name
            parameters = list(act.parameters)  # Convert to list for mutability

            if action_name == "approach":
                # Update the third parameter to match the current facing direction
                parameters[2] = current_facing
                current_facing = parameters[0]  # Update the current facing to the approached object
            elif action_name == "pick":
                current_facing = "nothing"  # After picking, robot faces nothing
            elif action_name == "pass_through_door":
                # Maintain facing direction as the doorway
                current_facing = parameters[2]  # The doorway remains the facing direction after passing through
            elif action_name == "place":
                current_facing = "nothing"  # After placing, reset facing to nothing
            
            # Create a corrected action by copying the existing action and updating parameters
            corrected_action = self.copy_action_with_parameters(act, tuple(parameters))
            corrected_plan.append(corrected_action)
        
        return corrected_plan

    def copy_action_with_parameters(self, action, parameters):
        """
        Create a copy of the given action with updated parameters.
        """
        return action.__class__(
            action.name,
            parameters,
            action.positive_preconditions,
            action.negative_preconditions,
            action.add_effects,
            action.del_effects
        )

    # def generate_plan_str(self, objects) -> str:
    #     """
    #     Generate a plan string by invoking the PDDL planner and applying post-processing.
    #     """
    #     problem_file = "temp_problem.pddl"  # Temporary filename for the problem
    #     with open(problem_file, 'w') as file:
    #         file.write(self.generate_problem_str(objects))

    #     # Call the PDDL planner to generate a plan
    #     plan = self.pddl_planner.solve(self._domain_path, problem_file)

    #     print(f"Plan from PDDLParser = {plan}")

    #     if plan is None:
    #         return "No plan found"

    #     # Post-process the plan to fix inconsistencies
    #     corrected_plan = self.post_process_plan(plan)

    #     # Convert the corrected plan to a string
    #     plan_str = ""
    #     for act in corrected_plan:
    #         plan_str += f"{act.name} {' '.join(act.parameters)}\n"

    #     return plan_str


    # def new_problem(self, objects: dict, current_state: dict):
    #     problem_str = self.generate_problem_str(objects, current_state)
    #     print("Generated problem file:")
    #     print(problem_str)

    #     plan_str = self.generate_plan_str(objects)
    #     print("Generated plan file:")
    #     print(plan_str)

    #     self.__create_action_generator(plan_str)

    
    def next_action(self) -> str:
        try:
            return next(self._action)
        except StopIteration:
            return None

    def __parse_predicates(self) -> None:
        for predicate, args in self._parser.predicates.items():
            self._predicates[predicate] = len(args)

    def __verify_predicates(self, function_dict: dict) -> None:
        predicate_keys = set(self._predicates.keys())
        function_keys = set(function_dict.keys())

        if predicate_keys != function_keys:
            raise ValueError("Predicate keys do not match function dictionary keys.")

    def __parse_actions(self) -> None:
        for action in self._parser.actions:
            self._actions.append(action.name)
            parameters = [param[0] for param in action.parameters]
            self._preconditions[action.name] = self.__build_boolean_function(action.positive_preconditions,
                                                                             action.negative_preconditions,
                                                                             parameters)
            self._effects[action.name] = self.__build_boolean_function(action.add_effects,
                                                                       action.del_effects,
                                                                       parameters)

    def __build_boolean_function(self, pos_conds: set, neg_conds: set, params: list):
        funcs = []
        for cond in pos_conds:
            predicate_name = cond[0]
            predicate_args = cond[1:]
            p_arg_indecies = []
            const_args = []
            for i, arg in enumerate(predicate_args):
                try:
                    p_arg_indecies.append(params.index(arg))
                except ValueError:
                    const_args.append((i, arg))

            def func(name, arg_indecies, consts, *args):
                args = list(args)
                args = [args[i] for i in arg_indecies]
                for (i, val) in consts:
                    args.insert(i, val)
                return self._predicate_funcs[name](*args)

            funcs.append({'name': predicate_name, 'arg_indecies': p_arg_indecies, 'consts': const_args, 'func': func})

        return funcs
    
    def __create_action_generator(self, plan_str):
        plan_lines = plan_str.strip().split('\n')
        actions = []
        for line in plan_lines:
            actions.append(line.split())
        self._action = (action for action in actions)


    def compute_plannable_states(self, plan, failed_operator):
        """
        Computes the plannable state set (S_r) given a plan and a failed operator.
        plan: list of operator objects
        failed_operator: operator object (failed)
        Returns: set of predicates
        """
        S_r = set()
        
        for op in reversed(plan):
            print("[ComputePlannableStates] Processing operator", op)
            if op.name == failed_operator.name and op.parameters == failed_operator.parameters or \
                failed_operator.add_effects.issuperset(op.positive_preconditions):
                print(f"[ComputePlannableStates] Skipping failed operator")
                continue  # skip failed operator itself
            print(f"[ComputePlannableStates] Adding positive preconditions {op.positive_preconditions}")
            S_r.update(op.positive_preconditions)
            print(f"[ComputePlannableStates] Updated set: {S_r}")
            for eff in op.add_effects:
                print(f"[ComputePlannableStates] Processing add effect: {eff}")
                if eff in S_r:
                    print(f"[ComputePlannableStates] Removing effect: {eff}")
                    S_r.remove(eff)
                    print(f"[ComputePlannableStates] Updated set: {S_r}")
        S_r.update(failed_operator.add_effects)
        return S_r

if __name__ == "__main__":
    from argparse import ArgumentParser
    import os

    # parser = ArgumentParser()
    # parser.add_argument("-a", "--action")
    # parser.add_argument("-p", "--param", nargs="*")

    # args = parser.parse_args()

    # action = args.action
    # params = args.param

    action, *params = sys.argv[1:]

    print(f"Action: {action}")
    print(f"Params: {params}")


    domain_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'knowledge', 'PDDL', 'recycle_bot', 'domain.pddl'))
    planner = Planner(domain_path=domain_path)

    result = planner.verify_preconditions(action, *params)

    print(f"Verification result: {result}")


