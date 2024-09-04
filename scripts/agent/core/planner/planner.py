import re
from os import mkdir
from os.path import exists, join
from pddl_parser.PDDL import PDDL_Parser
from pddl_parser.planner import Planner as PDDL_Planner

class Planner:
    def __init__(self, domain_path: str, predicate_funcs: dict):
        self._domain_path = domain_path
        self._predicates = {}
        self._preconditions = {}
        self._effects = {}
        self._actions = []  # Initialize _actions here

        self._parser = PDDL_Parser()
        self._parser.parse_domain(self._domain_path)

        self.__parse_predicates()
        self.__verify_predicates(predicate_funcs)

        self._predicate_funcs = predicate_funcs
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

    def generate_problem_str(self, objects: dict) -> str:
        """
        Generate a PDDL problem string based on the current state of the world.
        Args:
            objects (dict): Dictionary containing objects and their states.
        Returns:
            str: The problem file content as a string.
        """
        # Start building the problem definition
        problem_str = f"(define (problem recycle) (:domain recycle_bot)\n"
        problem_str += "(:objects \n"
        
        for obj_type, obj_list in objects.items():
            obj_str = " ".join(obj_list)
            problem_str += f"    {obj_str} - {obj_type}\n"
        
        problem_str += ")\n\n(:init\n"

        # Add fixed connections
        problem_str += "    (connect room_1 room_2 doorway_1)\n"
        problem_str += "    (connect room_2 room_1 doorway_1)\n"
        
        # Add initial states
        for obj_type, obj_list in objects.items():
            if obj_type == "room" or obj_type == "nothing":
                continue  # Skip rooms and 'nothing'
            for obj in obj_list:
                # Correct object placement
                if obj == "bin_1":
                    problem_str += f"    (at room_2 {obj})\n"
                else:
                    problem_str += f"    (at room_1 {obj})\n"
        
        problem_str += "    (facing nothing)\n"
        problem_str += "    (hold nothing)\n"
        problem_str += ")\n\n(:goal (and\n"
        problem_str += "    (contain ball_1 bin_1)\n"
        problem_str += "))\n)"

        return problem_str

    def generate_plan_str(self, objects) -> str:
        problem_file = "temp_problem.pddl"  # Temporary filename for the problem
        with open(problem_file, 'w') as file:
            file.write(self.generate_problem_str(objects))

        # Call the PDDL planner to generate a plan
        plan = self.pddl_planner.solve(self._domain_path, problem_file)

        print (f"plan = {plan}")
        
        if plan is None:
            return "No plan found"

        plan_str = ""
        for act in plan:
            plan_str += f"{act.name} {' '.join(act.parameters)}\n"

        return plan_str

    def new_problem(self, objects: dict):
        problem_str = self.generate_problem_str(objects)
        print("Generated problem file:")
        print(problem_str)

        plan_str = self.generate_plan_str(objects)
        print("Generated plan file:")
        print(plan_str)

        self.__create_action_generator(plan_str)
    
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