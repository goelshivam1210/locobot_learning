class ActionExecutionError(Exception):
    def __init__(self, reason: str, action_name: str, params: list):
        super().__init__(reason)
        self.action_name = action_name
        self.params = params
