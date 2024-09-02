

class BaseLearner:
    def __init__(self):
        pass

    def learn(self):
        """
        Method to start the learning process.
        Should be overridden by the specific learner implementation.
        """
        raise NotImplementedError("Learn method not implemented in BaseLearner")

    def save_model(self, file_path):
        """
        Save the model to a file.
        """
        raise NotImplementedError("Save method not implemented in BaseLearner")

    def load_model(self, file_path):
        """
        Load the model from a file.
        """
        raise NotImplementedError("Load method not implemented in BaseLearner")