import bipedal_locomotion_framework.bindings as blf
from abc import ABC, abstractmethod


class GenericTest(ABC):
    def __init__(self, name):
        self.outcome = False
        self.name = name

    @abstractmethod
    def configure(
        self, param_handler: blf.parameters_handler.IParametersHandler
    ) -> bool:
        pass

    @abstractmethod
    def run(self) -> bool:
        pass
