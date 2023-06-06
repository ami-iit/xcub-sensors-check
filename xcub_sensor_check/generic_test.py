import bipedal_locomotion_framework.bindings as blf
from abc import ABC, abstractmethod
from pathlib import Path


class GenericTest(ABC):
    def __init__(self, name: str, additional_output_folder: Path):
        self.outcome = False
        self.name = name
        self.additional_output_folder = additional_output_folder

    @abstractmethod
    def configure(
        self, param_handler: blf.parameters_handler.IParametersHandler
    ) -> bool:
        pass

    @abstractmethod
    def run(self) -> bool:
        pass
