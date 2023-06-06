from xcub_sensor_check.generic_test import GenericTest
import bipedal_locomotion_framework as blf


class IMUTest(GenericTest):
    def __init__(self, name):
        super().__init__(name=name)

    def configure(self, param_handler: blf.parameters_handler.IParametersHandler):
        pass

    def run(self):
        return True
