from xcub_sensor_check.generic_test import GenericTest
import bipedal_locomotion_framework as blf
import numpy as np
import h5py
from enum import Enum
import idyntree.bindings as idyn
from pathlib import Path
import matplotlib.pyplot as plt


class SignalType(Enum):
    def __str__(self):
        return str(self.name) + "s"

    unknown = 0
    accelerometer = 1
    gyro = 2
    orientation = 3


class GenericImuSignalTest(GenericTest):
    def __init__(
        self, name: str, additional_output_folder: Path, signal_type: SignalType
    ):
        super().__init__(name=name, additional_output_folder=additional_output_folder)

        self.model_path = ""
        self.sensor_name = ""
        self.frame_name = ""
        self.joints_name = []

        class JointState:
            self.positions = np.array([])
            self.velocities = np.array([])
            self.accelerations = np.array([])

        self.signal_type = signal_type
        self.joint_state = JointState()
        self.imu_signal = np.array([])
        self.expected_imu_signal = np.array([])

        self.accepted_mean_error = np.array([])
        self.accepted_std_error = np.array([])
        self.file_name = ""

    def configure(self, param_handler: blf.parameters_handler.IParametersHandler):
        self.sensor_name = param_handler.get_parameter_string("sensor_name")
        self.frame_name = param_handler.get_parameter_string("frame_name")
        self.model_path = param_handler.get_parameter_string("model_path")
        self.accepted_mean_error = param_handler.get_parameter_vector_float(
            "error_mean_tolerance"
        )
        self.accepted_std_error = param_handler.get_parameter_vector_float(
            "error_std_tolerance"
        )

        self.file_name = param_handler.get_parameter_string("dataset_file_name")
        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            joint_ref = root_variable["description_list"]
            self.joints_name = [
                "".join(chr(c[0]) for c in file[ref]) for ref in joint_ref[0]
            ]
            self.joint_state.positions = np.squeeze(
                np.array(root_variable["joints_state"]["positions"]["data"])
            )

            self.joint_state.velocities = np.squeeze(
                np.array(root_variable["joints_state"]["velocities"]["data"])
            )
            self.joint_state.accelerations = np.squeeze(
                np.array(root_variable["joints_state"]["accelerations"]["data"])
            )

        self.expected_imu_signal.resize((self.joint_state.positions.shape[0], 3))

    def get_kindyn(self) -> idyn.KinDynComputations:
        ml = idyn.ModelLoader()
        ml.loadReducedModelFromFile(self.model_path, self.joints_name)
        kindyn = idyn.KinDynComputations()
        kindyn.loadRobotModel(ml.model())
        kindyn.setFrameVelocityRepresentation(idyn.BODY_FIXED_REPRESENTATION)
        return kindyn


class GyroTest(GenericImuSignalTest):
    def __init__(self, name: str, additional_output_folder: Path):
        super().__init__(
            name=name,
            additional_output_folder=additional_output_folder,
            signal_type=SignalType.gyro,
        )

    def run(self):
        kindyn = self.get_kindyn()
        gravity = np.array([0, 0, -blf.math.StandardAccelerationOfGravitation])

        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            self.imu_signal = np.squeeze(
                np.array(root_variable[str(self.signal_type)][self.sensor_name]["data"])
            )

        for i in range(self.joint_state.positions.shape[0]):
            kindyn.setRobotState(
                self.joint_state.positions[i, :],
                self.joint_state.velocities[i, :],
                gravity,
            )

            tmp = kindyn.getFrameVel(self.frame_name).toNumPy()
            self.expected_imu_signal[i, :] = tmp[3:6]

        error = self.expected_imu_signal - self.imu_signal

        fig, axs = plt.subplots(3, 2)
        axis_name = ["x", "y", "z"]
        fig.suptitle(self.name.replace("_", " "), fontsize=16)
        fig.set_size_inches(10.5, 6.5)

        for i in range(3):
            axs[i][0].set(ylabel="ω" + axis_name[i] + " (rad/s)")
            axs[i][0].plot(self.expected_imu_signal[:, i], label="Kinematics")
            axs[i][0].plot(self.imu_signal[:, i], label="Sensor")
            axs[i][1].plot(error[:, i], label="Error")
            axs[i][0].legend()
            axs[i][1].legend()

        (self.additional_output_folder / self.name).mkdir(parents=True, exist_ok=True)

        plt.savefig(
            str(self.additional_output_folder / self.name / (self.name + ".png"))
        )
        plt.close()

        std = np.std(error, axis=0)
        mean = np.mean(error, axis=0)

        return (np.abs(mean) < self.accepted_mean_error).all() and (
            std < self.accepted_std_error
        ).all()


class OrientationTest(GenericImuSignalTest):
    def __init__(self, name: str, additional_output_folder: Path):
        super().__init__(
            name=name,
            additional_output_folder=additional_output_folder,
            signal_type=SignalType.orientation,
        )

    def run(self):
        kindyn = self.get_kindyn()
        gravity = np.array([0, 0, -blf.math.StandardAccelerationOfGravitation])

        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            imu_signal_tmp = np.squeeze(
                np.array(root_variable[str(self.signal_type)][self.sensor_name]["data"])
            )

            kindyn.setRobotState(
                self.joint_state.positions[0, :],
                self.joint_state.velocities[0, :],
                gravity,
            )

            I_R_FK = kindyn.getWorldTransform(self.frame_name).getRotation()
            I_R_I_IMU = (
                I_R_FK
                * (
                    idyn.Rotation.RPY(
                        imu_signal_tmp[0, 0], imu_signal_tmp[0, 1], imu_signal_tmp[0, 2]
                    )
                ).inverse()
            )

            self.imu_signal.resize((self.joint_state.positions.shape[0], 3))
            self.expected_imu_signal.resize((self.joint_state.positions.shape[0], 3))
            error = np.zeros((self.joint_state.positions.shape[0], 3))

            for i in range(self.joint_state.positions.shape[0]):
                kindyn.setRobotState(
                    self.joint_state.positions[i, :],
                    self.joint_state.velocities[i, :],
                    gravity,
                )

                expected_imu_signal_tmp = kindyn.getWorldTransform(
                    self.frame_name
                ).getRotation()
                self.expected_imu_signal[
                    i, :
                ] = expected_imu_signal_tmp.asRPY().toNumPy()

                imu_signal_temp = I_R_I_IMU * idyn.Rotation.RPY(
                    imu_signal_tmp[i, 0], imu_signal_tmp[i, 1], imu_signal_tmp[i, 2]
                )
                self.imu_signal[i, :] = imu_signal_temp.asRPY().toNumPy()
                error[i, :] = (
                    (expected_imu_signal_tmp * imu_signal_temp.inverse())
                    .log()
                    .toNumPy()
                )

        fig, axs = plt.subplots(3, 2)
        axis_name = ["r", "p", "y"]
        fig.suptitle(self.name.replace("_", " "), fontsize=16)
        fig.set_size_inches(10.5, 6.5)

        for i in range(3):
            axs[i][0].set(ylabel="Angle " + axis_name[i] + " (rad)")
            axs[i][0].plot(self.expected_imu_signal[:, i], label="Kinematics")
            axs[i][0].plot(self.imu_signal[:, i], label="Sensor")
            axs[i][1].plot(error[:, i], label="Error")
            axs[i][0].legend()
            axs[i][1].legend()

        (self.additional_output_folder / self.name).mkdir(parents=True, exist_ok=True)

        plt.savefig(
            str(self.additional_output_folder / self.name / (self.name + ".png"))
        )
        plt.close()

        std = np.std(error, axis=0)
        mean = np.mean(error, axis=0)

        return (np.abs(mean) < self.accepted_mean_error).all() and (
            std < self.accepted_std_error
        ).all()
