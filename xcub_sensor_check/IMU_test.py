from xcub_sensor_check.generic_test import GenericTest
import bipedal_locomotion_framework as blf
import numpy as np
import h5py
from enum import Enum
import idyntree.bindings as idyn
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter


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
        self.file_name = ""
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

        # Get optional parameters
        try:
            base_link_orientation_rpy_deg = param_handler.get_parameter_vector_float(
                "base_link_orientation_rpy_deg"
            )
        except:
            base_link_orientation_rpy_deg = [0, 0, 0]
            blf.log().info(
                "No 'base_link_orientation_rpy found' in the parameter handler. Assuming [0, 0, 0]"
            )

        # Convert the base link orientation from deg to rad
        base_link_orientation_rpy_rad = [
            np.deg2rad(x) for x in base_link_orientation_rpy_deg
        ]

        # Convert the base link orientation from RPY to Rotation
        self.base_link_orientation = idyn.Rotation.RPY(
            base_link_orientation_rpy_rad[0],
            base_link_orientation_rpy_rad[1],
            base_link_orientation_rpy_rad[2],
        )

        compute_joint_velocity_from_position = param_handler.get_parameter_bool(
            "compute_joint_velocity_from_position"
        )
        compute_joint_acceleration_from_position = param_handler.get_parameter_bool(
            "compute_joint_acceleration_from_position"
        )

        velocity_window_length = 0
        if compute_joint_velocity_from_position:
            velocity_window_length = param_handler.get_parameter_int(
                "velocity_svg_window_length"
            )

        acceleration_window_length = 0
        if compute_joint_acceleration_from_position:
            acceleration_window_length = param_handler.get_parameter_int(
                "acceleration_svg_window_length"
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

            sampling_time = np.average(
                np.diff(
                    np.squeeze(
                        np.array(
                            root_variable["joints_state"]["positions"]["timestamps"]
                        )
                    )
                )
            )

            if compute_joint_velocity_from_position:
                self.joint_state.velocities = (
                    savgol_filter(
                        x=self.joint_state.positions,
                        window_length=velocity_window_length,
                        polyorder=3,
                        axis=0,
                        deriv=1,
                    )
                    / sampling_time
                )
            else:
                self.joint_state.velocities = np.squeeze(
                    np.array(root_variable["joints_state"]["velocities"]["data"])
                )

            if compute_joint_acceleration_from_position:
                self.joint_state.accelerations = savgol_filter(
                    x=self.joint_state.positions,
                    window_length=acceleration_window_length,
                    polyorder=3,
                    axis=0,
                    deriv=2,
                ) / (sampling_time**2)
            else:
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

        I_T_base = idyn.Transform(self.base_link_orientation, idyn.Position.Zero())
        base_velocity = idyn.Twist.Zero()

        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            self.imu_signal = np.squeeze(
                np.array(root_variable[str(self.signal_type)][self.sensor_name]["data"])
            )

        for i in range(self.joint_state.positions.shape[0]):
            kindyn.setRobotState(
                I_T_base,
                self.joint_state.positions[i, :],
                base_velocity,
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
    """
    The OrientationTest checks the orientation of the IMU. The test is performed in the following way:
    - The orientation of the IMU is computed from the IMU itself and applying the transformation I_R_I_IMU
    - The expected orientation of the IMU is computed from the FK
    - The error is defined as log(I_R_FK * I_IMU_R_IMU.inverse())
    """

    def __init__(self, name: str, additional_output_folder: Path):
        super().__init__(
            name=name,
            additional_output_folder=additional_output_folder,
            signal_type=SignalType.orientation,
        )

    def run(self):
        kindyn = self.get_kindyn()
        gravity = np.array([0, 0, -blf.math.StandardAccelerationOfGravitation])

        I_T_base = idyn.Transform(self.base_link_orientation, idyn.Position.Zero())
        base_velocity = idyn.Twist.Zero()

        # Open the file and get the signal
        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            imu_signal_tmp = np.squeeze(
                np.array(root_variable[str(self.signal_type)][self.sensor_name]["data"])
            )

            # For the first sample compute the orientation of the frame and force the IMU inertial frame to be aligned with the base frame
            # In this context the following frames are defined:
            # I: inertial frame
            # I_IMU: inertial frame for the IMU
            # FK: frame where the IMU is attached for the forward kinematics
            # In details, for the first step we ask "I_R_FK = I_R_I_IMU * I_IMU_R_IMU"
            # where - I_R_FK can be computed using the forward kinematics
            #       - I_IMU_R_IMU is the orientation of the IMU in its inertial frame and it is retrieved from the imu itself

            kindyn.setRobotState(
                I_T_base,
                self.joint_state.positions[0, :],
                base_velocity,
                self.joint_state.velocities[0, :],
                gravity,
            )

            # Compute the orientation of the frame where the IMU is attached
            I_R_FK = kindyn.getWorldTransform(self.frame_name).getRotation()

            # We compute I_R_I_IMU = I_R_FK * I_IMU_R_IMU.inverse()
            I_R_I_IMU = (
                I_R_FK
                * (
                    idyn.Rotation.RPY(
                        imu_signal_tmp[0, 0], imu_signal_tmp[0, 1], imu_signal_tmp[0, 2]
                    )
                ).inverse()
            )

            # Resize the matrices
            self.imu_signal.resize((self.joint_state.positions.shape[0], 3))
            self.expected_imu_signal.resize((self.joint_state.positions.shape[0], 3))
            error = np.zeros((self.joint_state.positions.shape[0], 3))

            # Compute the orientation of the IMU for each sample
            for i in range(self.joint_state.positions.shape[0]):
                kindyn.setRobotState(
                    I_T_base,
                    self.joint_state.positions[i, :],
                    base_velocity,
                    self.joint_state.velocities[i, :],
                    gravity,
                )

                # The expected orientation of the IMU is computed from the FK
                expected_imu_signal_tmp = kindyn.getWorldTransform(
                    self.frame_name
                ).getRotation()

                self.expected_imu_signal[
                    i, :
                ] = expected_imu_signal_tmp.asRPY().toNumPy()

                # The orientation of the IMU is computed from the IMU itself and applying the transformation I_R_I_IMU
                imu_signal_temp = I_R_I_IMU * idyn.Rotation.RPY(
                    imu_signal_tmp[i, 0], imu_signal_tmp[i, 1], imu_signal_tmp[i, 2]
                )
                self.imu_signal[i, :] = imu_signal_temp.asRPY().toNumPy()

                # We define the error as log(I_R_FK * I_IMU_R_IMU.inverse())
                error[i, :] = (
                    (expected_imu_signal_tmp * imu_signal_temp.inverse())
                    .log()
                    .toNumPy()
                )

        fig, axs = plt.subplots(3, 2)
        axis_name = ["r", "p", "y"]
        fig.suptitle(self.name.replace("_", " "), fontsize=16)
        fig.set_size_inches(10.5, 6.5)

        # Plot the results
        for i in range(3):
            axs[i][0].set(ylabel="θ" + axis_name[i] + " (deg)")
            axs[i][0].plot(
                np.rad2deg(self.expected_imu_signal[:, i]), label="Kinematics"
            )
            axs[i][0].plot(np.rad2deg(self.imu_signal[:, i]), label="Sensor")
            axs[i][1].plot(np.rad2deg(error[:, i]), label="Error")
            axs[i][0].legend()
            axs[i][1].legend()

        # Save the figure
        (self.additional_output_folder / self.name).mkdir(parents=True, exist_ok=True)
        plt.savefig(
            str(self.additional_output_folder / self.name / (self.name + ".png"))
        )
        plt.close()

        # Compute the mean and the standard deviation of the error
        std = np.std(error, axis=0)
        mean = np.mean(error, axis=0)

        # Check if the error is within the accepted tolerance
        return (np.abs(mean) < self.accepted_mean_error).all() and (
            std < self.accepted_std_error
        ).all()


class AccTest(GenericImuSignalTest):
    def __init__(self, name: str, additional_output_folder: Path):
        super().__init__(
            name=name,
            additional_output_folder=additional_output_folder,
            signal_type=SignalType.accelerometer,
        )

    def run(self):
        kindyn = self.get_kindyn()
        gravity = np.array([0, 0, -blf.math.StandardAccelerationOfGravitation])

        I_T_base = idyn.Transform(self.base_link_orientation, idyn.Position.Zero())
        base_velocity = idyn.Twist.Zero()
        base_acceleration = idyn.Vector6()
        base_acceleration.zero()
        base_acceleration[2] = blf.math.StandardAccelerationOfGravitation

        with h5py.File(self.file_name, "r") as file:
            root_variable = file.get("robot_logger_device")
            self.imu_signal = np.squeeze(
                np.array(root_variable[str(self.signal_type)][self.sensor_name]["data"])
            )

        for i in range(self.joint_state.positions.shape[0]):
            kindyn.setRobotState(
                I_T_base,
                self.joint_state.positions[i, :],
                base_velocity,
                self.joint_state.velocities[i, :],
                gravity,
            )

            tmp = kindyn.getFrameAcc(
                self.frame_name, base_acceleration, self.joint_state.accelerations[i, :]
            ).toNumPy()
            self.expected_imu_signal[i, :] = tmp[:3]

        error = self.expected_imu_signal - self.imu_signal

        fig, axs = plt.subplots(3, 2)
        axis_name = ["x", "y", "z"]
        fig.suptitle(self.name.replace("_", " "), fontsize=16)
        fig.set_size_inches(10.5, 6.5)

        for i in range(3):
            axs[i][0].set(ylabel="x" + axis_name[i] + " (m/s^2)")
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
