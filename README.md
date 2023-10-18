# 🧪 xcub-sensors-check 🧪

xcub-sensor-check is a repository containing a set of of scripts to check the status of the sensors of the xCub robot (e.g., iCub, ergoCub).

## 🚀 Installation
Create a conda enviroment with the following command:

```bash
conda create -n xcub-sensors-check-env
```
and activate it with
```bash
conda activate xcub-sensors-check-env
```

Then, install the required packages with
```bash
conda install -c conda-forge bipedal-locomotion-framework resolve-robotics-uri-py h5py matplotlib scipy
```

## 📝 Usage
To use the application, run the following command:
```bash
python -m xcub_sensors_check --config <path_to_config_file> -o output
```

`output` represents the folder where the results of the tests will be stored. If not specified, the results will be stored in the current folder. While the `path_to_config_file` is the path to the configuration file.
The configuration file is required to configure the tests to be performed. Currently the following tests are available:
- `OrientationTest`: checks the orientation of a sensor with respect to a frame. The test is performed by comparing the mean and the standard deviation of the orientation of the sensor with respect to the frame with the expected values. The test is passed if the mean and the standard deviation of the error are below the specified thresholds.
- `GyroTest`: checks the angular velocity of a sensor with respect to a frame. The test is performed by comparing the mean and the standard deviation of the angular velocity of the sensor with respect to the frame with the expected values. The test is passed if the mean and the standard deviation of the error are below the specified thresholds.
- `AccTest`: checks the linear acceleration of a sensor with respect to a frame. The test is performed by comparing the mean and the standard deviation of the linear acceleration of the sensor with respect to the frame with the expected values. The test is passed if the mean and the standard deviation of the error are below the specified thresholds.

The follow example shows how to configure the tests 4 tests for the ergoCubSN000 robot. In detail we are checking the orientation of the front and rear feet and the angular velocity of the front and rear feet.

Here it is worth to note the following parameters:
- `test_list`: contains the list of the tests to be performed. The name of the tests must match the name of the sections (i.e., groups) in the configuration file.
- `model_package_path`: contains the path to the model package of the robot. This is required to load the model of the robot. You can find further information about the model package [here](https://github.com/ami-iit/resolve-robotics-uri-py)
- `dataset_path`: the relative path of a dataset collected with [`YarpRobotLoggerDevice`](https://github.com/ami-iit/bipedal-locomotion-framework/tree/7723da13e4ddab9ebafe53df1daacf2e04719126/devices/YarpRobotLoggerDevice)

Then for each test a set of parameters must be specified:
- `type`: the type of the test. Currently the following tests are available: `OrientationTest` and `GyroTest`
- `sensor_name`: the name of the sensor to be tested. The name must match the name of the sensor in the model of the robot.
- `frame_name`: the name of the frame with respect to which the sensor is tested. The name must match the name of the frame in the model of the robot.
- `error_mean_tolerance`: the tolerance on the mean of the error. The test is passed if the mean of the error is below the specified threshold.
- `error_std_tolerance`: the tolerance on the standard deviation of the error. The test is passed if the standard deviation of the error is below the specified threshold.

```toml
test_list = ["L_FOOT_FRONT_ORIENTATION", "L_FOOT_FRONT_GYRO", "L_FOOT_FRONT_ACC"]

model_package_path = "package://ergoCub/robots/ergoCubSN000/model.urdf"
dataset_path = "robot_logger_device_2023_10_16_11_39_22.mat"
compute_joint_velocity_from_position = true
compute_joint_acceleration_from_position = true
velocity_svg_window_length = 31
acceleration_svg_window_length = 31

[L_FOOT_FRONT_ORIENTATION]
type = "OrientationTest"
sensor_name = "l_foot_front_ft_eul"
frame_name = "l_foot_front_ft"
error_mean_tolerance = [0.1, 0.1, 0.1]
error_std_tolerance = [0.1, 0.1, 0.1]

[L_FOOT_FRONT_GYRO]
type = "GyroTest"
sensor_name = "l_foot_front_ft_gyro"
frame_name = "l_foot_front_ft"
error_mean_tolerance = [0.1, 0.1, 0.1]
error_std_tolerance = [0.1, 0.1, 0.1]

[L_FOOT_FRONT_ACC]
type = "AccTest"
sensor_name = "l_foot_front_ft_acc"
frame_name = "l_foot_front_ft"
error_mean_tolerance = [0.5, 0.5, 0.5]
error_std_tolerance = [0.5, 0.5, 0.5]
```

## 📊 Output
This a sample output of the application:
```
🧪 Running 3 tests...
├── Running test L_FOOT_FRONT_ORIENTATION...
├── Running test L_FOOT_FRONT_GYRO...
└── Running test L_FOOT_FRONT_ACC...

📊 Test outcomes:
├── 🔴 Test L_FOOT_FRONT_ORIENTATION: Failed
├── 🟢 Test L_FOOT_FRONT_GYRO: Passed
└── 🟢 Test L_FOOT_FRONT_ACC: Passed
```
Further details related to each test can be found in the `output` folder.

## 📜 License
The code is released under the [BSD 3-Clause License](./LICENSE).
