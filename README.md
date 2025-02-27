# RBF GNSS INS Driver

## Overview

The ROBINS/GNSS INS System ROS2 driver is a software component designed to interface with a Global Navigation Satellite System (GNSS) module combined with an Inertial Navigation System (INS). The driver enables ROS2-based applications to access and utilize the raw GNSS and INS data for localization, navigation, and other related tasks.

#### Author: [Robeff Technology](https://www.robeff.com)
#### Maintainer: [Robeff Technology](mailto:support@robeff.com)

## Table of Contents

- [Features](#features)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Install the RBF GNSS INS ROS2 Driver](#install-the-rbf-gnss-ins-ros2-driver)
- [Configuration File](#configuration-file)
- [Usage](#usage)
- [Customizing Configuration](#customizing-configuration-optional)
- [Troubleshooting](#troubleshooting)
- [Custom Messages](#custom-messages)
- [Standard Messages](#standard-messages)
- [Subscribed Topic](#subscribed-topic)
- [Contributing](#contributing)
- [License](#license)

## Features

- **ROS2 Compatibility**: Fully compatible with the ROS2 (Robot Operating System 2) ecosystem.
- **GNSS Data**: Reads raw data from the GNSS module, including satellite positions, timestamps, position, velocity, and other relevant information.
- **INS Data**: Retrieves data from the Inertial Navigation System, such as roll, pitch, and yaw angles.
- **6-DOF IMU Data**: Provides data from the built-in [ADIS16470](https://www.analog.com/media/en/technical-documentation/data-sheets/adis16470.pdf) 6-DOF IMU.
- **Fused Output**: Fuses GNSS and INS data using sensor fusion techniques (e.g., Extended Kalman Filter) for robust localization estimates.
- **Configurable Parameters**: Allows customization of various parameters to adapt to different GNSS/INS modules and user requirements.

## Installation

### Prerequisites

Before proceeding with the installation, ensure you have the following prerequisites:

1. **ROS2 Humble**: Ensure you have a working ROS2 Humble installation. Follow the [official installation instructions](https://docs.ros.org/en/humble/Installation.html) if needed.
2. **Build Tools**: Ensure you have the necessary build tools and dependencies installed on your system.

### Install the RBF GNSS INS ROS2 Driver

1. **Clone the Repository**: Clone the repository into your ROS2 workspace's source directory:
    ```sh
    cd /path_to_your_ros2_workspace/src
    git clone https://github.com/Robeff-Technology/rbf_gnss_ins_driver.git
    ```     
    **Clone ntrip driver Repository(Optional)**:
    ```sh
    git clone https://github.com/Robeff-Technology/rbf_ntrip_driver.git
    ```
2. **Instal Dependencies**: Use `rosdep` to install the required dependencies for the project:
     ```sh
    cd /path_to_your_ros2_workspace
    rosdep install --from-paths src --ignore-src -r -y
     ```

2. **Build the Workspace**: Navigate to your ROS2 workspace and build the packages:
    ```sh
    cd /path_to_your_ros2_ws
    colcon build --packages-select rbf_gnss_ins_driver
    ```  
3. **Source the Workspace**: Source your ROS2 workspace to make the newly built RBF GNSS INS driver node available:
    ```sh
    source /path_to_your_ros2_ws/install/setup.bash
    ```

    ## Configuration File

The `rbf_gnss_ins_driver.param.yaml` configuration file allows customization of the driver's behavior. Below is a table explaining each parameter:

| Parameter                    | Description                                      | Default Value                 |
|------------------------------|--------------------------------------------------|-------------------------------|
| **working_frequency**        | The working frequency of the driver.              | 200                           |
| **serial_config**            |                                                  |                               |
| ├─ `port`                    | Current port name of the device                  | `/dev/ttyUSB0`                |
| └─ `baudrate`                | Current baudrate of the device                   | 460800                        |
| **topic_config**             |                                                  |                               |
| ├─ `rtcm_topic`              | RTCM topic name                                  | `/sensing/gnss/ntrip/rtcm`    |
| ├─ `imu_topic`               | IMU topic name                                   | `robins/ros/imu`              |
| ├─ `nav_sat_fix_topic`       | NavSatFix topic name                             | `robins/ros/gps_nav_sat_fix`  |
| ├─ `twist_topic`             | TwistWithCovarianceStamped topic name            | `robins/ros/twist_with_covariance_stamped` |
| └─ `temperature_topic`       | Temperature topic name                           | `robins/ros/temperature`      |
| **frame_config**             |                                                  |                               |
| ├─ `gnss_frame`              | GNSS frame name                                  | `gnss_ins_link`               |
| └─ `imu_frame`               | IMU frame name                                   | `imu_link`                    |
| **time_config**              |                                                  |                               |
| └─ `use_ros_time`            | Use ROS time if true (not recommended)           | false                         |
| **odometry_config**          |                                                  |                               |
| ├─ `use_odometry`            | Use odometry if true                             | false                         |
| ├─ `odometry_topic`          | Odometry topic name                              | `robins/ros/odometry`         |
| └─ `odometry_frame`          | Odometry frame name                              | `odom`                        |
| **origin_config**            |                                                  |                               |
| ├─ `latitude`                | Latitude of the origin                           | 0.0                           |
| ├─ `longitude`               | Longitude of the origin                          | 0.0                           |
| └─ `altitude`                | Altitude of the origin                           | 0.0                           |
| **altitude_config**          |                                                  |                               |
| └─ `altitude_mode`           | Altitude mode (0: orthometric, 1: ellipsoid)     | 0                             |

## Usage

1. **Launch the RBF GNSS INS Driver Node**: Start the RBF GNSS INS driver node using the provided launch file. Run the following command:
    ```sh
    ros2 launch rbf_gnss_ins_driver gnss_ins_driver.launch.xml
    ```
    This will start the driver node and configure it based on the default parameters.

2. **Verify Data**: After launching the driver, verify that the GNSS and INS data are being published to the correct topics using tools like `ros2 topic echo`.

## Customizing Configuration (Optional)

To customize the behavior of the GNSS/INS driver:

1. **Locate the Configuration File**: The `gnss_ins.param.yaml` configuration file is in the package's `config` directory.
2. **Customize Parameters**: Edit the configuration file to modify the driver's behavior. Adjust parameters such as communication settings, INS settings, and output topics.
3. **Launch with Custom Configuration**: Launch the driver node with the updated configuration:
    ```sh
      ros2 launch rbf_gnss_ins_driver gnss_ins_driver.launch.xml
    ```
4. **Launch with 'rbf_ntrip_driver**: This driver works in coordination with the [rbf_ntrip_driver](https://github.com/Robeff-Technology/rbf_ntrip_driver#) to provide RTCM correction data via NTRIP. You can launch it using:
    ```sh
    ros2 launch rbf_gnss_ins_driver gnss_ins_driver_with_ntrip_client.launch.xml
    ```

## Troubleshooting

- **Build Errors**: Ensure all dependencies are properly installed and sourced.
- **Permissions**: Ensure the user has the necessary permissions to access the GNSS/INS module (e.g., `/dev/ttyUSB0`).
- **Driver-specific Issues**: Refer to the documentation or contact the developer for specific issues.

## Custom Messages

### `ECEF`
- Position and velocity in ECEF.

### `GnssStatus`
- Reports several status indicators, including differential age.

### `GnssVel`
- Best available GNSS velocity without INS, including status indicators.

### `Heading`
- Heading information of the receiver in motion.

### `ImuStatus`
- IMU status indicator.

### `Ins`
- Integrated navigation results and deviations.

### `RTCMStatus`
- Status of RTCM topic for debugging.

## Standard Messages

### `imu/Temp` [sensor_msgs/Temperature](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html)
- Temperature of the IMU. Requires `RAWIMU`.

### `TwistWithCovarianceStamped` [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)
- Velocity of the vehicle calculated with EKF. Requires `INSPVAX`.
- `/raw/ecef_twist`: Velocity of the vehicle in ECEF. Requires `ECEF`.

### `NavSatFix` [sensor_msgs/NavSatFix](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html)
- Position of the vehicle calculated with EKF. Requires `INSPVAX`.
- `/raw/nav_sat_fix`: Position of the vehicle without EKF. Requires `ECEF`.

### `Imu` [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
- IMU data calculated with EKF. Requires `RAWIMU`, `INSPVAX`.
- `/raw/imu`: IMU data without EKF. Requires `RAWIMU`.

### `Odometry` [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)
- Odometry data. Requires `INSPVAX`.

## Subscribed Topic

### `RTCM` [mavros_msgs::msg::RTCM](https://docs.ros.org/en/api/mavros_msgs/html/msg/RTCM.html)
- RTCM data for RTK. For more information, see [ntrip_client](https://github.com/Robeff-Technology/rbf_ntrip_driver).

## Contributing

If you would like to contribute to this project, please contact [Robeff Technology](mailto:support@robeff.com) or submit a pull request on the [GitHub repository](https://github.com/Robeff-Technology/rbf_gnss_ins_driver).

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.