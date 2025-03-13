# ROS2 driver for TSND121 IMU sensor

This dirver will config the sensor into "USB connected measure mode" and decode readout data via USB.

Decoded data will publish to topics `/sensor/tsnd121/Imu`(w/o quaternion), `/sensor/tsnd121/MagneticField`, `/sensor/tsnd121/AirTemperature` and `/sensor/tsnd121/AirPressure`

## Installation

Clone the driver code from repository.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/iDifferent-studio/tsnd2imu_ros2.git
```

Build the code.

```
cd ..
colcon build --packages-select tsnd2imu_ros2
```

## Run the node

Connect the TSND121 sensor to your computer via USB.

Long push the power button until the sensor ring to turn the sensor power on.

Run the command to run the node.

```
source install/setup.bash
ros2 run tsnd2imu_ros2 tsnd_imu_pub
```

If all gose well, the sensor will ring two times to indicate the sensor is starting sending out data.
