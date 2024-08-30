# sensehat_ros
![](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue) ![GitHub License](https://img.shields.io/github/license/adityakamath/sensehat_ros)
 ![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)
 
ROS 2 package for the Raspberry Pi [Sense HAT](https://www.raspberrypi.com/documentation/accessories/sense-hat.html) which includes an accelerometer, a gyroscope, a magnetometer, a barometric pressure sensor, a humidity sensor, a 5-button joystick and a 8x8 LED matrix. The [Sensor HAT v2](https://www.raspberrypi.com/products/sense-hat/) also includes a color and brightness sensor. This package provides support for both versions.

Note: This implementation is a bit over-engineered, as I have been experimenting with ROS 2 [managed/lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes using Python.

## Implementation details

* ```sensehat_publisher```: This executable uses the [python-sense-hat](https://github.com/astro-pi/python-sense-hat) library to access data from the Sense HAT over multiple I2C channels. This node publishes messages from the IMU sensors (accelerometer, gyroscope, magnetometer), the magnetometer only, the pressure sensor, the humidity sensor, and the 5-button joystick respectively. Additionally, it uses the pressure sensor and humidity sensor to calculate the ambient temperature and publishes them as topics. Finally, if the Sense HAT v2 is used, the color sensor measurements are also published. Each sensor/publisher can be enabled or disabled using boolean parameters. When everything is enabled, the following topics are published:

    * ```/imu```: IMU (accelerometer + gyroscope + magnetometer) readings [converted from NED to ENU](https://github.com/mavlink/mavros/issues/49) if needed - [sensor_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) message type
    * ```/mag```: Magnetometer readings converted to Teslas [converted from NED to ENU](https://github.com/mavlink/mavros/issues/49) if needed - [sensor_msgs/msg/MagneticField](https://docs.ros2.org/foxy/api/sensor_msgs/msg/MagneticField.html) message type
    * ```/pressure```: Pressure sensor readings converted to Pascals - [sensor_msgs/msg/FluidPressure](https://docs.ros2.org/foxy/api/sensor_msgs/msg/FluidPressure.html) message type
    * ```/humidity```: Humidity sensor readings converted from percentage to the range [0.0, 1.0] - [sensor_msgs/msg/RelativeHumidity](https://docs.ros2.org/foxy/api/sensor_msgs/msg/RelativeHumidity.html) message type
    * ```/temp_p```: Temperature readings from the pressure sensor in degrees Celcius - [sensor_msgs/msg/Temperature](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html) message type
    * ```/temp_h```: Temperature readings from the humidity sensor in degrees Celcius - [sensor_msgs/msg/Temperature](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html) message type
    * ```/joy```: 5-button joystick readings as an array of buttons - [sensor_msgs/msg/Joy](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) message type
    * ```/color```: Color sensor readings in the form of RGBA, each in the range [0, 256] - [std_msgs/msg/ColorRGBA](https://docs.ros2.org/foxy/api/std_msgs/msg/ColorRGBA.html) message type

* ```sensehat_display_handler```: This executable provides a handler for displaying different images/animations based on different subscribed topics on the 8x8 LED matrix. This is still a work in progress.

* ```sensehat_launch.py```: This is the launch file that launches ```sensehat_publisher``` as a  lifecycle node, loads its parameters, and then configures and activates it. The lifecycle node is first initialized, and then set to 'configure' from the launch file. When the 'inactive' state is reached, the registered event handler activates the node. This launch file has the following arguments:

    * ```ns```: Namespace of the system (default: ```''```)
    * ```frame_id```: Frame ID of the Sense HAT (default: ```sensehat_frame```)
    * ```config_path```: Path to the config file, if called from another package/launch file (default: ```config/sensehat_config.yaml```)

## Parameters

* ```en_imu```: Enable IMU publisher - Gyroscope, Accelerometer, Magnetometer (Default: ```True```)
* ```en_mag```: Enable Magnetometer publisher (Default: ```True```)
* ```en_pressure```: Enable Pressure publisher (Default: ```True```)
* ```en_humidity```: Enable Humidity publisher (Default: ```True```)
* ```en_joy```: Enable Joystick publisher (Default: ```False```)
* ```en_color```: Enable Color publisher, if v2 (Default: ```False```)
* ```frame_id```: Sense HAT Frame ID (Default: ```sensehat_frame```)
* ```timer_period```: Time period of the Timer in seconds (Default: ```0.02```)
* ```imu_transform```: Transform IMU coordinate system from NED to ENU (Default: ```True```)
* ```joy_once```: If True, sets Joy output once when pressed. If False, sets Joy output continuously as the button is pressed. (Default: ```True```)
* ```color_gain```:  Sets the sensitivity of the Color sensor [1, 4, 16, 60] (Default: ```60```)
* ```color_int_cycles```: Sets the time that the the sensor takes between measuring the light times 2.4 milliseconds [1, 256] (Default: ```64```)

## How to use

* Stack the Sense HAT onto the RPi GPIO 
* Follow the [Raspberry Pi Setup](https://gist.github.com/adityakamath/63eacf890381f9428f822742d49255c8) and the [Sense HAT Setup](https://gist.github.com/adityakamath/897d1933b3fe9ec5b7d388aabb7de9ef), forked from the [EnvTrackerNode](https://github.com/J-Pai/EnvTrackerNode) repository (I made a copy just in case the documentation in this repo is updated) 
* Clone this repository in a ROS 2 workspace. Check the ```sensor_params.yaml``` file in the config directory and ```sensehat_launch.py```, and make any necessary changes.
* Calibrate the IMU using the [RTIMULibCal tool](https://github.com/RPi-Distro/RTIMULib/tree/master/Linux/RTIMULibCal) ([RTIMULib](https://github.com/RPi-Distro/RTIMULib) must already be cloned to your device from the Sense HAT Setup steps from earlier). Follow the [Hardware Calibration](https://www.raspberrypi.com/documentation/accessories/sense-hat.html#hardware-calibration) steps and generate a ```RTIMULib.ini``` calibration file. OR you can use my calibration file, which is located in the config folder.
* Copy this calibration file to ```~/.config/sense_hat/RTIMULib.ini```. If this path does not exist, simply create it or run the launch file which will generate a default calibration file in this location.
* Build the package and run the launch file: ```ros2 launch sensehat_ros sensehat_launch.py```
* Launch arguments can be added like this: ```ros2 launch sensehat_ros sensehat_launch.py frame_id:='sensehat'```
