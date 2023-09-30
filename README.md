# sensehat_ros
ROS 2 package for the Raspberry Pi [Sense HAT](https://www.raspberrypi.com/documentation/accessories/sense-hat.html) which includes an accelerometer, a gyroscope, a magnetometer, a barometric pressure sensor, a humidity sensor, a 5-button joystick and a 8x8 LED matrix. The [Sensor HAT v2](https://www.raspberrypi.com/products/sense-hat/) also includes a color and brightness sensor. This package provides support for both versions.

Note: This implementation is a bit over-engineered, as I have been experimenting with ROS 2 [managed/lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes, [executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html#executors) and [composition](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/composed.py) using Python.

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

Some notes:
1. The IMU needs a calibration file for accurate and reliable results. This needs to be saved in the default calibration file location as defined in the [python-sense-hat](https://github.com/astro-pi/python-sense-hat) library: ```~/.config/sense_hat/RTIMULib.ini```. This default location cannot be changed.
2. This node only reads the sensors and buttons and publishes data as ROS 2 topics. This node does not touch the LED matrix at all, it is controlled using a separate node.
3. Each publisher uses the [Sensor Data QoS profile](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-profiles) as default.
4. This implementation is designed as a lifecycle component and can be run individually as well.


* ```sensehat_led_display```: This executable subscribes to several topics (like IMU and joystick messages from ```sensehat_publisher``` or elsewhere) and displays relevant measurements on the 8x8 LED matrix. This is still a work in progress.

* ```sensehat_node```: This executable creates instances of ```sensehat_publisher``` and ```sensehat_led_display``` and runs them both using a single threaded executor. 

* ```sensehat_launch.py```: This is the launch file that launches ```sensehat_node``` as a  lifecycle node, loads its parameters, and then configures and activates it. The lifecycle node is first initialized, then set to 'configure' from the launch file. When the 'inactive' state is reached, the registered event handler activates the node. This launch file has the following arguments:
    * ```ns```: Namespace of the system (default: ```''```)
    * ```frame_id```: Frame ID of the Sense HAT (default: ```sensehat_frame```)

## Parameters

* ```odom_topic```: Odometry topic name (Default: odom)
* ```timer_period```: Timer period in seconds (Default: 0.01)
* ```sensor_timeout```: Sensor timeout in seconds in case of no movement, or sensor failure (Default: 1.0)
* ```parent_frame```: Parent frame for the Odometry message and Transform (Default: odom)
* ```child_frame```: Child frame for the Odometry message and transform (Default: base_link)
* ```x_init```: Initial position in the X axis, in meters (Default: 0.0)
* ```y_init```: Initial position in the Y axis, in meters (Default: 0.0)
* ```z_height```: Height of the sensor from the ground, in meters (Default: 0.025)
* ```board```: Sensor type - pmw3901 or paa5100 (Default: paa5100)
* ```fov_deg```: FOV of the sensor aperture in degrees (Default: 42 for PMW3901/PAA5100)
* ```res_px```: Resolution of the sensor in pixels, assuming the same value in both directions (Default: 35 for PMW3901/PAA5100)
* ```scaler```: Scaling factor, i.e. the sensor value returned for 1 pixel move (Default: 5)
* ```spi_nr```: SPI port number (Default: 0)
* ```spi_slot```: SPI CS pin - front (BCM pin 7 on RPi) or back (BCM pin 8 on RPi) (Default: front)
* ```rotation```: Rotation of the sensor in 90 degree increments - 0, 90, 180, 270 (Default: 270)
* ```publish_tf```: Boolean value to turn transform publisher on/off (Default: true)

## How to use

* Stack the Sense HAT on to the RPi GPIO 
* Follow the [Raspberry Pi Setup](https://gist.github.com/adityakamath/63eacf890381f9428f822742d49255c8) and the [Sense HAT Setup](https://gist.github.com/adityakamath/897d1933b3fe9ec5b7d388aabb7de9ef), forked from the [EnvTrackerNode](https://github.com/J-Pai/EnvTrackerNode) repository (I made a copy just in case the documentation in this repo is updated) 
* Clone this repository in a ROS 2 workspace. Check the ```sensor_params.yaml``` file in the config directory and ```sensehat_launch.py```, and make any necessary changes.
* Calibrate the IMU using the [RTIMULibCal tool](https://github.com/RPi-Distro/RTIMULib/tree/master/Linux/RTIMULibCal) ([RTIMULib](https://github.com/RPi-Distro/RTIMULib) must already be cloned to your device from the Sense HAT Setup steps from earlier). Follow the steps in the tool and generate a ```RTIMULib.ini``` calibration file. OR you can use my calibration file, which is located in the config folder.
* Copy this calibration file to ```~/.config/sense_hat/RTIMULib.ini```. If this path does not exist, simply create it or run the launch file which will generate a default calibration file in this location.
* Build the package and run the launch file: ```ros2 launch sensehat_ros sensehat_launch.py```
* Launch arguments can be added like this: ```ros2 launch sensehat_ros sensehat_launch.py frame_id:='sensehat2'```

## Results

This package was tested using a [Sense HAT v1](https://sense-emu.readthedocs.io/en/v1.0/api.html) and a Raspberry  Pi 4 (4GB) running ROS 2 Humble on Ubuntu22.04 with a real-time kernel. The following observations were made:
* The output frequency of 50Hz was achieved with all sensors enabled
* Humidity and Pressure sensors work as expected, including temperature measurements. Temperature readings are not ambient temperature, since the RPi also heats up - but both temperature readings were observed to be nearly identical.
* IMU was calibrated using [RTIMULibCal](https://github.com/RPi-Distro/RTIMULib/tree/master/Linux/RTIMULibCal) and the calibration file was copied to the default location: ```~/.config/sense_hat/RTIMULib.ini```
* Only 1 of the 5 buttons of the Joystick work - but I have used the Sense HAT roughly in the past, so I guess it is human error, not a software bug
* The Joystick button that works (right button), also triggers some LEDs in the corner of the matrix (the corner closest to the joystick). I don't think it is human error this time, it could be a software bug
