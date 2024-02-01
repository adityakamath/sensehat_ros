# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import numpy as np
from typing import Optional
import rclpy
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, Temperature, MagneticField, Joy, FluidPressure as Pressure, RelativeHumidity as Humidity
from std_msgs.msg import Header, ColorRGBA
from sense_hat import SenseHat

class SenseHatPublisher(Node):
    def __init__(self, node_name='sensehat'):
        super().__init__(node_name)

        # declare publishers and timer
        self._imu_pub: Optional[Publisher] = None
        self._mag_pub: Optional[Publisher] = None
        self._pressure_pub: Optional[Publisher] = None
        self._humidity_pub: Optional[Publisher] = None
        self._temp_p_pub: Optional[Publisher] = None
        self._temp_h_pub: Optional[Publisher] = None
        self._joy_pub: Optional[Publisher] = None
        self._color_pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None

        # declare parameters and default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('en_imu', True),
                ('en_mag', True),
                ('en_pressure', True),
                ('en_humidity', True),
                ('en_temp', True),
                ('en_joy', True),
                ('en_color', True),
                ('frame_id', 'sensehat_frame'),
                ('timer_period', 0.02),  
                ('imu_transform', True),
                ('joy_once', True),
                ('color_gain', 60),
                ('color_int_cycles', 64),
            ]
        )

        # define global info, declare global objects and variables
        self._pub_info = [
            ('imu', Imu),
            ('mag', MagneticField),
            ('pressure', Pressure),
            ('temp_p', Temperature),
            ('humidity', Humidity),
            ('temp_h', Temperature),
            ('joy', Joy),
            ('color', ColorRGBA),
        ]
        self._joy_mapping = ['up', 'down', 'left', 'right', 'middle']
        self._time = self.get_clock().now().to_msg()
        self._sensehat = SenseHat()
        self._sensehat.set_rotation(180)
        
        self.get_logger().info('Initialized')

    def timer_callback(self):
        self._time = self.get_clock().now().to_msg()
        
        for index, (pub_name, msg_type) in enumerate(self._pub_info):
            # check if publisher exists and is activated
            if getattr(self, f'_{pub_name}_pub', None) and getattr(self, f'_{pub_name}_pub', None).is_activated:
                
                # create sensor message and populate header message 
                # (except for color message type which doesn't have a header field)
                sensor_msg = msg_type()
                
                if pub_name != 'color':
                    sensor_msg.header = Header(stamp = self._time, frame_id = self.get_parameter('frame_id').value)

                try:
                    # imu: accelerometer + gyroscope + magnetometer
                    if pub_name == 'imu':
                        gyro_raw = self._sensehat.get_gyroscope_raw() # Units: rad/s
                        acc_raw = self._sensehat.get_accelerometer_raw() # Units: g
                        orientation = self._sensehat.get_orientation_radians() # Euler angles: roll, pitch, yaw in radians
                        quat = quaternion_from_euler(orientation['roll'], orientation['pitch'], orientation['yaw'])

                        # Populate ENU messages according to REP-103. Convert from NED to ENU if needed.
                        if self.get_parameter('imu_transform').value:
                            # Convert NED to ENU: 
                            #     body-fixed NED → ROS ENU: (x y z)→(x -y -z) or (x y z w)→(x -y -z w)
                            #     local      NED → ROS ENU: (x y z)→(y x -z)  or (x y z w)→(y x -z w)
                            orientation = Quaternion(x=quat[1], y=quat[0], z=-quat[2], w=quat[3])
                            ang_vel = Vector3(x=gyro_raw['y'], y=gyro_raw['x'], z=-gyro_raw['z'])
                            lin_acc = Vector3(x=acc_raw['y']*9.81, y=acc_raw['x']*9.81, z=-acc_raw['z']*9.81) # convert from g to m/s^2
                        else:
                            # No conversion needed
                            orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                            ang_vel = Vector3(x=gyro_raw['x'], y=gyro_raw['y'], z=gyro_raw['z'])
                            lin_acc = Vector3(x=acc_raw['x']*9.81, y=acc_raw['y']*9.81, z=acc_raw['z']*9.81) # convert from g to m/s^2

                        sensor_msg.orientation = orientation
                        sensor_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                        sensor_msg.angular_velocity = ang_vel
                        sensor_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                        sensor_msg.linear_acceleration = lin_acc
                        sensor_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

                    # magnetometer
                    elif pub_name == 'mag':
                        mag_raw = self._sensehat.get_compass_raw()
                        if self.get_parameter('imu_transform').value:
                            # Convert NED to ENU according to REP-103
                            sensor_msg.magnetic_field = Vector3(x=mag_raw['y']/1e6, y=mag_raw['x']/1e6, z=-mag_raw['z']/1e6) # convert from uT to T
                        else:
                            # No conversion needed
                            sensor_msg.magnetic_field = Vector3(x=mag_raw['x']/1e6, y=mag_raw['y']/1e6, z=mag_raw['z']/1e6) # convert from uT to T

                    # barometer
                    elif pub_name == 'pressure': 
                        sensor_msg.fluid_pressure = self._sensehat.get_pressure() * 100.0 # convert from mBar to Pa

                    # hygrometer
                    elif pub_name == 'humidity':
                        sensor_msg.relative_humidity = self._sensehat.get_humidity() / 100.0 # convert [0%, 100%] to [0.0, 1.0]

                    # temperature from barometer
                    elif pub_name == 'temp_p':
                        sensor_msg.temperature = float(self._sensehat.get_temperature_from_pressure()) # degrees Celcius

                    # temperature from hygrometer
                    elif pub_name == 'temp_h':
                        sensor_msg.temperature = float(self._sensehat.get_temperature_from_humidity()) # degrees Celcius

                    # joystick button values
                    elif pub_name == 'joy':
                        sensor_msg.buttons = [0, 0, 0, 0, 0]
                        for event in self._sensehat.stick.get_events():
                            # check joy_once parameter - if true: check if joy was 'pressed', if false: check if 'held'
                            if event.action == ('pressed' if self.get_parameter('joy_once').value else 'held'):
                                sensor_msg.buttons = [event.direction == direction for direction in self._joy_mapping]

                    # color sensor values
                    elif pub_name == 'color':
                        sensor._sensehat.color.gain = self.get_parameter('color_gain').value
                        sensor._sensehat.color.integration_cycles = self.get_parameter('color_int_cycles').value

                    # publish message
                    getattr(self, f'_{pub_name}_pub', None).publish(sensor_msg)
                    
                except OSError:
                    self.get_logger().info(f'Failed to initialize sensor for {pub_name} data.')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            # read parameter server and set local variables (convention = l_{pub_name})
            l_imu = self.get_parameter('en_imu').value
            l_mag = self.get_parameter('en_mag').value
            l_pressure = self.get_parameter('en_pressure').value
            l_humidity = self.get_parameter('en_humidity').value
            l_temp = self.get_parameter('en_temp').value
            l_temp_p = l_temp and l_pressure
            l_temp_h = l_temp and l_humidity
            l_joy = self.get_parameter('en_joy').value
            l_color = self.get_parameter('en_color').value and self._sensehat.has_colour_sensor()

            # if temperature is enabled but both humidity and pressure sensors are disabled, issue warning
            if l_temp and not l_pressure and not l_humidity:
                self.get_logger().warn('Warning: Temperature enabled but Humidity/Pressure sensors disabled. Enable at least one to publish Temperature.')
            
            # configure sensor
            self._sensehat.set_imu_config(True, True, True) # set magnetometer, gyroscope, accelerometer to true on hardware

            # create timer
            self._timer = self.create_timer(self.get_parameter('timer_period').value, self.timer_callback)

            # create publishers
            for pub_name, msg_type in self._pub_info:
                if locals().get(f'l_{pub_name}'):
                    setattr(self, f'_{pub_name}_pub', self.create_lifecycle_publisher(msg_type, pub_name, qos_profile=qos_profile_sensor_data))
            
            self.get_logger().info('Configured')
            return TransitionCallbackReturn.SUCCESS
            
        except (OSError):
            self.get_logger().info('Configuration Failure: Unable to access Sense HAT')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activated')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:      
        self.get_logger().info('Deactivate')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.terminate()      
        self.get_logger().info('Clean Up Successful')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.terminate()
        self.get_logger().info('Shut Down Successful')
        return TransitionCallbackReturn.SUCCESS
        
    def terminate(self):        
        # destroy timer
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)

        # destroy publishers 
        for pub_name, msg_type in self._pub_info:
            pub = getattr(self, f'_{pub_name}_pub', None)
            if pub is not None:
                self.destroy_publisher(pub)

def main(args=None):
    rclpy.init(args=args)
    node = SenseHatPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.terminate()
        node.destroy_node()

if __name__ == '__main__':
    main()
