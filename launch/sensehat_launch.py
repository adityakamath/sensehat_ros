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

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    ld = LaunchDescription()
    
    sensehat_params_path = PathJoinSubstitution(
        [FindPackageShare("sensehat_ros"), "config", "sensor_params.yaml"])

    ns_arg = DeclareLaunchArgument(
        name='ns',
        default_value='', # Do not change, else config params and remappings need to be updated
        description='Namespace of the system')
        
    frame_id_arg = DeclareLaunchArgument(
        name='frame_id',
        default_value='sensehat_frame',
        description='Frame ID of the Sense HAT')

    ld.add_action(ns_arg)
    ld.add_action(frame_id_arg)

    # launch lifecycle node
    sensehat_node = LifecycleNode(
        package='sensehat_ros',
        executable='sensehat_node',
        name='sensehat',
        namespace=LaunchConfiguration('ns'), 
        output='screen',
        parameters=[
            sensehat_params_path,
            {'frame_id': LaunchConfiguration('frame_id')}]) # this overwrites the config file param

    # emit configure event
    emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher = matches_action(sensehat_node),
            transition_id = Transition.TRANSITION_CONFIGURE))

    # when inactive state is reached, emit activate event
    register_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sensehat_node, goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher = matches_action(sensehat_node),
                        transition_id = Transition.TRANSITION_ACTIVATE))]))

    ld.add_action(sensehat_node)
    ld.add_action(emit_configure_event)
    ld.add_action(register_activate_handler)

    # publish the imu_frame to base_link transform
    sensehat_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensehat_base_link_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', LaunchConfiguration('frame_id'), 'base_link'])  # Adjust the transform as needed

    ld.add_action(sensehat_tf)

    return ld