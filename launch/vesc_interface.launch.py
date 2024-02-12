# Copyright 2023 Amadeusz Szymko
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    vesc_param_file = LaunchConfiguration('vesc_interface_param_file').perform(context)
    vehicle_param_file = LaunchConfiguration('vehicle_param_file').perform(context)
    if not vesc_param_file:
        vesc_param_file = PathJoinSubstitution(
            [FindPackageShare('vesc_interface'), 'config', 'vesc_interface.param.yaml']
        ).perform(context)
    if not vehicle_param_file:
        vehicle_param_file = PathJoinSubstitution(
            [FindPackageShare('f1tenth_vehicle_description'), 'config', 'vehicle_info.param.yaml']
        ).perform(context)

    vesc_interface_node = Node(
        package='vesc_interface',
        executable='vesc_interface_node_exe',
        name='vesc_interface',
        namespace='vesc',
        parameters=[
            vesc_param_file,
            vehicle_param_file
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    return [
        vesc_interface_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('vesc_interface_param_file', '')
    add_launch_arg('vehicle_param_file', '')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
