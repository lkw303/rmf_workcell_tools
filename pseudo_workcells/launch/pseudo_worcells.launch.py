# Copyright 2022 ROS-Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ingestor_list = DeclareLaunchArgument(
        'ingestors',
        default_value="[\
            'product_dropoff_1',\
            'stock_holding_1', \
            'stock_holding_2',\
            'stock_holding_3',\
            'stock_holding_4',\
        ]"
    )

    dispenser_list = DeclareLaunchArgument(
        'dispensers', 
        default_value="[ \
            'storage_1',  \
            'storage_2', \
            'cnc_1', \
            'cnc_2', \
            'cnc_3' \
        ]"
    )

    _parameters=[{
        'ingestors': LaunchConfiguration('ingestors'),
        'dispensers': LaunchConfiguration('dispensers'),
        }]

    node = Node(
        package='pseudo_workcells',
        executable='pseudo_workcells',
        name='pseudo_workcells',
        parameters=_parameters
    )

    return LaunchDescription([
        ingestor_list,
        dispenser_list,
        node,
    ])
