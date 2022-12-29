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
from launch_ros.actions import  Node


def generate_launch_description():
    ingestor_list = [
        'product_dropoff_1',
        'product_dropoff_2',
        'stock_holding_1',
        'stock_holding_2',
        'stock_holding_3',
        'stock_holding_4',
    ]

    dispenser_list = [
        'storage_1_dispenser',
        'storage_2_dispenser',
        'cnc_1_dispenser',
        'cnc_2_dispenser',
        'cnc_3_dispenser'
    ]

    node_list = []
    # create dispenser nodes
    for dispenser in dispenser_list:

        _parameters = [{
            "workcell_name": dispenser
        }]
        node = Node(
            package='pseudo_workcells',
            executable='pseudo_dispenser',
            name=dispenser,
            parameters=_parameters
        )

        node_list.append(node)

    # create ingestor nodes
    for ingestor in ingestor_list:

        _parameters = [{
            "workcell_name": ingestor
        }]
        node = Node(
            package='pseudo_workcells',
            executable='pseudo_ingestor',
            name=ingestor,
            parameters=_parameters
        )

        node_list.append(node)

        node = Node(
            package='pseudo_workcells',
            executable='pseudo_dispenser',
            name=ingestor,
            parameters=_parameters
        )
        node_list.append(node)

    return LaunchDescription(node_list)
