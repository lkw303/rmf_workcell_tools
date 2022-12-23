// Copyright 2022 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pseudo_workcells/pseudo_workcells.hpp"

int main(int argc, char* argv[])
{   
    // consider creating a lifecycle node here just to extract
    // params instead of the constructor of the pseudo workcell
    rclcpp::init(argc, argv);
    auto workcells = std::make_shared<pseudo_workcells::PseudoWorkcells>("pseudo_workcells");
    workcells->init_workcells();
    workcells->spin_all();
    rclcpp::shutdown();
    return 0;
}