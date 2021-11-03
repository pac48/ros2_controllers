// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_SPECIALIZATIONS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_SPECIALIZATIONS_HPP_

#include <string>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace position_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


/**
 * \brief
 */
class JointTrajectoryController : joint_trajectory_controller::JointTrajectoryController
{
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  JointTrajectoryController();

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  CallbackReturn on_init(const std::string & controller_name) override;
};

}  // namespace position_controllers

#endif  // JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_SPECIALIZATIONS_HPP_
