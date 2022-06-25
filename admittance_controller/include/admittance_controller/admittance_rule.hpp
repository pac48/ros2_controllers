// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_

#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include "tf2_kdl/tf2_kdl/tf2_kdl.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "control_toolbox/filters.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "config/admittance_struct.h"

// kinematics plugins
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"


namespace {  // Utility namespace

// Numerical accuracy checks. Used as deadbands.
  static constexpr double WRENCH_EPSILON = 1e-10;
  static constexpr double POSE_ERROR_EPSILON = 1e-12;
  static constexpr double POSE_EPSILON = 1e-15;
  const double ROT_AXIS_EPSILON = 0.001;


}  // utility namespace

namespace admittance_controller {

  class AdmittanceRule {
  public:
    AdmittanceRule() = default;

    controller_interface::return_type configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int num_joint);
    controller_interface::return_type reset();

    /**
     * Calculate 'desired joint states' based on the 'measured force' and 'reference joint state'.
     *
     * \param[in] current_joint_state
     * \param[in] measured_wrench
     * \param[in] reference_joint_state
     * \param[in] period
     * \param[out] desired_joint_state
     */
    controller_interface::return_type update(
        const trajectory_msgs::msg::JointTrajectoryPoint &current_joint_state,
        const geometry_msgs::msg::Wrench &measured_wrench,
        const trajectory_msgs::msg::JointTrajectoryPoint &reference_joint_state,
        const rclcpp::Duration &period,
        trajectory_msgs::msg::JointTrajectoryPoint &desired_joint_states);

    control_msgs::msg::AdmittanceControllerState
    get_controller_state( control_msgs::msg::AdmittanceControllerState & state_message);


  public:
    // TODO(destogl): Add parameter for this
//    bool use_feedforward_commanded_input_ = true;

    // Dynamic admittance config
    std::shared_ptr<admittance_struct_parameters::admittance_struct> parameters_;

    // Filter parameter for exponential smoothing
//    const double alpha = 0.005; // TODO make a ros param


  protected:
    /**
     * All values are in the controller frame
     */
    void calculate_admittance_rule(
        const Eigen::Matrix<double,3,2> &wrench,
        const Eigen::Matrix<double,3,2> &desired_vel,
        const double dt
    );

    void process_wrench_measurements(
        const geometry_msgs::msg::Wrench &measured_wrench, const Eigen::Matrix<double, 3, 3>& sensor_rot, const Eigen::Matrix<double, 3, 3>& cog_rot
    );

    Eigen::Vector3d get_rotation_axis(const Eigen::Matrix3d& R) const;
    void convert_cartesian_deltas_to_joint_deltas(const std::vector<double>& positions,
                                          const Eigen::Matrix<double, 3,2> & cartesian_delta, std::vector<double>& joint_delta, bool & success);
    Eigen::Matrix<double, 3, 2>  convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &positions,
                                                                  const std::vector<double> &joint_delta,
                                                                  bool &success);
    void normalize_rotation(Eigen::Matrix<double,3,3,Eigen::ColMajor>& R);
    Eigen::Matrix<double,4,4,Eigen::ColMajor> invert_transform(Eigen::Matrix<double,4,4,Eigen::ColMajor> &T);
    Eigen::Matrix<double,4,4,Eigen::ColMajor> get_transform(const std::vector<double>& positions, const std::string & link_name, bool & success);
    void eigen_to_msg(const Eigen::Matrix<double, 3, 2>& wrench, const std::string& frame_id, geometry_msgs::msg::WrenchStamped& wrench_msg);
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor>
    find_transform(const std::string &target_frame, const std::string &source_frame, bool &success);

    // Kinematics interface plugin loader
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> kinematics_loader_;
    std::unique_ptr<kinematics_interface::KinematicsBaseClass> kinematics_;

    // number of robot joint
    int num_joints_;

    // buffers to pass data to kinematics interface
    std::vector<double> transform_buffer_vec;
    std::vector<double> joint_buffer_vec;
    std::vector<double> cart_buffer_vec;

    // admittance controller values
    Eigen::Matrix<double,3,2> admittance_acceleration_;
    Eigen::Matrix<double,3,2> admittance_velocity_;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> admittance_position_;

    // transforms
    Eigen::Matrix<double,4,4,Eigen::ColMajor> ee_transform;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> reference_ee_transform;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> sensor_transform;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> control_transform;
    Eigen::Matrix<double,4,4,Eigen::ColMajor> cog_transform;
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> base_link_transform_;

    // external force
    Eigen::Matrix<double,3,2> wrench_;
    // position of center of gravity in cog_frame
    Eigen::Matrix<double,3,1> cog_;
    // force applied to sensor due to weight of end effector
    Eigen::Matrix<double,3,1> ee_weight;

    // admittance controller values in joint space
    std::vector<double> joint_vel;
    std::vector<double> joint_acc;
    std::vector<double> joint_pos;

    std::vector<double> damping_;
    std::vector<double> mass_;
    std::vector<bool> selected_axes_;
    std::vector<double> stiffness_;

    // ROS
    trajectory_msgs::msg::JointTrajectoryPoint admittance_rule_calculated_values_;
    control_msgs::msg::AdmittanceControllerState state_message_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  };

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_HPP_
