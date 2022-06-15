// Copyright (c) 2022, PickNik, Inc.
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
/// \author: Andy Zelenak, Paul Gesel


#include <rclcpp/rclcpp.hpp>
#include "kdl_parser/kdl_parser.hpp"

#include "kdl/chainfksolverpos_recursive.hpp"
#include "kinematics_plugin/kinematics_plugin.h"

namespace kinematics_plugin {

    rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_plugin");

    bool Kinematics::initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string &end_effector_name) {
        end_effector_name_ = end_effector_name;

        // get robot description
        node->declare_parameter("robot_description", "");
        auto robot_param = rclcpp::Parameter();
        if (!node->get_parameter("robot_description", robot_param)){
            RCLCPP_ERROR(LOGGER, "node parameter robot_description not set");
            return false;
        }
        auto robot_description = robot_param.as_string();

        // get alpha damping term
        node->declare_parameter("alpha", 0.005);
        auto alpha_param = rclcpp::Parameter();
        node->get_parameter("alpha", alpha_param);
        alpha = alpha_param.as_double();

        // create kinematic chain
        KDL::Tree robot_tree;
        kdl_parser::treeFromString(robot_description, robot_tree);
        auto root_name = robot_tree.getRootSegment()->first;
        if (!robot_tree.getChain(root_name, end_effector_name, chain_)){
            RCLCPP_ERROR(LOGGER, "failed to find chain from robot root %s to end effector %s",
                         root_name.c_str(), end_effector_name.c_str());
            return false;
        }

        for (auto i = 0u; i < chain_.getNrOfSegments(); i++){
            name_segment_map_[chain_.getSegment(i).getName()] = i;
        }

        fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_shared<KDL::TreeJntToJacSolver>(robot_tree);
        jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);

        // create joint array
        num_joints_ = chain_.getNrOfJoints();
        q_ = KDL::JntArray(num_joints_);

        return true;
    }


    bool Kinematics::convert_joint_deltas_to_cartesian_deltas(std::vector<double> &delta_theta_vec,
                                                              std::vector<double> &delta_x_vec) {

        auto delta_theta_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(delta_theta_vec.data(), delta_theta_vec.size());

        // calculate jacobian from last set state
        jac_solver_->JntToJac(q_, *jacobian_, end_effector_name_);

        auto J = jacobian_->data;
        Eigen::VectorXd  delta_x = J * delta_theta_;

        std::vector<double> delta_x_v(&delta_x[0], delta_x.data() + delta_x.cols() * delta_x.rows());
        delta_x_vec = delta_x_v;

        return true;
    }

    bool Kinematics::convert_cartesian_deltas_to_joint_deltas(std::vector<double> &delta_x_vec,
                                                              std::vector<double> &delta_theta_vec) {

       // create eigen vec type from input
        auto delta_x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(delta_x_vec.data(), delta_x_vec.size());

        // calculate jacobian from last set state
        jac_solver_->JntToJac(q_, *jacobian_, end_effector_name_);

        auto J = jacobian_->data;

        auto I = Eigen::MatrixXd(num_joints_, num_joints_);
        I.setIdentity();
        // damped inverse
        auto J_inverse = (J.transpose()*J + alpha*I).inverse()*J.transpose();

        Eigen::VectorXd  delta_theta = J_inverse * delta_x;

        std::vector<double> delta_theta_v(&delta_theta[0], delta_theta.data() + delta_theta.cols() * delta_theta.rows());
        delta_theta_vec = delta_theta_v;

        return true;
    }

    bool Kinematics::calculate_segment_transform(std::vector<double> &transform_vec, const std::string & segment_name) {

        fk_pos_solver_->JntToCart(q_, frame_, name_segment_map_[segment_name]);
        transform_vec[0] = frame_.p.x();
        transform_vec[1] = frame_.p.y();
        transform_vec[2] = frame_.p.z();
        for (int i=0; i < 9; i++){
            transform_vec[i+3] = frame_.M.data[i];
        }

        return true;
    }

    bool Kinematics::update_robot_state(const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
    {
        // maybe need to check if init was called yet
        if (current_joint_state.positions.size() != num_joints_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Vector size mismatch in update_robot_state()");
            return false;
        }

        q_ = convert_vector_to_kdl_joint_array(current_joint_state.positions);

        return true;
    }

    KDL::JntArray Kinematics::convert_vector_to_kdl_joint_array(std::vector<double> vec){
        KDL::JntArray joint_array;
        joint_array.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
        return {joint_array};
    }

    KDL::JntArrayVel Kinematics::convert_vector_to_kdl_joint_array_vel(std::vector<double> vec){
        KDL::JntArray joint_array;
        joint_array.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
        return KDL::JntArrayVel(joint_array);
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kinematics_plugin::Kinematics,
                       kinematics_interface::KinematicsBaseClass)