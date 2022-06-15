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

#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

#include "kinematics_interface/kinematics_interface_plugin_base.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/treejnttojacsolver.hpp"


namespace kinematics_plugin
{

    class Kinematics : public kinematics_interface::KinematicsBaseClass
    {
    public:
        /**
         * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
         * It uses the Jacobian from MoveIt.
         */
        bool initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & group_name);


        /**
         * \brief Calculates the joint transform specified by segment name using the last set robot state.
         * \param transform_vec output vector with three element of the segment's position and 9 elements of
         * the segments rotation matrix in column major format.
         * \param segment_name the name of the segment to find the transform for
         * \return true if successful
         */
        bool
        calculate_segment_transform(std::vector<double> & transform_vec, const std::string & segment_name);

        /**
         * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
         * \param delta_x_vec input Cartesian deltas (x, y, z, rx, ry, rz)
         * \param control_frame_to_ik_base transform the requested delta_x to MoveIt's ik_base frame
         * \param delta_theta_vec output vector with joint states
         * \return true if successful
         */
        bool
        convert_cartesian_deltas_to_joint_deltas(
                std::vector<double> & delta_x_vec,
                std::vector<double> & delta_theta_vec);

        /**
         * \brief Convert joint delta-theta to Cartesian delta-x, using the Jacobian.
         * \param[in] delta_theta_vec vector with joint states
         * \param[in] tf_ik_base_to_desired_cartesian_frame transformation to the desired Cartesian frame. Use identity matrix to stay in the ik_base frame.
         * \param[out] delta_x_vec  Cartesian deltas (x, y, z, rx, ry, rz)
         * \return true if successful
         */
        bool
        convert_joint_deltas_to_cartesian_deltas(
                std::vector<double> &  delta_theta_vec,
                std::vector<double> & delta_x_vec);

        bool update_robot_state(const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state);

    private:
        /** \brief Possibly calculate a velocity scaling factor, due to proximity of
         * singularity and direction of motion
      //   */
        static static KDL::JntArray convert_vector_to_kdl_joint_array(std::vector<double> vec);
        KDL::JntArrayVel convert_vector_to_kdl_joint_array_vel(std::vector<double> vec);
//        void assign_vector_from_kdl_frame_vel(const KDL::FrameVel & frame_vel, std::vector<double>& vec);

        std::string end_effector_name_;
        size_t num_joints_;
        KDL::Chain chain_;
        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
        KDL::JntArray q_;
        KDL::JntArrayVel q_vel_;
        KDL::Frame frame_;
        std::shared_ptr<KDL::Jacobian> jacobian_;
        std::shared_ptr<KDL::TreeJntToJacSolver> jac_solver_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::unordered_map<std::string, int> name_segment_map_;
        double alpha; // damping term for Jacobian inverse
    };

}  // namespace moveit_differential_ik_plugin
