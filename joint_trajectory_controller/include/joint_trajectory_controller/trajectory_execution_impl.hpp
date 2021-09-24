// Copyright (c) 2021 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_EXECUTION_IMPL_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_EXECUTION_IMPL_HPP_

// TODO: Clean up includes
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_trajectory_controller/tolerances.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "joint_trajectory_controller/visibility_control.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_trajectory_controller
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_execution_impl");

class TrajectoryExecutionImpl
{
protected:
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;
  using JointLimiter = joint_limits::JointLimiterInterface<joint_limits::JointLimits>;

  bool reset()
  {
    subscriber_is_active_ = false;
    joint_command_subscriber_.reset();

    // iterator has no default value
    // prev_traj_point_ptr_;
    traj_point_active_ptr_ = nullptr;
    traj_external_point_ptr_.reset();
    traj_home_point_ptr_.reset();
    traj_msg_home_ptr_.reset();

    return true;
  }

  void check_for_new_trajectory(
    std::vector<std::string> joint_names,
    std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>>
      joint_command_interface)
  {
    // Check if a new external message has been received from nonRT threads
    auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
    auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
    if (current_external_msg != *new_external_msg)
    {
      fill_partial_goal(*new_external_msg, joint_names, joint_command_interface);
      sort_to_local_joint_order(*new_external_msg, joint_names);
      traj_external_point_ptr_->update(*new_external_msg);
    }
  }

  bool have_trajectory()
  {
    return traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg();
  }

  void set_point_before_trajectory_msg(
    bool open_loop_control, rclcpp::Time now,
    trajectory_msgs::msg::JointTrajectoryPoint state_current,
    trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state)
  {
    // If we will be sampling for the first time, prefix the trajectory with the current state
    // currently carrying out a trajectory
    if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
    {
      // if sampling the first time, set the point before you sample
      if (!(*traj_point_active_ptr_)->is_sampled_already())
      {
        if (open_loop_control)
        {
          (*traj_point_active_ptr_)->set_point_before_trajectory_msg(now, last_commanded_state);
        }
        else
        {
          (*traj_point_active_ptr_)->set_point_before_trajectory_msg(now, state_current);
        }
      }
    }
  }

  bool sample_trajectory(
    const rclcpp::Time & now, trajectory_msgs::msg::JointTrajectoryPoint & state_desired,
    TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr,
    const std::unique_ptr<joint_limits::JointLimiterInterface<joint_limits::JointLimits>> &
      joint_limiter = nullptr)
  {
    return (*traj_point_active_ptr_)
      ->sample(now, state_desired, start_segment_itr, end_segment_itr, joint_limiter);
  }

  bool is_before_last_point(TrajectoryPointConstIter & end_segment_itr)
  {
    return end_segment_itr != (*traj_point_active_ptr_)->end();
  }

  void perform_action_server_update(
    bool before_last_point, bool abort, bool outside_state_tolerance, double goal_time_tolerance,
    rclcpp::Time now, std::vector<std::string> joint_names,
    trajectory_msgs::msg::JointTrajectoryPoint state_current,
    trajectory_msgs::msg::JointTrajectoryPoint state_desired,
    trajectory_msgs::msg::JointTrajectoryPoint state_error,
    TrajectoryPointConstIter start_segment_itr)
  {
    const auto active_goal = *rt_active_goal_.readFromRT();
    if (active_goal)
    {
      // send feedback
      auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
      feedback->header.stamp = now;
      feedback->joint_names = joint_names;

      feedback->actual = state_current;
      feedback->desired = state_desired;
      feedback->error = state_error;
      active_goal->setFeedback(feedback);

      // check abort
      if (abort || outside_state_tolerance)
      {
        auto result = std::make_shared<FollowJTrajAction::Result>();

        if (abort)
        {
          RCLCPP_WARN(LOGGER, "Aborted due to state tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
        }
        else if (outside_state_tolerance)
        {
          RCLCPP_WARN(LOGGER, "Aborted due to goal tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
        }
        active_goal->setAborted(result);
        // TODO(matthew-reynolds): Need a lock-free write here
        // See https://github.com/ros-controls/ros2_controllers/issues/168
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

        // check goal tolerance
      }
      else if (!before_last_point)
      {
        if (!outside_state_tolerance)
        {
          auto res = std::make_shared<FollowJTrajAction::Result>();
          res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
          active_goal->setSucceeded(res);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

          RCLCPP_INFO(LOGGER, "Goal reached, success!");
        }
        else if (goal_time_tolerance != 0.0)
        {
          // if we exceed goal_time_toleralance set it to aborted
          const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
          const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

          const double difference = now.seconds() - traj_end.seconds();
          if (difference > goal_time_tolerance)
          {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            RCLCPP_WARN(
              LOGGER, "Aborted due goal_time_tolerance exceeding by %f seconds", difference);
          }
        }
      }
    }
  }

  rclcpp_action::GoalResponse goal_callback(
    // const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
    const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal,
    const controller_interface::ControllerInterface * controller_interface,
    bool allow_partial_joints_goal, std::vector<std::string> joint_names,
    bool allow_integration_in_goal_trajectories, rclcpp::Node::SharedPtr node)
  {
    RCLCPP_INFO(LOGGER, "Received new action goal");

    // Precondition: Running controller
    if (
      controller_interface->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Can't accept new action goals. Controller is not running. It is " << controller_interface->get_state().label());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!validate_trajectory_msg(
          goal->trajectory, allow_partial_joints_goal, joint_names,
          allow_integration_in_goal_trajectories, node->now()))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    // TODO(denis): is here the following line missing?
    //   add_new_trajectory_msg(std::make_shared(goal->trajectory));

    RCLCPP_INFO(LOGGER, "Accepted new action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
  {
    RCLCPP_INFO(LOGGER, "Got request to cancel goal");

    // Check that cancel request refers to currently active goal (if any)
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle)
    {
      // Controller uptime
      // Enter hold current position mode
      set_hold_position();

      RCLCPP_DEBUG(LOGGER, "Canceling active action goal because cancel callback received.");

      // Mark the current goal as canceled
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void feedback_setup_callback(
    // std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle,
    rclcpp::Node::SharedPtr node, std::vector<std::string> joint_names,
    rclcpp::Duration action_monitor_period)
  {
    // Update new trajectory
    {
      preempt_active_goal();
      auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
        goal_handle->get_goal()->trajectory);

      add_new_trajectory_msg(traj_msg);
    }

    // Update the active goal
    RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
    rt_goal->preallocated_feedback_->joint_names = joint_names;
    rt_goal->execute();
    rt_active_goal_.writeFromNonRT(rt_goal);

    // Setup goal status checking timer
    goal_handle_timer_ = node->create_wall_timer(
      action_monitor_period.to_chrono<std::chrono::seconds>(),
      std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  }

  void create_action_server(
    rclcpp::Node::SharedPtr node,
    const controller_interface::ControllerInterface * controller_interface,
    double action_monitor_rate, bool allow_partial_joints_goal,
    std::vector<std::string> joint_names, bool allow_integration_in_goal_trajectories)
  {
    RCLCPP_INFO(LOGGER, "Action status changes will be monitored at %.2f Hz.", action_monitor_rate);
    rclcpp::Duration action_monitor_period =
      rclcpp::Duration::from_seconds(1.0 / action_monitor_rate);

    //   using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(),
      std::string(node->get_name()) + "/follow_joint_trajectory",
      std::bind(
        &TrajectoryExecutionImpl::goal_callback, this, std::placeholders::_1, std::placeholders::_2,
        controller_interface, allow_partial_joints_goal, joint_names,
        allow_integration_in_goal_trajectories, node),
      std::bind(&TrajectoryExecutionImpl::cancel_callback, this, std::placeholders::_1),
      std::bind(
        &TrajectoryExecutionImpl::feedback_setup_callback, this, std::placeholders::_1, node,
        joint_names, action_monitor_period));
  }

  void release_action_server() {
    action_server_.reset();
  }

  void fill_partial_goal(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg,
    std::vector<std::string> joint_names,
    std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>>
      joint_command_interface) const
  {
    // joint names in the goal are a subset of existing joints, as checked in goal_callback
    // so if the size matches, the goal contains all controller joints
    if (joint_names.size() == trajectory_msg->joint_names.size())
    {
      return;
    }

    trajectory_msg->joint_names.reserve(joint_names.size());

    for (auto index = 0ul; index < joint_names.size(); ++index)
    {
      {
        if (
          std::find(
            trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
            joint_names[index]) != trajectory_msg->joint_names.end())
        {
          // joint found on msg
          continue;
        }
        trajectory_msg->joint_names.push_back(joint_names[index]);

        for (auto & it : trajectory_msg->points)
        {
          // Assume hold position with 0 velocity and acceleration for missing joints
          it.positions.push_back(joint_command_interface[0][index].get().get_value());
          if (!it.velocities.empty())
          {
            it.velocities.push_back(0.0);
          }
          if (!it.accelerations.empty())
          {
            it.accelerations.push_back(0.0);
          }
          if (!it.effort.empty())
          {
            it.effort.push_back(0.0);
          }
        }
      }
    }
  }

  void sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg,
    std::vector<std::string> joint_names)
  {
    // rearrange all points in the trajectory message based on mapping
    std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, joint_names);
    auto remap = [this](
                   const std::vector<double> & to_remap,
                   const std::vector<size_t> & mapping) -> std::vector<double> {
      if (to_remap.empty())
      {
        return to_remap;
      }
      if (to_remap.size() != mapping.size())
      {
        RCLCPP_WARN(LOGGER, "Invalid input size (%zu) for sorting", to_remap.size());
        return to_remap;
      }
      std::vector<double> output;
      output.resize(mapping.size(), 0.0);
      for (auto index = 0ul; index < mapping.size(); ++index)
      {
        auto map_index = mapping[index];
        output[map_index] = to_remap[index];
      }
      return output;
    };

    for (auto index = 0ul; index < trajectory_msg->points.size(); ++index)
    {
      trajectory_msg->points[index].positions =
        remap(trajectory_msg->points[index].positions, mapping_vector);

      trajectory_msg->points[index].velocities =
        remap(trajectory_msg->points[index].velocities, mapping_vector);

      trajectory_msg->points[index].accelerations =
        remap(trajectory_msg->points[index].accelerations, mapping_vector);

      trajectory_msg->points[index].effort =
        remap(trajectory_msg->points[index].effort, mapping_vector);
    }
  }

  bool validate_trajectory_point_field(
    size_t joint_names_size, const std::vector<double> & vector_field,
    const std::string & string_for_vector_field, size_t i, bool allow_empty) const
  {
    if (allow_empty && vector_field.empty())
    {
      return true;
    }
    if (joint_names_size != vector_field.size())
    {
      RCLCPP_ERROR(
        LOGGER, "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.", joint_names_size,
        string_for_vector_field.c_str(), vector_field.size(), i);
      return false;
    }
    return true;
  }

  bool validate_trajectory_msg(
    const trajectory_msgs::msg::JointTrajectory & trajectory, bool allow_partial_joints_goal,
    std::vector<std::string> joint_names, bool allow_integration_in_goal_trajectories,
    rclcpp::Time now) const
  {
    // If partial joints goals are not allowed, goal should specify all controller joints
    if (!allow_partial_joints_goal)
    {
      if (trajectory.joint_names.size() != joint_names.size())
      {
        RCLCPP_ERROR(LOGGER, "Joints on incoming trajectory don't match the controller joints.");
        return false;
      }
    }

    if (trajectory.joint_names.empty())
    {
      RCLCPP_ERROR(LOGGER, "Empty joint names on incoming trajectory.");
      return false;
    }

    const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
    // If the starting time it set to 0.0, it means the controller should start it now.
    // Otherwise we check if the trajectory ends before the current time,
    // in which case it can be ignored.
    if (trajectory_start_time.seconds() != 0.0)
    {
      auto trajectory_end_time = trajectory_start_time;
      for (const auto & p : trajectory.points)
      {
        trajectory_end_time += p.time_from_start;
      }
      if (trajectory_end_time < now)
      {
        RCLCPP_ERROR(
          LOGGER,
          "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
          trajectory_start_time.seconds(), trajectory_end_time.seconds());
        return false;
      }
    }

    for (auto i = 0ul; i < trajectory.joint_names.size(); ++i)
    {
      const std::string & incoming_joint_name = trajectory.joint_names[i];

      auto it = std::find(joint_names.begin(), joint_names.end(), incoming_joint_name);
      if (it == joint_names.end())
      {
        RCLCPP_ERROR(
          LOGGER, "Incoming joint %s doesn't match the controller's joints.",
          incoming_joint_name.c_str());
        return false;
      }
    }

    rclcpp::Duration previous_traj_time(rclcpp::Duration::from_seconds(0));
    for (auto i = 0ul; i < trajectory.points.size(); ++i)
    {
      if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
      {
        RCLCPP_ERROR(
          LOGGER,
          "Time between points %zu and %zu is not strictly increasing, it is %f and %f "
          "respectively",
          i - 1, i, previous_traj_time.seconds(),
          rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
        return false;
      }
      previous_traj_time = trajectory.points[i].time_from_start;

      const size_t joint_count = trajectory.joint_names.size();
      const auto & points = trajectory.points;
      // TODO(anyone): This currently supports only position, velocity and acceleration inputs
      if (allow_integration_in_goal_trajectories)
      {
        const bool all_empty = points[i].positions.empty() && points[i].velocities.empty() &&
                               points[i].accelerations.empty();
        const bool position_error =
          !points[i].positions.empty() &&
          !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false);
        const bool velocity_error = !points[i].velocities.empty() &&
                                    !validate_trajectory_point_field(
                                      joint_count, points[i].velocities, "velocities", i, false);
        const bool acceleration_error =
          !points[i].accelerations.empty() &&
          !validate_trajectory_point_field(
            joint_count, points[i].accelerations, "accelerations", i, false);
        if (all_empty || position_error || velocity_error || acceleration_error)
        {
          return false;
        }
      }
      else if (
        !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
        !validate_trajectory_point_field(
          joint_count, points[i].velocities, "velocities", i, true) ||
        !validate_trajectory_point_field(
          joint_count, points[i].accelerations, "accelerations", i, true) ||
        // TODO(denis): should this be deleted, since effort goals are not supported?
        !validate_trajectory_point_field(joint_count, points[i].effort, "effort", i, true))
      {
        return false;
      }
    }
    return true;
  }

  void add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
  {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
  }

  void preempt_active_goal()
  {
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
      action_res->set__error_string("Current goal cancelled due to new incoming action.");
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }

  void set_hold_position()
  {
    trajectory_msgs::msg::JointTrajectory empty_msg;
    empty_msg.header.stamp = rclcpp::Time(0);

    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg);
    add_new_trajectory_msg(traj_msg);
  }

  void resize_joint_trajectory_point(
    trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size,
    bool has_velocity_state_interface, bool has_acceleration_state_interface)
  {
    point.positions.resize(size);
    if (has_velocity_state_interface)
    {
      point.velocities.resize(size);
    }
    if (has_acceleration_state_interface)
    {
      point.accelerations.resize(size);
    }
  }

  // TODO(karsten1987): eventually activate and deactivate subscriber directly when its supported
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_ =
    nullptr;

  std::shared_ptr<Trajectory> * traj_point_active_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_home_point_ptr_ = nullptr;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg_home_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;

  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;

  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
};

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__TRAJECTORY_EXECUTION_IMPL_HPP_
