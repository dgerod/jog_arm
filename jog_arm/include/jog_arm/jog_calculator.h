///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.h
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Blake Anderson, Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

// Server node for arm jogging with MoveIt.

#ifndef JOG_ARM_JOG_CACULATOR_H
#define JOG_ARM_JOG_CACULATOR_H

#include <Eigen/Eigenvalues>
#include <ros/ros.h>
#include <jog_msgs/JogJoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <jog_arm/jog_parameters.h>
#include <jog_arm/shared_variables.h>
#include <jog_arm/low_pass_filter.h>
#include <jog_arm/collision_checker.h>

namespace jog_arm
{
/**
 * Class JogCalcs - Perform the Jacobian calculations.
 */
class JogCalcs
{
public:
  JogCalcs(const std::string& name, const jog_arm_parameters& parameters, jog_arm_shared& shared_variables,
           const std::unique_ptr<robot_model_loader::RobotModelLoader>& model_loader_ptr);

protected:
  bool cartesianJogCalcs(const geometry_msgs::TwistStamped& cmd, jog_arm_shared& shared_variables);
  bool jointJogCalcs(const jog_msgs::JogJoint& cmd, jog_arm_shared& shared_variables);
  // Parse the incoming joint msg for the joints of our MoveGroup
  bool updateJoints();
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;
  Eigen::VectorXd scaleJointCommand(const jog_msgs::JogJoint& command) const;
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J) const;
  // This pseudoinverse calculation is more stable near stabilities. See Golub, 1965, "Calculating the Singular Values..."
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& u_matrix, const Eigen::MatrixXd& v_matrix, const Eigen::MatrixXd& s_diagonals) const;
  void enforceJointVelocityLimits(Eigen::VectorXd& calculated_joint_vel);
  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;
  // Reset the data stored in low-pass filters so the trajectory won't jump when
  // jogging is resumed.
  void resetVelocityFilters();
  // Avoid a singularity or other issue.
  // Needs to be handled differently for position vs. velocity control
  void halt(trajectory_msgs::JointTrajectory& jt_traj);
  void publishWarning(bool active) const;
  bool checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory_<std::allocator<void>>& new_jt_traj);
  // Possibly calculate a velocity scaling factor, due to proximity of
  // singularity and direction of motion
  double decelerateForSingularity(Eigen::MatrixXd jacobian, const Eigen::VectorXd commanded_velocity);
  // Apply velocity scaling for proximity of collisions and singularities
  bool applyVelocityScaling(jog_arm_shared& shared_variables, trajectory_msgs::JointTrajectory& new_jt_traj,
                            const Eigen::VectorXd& delta_theta, double singularity_scale);
  trajectory_msgs::JointTrajectory composeOutgoingMessage(sensor_msgs::JointState& joint_state,
                                                          const ros::Time& stamp) const;
  void lowPassFilterVelocities(const Eigen::VectorXd& joint_vel);
  void lowPassFilterPositions();
  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const;
  const robot_state::JointModelGroup* joint_model_group_;

  const std::string node_name_;
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  sensor_msgs::JointState incoming_jts_;

  robot_state::RobotStatePtr kinematic_state_;
  sensor_msgs::JointState jt_state_, original_jts_;
  trajectory_msgs::JointTrajectory new_traj_;
  tf::TransformListener listener_;
  std::vector<jog_arm::LowPassFilter> velocity_filters_;
  std::vector<jog_arm::LowPassFilter> position_filters_;
  ros::Publisher warning_pub_;
  jog_arm_parameters parameters_;
};

}

#endif