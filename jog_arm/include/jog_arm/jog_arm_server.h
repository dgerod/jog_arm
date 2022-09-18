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

#ifndef JOG_ARM_SERVER_H
#define JOG_ARM_SERVER_H

#include <Eigen/Eigenvalues>
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
#include <jog_arm/jog_calculator.h>

namespace jog_arm
{

class JogROSInterface
{
public:
  JogROSInterface(const std::string& name);

private:
  void deltaCartesianCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void deltaJointCmdCB(const jog_msgs::JogJointConstPtr& msg);
  void jointsCB(const sensor_msgs::JointStateConstPtr& msg);

  bool readParameters(ros::NodeHandle& n);

  static void* jogCalcThread(void* thread_id);
  static void* CollisionCheckThread(void* thread_id);

  static std::string node_name_;
  static struct jog_arm_parameters ros_parameters_;
  static struct jog_arm_shared shared_variables_;
  static std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr_;
};

}

#endif