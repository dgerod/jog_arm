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

#ifndef JOG_ARM_SHARED_VARIABLES_H
#define JOG_ARM_SHARED_VARIABLES_H

#include <moveit/robot_state/robot_state.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <jog_msgs/JogJoint.h>

namespace jog_arm
{

struct jog_arm_shared
{
  geometry_msgs::TwistStamped command_deltas;
  pthread_mutex_t command_deltas_mutex;

  jog_msgs::JogJoint joint_command_deltas;
  pthread_mutex_t joint_command_deltas_mutex;

  sensor_msgs::JointState joints;
  pthread_mutex_t joints_mutex;

  double collision_velocity_scale = 1;
  pthread_mutex_t collision_velocity_scale_mutex;

  // Indicates that an incoming Cartesian command is all zero velocities
  bool zero_cartesian_cmd_flag = true;
  pthread_mutex_t zero_cartesian_cmd_flag_mutex;

  // Indicates that an incoming joint angle command is all zero velocities
  bool zero_joint_cmd_flag = true;
  pthread_mutex_t zero_joint_cmd_flag_mutex;

  // Indicates that we have not received a new command in some time
  bool command_is_stale = false;
  pthread_mutex_t command_is_stale_mutex;

  // The new trajectory which is calculated
  trajectory_msgs::JointTrajectory new_traj;
  pthread_mutex_t new_traj_mutex;

  // Timestamp of incoming commands
  ros::Time incoming_cmd_stamp = ros::Time(0.);
  pthread_mutex_t incoming_cmd_stamp_mutex;

  bool ok_to_publish = false;
  pthread_mutex_t ok_to_publish_mutex;
};

}

#endif