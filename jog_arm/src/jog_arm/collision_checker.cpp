///////////////////////////////////////////////////////////////////////////////
//      Title     : jog_arm_server.cpp
//      Project   : jog_arm
//      Created   : 3/9/2017
//      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
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

#include <jog_arm/collision_checker.h>

#include <memory>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <jog_arm/low_pass_filter.h>

using namespace jog_arm;

namespace {

void wait_until_ready(const std::string& log_name, const std::string& joint_state_topic,
                      robot_model_loader::RobotModelLoaderPtr& model_loader)
{
   while (ros::ok() && !model_loader)
    {
      ROS_WARN_THROTTLE_NAMED(5, log_name, "Waiting for a non-null robot_model_loader pointer");
      ros::Duration(0.1).sleep();
    }

    ROS_INFO_NAMED(log_name, "[CollisionCheckThread::CollisionCheckThread] Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic);
    ROS_INFO_NAMED(log_name, "[CollisionCheckThread::CollisionCheckThread] Received first joint msg.");
}

}

CollisionCheckThread::CollisionCheckThread(const std::string& log_name,
    const jog_arm_parameters& parameters, jog_arm_shared& shared_variables,
    robot_model_loader::RobotModelLoaderPtr& model_loader)
    : log_name_(log_name)
{
    ROS_INFO_NAMED(log_name_, "[CollisionCheckThread::CollisionCheckThread] Start collision checker? %d",
                   parameters.check_collisions);

    const std::string PLANNING_SCENE_TOPIC("/planning_scene");

    if (not parameters.check_collisions)
    {
        ROS_WARN_NAMED(log_name_, "Collision checker is disabled");
        return;
    }

    wait_until_ready(log_name_, parameters.joint_topic, model_loader);

    const robot_model::RobotModelPtr& kinematic_model = model_loader->getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = parameters.move_group_name;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(model_loader));
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();

    if (planning_scene_monitor->getPlanningScene())
    {
      planning_scene_monitor->startSceneMonitor(PLANNING_SCENE_TOPIC);
      planning_scene_monitor->startWorldGeometryMonitor();
      planning_scene_monitor->startStateMonitor();
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(log_name_, "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    double velocity_scale_coefficient = -log(0.001) / parameters.collision_proximity_threshold;
    ros::Rate collision_rate(parameters.collision_check_rate);

    while (ros::ok())
    {
        pthread_mutex_lock(&shared_variables.mutex);
        sensor_msgs::JointState jts = shared_variables.joints;
        pthread_mutex_unlock(&shared_variables.mutex);

        for (std::size_t i = 0; i < jts.position.size(); ++i)
        {
            current_state.setJointPositions(jts.name[i], &jts.position[i]);
        }

        collision_result.clear();
        planning_scene_monitor->getPlanningScene()->checkCollision(collision_request, collision_result, current_state);

        // Scale robot velocity according to collision proximity and user-defined thresholds.
        // I scaled exponentially (cubic power) so velocity drops off quickly after the threshold.
        // Proximity decreasing --> decelerate
        double velocity_scale = 1;

        // If we are far from a collision, velocity_scale should be 1.
        // If we are very close to a collision, velocity_scale should be ~zero.
        // When collision_proximity_threshold is breached, start decelerating exponentially.
        if (collision_result.distance < parameters.collision_proximity_threshold)
        {
            // velocity_scale = e ^ k * (collision_distance - threshold)
            // k = - ln(0.001) / collision_proximity_threshold
            // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
            // velocity_scale should equal 0.001 when collision_distance is at zero.
            velocity_scale =
                 exp(velocity_scale_coefficient * (collision_result.distance - parameters.collision_proximity_threshold));
        }

        pthread_mutex_lock(&shared_variables.mutex);
        shared_variables.collision_velocity_scale = velocity_scale;
        pthread_mutex_unlock(&shared_variables.mutex);

        collision_rate.sleep();
    }
}
