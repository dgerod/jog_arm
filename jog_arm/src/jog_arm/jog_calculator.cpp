#include <jog_arm/jog_calculator.h>
#include <jog_arm/jog_arm_server.h>

using namespace jog_arm;

namespace {

static const int GAZEBO_REDUNTANT_MESSAGE_COUNT = 30;

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

JogCalculatorWorker::JogCalculatorWorker(const std::string& name, const jog_arm_parameters& parameters, jog_arm_shared& shared_variables,
                                         robot_model_loader::RobotModelLoaderPtr& model_loader)
  : move_group_(parameters.move_group_name)
  , log_name_(name)
  , parameters_(parameters)
{
  ROS_INFO_NAMED(log_name_, "[JogCalcs::JogCalcs] BEGIN");

  wait_until_ready(log_name_, parameters.joint_topic, model_loader);

  // Publish collision status
  warning_pub_ = nh_.advertise<std_msgs::Bool>(parameters_.warning_topic, 1);

  const robot_model::RobotModelPtr& kinematic_model = model_loader->getModel();
  kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);
  kinematic_state_->setToDefaultValues();

  joint_model_group_ = kinematic_model->getJointModelGroup(parameters_.move_group_name);

  std::vector<double> dummy_joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_, dummy_joint_values);

  resetVelocityFilters();

  jt_state_.name = move_group_.getJointNames();
  jt_state_.position.resize(jt_state_.name.size());
  jt_state_.velocity.resize(jt_state_.name.size());
  jt_state_.effort.resize(jt_state_.name.size());

  // Low-pass filters for the joint positions & velocities
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    velocity_filters_.emplace_back(parameters_.low_pass_filter_coeff);
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // Initialize the position filters to initial robot joints
  while (!updateJoints() && ros::ok())
  {
    pthread_mutex_lock(&shared_variables.mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.mutex);
    ros::Duration(0.001).sleep();
  }
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    position_filters_[i].reset(jt_state_.position[i]);
  }

  // Wait for the first jogging cmd.
  // Store it in a class member for further calcs.
  // Then free up the shared variable again.
  geometry_msgs::TwistStamped cartesian_deltas;
  jog_msgs::JogJoint joint_deltas;

  ROS_INFO("[JogCalcs::JogCalcs] Wait for the first jogging cmd.");

  while (ros::ok() && cartesian_deltas.header.stamp == ros::Time(0.) && joint_deltas.header.stamp == ros::Time(0.))
  {
    ros::Duration(0.05).sleep();

    pthread_mutex_lock(&shared_variables.mutex);
    cartesian_deltas = shared_variables.command_deltas;
    joint_deltas = shared_variables.joint_command_deltas;
    pthread_mutex_unlock(&shared_variables.mutex);

    //ROS_INFO("[JogCalcs::JogCalcs] time: %s", ros::Time(0.));
    //ROS_INFO("[JogCalcs::JogCalcs] cartesian_deltas" << cartesian_deltas);
    //ROS_INFO("[JogCalcs::JogCalcs] joint_deltas" << joint_deltas);

    ROS_INFO("[JogCalcs::JogCalcs] Wait.");
  }

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count = 0;
  int num_zero_cycles_to_publish = 4;

  ROS_INFO("[JogCalcs::JogCalcs] Now do jogging calcs.");

  // Now do jogging calcs
  while (ros::ok())
  {
    ROS_INFO("[JogCalcs::JogCalcs] LOOP");

    // If user commands are all zero, reset the low-pass filters
    // when commands resume
    pthread_mutex_lock(&shared_variables.mutex);
    bool zero_cartesian_traj_flag = shared_variables.zero_cartesian_cmd_flag;
    bool zero_joint_traj_flag = shared_variables.zero_joint_cmd_flag;
    pthread_mutex_unlock(&shared_variables.mutex);

    if (zero_cartesian_traj_flag && zero_joint_traj_flag)
    {
        resetVelocityFilters();
    }

    // Pull data from the shared variables.
    pthread_mutex_lock(&shared_variables.mutex);
    incoming_jts_ = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.mutex);

    // Initialize the position filters to initial robot joints
    while (!updateJoints() && ros::ok())
    {
      pthread_mutex_lock(&shared_variables.mutex);
      incoming_jts_ = shared_variables.joints;
      pthread_mutex_unlock(&shared_variables.mutex);
      ros::Duration(0.001).sleep();
    }

    // If there have not been several consecutive cycles of all zeros and joint
    // jogging commands are empty
    if ((zero_velocity_count <= num_zero_cycles_to_publish) && zero_joint_traj_flag)
    {
      cartesian_deltas = shared_variables.command_deltas;
      //ROS_INFO("Cartesian deltas: " << cartesian_deltas);

      if (!cartesianJogCalcs(cartesian_deltas, shared_variables)) { continue; }
    }
    // If there have not been several consecutive cycles of all zeros and joint
    // jogging commands are not empty
    else if ((zero_velocity_count <= num_zero_cycles_to_publish) && !zero_joint_traj_flag)
    {
      joint_deltas = shared_variables.joint_command_deltas;
      //ROS_INFO("Joint deltas: " << joint_deltas);

      if (!jointJogCalcs(joint_deltas, shared_variables)) { continue; }
    }

    ROS_INFO("[JogCalcs::JogCalcs] Command stale: %d", shared_variables.command_is_stale);
    ROS_INFO("[JogCalcs::JogCalcs] Zero cartesian trajectory: %d", zero_cartesian_traj_flag);
    ROS_INFO("[JogCalcs::JogCalcs] Zero joint trajectory: %d", zero_joint_traj_flag);

    // Halt if the command is stale or inputs are all zero, or commands were
    // zero
    if (shared_variables.command_is_stale || (zero_cartesian_traj_flag && zero_joint_traj_flag))
    {
      halt(new_traj_);
      zero_cartesian_traj_flag = true;
      zero_joint_traj_flag = true;
    }

    // Has the velocity command been zero for several cycles in a row?
    // If so, stop publishing so other controllers can take over
    bool valid_nonzero_trajectory =
        !((zero_velocity_count > num_zero_cycles_to_publish) && zero_cartesian_traj_flag && zero_joint_traj_flag);

    ROS_INFO("[JogCalcs::JogCalcs] Zero valid_nonzero_trajectory: %d", valid_nonzero_trajectory);
    ROS_INFO("[JogCalcs::JogCalcs] new_traj is empty: %d", new_traj_.joint_names.empty());

    // Send the newest target joints
    if (!new_traj_.joint_names.empty())
    {
      // If everything normal, share the new traj to be published
      if (valid_nonzero_trajectory)
      {
        pthread_mutex_lock(&shared_variables.mutex);
        shared_variables.new_traj = new_traj_;
        shared_variables.ok_to_publish = true;
        pthread_mutex_unlock(&shared_variables.mutex);
      }
      // Skip the jogging publication if all inputs have been zero for several
      // cycles in a row
      else if (zero_velocity_count > num_zero_cycles_to_publish)
      {
        pthread_mutex_lock(&shared_variables.mutex);
        shared_variables.ok_to_publish = false;
        pthread_mutex_unlock(&shared_variables.mutex);
      }

      ROS_INFO("[JogCalcs::JogCalcs] Zero ok_to_publish: %d", shared_variables.ok_to_publish);

      // Store last zero-velocity message flag to prevent superfluous warnings.
      // Cartesian and joint commands must both be zero.
      if (zero_cartesian_traj_flag && zero_joint_traj_flag)
        zero_velocity_count += 1;
      else
        zero_velocity_count = 0;
    }

    // Add a small sleep to avoid 100% CPU usage
    ros::Duration(0.005).sleep();
  }
}

bool JogCalculatorWorker::cartesianJogCalcs(const geometry_msgs::TwistStamped& cmd, jog_arm_shared& shared_variables)
{
  ROS_INFO("[JogCalcs::cartesianJogCalcs] BEGIN");

  // Check for nan's in the incoming command
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    ROS_WARN_STREAM_NAMED(log_name_, "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_.command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      ROS_WARN_STREAM_NAMED(log_name_, "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  // Convert the cmd to the MoveGroup planning frame.
  try
  {
    ROS_INFO("[JogCalcs::cartesianJogCalcs] Convert the cmd to the MoveGroup planning frame.");
    listener_.waitForTransform(cmd.header.frame_id, parameters_.planning_frame, ros::Time::now(), ros::Duration(0.2));
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(log_name_, ros::this_node::getName() << ": " << ex.what());
    return 0;
  }

  ROS_INFO("[JogCalcs::cartesianJogCalcs] 2");

  // To transform, these vectors need to be stamped. See answers.ros.org
  // Q#199376
  // Transform the linear component of the cmd message
  geometry_msgs::Vector3Stamped lin_vector;
  lin_vector.vector = cmd.twist.linear;
  lin_vector.header.frame_id = cmd.header.frame_id;
  try
  {
    listener_.transformVector(parameters_.planning_frame, lin_vector, lin_vector);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(log_name_, ros::this_node::getName() << ": " << ex.what());
    return 0;
  }

 ROS_INFO("[JogCalcs::cartesianJogCalcs] 3");

  geometry_msgs::Vector3Stamped rot_vector;
  rot_vector.vector = cmd.twist.angular;
  rot_vector.header.frame_id = cmd.header.frame_id;
  try
  {
    listener_.transformVector(parameters_.planning_frame, rot_vector, rot_vector);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR_STREAM_NAMED(log_name_, ros::this_node::getName() << ": " << ex.what());
    return 0;
  }

  ROS_INFO("[JogCalcs::cartesianJogCalcs] 4");

  // Put these components back into a TwistStamped
  geometry_msgs::TwistStamped twist_cmd;
  twist_cmd.header.stamp = cmd.header.stamp;
  twist_cmd.header.frame_id = parameters_.planning_frame;
  twist_cmd.twist.linear = lin_vector.vector;
  twist_cmd.twist.angular = rot_vector.vector;

  const Eigen::VectorXd delta_x = scaleCartesianCommand(twist_cmd);

  kinematic_state_->setVariableValues(jt_state_);
  original_jts_ = jt_state_;

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd delta_theta = pseudoInverse(svd.matrixU(), svd.matrixV(), svd.singularValues().asDiagonal()) * delta_x;

  enforceJointVelocityLimits(delta_theta);
  if (!addJointIncrements(jt_state_, delta_theta))
    return 0;

  // Include a velocity estimate for velocity-controlled robots
  Eigen::VectorXd joint_vel(delta_theta / parameters_.publish_period);

  lowPassFilterVelocities(joint_vel);
  lowPassFilterPositions();

  const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_period);
  new_traj_ = composeOutgoingMessage(jt_state_, next_time);

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(shared_variables, new_traj_, delta_theta, decelerateForSingularity(jacobian, delta_x));

  ROS_INFO("[JogCalcs::cartesianJogCalcs] 5");

  if (!checkIfJointsWithinBounds(new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }
  else
  {
    publishWarning(false);
  }

  // If using Gazebo simulator, insert redundant points
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(new_traj_, GAZEBO_REDUNTANT_MESSAGE_COUNT);
  }

  ROS_INFO("[JogCalcs::cartesianJogCalcs] END");
  return true;
}

bool JogCalculatorWorker::jointJogCalcs(const jog_msgs::JogJoint& cmd, jog_arm_shared& shared_variables)
{
  ROS_INFO("[JogCalcs::jointJogCalcs] BEGIN");

  // Check for nan's or |delta|>1 in the incoming command
  for (std::size_t i = 0; i < cmd.deltas.size(); ++i)
  {
    if (std::isnan(cmd.deltas[i]) || (fabs(cmd.deltas[i]) > 1))
    {
      ROS_WARN_STREAM_NAMED(log_name_, "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }

  // Apply user-defined scaling
  const Eigen::VectorXd delta = scaleJointCommand(cmd);

  kinematic_state_->setVariableValues(jt_state_);
  original_jts_ = jt_state_;

  if (!addJointIncrements(jt_state_, delta))
  {
    return false;
  }

  // Include a velocity estimate for velocity-controlled robots
  Eigen::VectorXd joint_vel(delta / parameters_.publish_period);

  lowPassFilterVelocities(joint_vel);
  lowPassFilterPositions();

  // update joint state with new values
  kinematic_state_->setVariableValues(jt_state_);

  const ros::Time next_time = ros::Time::now() + ros::Duration(parameters_.publish_delay);
  new_traj_ = composeOutgoingMessage(jt_state_, next_time);

  // check if new joint state is valid
  if (!checkIfJointsWithinBounds(new_traj_))
  {
    halt(new_traj_);
    publishWarning(true);
  }
  else
  {
    publishWarning(false);
  }

  // done with calculations
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(new_traj_, GAZEBO_REDUNTANT_MESSAGE_COUNT);
  }

  ROS_INFO("[JogCalcs::jointJogCalcs] END");
  return true;
}

// Spam several redundant points into the trajectory. The first few may be
// skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo
// simulation.
// Start from 2 because the first point's timestamp is already
// 1*parameters_.publish_period
void JogCalculatorWorker::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& trajectory, int count) const
{
  auto point = trajectory.points[0];
  // Start from 2 because we already have the first point. End at count+1 so
  // total # == count
  for (int i = 2; i < count + 1; ++i)
  {
    point.time_from_start = ros::Duration(i * parameters_.publish_period);
    trajectory.points.push_back(point);
  }
}

void JogCalculatorWorker::lowPassFilterPositions()
{
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_.position[i] = position_filters_[i].filter(jt_state_.position[i]);

    // Check for nan's
    if (std::isnan(jt_state_.position[i]))
    {
      jt_state_.position[i] = original_jts_.position[i];
      jt_state_.velocity[i] = 0.;
    }
  }
}

void JogCalculatorWorker::lowPassFilterVelocities(const Eigen::VectorXd& joint_vel)
{
  for (size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    jt_state_.velocity[i] = velocity_filters_[i].filter(joint_vel[static_cast<long>(i)]);

    // Check for nan's
    if (std::isnan(jt_state_.velocity[static_cast<long>(i)]))
    {
      jt_state_.position[i] = original_jts_.position[i];
      jt_state_.velocity[i] = 0.;
      ROS_WARN_STREAM_NAMED(log_name_, "nan in velocity filter");
    }
  }
}

trajectory_msgs::JointTrajectory JogCalculatorWorker::composeOutgoingMessage(sensor_msgs::JointState& joint_state,
                                                                             const ros::Time& stamp) const
{
  trajectory_msgs::JointTrajectory new_jt_traj;
  new_jt_traj.header.frame_id = parameters_.planning_frame;
  new_jt_traj.header.stamp = stamp;
  new_jt_traj.joint_names = joint_state.name;

  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);
  if (parameters_.publish_joint_positions)
    point.positions = joint_state.position;
  if (parameters_.publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (parameters_.publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(joint_state.velocity.size());
    point.accelerations = acceleration;
  }
  new_jt_traj.points.push_back(point);

  return new_jt_traj;
}

// Apply velocity scaling for proximity of collisions and singularities.
// Scale for collisions is read from a shared variable.
// Key equation: new_velocity =
// collision_scale*singularity_scale*previous_velocity
bool JogCalculatorWorker::applyVelocityScaling(jog_arm_shared& shared_variables, trajectory_msgs::JointTrajectory& new_jt_traj,
                                               const Eigen::VectorXd& delta_theta, double singularity_scale)
{
  double collision_scale = shared_variables.collision_velocity_scale;

  for (size_t i = 0; i < jt_state_.velocity.size(); ++i)
  {
    if (parameters_.publish_joint_positions)
    {
      // If close to a singularity or joint limit, undo any change to the joint
      // angles
      new_jt_traj.points[0].positions[i] =
          new_jt_traj.points[0].positions[i] -
          (1. - singularity_scale * collision_scale) * delta_theta[static_cast<long>(i)];
    }
    if (parameters_.publish_joint_velocities)
      new_jt_traj.points[0].velocities[i] *= singularity_scale * collision_scale;
  }

  return 1;
}


void JogCalculatorWorker::enforceJointVelocityLimits(Eigen::VectorXd& calculated_joint_vel)
{
  double maximum_joint_vel = calculated_joint_vel.cwiseAbs().maxCoeff();
  if(maximum_joint_vel > parameters_.joint_scale)
  {
    // Scale the entire joint velocity vector so that each joint velocity is below min, and the output movement is scaled uniformly to match expected motion
    calculated_joint_vel = calculated_joint_vel * parameters_.joint_scale / maximum_joint_vel;
  }
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity
// and direction of motion
double JogCalculatorWorker::decelerateForSingularity(Eigen::MatrixXd jacobian, const Eigen::VectorXd commanded_velocity)
{
  double velocity_scale = 1;

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points away from the
  // singularity
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(5);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity
  // in the Singular Value Decomposition"
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction.
  // Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(6);
  double scale = 100;
  delta_x[0] = vector_toward_singularity[0] / scale;
  delta_x[1] = vector_toward_singularity[1] / scale;
  delta_x[2] = vector_toward_singularity[2] / scale;
  delta_x[3] = vector_toward_singularity[3] / scale;
  delta_x[4] = vector_toward_singularity[4] / scale;
  delta_x[5] = vector_toward_singularity[5] / scale;

  // Calculate a small change in joints
  Eigen::VectorXd delta_theta = pseudoInverse(jacobian) * delta_x;

  double theta[6];
  const double* prev_joints = kinematic_state_->getVariablePositions();
  for (std::size_t i = 0, size = static_cast<std::size_t>(delta_theta.size()); i < size; ++i)
    theta[i] = prev_joints[i] + delta_theta(i);

  kinematic_state_->setVariablePositions(theta);
  jacobian = kinematic_state_->getJacobian(joint_model_group_);
  svd = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian);
  double new_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity.
  //  Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity[0] *= -1;
    vector_toward_singularity[1] *= -1;
    vector_toward_singularity[2] *= -1;
    vector_toward_singularity[3] *= -1;
    vector_toward_singularity[4] *= -1;
    vector_toward_singularity[5] *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==>
  // decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between
    // lower_singularity_threshold and
    // hard_stop_singularity_threshold, and we're moving towards the singularity
    if ((ini_condition > parameters_.lower_singularity_threshold) &&
        (ini_condition < parameters_.hard_stop_singularity_threshold))
    {
      velocity_scale = 1. -
                       (ini_condition - parameters_.lower_singularity_threshold) /
                           (parameters_.hard_stop_singularity_threshold - parameters_.lower_singularity_threshold);
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_.hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      ROS_WARN_NAMED(log_name_, "Close to a singularity. Halting.");
    }
  }

  return velocity_scale;
}

bool JogCalculatorWorker::checkIfJointsWithinBounds(trajectory_msgs::JointTrajectory& new_jt_traj)
{
  bool halting = false;
  for (auto joint : joint_model_group_->getJointModels())
  {
    if (!kinematic_state_->satisfiesVelocityBounds(joint))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, log_name_, ros::this_node::getName() << " " << joint->getName() << " "
                                                                             << " close to a "
                                                                                " velocity limit. Enforcing limit.");
      kinematic_state_->enforceVelocityBounds(joint);
      for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c)
      {
        if (new_jt_traj.joint_names[c] == joint->getName())
        {
          new_jt_traj.points[0].velocities[c] = kinematic_state_->getJointVelocities(joint)[0];
          break;
        }
      }
    }

    // Halt if we're past a joint margin and joint velocity is moving even
    // farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < new_jt_traj.joint_names.size(); ++c)
    {
      if (original_jts_.name[c] == joint->getName())
      {
        joint_angle = original_jts_.position.at(c);
        break;
      }
    }

    if (!kinematic_state_->satisfiesPositionBounds(joint,
                                                   -parameters_.joint_limit_margin))
    {
      const std::vector<moveit_msgs::JointLimits> limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (limits.size() > 0)
      {
        if ((kinematic_state_->getJointVelocities(joint)[0] < 0 &&
             (joint_angle < (limits[0].min_position + parameters_.joint_limit_margin))) ||
            (kinematic_state_->getJointVelocities(joint)[0] > 0 &&
             (joint_angle > (limits[0].max_position - parameters_.joint_limit_margin))))
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(2, log_name_, ros::this_node::getName() << " " << joint->getName()
                                                                                  << " close to a "
                                                                                     " position limit. Halting.");
          halting = true;
        }
      }
    }
  }

  return !halting;
}

void JogCalculatorWorker::publishWarning(const bool active) const
{
  std_msgs::Bool status;
  status.data = static_cast<std_msgs::Bool::_data_type>(active);
  warning_pub_.publish(status);
}

// Avoid a singularity or other issue.
// Needs to be handled differently for position vs. velocity control
void JogCalculatorWorker::halt(trajectory_msgs::JointTrajectory& jt_traj)
{
  for (std::size_t i = 0; i < jt_state_.velocity.size(); ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good
    // state
    if (parameters_.publish_joint_positions)
      jt_traj.points[0].positions[i] = original_jts_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_.publish_joint_velocities)
      jt_traj.points[0].velocities[i] = 0;
  }
}

// Reset the data stored in filters so the trajectory won't jump when jogging is
// resumed.
void JogCalculatorWorker::resetVelocityFilters()
{
  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
    velocity_filters_[i].reset(0);  // Zero velocity
}

// Parse the incoming joint msg for the joints of our MoveGroup
bool JogCalculatorWorker::updateJoints()
{
  // Check if every joint was zero. Sometimes an issue.
  bool all_zeros = true;

  // Check that the msg contains enough joints
  if (incoming_jts_.name.size() < jt_state_.name.size())
    return 0;

  // Store joints in a member variable
  for (std::size_t m = 0; m < incoming_jts_.name.size(); ++m)
  {
    for (std::size_t c = 0; c < jt_state_.name.size(); ++c)
    {
      if (incoming_jts_.name[m] == jt_state_.name[c])
      {
        jt_state_.position[c] = incoming_jts_.position[m];
        // Make sure there was at least one nonzero value
        if (incoming_jts_.position[m] != 0.)
          all_zeros = false;
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }

  return !all_zeros;
}

Eigen::VectorXd JogCalculatorWorker::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_.command_in_type == "unitless")
  {
    result[0] = parameters_.linear_scale * command.twist.linear.x;
    result[1] = parameters_.linear_scale * command.twist.linear.y;
    result[2] = parameters_.linear_scale * command.twist.linear.z;
    result[3] = parameters_.rotational_scale * command.twist.angular.x;
    result[4] = parameters_.rotational_scale * command.twist.angular.y;
    result[5] = parameters_.rotational_scale * command.twist.angular.z;
  }
  // Otherwise, commands are in m/s and rad/s
  else if (parameters_.command_in_type == "speed_units")
  {
    result[0] = command.twist.linear.x * parameters_.publish_period;
    result[1] = command.twist.linear.y * parameters_.publish_period;
    result[2] = command.twist.linear.z * parameters_.publish_period;
    result[3] = command.twist.angular.x * parameters_.publish_period;
    result[4] = command.twist.angular.y * parameters_.publish_period;
    result[5] = command.twist.angular.z * parameters_.publish_period;
  }
  else
    ROS_ERROR_STREAM_NAMED(log_name_, "Unexpected command_in_type");

  return result;
}

Eigen::VectorXd JogCalculatorWorker::scaleJointCommand(const jog_msgs::JogJoint& command) const
{
  Eigen::VectorXd result(jt_state_.name.size());

  for (std::size_t i = 0; i < jt_state_.name.size(); ++i)
  {
    result[i] = 0.0;
  }

  // Store joints in a member variable
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    for (std::size_t c = 0; c < jt_state_.name.size(); ++c)
    {
      if (command.joint_names[m] == jt_state_.name[c])
      {
        // Apply user-defined scaling if inputs are unitless [-1:1]
        if (parameters_.command_in_type == "unitless")
          result[c] = command.deltas[m] * parameters_.joint_scale;
        // Otherwise, commands are in m/s and rad/s
        else if (parameters_.command_in_type == "speed_units")
          result[c] = command.deltas[m] * parameters_.publish_period;
        else
          ROS_ERROR_STREAM_NAMED(log_name_, "Unexpected command_in_type");
        goto NEXT_JOINT;
      }
    }
  NEXT_JOINT:;
  }

  return result;
}

Eigen::MatrixXd JogCalculatorWorker::pseudoInverse(const Eigen::MatrixXd& J) const
{
  return J.transpose() * (J * J.transpose()).inverse();
}

Eigen::MatrixXd JogCalculatorWorker::pseudoInverse(const Eigen::MatrixXd& u_matrix,
                                                   const Eigen::MatrixXd& v_matrix, const Eigen::MatrixXd& s_diagonals) const
{
  return v_matrix * s_diagonals.inverse() * u_matrix.transpose();
}

bool JogCalculatorWorker::addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const
{
  for (std::size_t i = 0, size = static_cast<std::size_t>(increments.size()); i < size; ++i)
  {
    try
    {
      output.position[i] += increments[static_cast<long>(i)];
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_NAMED(log_name_, ros::this_node::getName() << " Lengths of output and "
                                                                     "increments do not match.");
      return 0;
    }
  }

  return 1;
}
