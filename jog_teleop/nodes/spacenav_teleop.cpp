#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "jog_msgs/JogJoint.h"

namespace to_twist
{

class spaceNavToTwist
{
public:
  spaceNavToTwist() : spinner_(1)
  {
    joy_sub_ = n_.subscribe("spacenav/joy", 1, &spaceNavToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("jog_arm_server/cartesian_cmd", 1);
    joint_delta_pub_ = n_.advertise<jog_msgs::JogJoint>("jog_arm_server/joint_cmd", 1);

    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_, joint_delta_pub_;
  ros::AsyncSpinner spinner_;

  // Convert incoming joy commands to TwistStamped commands for jogging.
  // The TwistStamped component goes to jogging, while buttons 0 & 1 control
  // joints directly.
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging with the axes
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = msg->axes[0];
    twist.twist.linear.y = msg->axes[1];
    twist.twist.linear.z = msg->axes[2];

    twist.twist.angular.x = msg->axes[3];
    twist.twist.angular.y = msg->axes[4];
    twist.twist.angular.z = msg->axes[5];

    // Joint jogging with the buttons
    jog_msgs::JogJoint joint_deltas;
    // This example is for a Motoman SIA5. joint_s is the base joint.
    joint_deltas.joint_names.push_back("joint_s");

    // Button 0: positive on the wrist joint
    // Button 1: negative on the wrist joint
    joint_deltas.deltas.push_back(msg->buttons[0] - msg->buttons[1]);
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spacenav_teleop");

  to_twist::spaceNavToTwist to_twist;

  return 0;
}