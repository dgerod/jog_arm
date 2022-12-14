#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include "jog_msgs/JogJoint.h"

namespace to_twist
{

class dragonriseToTwist
{
public:
  dragonriseToTwist() : spinner_(1)
  {
    joy_sub_ = n_.subscribe("joy", 1, &dragonriseToTwist::joyCallback, this);
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

  // Convert incoming joy commands to TwistStamped commands for jogging
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();

    twist.twist.linear.x = -msg->axes[0];  // left stick x (inversed)
    twist.twist.linear.y = msg->axes[1];   // left stick y
    twist.twist.linear.z = msg->axes[4];   // right stick y

    // buttons
    twist.twist.angular.x = -msg->axes[5];  // dpad x (inversed)
    twist.twist.angular.y = msg->axes[6];   // dpad y
    // A binary button
    twist.twist.angular.z = -msg->buttons[6] + msg->buttons[4];  // buttons L2 L1

    // Joint jogging
    jog_msgs::JogJoint joint_deltas;
    // This example is for a Phoenix hexapod : "femur_joint_r1" is the R1 femur
    // joint (move leg up/down)
    joint_deltas.joint_names.push_back("femur_joint_r1");
    joint_deltas.deltas.push_back(msg->buttons[5] - msg->buttons[7]);  // buttons R2 R1
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragonrise_teleop");

  to_twist::dragonriseToTwist to_twist;

  return 0;
}
