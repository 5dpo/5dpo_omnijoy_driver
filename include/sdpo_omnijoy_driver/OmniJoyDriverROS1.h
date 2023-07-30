#pragma once

#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>



namespace sdpo_omnijoy_driver
{

class OmniJoyDriverROS1
{

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  int linearx_;
  int lineary_;
  int angular_;
  int deadman_axis_;
  int turbo_axis_;
  int turbo_up_axis_;
  int turbo_down_axis_;

  double l_scale_;
  double a_scale_;
  double l_turbo_scale_;
  double l_turbo_maxscale_;
  double a_turbo_scale_;
  double a_turbo_maxscale_;



  geometry_msgs::Twist last_published_;

  std::mutex publish_mutex_;



  bool deadman_pressed_;
  bool turbo_pressed_;
  bool turbo_up_pressed_;
  bool turbo_down_pressed_;
  bool zero_twist_published_;



  ros::Timer timer_;





 public:

  OmniJoyDriverROS1();



 private:

  void readParam();

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void publish();

};

} // namespace sdpo_omnijoy_driver
