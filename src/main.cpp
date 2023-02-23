#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sdpo_driver_omnijoy");

  ros::spin();
}
