#include "sdpo_omnijoy_driver/OmniJoyDriverROS1.h"

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "sdpo_omnijoy_driver");

  sdpo_omnijoy_driver::OmniJoyDriverROS1 node;

  ros::spin();

  ros::shutdown();

  return 0;

}
