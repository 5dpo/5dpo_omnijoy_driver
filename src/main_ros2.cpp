#include "sdpo_omnijoy_driver/OmniJoyDriverROS2.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_omnijoy_driver::OmniJoyDriverROS2>());

  rclcpp::shutdown();

  return 0;

}
