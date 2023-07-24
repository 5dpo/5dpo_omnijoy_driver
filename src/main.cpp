#include "sdpo_omnijoy_driver/OmniJoyDriverROS.h"

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_omnijoy_driver::OmniJoyDriverROS>());

  rclcpp::shutdown();

  return 0;

}
