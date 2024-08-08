#include "optical_flow/optical_flow.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optical_flow::OpticalFlow>());
  rclcpp::shutdown();
  return 0;
}
