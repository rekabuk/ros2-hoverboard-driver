#include "ros2-hoverboard-driver/hoverboard.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node after ros init
  auto hoverboard_node = std::make_shared<Hoverboard>(); // What is auto??

  while (rclcpp::ok())
  {
    if (hoverboard_node->read()==1)
    {
      hoverboard_node->write();
    }
    rclcpp::spin_some(hoverboard_node);
  }  
  rclcpp::shutdown();

  return 0;
}
