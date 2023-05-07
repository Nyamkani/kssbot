cmake_minimum_required(VERSION 3.5)
project(kssbot_description)

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY config ros2_control urdf gazebo_worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
