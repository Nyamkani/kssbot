cmake_minimum_required(VERSION 3.5)
project(kssbot_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# add moudules
add_subdirectory(modules/kssbot_diffdrive)
#add ros2_control library here


#using this for linking this cmakelist
add_library(${PROJECT_NAME} SHARED
  src/test.cpp
)

# link modules
target_link_libraries(${PROJECT_NAME} 
  kssbot_diffdrive
  #add ros2_control library here
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  include 
)


ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE 
  "KSSBOT_HARDWARE_BUILDING_DLL"
)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface 
  interface/kssbot_hardware.xml
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()