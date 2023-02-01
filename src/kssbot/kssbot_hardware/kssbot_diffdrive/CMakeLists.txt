cmake_minimum_required(VERSION 3.5)
project(kssbot_diffdrive)

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


# Link against wiringPi
SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib -lwiringPi")

# COMPILE
add_library(${PROJECT_NAME}_lib SHARED
  src/kssbot_diffdrive.cpp 
  src/raspmotorctrl.cpp 
)

find_library(WIRINGPI_LIBRARIES wiringPi)

target_link_libraries(${PROJECT_NAME}_lib ${WIRINGPI_LIBRARIES})


target_include_directories(${PROJECT_NAME}_lib PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}_lib
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

#target_link_libraries(${PROJECT_NAME} ${/usr/local/lib/libwiringPi.so})


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE "KSSBOT_HARDWARE_BUILDING_DLL")

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface interface/kssbot_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()