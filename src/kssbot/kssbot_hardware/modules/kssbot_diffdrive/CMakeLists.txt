# Moudules/kssbot_diffdrive
cmake_minimum_required(VERSION 3.5)
project(kssbot_diffdrive)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


# Link against wiringPi
SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib -lwiringPi")

find_library(WIRINGPI_LIBRARIES wiringPi)

# COMPILE
add_library(${PROJECT_NAME} SHARED
  src/kssbot_diffdrive.cpp 
  src/raspmotorctrl.cpp 
)

target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(
  DIRECTORY include/
  DESTINATION include
)
