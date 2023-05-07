# Moudules/kssbot_led
cmake_minimum_required(VERSION 3.5)
project(kssbot_led)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(controller_manager REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

# COMPILE
add_library(${PROJECT_NAME} SHARED
  #src/kssbot_led.cpp 
  #src/raspledctrl.cpp 
  src/ws2812-rpi.cpp
)

set(SOURCE src/ws2812-rpi-test.cpp)

add_executable(main_program ${SOURCE})

target_link_libraries(main_program ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
)

#target_compile_options(main_program PRIVATE -Wall -Werror)

ament_target_dependencies(
  #main_program
  ${PROJECT_NAME}
  # hardware_interface
  # controller_manager
  # pluginlib
  # rclcpp
  # rclcpp_lifecycle
)

# INSTALL
install(
  TARGETS main_program
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)
