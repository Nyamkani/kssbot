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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


# Link against wiringPi
SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib -lwiringPi")

find_library(WIRINGPI_LIBRARIES wiringPi)


#add_library(WIRINGPI_LIBRARIES STATIC sdk/raspmotorctrl/raspmotorctrl.cpp)

#target_include_directories(WIRINGPI_LIBRARIES PUBLIC ${CMAKE_SOURCE_DIR}/includes)

#target_compile_options(WIRINGPI_LIBRARIES PRIVATE -Wall -Werror)


#TARGET_LINK_LIBRARIES(kssbot_diffdrive WIRINGPI_LIBRARIES)

## COMPILE
add_library(
  ${PROJECT_NAME} 
  SHARED
  ${WIRINGPI_LIBRARIES}
  src/kssbot_diffdrive.cpp 
  src/raspmotorctrl.cpp 
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  WIRINGPI_LIBRARIES
  #sdk/raspmotorctrl
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KSSBOT_HARDWARE_BUILDING_DLL")

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface interface/kssbot_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
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
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()