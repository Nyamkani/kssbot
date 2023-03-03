// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kssbot_diffdrive/kssbot_diffdrive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace kssbot_hardware
{
hardware_interface::CallbackReturn kssbot_diffdrive_rasp4::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
 
  //get data from launch parameters
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);

  // Set up the wheels
  l_wheel_.setup(cfg_.left_wheel_name);
  r_wheel_.setup(cfg_.right_wheel_name);

  rclcpp::get_logger("kssbot_hardware Initialize");

  //main rasp4 motor end

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Kssbot_diffdrive"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Kssbot_diffdrive"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Kssbot_diffdrive"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("kssbot_hardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("kssbot_hardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> kssbot_diffdrive_rasp4::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> kssbot_diffdrive_rasp4::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}



hardware_interface::CallbackReturn kssbot_diffdrive_rasp4::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rclcpp::get_logger("Activating Controller!");
  //rasp4 motor driver initialize

  //1. main rasp4 motor start
  if(!(this->raspmotor_)) this->raspmotor_ = std::make_unique<raspmotor>(BCM, 20, 20);

  //2. init rasp4 motor
  this->raspmotor_->Initialize();
  
  //3. check rasp4 motor class load 
  if(!(this->raspmotor_)) return hardware_interface::CallbackReturn::ERROR;

  //4. set main loop online
  this->raspmotor_->is_run_ = true;

  //5. make thread

  std::thread drive_loop_(&kssbot_diffdrive_rasp4::DriveMotor, this);

  drive_loop_.detach();

  // set some default values
  // l_wheel_.cmd = 0;
  // l_wheel_.pos = 0;
  // l_wheel_.vel = 0;

  // r_wheel_.cmd = 0;
  // r_wheel_.pos = 0;
  // r_wheel_.vel = 0;

  RCLCPP_INFO(rclcpp::get_logger("kssbot_hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn kssbot_diffdrive_rasp4::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //deinitalize

   //1. main loop is offline
  this->raspmotor_->is_run_ = false;


  RCLCPP_INFO(rclcpp::get_logger("kssbot_hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::return_type kssbot_diffdrive_rasp4::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if(!(this->raspmotor_)) return hardware_interface::return_type::ERROR;

  //due to rasp4 motor has no encoder, we cant get any data


  //rasp4motor.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  // double pos_prev = l_wheel_.pos;
  // l_wheel_.pos = l_wheel_.calcEncAngle();
  // l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  // pos_prev = r_wheel_.pos;
  // r_wheel_.pos = r_wheel_.calcEncAngle();
  // r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;


  //---------------------------------------------------------------
  // double radius = 0.017;  // radius of the wheels 
  // double dist_w = 0.1;   // distance between the wheels

  // for (uint i = 0; i < hw_commands_.size(); i++)
  // {
  //   // Simulate DiffBot wheels's movement as a first-order system
  //   // Update the joint status: this is a revolute joint without any limit.
  //   // Simply integrates
  //   hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_commands_[i];
  //   hw_velocities_[i] = hw_commands_[i];

  // }

  // // Update the free-flyer, i.e. the base notation using the classical
  // // wheel differentiable kinematics
  // double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  // double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  // double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  // base_x_ += base_dx * period.seconds();
  // base_y_ += base_dy * period.seconds();
  // base_theta_ += base_dtheta * period.seconds();

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type kssbot_hardware::kssbot_diffdrive_rasp4::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(!(this->raspmotor_)) return hardware_interface::return_type::ERROR;

  this->raspmotor_->LinkRosToRasp(l_wheel_.cmd, r_wheel_.cmd);

  return hardware_interface::return_type::OK;
}

  KSSBOT_HARDWARE_PUBLIC
  hardware_interface::return_type kssbot_hardware::kssbot_diffdrive_rasp4::DriveMotor()
  {
    if(!(this->raspmotor_)) return hardware_interface::return_type::ERROR;

    while(1)
    {
      if(!(this->raspmotor_->is_run_)) break;

      this->raspmotor_->Drive();

      usleep(10000);
    }


    return hardware_interface::return_type::OK;
  }


}  // namespace kssbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kssbot_hardware::kssbot_diffdrive_rasp4, hardware_interface::SystemInterface)
