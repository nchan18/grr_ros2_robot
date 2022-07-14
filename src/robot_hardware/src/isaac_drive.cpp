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

#include "robot_hardware/isaac_drive.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using std::placeholders::_1;

namespace robot_hardware
{
CallbackReturn IsaacDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  node_ = rclcpp::Node::make_shared("isaac_hardware_interface");
  publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("isaac_joint_commands", custom_qos_profile.depth);
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("isaac_joint_states", custom_qos_profile.depth,
    std::bind(&IsaacDriveHardware::topic_callback, this, _1));

  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // joint_names_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IsaacDriveHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> IsaacDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> IsaacDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}



CallbackReturn IsaacDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("IsaacDriveHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  isaac_joint_names_ = joint_names_;
  isaac_positions_ = hw_positions_;
  isaac_velocities_ = hw_velocities_;

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}



CallbackReturn IsaacDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("IsaacDriveHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/

hardware_interface::return_type IsaacDriveHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Reading...");

  // double radius = 0.02;  // radius of the wheels
  // double dist_w = 0.1;   // distance between the wheels
  // double dt = 0.01;      // Control period
  // for (uint i = 0; i < hw_commands_.size(); i++)
  // {
  //   // Simulate DiffBot wheels's movement as a first-order system
  //   // Update the joint status: this is a revolute joint without any limit.
  //   // Simply integrates
  //   hw_positions_[i] = hw_positions_[1] + dt * hw_commands_[i];
  //   hw_velocities_[i] = hw_commands_[i];

  //   // START: This part here is for exemplary purposes - Please do not copy to your production code
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("IsaacDriveHardware"),
  //     "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
  //     hw_velocities_[i], joint_names_[i].c_str());
  //   // END: This part here is for exemplary purposes - Please do not copy to your production code
  // }

  // // Update the free-flyer, i.e. the base notation using the classical
  // // wheel differentiable kinematics
  // double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  // double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  // double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  // base_x_ += base_dx * dt;
  // base_y_ += base_dy * dt;
  // base_theta_ += base_dtheta * dt;
  rclcpp::spin_some(node_);
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    for (auto y = 0u; y < hw_commands_.size(); y++) {
      if (joint_names_[i] == isaac_joint_names_[y]) {
        hw_positions_[i] = isaac_positions_[y];
        hw_velocities_[i] = isaac_velocities_[y];
        break;
      }
    }
  }

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Read: %s pos: %.5f vel: %.5f",
      joint_names_[i].c_str(), hw_positions_[i], hw_velocities_[i]);
  }
  

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type robot_hardware::IsaacDriveHardware::write()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Writing...");

  auto joint_commands = sensor_msgs::msg::JointState();
  joint_commands.name = joint_names_;
  joint_commands.velocity = hw_commands_;
  publisher_->publish(joint_commands);
  rclcpp::spin_some(node_);

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("IsaacDriveHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("IsaacDriveHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

void IsaacDriveHardware::topic_callback(const sensor_msgs::msg::JointState & state)
{
  isaac_joint_names_ = state.name;
  isaac_positions_ = state.position;
  isaac_velocities_ = state.velocity;
}

}  // namespace robot_hardware



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware::IsaacDriveHardware, hardware_interface::SystemInterface)