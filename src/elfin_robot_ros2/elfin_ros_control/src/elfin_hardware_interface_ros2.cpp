#include "elfin_ros_control/elfin_hw_interface_ros2.h"
#include <pluginlib/class_list_macros.hpp>

namespace elfin_hardware_interface
{

CallbackReturn ElfinHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Initializing Elfin hardware interface");
  
  // Initialize command and state vectors
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // Get hardware parameters
  ethernet_name_ = info_.hardware_parameters["ethernet_name"];
  
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), 
              "Hardware interface initialized with %zu joints", info_.joints.size());
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Configuring Elfin hardware interface");
  
  // TODO: Parse driver names from parameters
  // For now, create a simple configuration
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Activating Elfin hardware interface");
  
  // Initialize EtherCAT manager
  try
  {
    ethercat_manager_ = std::make_unique<elfin_ethercat_driver::EtherCatManager>(ethernet_name_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), 
                "EtherCAT manager initialized on interface: %s", ethernet_name_.c_str());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ElfinHWInterface"), 
                 "Failed to initialize EtherCAT manager: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  // TODO: Initialize EtherCAT drivers based on configuration
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Deactivating Elfin hardware interface");
  
  // Clean up EtherCAT drivers
  ethercat_drivers_.clear();
  
  // Clean up EtherCAT manager
  ethercat_manager_.reset();
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Cleaning up Elfin hardware interface");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface"), "Shutting down Elfin hardware interface");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElfinHWInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("ElfinHWInterface"), "Error in Elfin hardware interface");
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> ElfinHWInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_states_[i]
      )
    );
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_states_[i]  // Using same pointer for simplicity
      )
    );
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_states_[i]  // Using same pointer for simplicity
      )
    );
  }
  
  return state_interfaces;
}

std::vector<CommandInterface> ElfinHWInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]
      )
    );
  }
  
  return command_interfaces;
}

return_type ElfinHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read actual joint states from EtherCAT drivers
  // For now, just echo commands as states (simulation)
  for (size_t i = 0; i < hw_states_.size(); ++i)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      hw_states_[i] = hw_commands_[i];
    }
  }
  
  return return_type::OK;
}

return_type ElfinHWInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Write commands to EtherCAT drivers
  // For now, just log the commands
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      // In a real implementation, send commands to EtherCAT drivers
      RCLCPP_DEBUG(rclcpp::get_logger("ElfinHWInterface"), 
                   "Command for joint %zu: %f", i, hw_commands_[i]);
    }
  }
  
  return return_type::OK;
}

} // namespace elfin_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  elfin_hardware_interface::ElfinHWInterface,
  hardware_interface::SystemInterface)