#ifndef ELFIN_ROS2_CONTROL_HW_INTERFACE_ROS2
#define ELFIN_ROS2_CONTROL_HW_INTERFACE_ROS2

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <elfin_ethercat_driver/elfin_ethercat_driver.h>

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

namespace elfin_hardware_interface
{

class ElfinHWInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElfinHWInterface)

  // Lifecycle methods
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interface methods
  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  
  // EtherCAT driver
  std::unique_ptr<elfin_ethercat_driver::EtherCatManager> ethercat_manager_;
  std::vector<std::unique_ptr<elfin_ethercat_driver::ElfinEtherCATDriver>> ethercat_drivers_;
  
  // Node for ROS2 communication
  rclcpp::Node::SharedPtr node_;
  
  // Configuration
  std::string ethernet_name_;
  std::vector<std::string> driver_names_;
};

} // namespace elfin_hardware_interface

#endif // ELFIN_ROS2_CONTROL_HW_INTERFACE_ROS2