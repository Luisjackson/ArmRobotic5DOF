#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include <vector>
#include <string>
#include <libserial/SerialPort.h>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware
{
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    LibSerial::SerialPort arduino_serial_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    std::string serial_port_name_;
};
}
#endif

