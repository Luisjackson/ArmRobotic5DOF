#include "arm_hardware/arm_hardware_interface.hpp"
#include <sstream>
#include <vector>
#include <string>


double map_value(double value, double from_min, double from_max, double to_min, double to_max) {
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

namespace arm_hardware
{

// Junta 0: arm_shoulder_pan_joint
int pan_rad_to_ticks(double rad) { return static_cast<int>(map_value(rad, -2.617, 2.617, 200, 815)); }
double pan_ticks_to_rad(int ticks) { return map_value(ticks, 200, 815, -2.617, 2.617); }

// Junta 1: arm_shoulder_lift_joint
int lift_rad_to_ticks(double rad) { return static_cast<int>(map_value(rad, -2.200, 2.160, 165, 900)); }
double lift_ticks_to_rad(int ticks) { return map_value(ticks, 165, 900, -2.200, 2.160); }

// Junta 2: arm_elbow_flex_joint
int elbow_rad_to_ticks(double rad) { return static_cast<int>(map_value(rad, -2.420, 2.420, 300, 900)); }
double elbow_ticks_to_rad(int ticks) { return map_value(ticks, 300, 900, -2.420, 2.420); }

// Junta 3: arm_wrist_flex_joint
int wrist_rad_to_ticks(double rad) { return static_cast<int>(map_value(rad, -1.720, 1.720, 200, 820)); }
double wrist_ticks_to_rad(int ticks) { return map_value(ticks, 200, 820, -1.720, 1.720); }

// Junta 4: crane_plus_moving_finger_joint (Garra)
int gripper_rad_to_ticks(double rad) { return static_cast<int>(map_value(rad, -0.600, 0.508, 390, 600)); }
double gripper_ticks_to_rad(int ticks) { return map_value(ticks, 390, 600, -0.600, 0.508); }

hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);
    serial_port_name_ = info_.hardware_parameters.at("serial_port_name");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, "position", &hw_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, "position", &hw_commands_[i]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Ativando hardware...");
    try {
        arduino_serial_.Open(serial_port_name_);
        arduino_serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"), "Falha ao abrir porta serial %s: %s", serial_port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Ao ativar, envia o robô para a posição HOME
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Enviando para a posição HOME...");
    std::stringstream home_command;
    home_command << "P 505 560 515 515 505\n"; // Pan, Lift, Elbow, Wrist, Gripper
    arduino_serial_.Write(home_command.str());

    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Hardware ativado com sucesso na posição HOME.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Desativando hardware...");
    if (arduino_serial_.IsOpen()) {
        arduino_serial_.Close();
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Hardware desativado.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < hw_states_.size(); ++i) {
        hw_states_[i] = hw_commands_[i];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!arduino_serial_.IsOpen()) return hardware_interface::return_type::ERROR;
    
    std::stringstream command_stream;
    command_stream << "P ";
    
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
        int ticks = 0;
        // Chama a função de conversão correta para cada junta
        switch(i) {
            case 0: ticks = pan_rad_to_ticks(hw_commands_[i]); break;
            case 1: ticks = lift_rad_to_ticks(hw_commands_[i]); break;
            case 2: ticks = elbow_rad_to_ticks(hw_commands_[i]); break;
            case 3: ticks = wrist_rad_to_ticks(hw_commands_[i]); break;
            case 4: ticks = gripper_rad_to_ticks(hw_commands_[i]); break;
        }
        command_stream << ticks;
        if (i < hw_commands_.size() - 1) {
            command_stream << " ";
        }
    }
    command_stream << "\n";

    try {
        arduino_serial_.Write(command_stream.str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"), "Falha ao enviar comando: %s", e.what());
    }

    return hardware_interface::return_type::OK;
}

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardwareInterface,
    hardware_interface::SystemInterface
)