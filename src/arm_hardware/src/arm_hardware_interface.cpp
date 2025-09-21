#include "arm_hardware/arm_hardware_interface.hpp"
#include <sstream>
#include <vector>
#include <string>


double map_value(double value, double from_min, double from_max, double to_min, double to_max) {
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

namespace arm_hardware
{
// MODIFICADO: Funções renomeadas para corresponder aos nomes de juntas do projeto antigo (o nosso padrão)

// ajuste cada um de acordo com a calibração real do servo
const int WAIST_OFFSET     = 10;   // antigo 10, testar -1, testar 36
const int SHOULDER_OFFSET    = 34; // Antiga LIFT_OFFSET
const int ELBOW_OFFSET   = -88; 
const int WRIST_OFFSET   = -16; 
const int GRIPPER_OFFSET = -2; 

// WAIST (Antiga PAN)
int waist_rad_to_ticks(double rad) {
    return static_cast<int>(map_value(rad, -2.617, 2.617, 220, 815)) + WAIST_OFFSET; // 230 - 815 
}
double waist_ticks_to_rad(int ticks) { 
    return map_value(ticks - WAIST_OFFSET, 200, 815, -2.617, 2.617); 
}

// SHOULDER (Antiga LIFT)
int shoulder_rad_to_ticks(double rad) { 
    return static_cast<int>(map_value(rad, -2.200, 2.160, 165, 900)) + SHOULDER_OFFSET; 
}
double shoulder_ticks_to_rad(int ticks) { 
    return map_value(ticks - SHOULDER_OFFSET, 165, 900, -2.200, 2.160); 
}

// ELBOW
int elbow_rad_to_ticks(double rad) { 
    return static_cast<int>(map_value(rad, -2.420, 2.420, 300, 900)) + ELBOW_OFFSET; 
}
double elbow_ticks_to_rad(int ticks) { 
    return map_value(ticks - ELBOW_OFFSET, 300, 900, -2.420, 2.420); 
}

// WRIST
int wrist_rad_to_ticks(double rad) { 
    return static_cast<int>(map_value(rad, -1.720, 1.720, 200, 820)) + WRIST_OFFSET; 
}
double wrist_ticks_to_rad(int ticks) { 
    return map_value(ticks - WRIST_OFFSET, 200, 820, -1.720, 1.720); 
}

// GRIPPER
int gripper_rad_to_ticks(double rad) { 
    return static_cast<int>(map_value(rad, -0.600, 0.508, 390, 600)) + GRIPPER_OFFSET; 
}
double gripper_ticks_to_rad(int ticks) { 
    return map_value(ticks - GRIPPER_OFFSET, 390, 600, -0.600, 0.508); 
}


hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_states_.resize(info_.joints.size(), 0.0);
    // MODIFICADO & CORRIGIDO: O nome do parâmetro no seu XACRO é "serial_port", não "serial_port_name". 
    // Usar .at() com o nome errado causaria uma falha.
    serial_port_name_ = info_.hardware_parameters.at("serial_port");
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
         arduino_serial_.FlushIOBuffers(); 
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"), "Falha ao abrir porta serial %s: %s", serial_port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"), "Enviando para a posição HOME...");
    std::stringstream home_command;
    home_command << "P 518 570 513 493 500\n"; // Waist, Shoulder, Elbow, Wrist, Gripper
    arduino_serial_.Write(home_command.str());

    // Espera um pouco e lê o estado inicial para sincronizar
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    for(size_t i = 0; i < hw_states_.size(); i++) {
        hw_commands_[i] = hw_states_[i];
    }

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
    if (!arduino_serial_.IsOpen()) return hardware_interface::return_type::ERROR;

    try {
        std::string response;
        arduino_serial_.ReadLine(response, '\n', 100);

        if (response.rfind("S ", 0) == 0) {
            std::stringstream ss(response.substr(2));
            std::vector<int> new_ticks;
            int tick_value;
            while(ss >> tick_value) {
                new_ticks.push_back(tick_value);
            }

            if (new_ticks.size() == 5) {
                for (size_t i = 0; i < hw_states_.size(); ++i) {
                    // MODIFICADO: Chamando as funções com os nomes padronizados
                    switch(i) {
                        case 0: hw_states_[i] = waist_ticks_to_rad(new_ticks[i]); break;
                        case 1: hw_states_[i] = shoulder_ticks_to_rad(new_ticks[i]); break;
                        case 2: hw_states_[i] = elbow_ticks_to_rad(new_ticks[i]); break;
                        case 3: hw_states_[i] = wrist_ticks_to_rad(new_ticks[i]); break;
                        case 4: hw_states_[i] = gripper_ticks_to_rad(new_ticks[i]); break;
                    }
                }
            }
        }
    } catch (const LibSerial::ReadTimeout &) {
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
        // MODIFICADO: Chamando as funções com os nomes padronizados
        switch(i) {
            case 0: ticks = waist_rad_to_ticks(hw_commands_[i]); break;
            case 1: ticks = shoulder_rad_to_ticks(hw_commands_[i]); break;
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
