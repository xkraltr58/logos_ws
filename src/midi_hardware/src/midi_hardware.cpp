/**
 * @file midi_hardware.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "midi_hardware/midi_hardware.hpp"

hardware_interface::CallbackReturn midi_hardware::MidiSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
{

    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
        return CallbackReturn::ERROR;   
    }

    m_HardwareInfoNode = std::make_shared<HardwareInfoNode>(

    );

    bool hwInitOk = m_HardwareInfoNode->init();

    if(!hwInitOk){
        return CallbackReturn::ERROR;
    }



    for(const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        m_JointsMap[joint.name] = Joint();
        m_JointsMap.at(joint.name).jointName = joint.name;
        m_JointsMap.at(joint.name).currentPosition = std::numeric_limits<double>::quiet_NaN();
        m_JointsMap.at(joint.name).currentVelocity = std::numeric_limits<double>::quiet_NaN();
        m_JointsMap.at(joint.name).targetPosition = std::numeric_limits<double>::quiet_NaN();
        m_JointsMap.at(joint.name).targetVelocity = std::numeric_limits<double>::quiet_NaN();
    }

    m_HwParams = m_HardwareInfoNode->getHardwareParams();

    /* m_MapSlope = (m_MaxPWM - m_MinPWM) / (m_HwParams.maxRPM - m_HwParams.minRPM); */
    m_MapSlope = (255 - 0) / (1000- 0);

    m_HardwareNodeSpinner.add_node(m_HardwareInfoNode);

    return hardware_interface::CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> midi_hardware::MidiSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

  /* for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces; */
    for(auto& [key, value] : m_JointsMap)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                value.jointName,
                hardware_interface::HW_IF_POSITION,
                &value.currentPosition     
            )
        );

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                value.jointName,
                hardware_interface::HW_IF_VELOCITY,
                &value.currentVelocity     
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> midi_hardware::MidiSystemHardware::export_command_interfaces()
{

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(auto& [key, value] : m_JointsMap)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                value.jointName,
                hardware_interface::HW_IF_VELOCITY,
                &value.targetVelocity
            )
        );
    }
    
    return command_interfaces;
}

hardware_interface::CallbackReturn midi_hardware::MidiSystemHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    for(auto& [key, value] : m_JointsMap)
    {
        value.currentPosition = 0.0;
        value.currentVelocity = 0.0;
        value.targetPosition = 0.0;
        value.targetVelocity = 0.0;
    }

    std::thread([this](){m_HardwareNodeSpinner.spin();}).detach();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn midi_hardware::MidiSystemHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type midi_hardware::MidiSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    midi_custom_interfaces::msg::MotorCommand pwms;
    
    
    
    auto velCommandsRPM = wheelAngVelToMotorRPM(
        m_JointsMap.at(m_HwParams.leftWheelName).targetVelocity,
        m_JointsMap.at(m_HwParams.rightWheelName).targetVelocity
    );

    pwms = mapRpmToPwm(velCommandsRPM);
    RCLCPP_INFO(rclcpp::get_logger("MidiSystemHardware"), "Target velocities: %d, %d", pwms.left_pwm, pwms.right_pwm);
    
    m_HardwareInfoNode->sendCommands(
        pwms
    );
    

    return hardware_interface::return_type::OK;

}

hardware_interface::return_type midi_hardware::MidiSystemHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    auto encoderData = m_HardwareInfoNode->getEncoderData();

    if(encoderData.data.size() != 0){
        m_JointsMap.at(m_HwParams.leftWheelName).currentPosition = motorIncrementToWheelPosition(encoderData.data.at(0));
    m_JointsMap.at(m_HwParams.rightWheelName).currentPosition = motorIncrementToWheelPosition(encoderData.data.at(1));
    }
    
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  midi_hardware::MidiSystemHardware, hardware_interface::SystemInterface)