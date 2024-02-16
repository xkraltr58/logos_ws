/**
 * @file hardware_info_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "midi_hardware/hardware_info_node.hpp"

HardwareInfoNode::HardwareInfoNode()
    : rclcpp::Node("midi_hardware")
{

    this->declare_parameter("left_joint_name", rclcpp::PARAMETER_STRING);
    this->declare_parameter("right_joint_name", rclcpp::PARAMETER_STRING);

    this->declare_parameter("pulse_per_revolution", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("gear_ratio", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("max_rpm", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("min_rpm", rclcpp::PARAMETER_INTEGER);

    auto paramsFromServer = this->get_parameters(
        {
            "left_joint_name",
            "right_joint_name",
            "pulse_per_revolution",
            "wheel_radius",
            "gear_ratio",
            "max_rpm",
            "min_rpm"
        }
    );

    HardwareParams hwParams;
    hwParams.leftWheelName = paramsFromServer.at(0).as_string();
    hwParams.rightWheelName = paramsFromServer.at(1).as_string();
    hwParams.pulsePerRevolution = paramsFromServer.at(2).as_int();
    hwParams.wheelRadius = paramsFromServer.at(3).as_double();
    hwParams.gearRatio = paramsFromServer.at(4).as_double();
    hwParams.maxRPM = paramsFromServer.at(5).as_int();
    hwParams.minRPM = paramsFromServer.at(6).as_int();
    
    m_HwParams = hwParams;
}

bool HardwareInfoNode::init()
{
    rclcpp::QoS qos(10);
    qos.reliable().transient_local();

    m_MotorCommandPub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor_command",
        qos
    );

    m_RtMotorCommandPub = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        m_MotorCommandPub
    );

    auto& msg = m_RtMotorCommandPub->msg_;
    msg.data.resize(2);

    m_EncoderDataSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/encoder_data",
        qos,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr encoder_data){
            m_EncoderDataBuffer.writeFromNonRT(
                *encoder_data
            );
        }
    );

    return true;

}

const std_msgs::msg::Float64MultiArray HardwareInfoNode::getEncoderData()
{
    
    const auto currentEncoderData = *m_EncoderDataBuffer.readFromRT();
    std_msgs::msg::Float64MultiArray pubMsg;
    pubMsg.data.resize(2);
    if(currentEncoderData.data.size() != 0){
        pubMsg.data.at(0) = currentEncoderData.data.at(0);
        pubMsg.data.at(1) = currentEncoderData.data.at(1);
    }
    
    return pubMsg;
}

bool HardwareInfoNode::sendCommands(const midi_custom_interfaces::msg::MotorCommand& motor_command_msg)
{   
    
    if(m_RtMotorCommandPub->trylock()){
        auto& msg = m_RtMotorCommandPub->msg_;
        msg.data.at(0) = motor_command_msg.left_pwm;
        msg.data.at(1) = motor_command_msg.right_pwm;
        m_RtMotorCommandPub->unlockAndPublish();
        return true;
    }
    
    return false;
}