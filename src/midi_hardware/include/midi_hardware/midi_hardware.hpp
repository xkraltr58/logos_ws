/**
 * @file midi_hardware.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MIDI_HARDWARE_HPP_
#define MIDI_HARDWARE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "midi_hardware/midi_hardware_defs.hpp"
#include "midi_hardware/visibility_control.h"
#include "midi_hardware/hardware_info_node.hpp"

namespace midi_hardware
{

  class MidiSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MidiSystemHardware);

    MIDI_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    MIDI_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MIDI_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MIDI_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    MIDI_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    MIDI_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    MIDI_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // std::vector<double> hw_commands_;
    // std::vector<double> hw_positions_;
    // std::vector<double> hw_velocities_;

    std::unordered_map<std::string, Joint> m_JointsMap;
    std::map<std::string, std::string> m_JointNamesMap;
    std::shared_ptr<HardwareInfoNode> m_HardwareInfoNode;

    rclcpp::executors::SingleThreadedExecutor m_HardwareNodeSpinner;

    HardwareParams m_HwParams;

    double m_MapSlope = 0.0;

    const int32_t m_MinPWM = 0.0;
    const int32_t m_MaxPWM = 255.0;

    inline const double motorIncrementToWheelPosition(const int64_t motor_increment) const
    {
      return (double)(((double)motor_increment / (double)m_HwParams.pulsePerRevolution) * (2.0 * M_PI) / m_HwParams.gearRatio);
    }

    // inline const int32_t motorVelocityToWheelVelocity(const int64_t motor_increment) const
    //{
    //
    // }

    /* inline const midi_custom_interfaces::msg::MotorCommand angularVelToPWM(
      const double left_wheel_ang_vel,
      const double right_wheel_ang_vel
    )
    {

    } */

    inline const std::pair<int32_t, int32_t> wheelAngVelToMotorRPM(
        const double left_wheel_ang_vel,
        const double right_wheel_ang_vel)
    {
      double leftMotorRPM, rightMotorRPM = 0.0;
      return std::make_pair(
          (int32_t)(left_wheel_ang_vel * 60.0 * m_HwParams.gearRatio) / (2.0 * M_PI),
          (int32_t)(right_wheel_ang_vel * 60.0 * m_HwParams.gearRatio) / (2.0 * M_PI));
    }

    const midi_custom_interfaces::msg::MotorCommand mapRpmToPwm(
        const std::pair<int32_t, int32_t> &rpms)
    {

      midi_custom_interfaces::msg::MotorCommand newCmd;
      int16_t leftPWM, rightPWM = 0;
      leftPWM = m_MinPWM + ((255.0 - 0.0) / (1000.0 - 0.0)) * (rpms.first - m_HwParams.minRPM);
      rightPWM = m_MinPWM + ((255.0 - 0.0) / (1000.0 - 0.0)) * (rpms.second - m_HwParams.minRPM);

      newCmd.left_pwm = leftPWM;
      newCmd.right_pwm = rightPWM;

      return newCmd;
    }
  };

} // End of namespace midi_hardware

#endif // MIDI_HARDWARE_HPP_