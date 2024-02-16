/**
 * @file midi_hardware_defs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MIDI_HARDWARE_DEFS_HPP_
#define MIDI_HARDWARE_DEFS_HPP_

#include <string>

struct Joint
{
    
    double currentPosition;
    double currentVelocity;
    double targetPosition;
    double targetVelocity;

    std::string jointName;

    Joint()
        : currentPosition(0.0), currentVelocity(0.0), targetPosition(0.0), targetVelocity(0.0)
    {
        
    }

};

struct HardwareParams
{
    std::string leftWheelName;
    std::string rightWheelName;

    int64_t pulsePerRevolution;
    double wheelRadius;

    double gearRatio;

    int32_t maxRPM;
    int32_t minRPM;

};  


#endif // MIDI_HARDWARE_DEFS_HPP_