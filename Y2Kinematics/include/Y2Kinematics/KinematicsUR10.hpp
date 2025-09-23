#pragma once

#include "Y2Kinematics/Kinematics.hpp"

class KinematicsUR10 : public Kinematics {
private:
    // UR10 Kinematics parameters (mm)
    // Link lengths
    static constexpr double a2 = -612.0;    // Upper arm length
    static constexpr double a3 = -572.3;    // Forearm length
    
    // Link offsets
    static constexpr double d1 = 128.0;    // Base height
    static constexpr double d4 = 163.9;    // Wrist 1 offset
    static constexpr double d5 = 115.7;    // Wrist 2 offset  
    static constexpr double d6 = 92.2;     // Wrist 3 offset
    
public:
    KinematicsUR10(double SamplingTime_ = 0.01, size_t numOfAxis_ = 6, const YMatrix& EE2TCP_ = YMatrix::identity(4)) 
    : Kinematics(SamplingTime_, numOfAxis_, EE2TCP_) {}

    // Forward Kinematics - T06 transformation matrix
    YMatrix forwardKinematics(const std::vector<double>& q) override;
    
    // Jacobian calculation 
    YMatrix calculateJacobian(const std::vector<double>& q) override;
};