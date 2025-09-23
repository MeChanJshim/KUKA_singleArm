#pragma once

#include "Y2Kinematics/Kinematics.hpp"

class KinematicsKUKAiiwa : public Kinematics {
private:
    // Kinematics parameters (mm)
    static constexpr double d1 = 340.0;
    static constexpr double d3 = 400.0;
    static constexpr double d5 = 400.0;
    static constexpr double d7 = 126.0;
    
    
    
public:
    KinematicsKUKAiiwa(double SamplingTime_ = 0.01, size_t numOfAxis_ = 7, const YMatrix& EE2TCP_ = YMatrix::identity(4)) 
    : Kinematics(SamplingTime_, numOfAxis_, EE2TCP_) {}

    // Complete Forward Kinematics (as shown in previous artifact)
    YMatrix forwardKinematics(const std::vector<double>& q) override;
    
    // Complete Jacobian calculation 
    YMatrix calculateJacobian(const std::vector<double>& q) override;
    
    
};