#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/state.hpp"
#include "lynx-v2/sort.hpp"
#include "lynx-v2/pto.hpp"

namespace global {

    inline pros::Rotation horizontal_pod(10);
    inline pros::Rotation vertical_pod(16);
    inline pros::Imu imu(14);
    inline pros::adi::Pneumatics matchLoaderP('A', false);
    inline pros::Controller con(pros::E_CONTROLLER_MASTER);

    inline pros::MotorGroup intake({3, 4}, pros::v5::MotorGears::blue);
    
    inline int opticalPort = 1;
    inline char sorterPistonPort = 'B';
    inline lynx::ColorSort colorSort(&opticalPort, &sorterPistonPort);

    inline pros::adi::Pneumatics intakeRamp('F', false);
    inline pros::adi::Pneumatics wingPiston('C', false);
    inline pros::adi::Pneumatics fourBarPiston('A', false);

    inline lynx::drive chassis {
        //left motors
        {
            {13, pros::v5::MotorGears::blue},
            {-12, pros::v5::MotorGears::blue},
            {-20, pros::v5::MotorGears::blue}
        },
        //right motors
        {
            {1, pros::v5::MotorGears::blue},
            {-2, pros::v5::MotorGears::blue},
            {10, pros::v5::MotorGears::blue}
        },

        //chassis params
        2.75,   // wheel diameter
        1,      // external gear ratio
        12.0,   // track width
        &imu,
        &vertical_pod
    };

    inline lynx::PTO drivetrainPTO('D', false);

    // ------------------------------------------------------------
    // ODOMETRY CONFIG (unchanged)
    // ------------------------------------------------------------
    inline lynx::odometry odom{
        2,      // tracking wheel diameter? (your current value)
        0,      // x offset
        0,      // y offset
        &horizontal_pod,
        &vertical_pod
    };



} // namespace global
