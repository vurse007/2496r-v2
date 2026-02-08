#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/state.hpp"
#include "lynx-v2/sort.hpp"

namespace global {

    inline pros::Rotation horizontal_pod(10);
    inline pros::Rotation vertical_pod(6);
    inline pros::Imu imu(21);
    inline pros::adi::Pneumatics matchLoaderP('H', false);
    inline pros::Controller con(pros::E_CONTROLLER_MASTER);
    
    inline int opticalPort = 1;
    inline char sorterPistonPort = 'B';
    inline lynx::ColorSort colorSort(&opticalPort, &sorterPistonPort);

    inline pros::adi::Pneumatics intakeRamp('E', false);
    inline pros::adi::Pneumatics wingPiston('B', false);
    inline pros::adi::Pneumatics fourBarPiston('A', false);

    // ------------------------------------------------------------
    // STATE DRIVE CONFIGURATION (4 fixed chassis + 4 shiftable)
    // ------------------------------------------------------------
    inline lynx::state_drive chassis {
        {
            {18, pros::v5::MotorGears::blue},   // left permanent 1
            {-17, pros::v5::MotorGears::blue},    // left permanent 2
            {-19, pros::v5::MotorGears::blue}
        },
        {
            {-8, pros::v5::MotorGears::blue},   // right permanent 1
            {7, pros::v5::MotorGears::blue},    // right permanent 2
            {10, pros::v5::MotorGears::blue}
        },

        // ---------------------
        // CHASSIS PARAMETERS
        // ---------------------
        2.75,   // wheel diameter
        1,   // external gear ratio
        12.0,   // track width
        &imu,
        &vertical_pod,

        // ---------------------
        // PISTON PORTS
        // ---------------------
        'B',    // pistonA → routes extraA (chassis <-> intake)
        // ---------------------
        // SHIFTABLE MOTOR GROUP A (piston A)
        // ---------------------
        std::vector<lynx::motor_specs>{
            {20, pros::v5::MotorGears::blue}, // LEFT
            {-9, pros::v5::MotorGears::blue}  // RIGHT
        }
    };

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
