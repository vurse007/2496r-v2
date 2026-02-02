#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/state.hpp"
#include "lynx-v2/sort.hpp"

namespace global {

    inline pros::Rotation horizontal_pod(10);
    inline pros::Rotation vertical_pod(16);
    inline pros::Imu imu(14);
    inline pros::adi::Pneumatics matchLoaderP('A', false);
    inline pros::Controller con(pros::E_CONTROLLER_MASTER);
    
    inline int opticalPort = 1;
    inline char sorterPistonPort = 'B';
    inline lynx::ColorSort colorSort(&opticalPort, &sorterPistonPort);

    inline pros::adi::Pneumatics intakeRamp('F', false);
    inline pros::adi::Pneumatics wingPiston('C', false);
    inline pros::adi::Pneumatics fourBarPiston('A', false);

    // ------------------------------------------------------------
    // STATE DRIVE CONFIGURATION (4 fixed chassis + 4 shiftable)
    // ------------------------------------------------------------
    inline lynx::state_drive chassis {
        {
            {13, pros::v5::MotorGears::blue},   // left permanent 1
            { -12, pros::v5::MotorGears::blue}    // left permanent 2
        },
        {
            {1, pros::v5::MotorGears::blue},   // right permanent 1
            {-2, pros::v5::MotorGears::blue}    // right permanent 2
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
        'X',    // pistonB → routes extraB (chassis <-> flywheel)

        // ---------------------
        // SHIFTABLE MOTOR GROUP A (piston A)
        // ---------------------
        std::vector<lynx::motor_specs>{
            {-20, pros::v5::MotorGears::blue}, // LEFT
            {10, pros::v5::MotorGears::blue}  // RIGHT
        },

        // ---------------------
        // SHIFTABLE MOTOR GROUP B (piston B)
        // ---------------------
        std::vector<lynx::motor_specs>{
            {-11, pros::v5::MotorGears::blue}, // LEFT
            {3, pros::v5::MotorGears::blue}   // RIGHT
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
