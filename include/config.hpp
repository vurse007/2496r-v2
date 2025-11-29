#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/state.hpp"

namespace global {

    inline pros::Rotation horizontal_pod(10);
    inline pros::Rotation vertical_pod(16);
    inline pros::Imu imu(7);
    inline pros::adi::DigitalOut matchLoaderP('A', true);
    inline pros::Controller con(pros::E_CONTROLLER_MASTER);

    // ------------------------------------------------------------
    // STATE DRIVE CONFIGURATION (4 fixed chassis + 4 shiftable)
    // ------------------------------------------------------------
    inline lynx::state_drive chassis {
        {
            {-11, pros::v5::MotorGears::blue},   // left permanent 1
            { 12, pros::v5::MotorGears::blue}    // left permanent 2
        },
        {
            {-13, pros::v5::MotorGears::blue},   // right permanent 1
            { 18, pros::v5::MotorGears::blue}    // right permanent 2
        },

        // ---------------------
        // CHASSIS PARAMETERS
        // ---------------------
        3.25,   // wheel diameter
        0.75,   // external gear ratio
        12.0,   // track width
        &imu,
        &vertical_pod,

        // ---------------------
        // PISTON PORTS
        // ---------------------
        'H',    // pistonA → routes extraA (chassis <-> flywheel)
        'G',    // pistonB → routes extraB (chassis <-> intake)

        // ---------------------
        // SHIFTABLE MOTOR GROUP A (piston A)
        // ---------------------
        std::vector<lynx::motor_specs>{
            {-19, pros::v5::MotorGears::blue}, // LEFT
            { 20, pros::v5::MotorGears::blue}  // RIGHT
        },

        // ---------------------
        // SHIFTABLE MOTOR GROUP B (piston B)
        // ---------------------
        std::vector<lynx::motor_specs>{
            {-1, pros::v5::MotorGears::blue}, // LEFT
            {2, pros::v5::MotorGears::blue}   // RIGHT
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
