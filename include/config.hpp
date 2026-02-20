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
    // inline char sorterPistonPort = 'B';
    // inline lynx::ColorSort colorSort(&opticalPort, &sorterPistonPort);

    inline pros::adi::Pneumatics intakeRampLower('E', false);
    inline pros::adi::Pneumatics intakeRampUpper('F', false);
    inline pros::adi::Pneumatics wingPiston('B', false);

    enum class intakeState {
        MID_GOAL,
        STORAGE,
        HIGH_GOAL
    };

    inline void set_intake(intakeState state){
        switch(state){
            case intakeState::MID_GOAL:
                intakeRampLower.set_value(false);
                intakeRampUpper.set_value(false);
            case intakeState::STORAGE:
                intakeRampLower.set_value(true);
                intakeRampUpper.set_value(false);
            case intakeState::HIGH_GOAL:
                intakeRampLower.set_value(true);
                intakeRampUpper.set_value(true);
        }
    }

    // ------------------------------------------------------------
    // STATE DRIVE CONFIGURATION (4 fixed chassis + 4 shiftable)
    // ------------------------------------------------------------
    inline lynx::state_drive chassis {
        {
            {3, pros::v5::MotorGears::blue},   // left permanent 1
            {-4, pros::v5::MotorGears::blue},    // left permanent 2
            {-2, pros::v5::MotorGears::blue}
        },
        {
            {-8, pros::v5::MotorGears::blue},   // right permanent 1
            {9, pros::v5::MotorGears::blue},    // right permanent 2
            {7, pros::v5::MotorGears::blue}
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
            {1, pros::v5::MotorGears::blue}, // LEFT
            {-10, pros::v5::MotorGears::blue}  // RIGHT
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
