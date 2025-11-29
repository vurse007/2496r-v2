#pragma once
#include "main.h"
#include "lynx.hpp"

// ============================================================================
// DRIVER CONTROL
// ============================================================================
/*
| Button | Behavior  | Transmission               | Intake | Flywheel |
| ------ | --------- | -------------------------- | ------ | -------- |
| **R1** | Storage   | 6 + 2                      | +127   | off      |
| **R2** | Outtake   | 6 + 2                      | -127   | off      |
| **L1** | Low Goal  | 6 + 2                      | +127   | off      |
| **L2** | High Goal | 4 + 2 + 2                  | +127   | +127     |
| None   | Neutral   | stays in last forced state | off    | off      |
*/

using namespace lynx;

inline void driver() {

    // ========================================================================
    // 1. STORAGE MODE  — R1 (HOLD)
    // ========================================================================
    if (global::con.get_digital(DIGITAL_R1)) {

        // Force to 6-drive, 2-intake (no reverting on release)
        global::chassis.set_state(DriveState::CHASSIS_6_INTAKE_2);

        // -------- piston logic here --------

        // Intake forward
        global::chassis.move_intake(127);
    }

    // ========================================================================
    // 2. OUTTAKE — R2 (HOLD)
    // ========================================================================
    else if (global::con.get_digital(DIGITAL_R2)) {

        global::chassis.set_state(DriveState::CHASSIS_6_INTAKE_2);

        // -------- piston logic here --------

        global::chassis.move_intake(-127);
    }

    // ========================================================================
    // 3. SCORING MODE — LOW GOAL (L1 HOLD)
    // ========================================================================
    else if (global::con.get_digital(DIGITAL_L1)) {

        global::chassis.set_state(DriveState::CHASSIS_6_INTAKE_2);

        // -------- piston logic here --------

        global::chassis.move_intake(127);
    }

    // ========================================================================
    // 4. SCORING MODE — HIGH GOAL (L2 HOLD)
    // ========================================================================
    else if (global::con.get_digital(DIGITAL_L2)) {

        global::chassis.set_state(DriveState::CHASSIS_4_INTAKE_2_FLYWHEEL_2);

        // -------- piston logic here --------

        global::chassis.move_intake(127);
        global::chassis.move_flywheel(127);
    }

    // ========================================================================
    // 5. NO BUTTON → STOP MECHANISMS
    // ========================================================================
    else {
        global::chassis.move_intake(0);
        global::chassis.move_flywheel(0);
    }


    // ========================================================================
    // ======================== DRIVETRAIN CONTROL ============================
    // ========================================================================

    // Single-stick arcade
    double forward = global::con.get_analog(ANALOG_LEFT_Y);
    double turn    = global::con.get_analog(ANALOG_RIGHT_X);

    // Deadband
    if (std::abs(forward) < 5) forward = 0;
    if (std::abs(turn)    < 5) turn    = 0;

    // Arcade mixing
    int left_power  = forward + turn;
    int right_power = forward - turn;

    left_power  = std::clamp(left_power,  -127, 127);
    right_power = std::clamp(right_power, -127, 127);

    // state_drive overrides move() so routing is automatic
    global::chassis.move(left_power, right_power);
}


