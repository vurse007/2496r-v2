#pragma once
#include "main.h"
#include "lynx.hpp"

// ============================================================================
// DRIVER CONTROL
// ============================================================================
/*
| Button | Behavior  | Transmission               | Intake | Flywheel | Chassis |
| ------ | --------- | -------------------------- | ------ | -------- | ------- |
| **R1** | Storage   | 6 + 2 (intake)             | +127   | off      |   600   |
| **R2** | Outtake   | 6 + 2 (intake)             | -127   | off      |   600   |
| **L1** | Low Goal  | 6 + 2 (intake)             | +127   | off      |   600   |
| **L2** | High Goal | 4 + 2 + 2                  | +127   | +127     |   200   |
| **B**  | Neutral   | 6 + 2 (flywheel)           | N/A    | N/A      |   200   |
| *DOWN* | Neutral   | 8                          | N/A    | N/A      |   600   |
| *None* | Neutral   | stays in last forced state | off    | off      |   N/A   |
*/

using namespace lynx;

//DELETE LATERRRRR
inline int cookedCount = 0;

inline void driver() {

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

inline void intakeCon(){   
    if (global::con.get_digital(DIGITAL_R1)){
        global::intake.move(127);
    } else if (global::con.get_digital(DIGITAL_R2)){
        global::intake.move(-127);
    } else {
        global::intake.move(0);
    }
}

inline void stateCon(){
    if (global::con.get_digital_new_press(DIGITAL_DOWN)) global::drivetrainPTO.toggle();             // 8m 600
    
    if (global::con.get_digital_new_press(DIGITAL_RIGHT)) global::matchLoaderP.toggle();

    if (global::con.get_digital_new_press(DIGITAL_B)){
        global::wingPiston.toggle();
    }
}

inline void otherCon(){
    if (global::con.get_digital_new_press(DIGITAL_LEFT)) {global::colorSort.toggle();} 
}

inline void driverCon(){
    driver();
    intakeCon();
    stateCon();
}

