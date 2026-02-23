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

inline int manualIntakeRamp = 0;

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

inline void printTemps(){
    lynx::util::print_info(
        pros::millis(),
        &global::con,
        std::vector<std::string>{"ChasRT", "ChasLT", "IntRT", "IntLT"},
        std::vector<double>{global::chassis.right.get_avg_temp(), global::chassis.left.get_avg_temp(), global::chassis.get_extra_temp(0), global::chassis.get_extra_temp(1)}
    );
}

inline void intakeCon(){  
    if(global::con.get_digital_new_press(DIGITAL_X)){
        manualIntakeRamp++;
    }
    if (global::con.get_digital(DIGITAL_R1) && global::con.get_digital(DIGITAL_R2)){
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(-127);
    }
    else if (global::con.get_digital(DIGITAL_R1)){
        global::set_intake(global::intakeState::STORAGE);
        manualIntakeRamp = 0;
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(127);
    }
    else if (global::con.get_digital(DIGITAL_R2)){
        global::set_intake(global::intakeState::HIGH_GOAL);
        manualIntakeRamp = 1;
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(127);
    }
    else if (global::con.get_digital(DIGITAL_L2)){
        global::set_intake(global::intakeState::MID_GOAL);
        manualIntakeRamp = 2;
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(90);
    }
    else {
        global::chassis.move_intake(0);
        if (manualIntakeRamp == 0){
            global::set_intake(global::intakeState::STORAGE);
        }
        else if (manualIntakeRamp == 1){
            global::set_intake(global::intakeState::HIGH_GOAL);
        }
        else if (manualIntakeRamp == 2){
            global::set_intake(global::intakeState::MID_GOAL);
        }
        else {
            manualIntakeRamp = 0;
        }
    }
}

inline void stateCon(){
    if (global::con.get_digital_new_press(DIGITAL_Y)) {
        global::chassis.set_state(DriveState::CHASSIS_8);
    }
    
    if (global::con.get_digital_new_press(DIGITAL_RIGHT)) global::matchLoaderP.toggle();

    global::wingPiston.set_value(global::con.get_digital(DIGITAL_L1));
}

inline void otherCon(){
    //if (global::con.get_digital_new_press(DIGITAL_LEFT)) {global::colorSort.toggle();} 
}

inline void driverCon(){
    driver();
    intakeCon();
    stateCon();
    //printTemps();
}

