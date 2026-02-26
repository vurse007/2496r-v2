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

inline double apply_turn_curve(double stick_value) {
    // Input is already in ≈ [-127, 127] from controller
    double abs_x = std::abs(stick_value);
    if (abs_x < 1e-6) return 0.0;  // avoid div-by-zero or noise

    // Your function, computed on positive side only
    double curve_positive = abs_x + 0.00004 * abs_x * (std::pow(abs_x, 2) - 127.0 * 127.0)
                                       / (1.0 + std::pow(abs_x / 50.0, 6));
// smelly ooh
    // Mirror sign of original stick
    double output = (stick_value >= 0.0) ? curve_positive : -curve_positive;

    // Final safety clamp (shouldn't be necessary, but good practice)
    return std::clamp(output, -127.0, 127.0);
}

inline void driver() {

    double forward = global::con.get_analog(ANALOG_LEFT_Y);
    double turn_raw = global::con.get_analog(ANALOG_RIGHT_X);

    // Deadband (your original logic)
    if (std::abs(forward) < 5)   forward = 0;
    if (std::abs(turn_raw) < 5)  turn_raw = 0;

    // Apply your non-linear curve **only to the turn axis**
    double turn_curved = apply_turn_curve(turn_raw);

    // Classic arcade mixing
    double left_power  = forward + turn_curved;
    double right_power = forward - turn_curved;

    // Final clamping to motor range
    left_power  = std::clamp(left_power,  -127.0, 127.0);
    right_power = std::clamp(right_power, -127.0, 127.0);

    // Send to chassis
    global::chassis.move(static_cast<int>(left_power),
                            static_cast<int>(right_power));

}

inline void printTemps(){
    lynx::util::print_info(
        pros::millis(),
        &global::con,
        std::vector<std::string>{"CRT", "CLT", "RT", "LT"},
        std::vector<double>{global::chassis.right.get_avg_temp(), global::chassis.left.get_avg_temp(), global::chassis.get_extra_temp(0), global::chassis.get_extra_temp(1)}
    );
}



inline void intakeCon(){  

    static lynx::util::timer intake_reverse_timer(50);
    static bool intake_reversing = false;
    static bool intake_forward_after_burst = false;

    if(global::con.get_digital_new_press(DIGITAL_X)){
        manualIntakeRamp++;
    }

    // ============================
    // 1️⃣ Reverse Burst Phase
    // ============================
    if (intake_reversing) {

        if (!intake_reverse_timer.has_elapsed()) {
            global::chassis.move_intake(-127);
            return;
        }

        // reverse finished → enter forward hold state
        intake_reversing = false;
        intake_forward_after_burst = true;

        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
    }

    // ============================
    // 2️⃣ Forward After Burst
    // ============================
    if (intake_forward_after_burst) {

        global::chassis.move_intake(127);

        // optional cancel if driver presses something else
        if (!global::con.get_digital(DIGITAL_L2)) {
            intake_forward_after_burst = false;
        }

        return;
    }

    // ============================
    // 3️⃣ Manual Controls
    // ============================

    if (global::con.get_digital(DIGITAL_R1) && global::con.get_digital(DIGITAL_R2)){
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(-127);
        return;
    }

    if (global::con.get_digital(DIGITAL_R1)){
        global::set_intake(global::intakeState::STORAGE);
        manualIntakeRamp = 0;
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(127);
        return;
    }

    if (global::con.get_digital(DIGITAL_R2)) {
        global::set_intake(global::intakeState::MID_GOAL);
        manualIntakeRamp = 2;
        global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);
        global::chassis.move_intake(127);
        return;
    }

    if (global::con.get_digital(DIGITAL_L2)){
        global::set_intake(global::intakeState::HIGH_GOAL);
        manualIntakeRamp = 1;

        intake_reverse_timer.restart();
        intake_reversing = true;

        global::chassis.move_intake(-127);
        return;
    }

    // ============================
    // 4️⃣ Idle
    // ============================
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

inline void stateCon(){
    if (global::con.get_digital_new_press(DIGITAL_Y)) {
        global::chassis.set_state(DriveState::CHASSIS_8);
    }
    
    if (global::con.get_digital_new_press(DIGITAL_RIGHT)) global::matchLoaderP.toggle();
    static bool firstToggle = false;
    if (firstToggle){global::wingPiston.set_value(!global::con.get_digital(DIGITAL_L1));}
    else{
        if(global::con.get_digital(DIGITAL_L1)){
            firstToggle = true;
        }
    }
    
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

