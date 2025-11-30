#pragma once
#include "chassis.hpp"

namespace lynx {

// -----------------------------------------------------------------------------
// DriveState - logical transmission modes
// -----------------------------------------------------------------------------
enum class DriveState {
    CHASSIS_8,                      // 8 motors on drivetrain (4 fixed + 4 shiftable)
    CHASSIS_6_INTAKE_2,             // 6 drivetrain (4 fixed + extraB), 2 intake (extraA)
    CHASSIS_6_FLYWHEEL_2,           // 6 drivetrain (4 fixed + extraA), 2 flywheel (extraB)
    CHASSIS_4_INTAKE_2_FLYWHEEL_2   // 4 drivetrain (fixed only), 2 intake (extraA), 2 flywheel (extraB)
};

// -----------------------------------------------------------------------------
// state_drive - extends drive with 2 pistons + 4 shiftable motors
// -----------------------------------------------------------------------------
class state_drive : public drive {
private:
    // Pneumatics for routing extra motors
    pros::adi::Pneumatics pistonA;   // routes extraA between chassis <-> intake
    pros::adi::Pneumatics pistonB;   // routes extraB between chassis <-> flywheel

    // The 4 shiftable motors (each has 1 left + 1 right)
    group extraA;   // controlled by piston A
    group extraB;   // controlled by piston B

    DriveState curr_state = DriveState::CHASSIS_6_INTAKE_2;

public:

    // --------------------------------------------------------------
    // CONSTRUCTOR
    // --------------------------------------------------------------
    state_drive(
        const std::vector<motor_specs>& left_fixed,
        const std::vector<motor_specs>& right_fixed,
        double wheel_diameter,
        double external_ratio,
        double track_width,
        pros::Imu* imu,
        pros::Rotation* distance_pod,
        char pistonA_port,
        char pistonB_port,
        const std::vector<motor_specs>& extraA_specs,
        const std::vector<motor_specs>& extraB_specs
    )
    : drive(left_fixed, right_fixed, wheel_diameter, external_ratio, track_width, imu, distance_pod),
      pistonA(pistonA_port, false),
      pistonB(pistonB_port, false),
      extraA(extraA_specs),
      extraB(extraB_specs)
    {
        // default: 6 drive + 2 intake
        curr_state = DriveState::CHASSIS_6_INTAKE_2;
        pistonA.set_value(false);  // extraA → intake
        pistonB.set_value(true);   // extraB → drivetrain
    }

    // --------------------------------------------------------------
    // STATE CONTROL - sets pistons according to logical state
    // --------------------------------------------------------------
    void set_state(DriveState new_state) {
        curr_state = new_state;

        switch (new_state) {
            case DriveState::CHASSIS_8:
                // A: chassis, B: chassis
                pistonA.set_value(true);    // extraA → drivetrain
                pistonB.set_value(true);    // extraB → drivetrain
                break;
            case DriveState::CHASSIS_6_INTAKE_2:
                // A: intake, B: chassis
                pistonA.set_value(false);   // extraA → intake
                pistonB.set_value(true);    // extraB → drivetrain
                break;
            case DriveState::CHASSIS_6_FLYWHEEL_2:
                // A: chassis, B: flywhel
                pistonA.set_value(true);    // extraA → drivetrain
                pistonB.set_value(false);   // extraB → flywheel
                break;
            case DriveState::CHASSIS_4_INTAKE_2_FLYWHEEL_2:
                // A: intake, B: flywheel
                pistonA.set_value(false);   // extraA → intake
                pistonB.set_value(false);   // extraB → flywheel
                break;
        }
    }

    DriveState get_state() const {
        return curr_state;
    }

    // --------------------------------------------------------------
    // DRIVE OVERRIDE — routes motors based on current state
    // --------------------------------------------------------------
    void move(int left_power, int right_power) override {

        // Always move the fixed 4 chassis motors
        drive::move(left_power, right_power);

        switch (curr_state) {

            case DriveState::CHASSIS_8:
                // extraA assists drivetrain
                if (auto mL = extraA.get_motor(0)) mL->move(left_power);
                if (auto mR = extraA.get_motor(1)) mR->move(right_power);

                // extraB assists drivetrain
                if (auto mL = extraB.get_motor(0)) mL->move(left_power);
                if (auto mR = extraB.get_motor(1)) mR->move(right_power);
                break;
            case DriveState::CHASSIS_6_INTAKE_2:
                // extraB helps drivetrain
                if (auto mL = extraB.get_motor(0)) mL->move(left_power);
                if (auto mR = extraB.get_motor(1)) mR->move(right_power);

                // extraA is intake → do NOT move here
                break;

            case DriveState::CHASSIS_4_INTAKE_2_FLYWHEEL_2:
                // No shiftable motors help drivetrain in this mode
                break;
        }
    }

    // --------------------------------------------------------------
    // INTAKE CONTROL — extraA is ALWAYS intake in states 2 and 3
    // --------------------------------------------------------------
    void move_intake(int power) {
        switch (curr_state) {
            case DriveState::CHASSIS_8:
                break; // no intake in pure drivetrain mode

            case DriveState::CHASSIS_6_INTAKE_2:
            case DriveState::CHASSIS_6_FLYWHEEL_2:
            case DriveState::CHASSIS_4_INTAKE_2_FLYWHEEL_2:
                if (auto mL = extraA.get_motor(0)) mL->move(power);
                if (auto mR = extraA.get_motor(1)) mR->move(power);
                break;
        }
    }

    // --------------------------------------------------------------
    // FLYWHEEL CONTROL — extraB is ONLY flywheel in state 3
    // --------------------------------------------------------------
    void move_flywheel(int power) {
        switch (curr_state) {
            case DriveState::CHASSIS_8:
            case DriveState::CHASSIS_6_INTAKE_2:
            case DriveState::CHASSIS_6_FLYWHEEL_2:
                break; // no flywheel in these modes

            case DriveState::CHASSIS_4_INTAKE_2_FLYWHEEL_2:
                if (auto mL = extraB.get_motor(0)) mL->move(power);
                if (auto mR = extraB.get_motor(1)) mR->move(power);
                break;
        }
    }

}; // class state_drive

} // namespace lynx
