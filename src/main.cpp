#include "main.h"
#include "config.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    global::imu.reset();              // starts internal calibration
    while (global::imu.is_calibrating()) {
        pros::delay(20);
    }

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    global::chassis.tare();
    global::chassis.straight(20);
}

using namespace global;
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

long long int counter = 0;

void opcontrol() {
    global::con.clear();
    global::odom.reset();


    while (true) {
        // Update odometry
        // con.print(0, 0, "IMU: %.2f", imu.get_heading());
        // pros::delay(100);
        global::odom.update();

    if (counter % 50 == 0 && counter % 100 != 0 && counter % 150 != 0) {
        con.print(0, 0, "X: %.2f | Y: %.2f", odom.current_pos.x, odom.current_pos.y);
    }
    else if (counter % 50 != 0 && counter % 100 != 0 && counter % 150 == 0) {
        con.print(0, 0, "I: %.2f", odom.current_pos.theta);
    }
    

    counter += 1;
    pros::delay(5);
    }
}