#include "main.h"
#include "lynx.hpp"
#include "lynx-v2/util.hpp"

extern std::vector<Auton> autons;
Auton* auton = nullptr;
std::string names;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    global::chassis.set_state(lynx::DriveState::CHASSIS_6_INTAKE_2);

    global::odom.reset();
    global::imu.reset();
    
    while (global::imu.is_calibrating()) {
        pros::delay(20);
    }

    static Auton temp = autons[auton_selector(autons, global::con)];
    names = temp.get_name1() + " " + temp.get_name2();  // Save display name
    auton = &temp;
    
    
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
    if (auton) {(*auton).run();} 
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
lynx::util::timer driver_time;

void opcontrol() {
    global::con.clear();
    global::odom.reset();
    driver_time.restart();

    while (true) {
        // Update odometry
        // con.print(0, 0, "IMU: %.2f", imu.get_heading());
        // pros::delay(100);        
        global::odom.update();

        lynx::util::print_info(
            driver_time.elapsed(), 
            &global::con, 
            {"X", "Y", "Imu"}, 
            {odom.current_pos.x, odom.current_pos.y, lynx::util::to_deg(odom.current_pos.theta)}
        );

        if (global::con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
            (*auton).run();
        }
        
        // if (counter % 50 == 0 && counter % 100 != 0 && counter % 150 != 0) {
        //     con.print(0, 0, "X: %.2f | Y: %.2f", odom.current_pos.x, odom.current_pos.y);
        // }
        // else if (counter % 50 != 0 && counter % 100 != 0 && counter % 150 == 0) {
        //     con.print(0, 0, "I: %.2f", odom.current_pos.theta);
        // }

    counter += 1;
    pros::delay(5);
    }
}