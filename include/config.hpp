#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"

namespace global {

    pros::Rotation horizontal_pod(10);
    pros::Rotation vertical_pod(16);
    pros::Imu imu(7);

    pros::Controller con(pros::E_CONTROLLER_MASTER);

    inline lynx::drive chassis {
        {{-11, pros::v5::MotorGears::blue}, {12, pros::v5::MotorGears::blue}, {-13, pros::v5::MotorGears::blue}},//left motors
        {{18, pros::v5::MotorGears::blue}, {-19, pros::v5::MotorGears::blue}, {20, pros::v5::MotorGears::blue}},//right motors
        3.25, //wheel diameter in inches
        0.75, //external gear ratio
        12.0, //track width in inches
        &imu,
        &vertical_pod
    };

    inline lynx::odometry odom{
        2,
        0,
        0,
        &horizontal_pod,
        &vertical_pod
    };

}

// Implement pods_to_inches here after all types are fully defined
namespace lynx {
    namespace util {
        inline double pods_to_inches(double ticks, std::string wheel_type) { 
            if (wheel_type == "odom") {
                return (ticks / 36000.0) * (M_PI * global::odom.odom_wheel_diameter); 
            }
            else if (wheel_type == "motor") {
                return (ticks / 300.0) * (M_PI * global::chassis.wheel_diameter);
            }
            return 0.0;
        }
    }
}

// Implement drive::straight() here after global::chassis is fully defined
// This is an inline member function definition outside the class
inline void lynx::drive::straight(double target, int timeout, double scale) {
    drive_pid.settle_timer.restart();
    util::timer safety_timer(timeout);
    safety_timer.start();
    double init_pos = global::chassis.get_position();
    double curr_pos;
    double init_heading = global::chassis.imu->get_heading();
    int loop_count = 0;
    global::con.clear();
    while (true){
        if (global::chassis.distance_pod != nullptr){ //if it is using the rotation sensor
            curr_pos = util::pods_to_inches((global::chassis.get_position() - init_pos), "odom");
        }
        else { //using chassis encoders
            curr_pos = util::pods_to_inches((global::chassis.get_position() - init_pos), "motor");
        }

        double drive_speed = drive_pid.calculate(target, curr_pos, scale);
        double heading_error = fmod((init_heading - global::chassis.imu->get_heading() + 540), 360) - 180;
        double heading_correction = heading_correction_pid.calculate(heading_error, 0);
        
        int left_motor = (int)(drive_speed + heading_correction);
        int right_motor = (int)(drive_speed - heading_correction);
    
        global::con.print(0, 0, "Init pos: %.2f", init_pos);
        global::con.print(1, 0, "L: %d, R: %d", left_motor, right_motor);
        global::con.print(2, 0, "Error: %.2f", target - curr_pos);
        
        global::chassis.move(left_motor, right_motor); // left, right

        if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target)) break;
        if (safety_timer.has_elapsed()) break;
        loop_count++;
        pros::delay(5);
    }
    global::chassis.move(0,0);
}