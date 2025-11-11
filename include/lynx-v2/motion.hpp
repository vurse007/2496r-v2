#pragma once
#include "chassis.hpp"
#include "odom.hpp"
#include "util.hpp"

namespace lynx::util {
    template <typename T>
    inline double get_inches(const T& device) {
        if constexpr(std::is_same_v<T, pros::Rotation>) {
            double ticks = device.get_position();
            return (ticks / 36000.0) * (M_PI * global::odom.odom_wheel_diameter);
        }
        else if constexpr (std::is_same_v<T, lynx::drive>) {
            if (device.distance_pod != nullptr) {
                return get_inches(*device.distance_pod);
            } else {
                return device.get_position();
            }
        }
        else { //else case for anything else - assuming that a individual chassis motor was passed in cus what else would it be
            double ticks = device.get_position();
            return (ticks / 300.0) * (M_PI * global::chassis.wheel_diameter);
        }
    }
}

inline void lynx::drive::straight(double target, int timeout, double scale) {
    drive_pid.settle_timer.restart();
    global::con.clear();

    util::timer safety_timer(timeout);
    safety_timer.restart();
    
    double curr_pos;
    double init_heading = global::chassis.imu->get_heading();
    double init_pos = util::get_inches(global::chassis);

    while (true){
        global::odom.update();
        curr_pos = util::get_inches(global::chassis) - init_pos;

        double drive_speed = drive_pid.calculate(target, curr_pos, scale);
        double heading_error = util::absolute_logic(init_heading, &global::imu);
        double heading_correction = turn_pid.calculate(heading_error, 0);
        
        int left_motor = (int)(drive_speed + heading_correction);
        int right_motor = (int)(drive_speed - heading_correction);
        global::chassis.move(left_motor, right_motor);
        
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"Init", "L", "R", "Err"}, 
            {init_pos, (double)left_motor, (double)right_motor, target - curr_pos}
        );

        // if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target)) break;
        if (safety_timer.has_elapsed()) break;
        pros::delay(5);
    }
    global::chassis.move(0,0);
}

inline void lynx::drive::turn_abs(double target, int timeout, double scale) {
    turn_pid.settle_timer.restart();
    global::con.clear();

    util::timer safety_timer(timeout);
    safety_timer.restart();
    double heading_error;

    while (true){
        heading_error = util::absolute_logic(target, &global::imu);
        double turn_speed = turn_pid.calculate(0, heading_error, scale);
        
        int left_motor = (int)(turn_speed);
        int right_motor = (int)(-turn_speed);
        global::chassis.move(left_motor, right_motor);
        
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"Err", "L", "R"}, 
            {heading_error, (double)left_motor, (double)right_motor}
        );

        //if (turn_pid.settle_timer.has_elapsed(turn_pid.settle_timer_target)) break;
        if (safety_timer.has_elapsed()) break;
        pros::delay(5);
    }
    global::chassis.move(0,0);
}

inline void lynx::drive::turn_rel(double delta_deg, int timeout, double scale) {
    turn_abs(fmod(global::imu.get_heading() + delta_deg + 360, 360), timeout, scale);
}