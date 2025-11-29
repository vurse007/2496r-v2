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
        else if constexpr (std::is_same_v<T, lynx::drive> || std::is_same_v<T, lynx::state_drive>)  {
            if (device.distance_pod != nullptr) {
                return get_inches(*device.distance_pod);
            } else {
                return device.get_position();
            }
        }
        else { 
            // assuming individual chassis motor (300 ticks/rev)
            double ticks = device.get_position();
            return (ticks / 300.0) * (M_PI * global::chassis.wheel_diameter);
        }
    }
}

// ==========================================
// DRIVE STRAIGHT
// ==========================================
inline void lynx::drive::straight(double target, int timeout, double scale) {
    drive_pid.settle_timer.reset();
    global::con.clear();

    util::timer safety_timer(timeout);
    safety_timer.restart();
    
    double curr_pos;
    double init_heading = global::chassis.imu->get_heading();
    double init_pos     = util::get_inches(global::chassis);

    while (true) {
        global::odom.update();
        curr_pos = util::get_inches(global::chassis) - init_pos;

        // Forward PID
        double drive_speed = drive_pid.calculate(target, curr_pos, scale);

        // Heading correction around initial heading
        double heading_error = util::absolute_logic(init_heading, &global::imu);
        double heading_correction = turn_pid.calculate(heading_error, 0.0, 1.0);

        int left_motor  = static_cast<int>(drive_speed + heading_correction);
        int right_motor = static_cast<int>(drive_speed - heading_correction);
        global::chassis.move(left_motor, right_motor);
        
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"Init", "L", "R", "Err"}, 
            {init_pos, (double)left_motor, (double)right_motor, target - curr_pos}
        );

        // Settle logic
        if (std::fabs(target - curr_pos) <= drive_pid.refined_range) {
            drive_pid.settle_timer.start();
        }

        if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target)) break;
        if (safety_timer.has_elapsed()) break;

        pros::delay(5);
    }
    global::chassis.move(0, 0);
}

// ==========================================
// ABSOLUTE TURN
// ==========================================
inline void lynx::drive::turn_abs(double target, int timeout, double scale) {
    global::con.clear();

    // Safety timer (optional timeout)
    util::timer safety_timer(timeout);
    if (timeout > 0) safety_timer.start();

    // Settle timer (we fix the logic here)
    turn_pid.settle_timer.reset();

    double heading_error = 0.0;

    while (true) {

        // Compute error (degrees)
        heading_error = util::absolute_logic(target, &global::imu);

        // PID: target = heading_error, current = 0
        double turn_speed = turn_pid.calculate(heading_error, 0.0, scale);

        // Motor outputs
        int left_motor  = static_cast<int>( turn_speed);
        int right_motor = static_cast<int>(-turn_speed);
        global::chassis.move(left_motor, right_motor);

        // Debug print
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"Err", "L", "R"}, 
            {heading_error, (double)left_motor, (double)right_motor}
        );

        // ============================
        //     FIXED SETTLE LOGIC 
        // ============================
        if (std::fabs(heading_error) <= turn_pid.refined_range) {
            // FIRST time into refined range? Start settle timer.
            if (!turn_pid.settle_timer.running)
                turn_pid.settle_timer.start();
        }
        else {
            // Left refined range → reset settle timer
            if (turn_pid.settle_timer.running)
                turn_pid.settle_timer.reset();
        }

        // Finish if settle timer exceeded target ms
        if (turn_pid.settle_timer.running &&
            turn_pid.settle_timer.has_elapsed(turn_pid.settle_timer_target))
        {
            break;
        }

        // Safety timeout  
        if (timeout > 0 && safety_timer.has_elapsed())
            break;

        pros::delay(5);
    }

    global::chassis.move(0, 0);
}


// ==========================================
// RELATIVE TURN
// ==========================================
inline void lynx::drive::turn_rel(double delta_deg, int timeout, double scale) {
    double current = global::imu.get_heading();
    double target  = std::fmod(current + delta_deg + 360.0, 360.0);
    turn_abs(target, timeout, scale);
}
