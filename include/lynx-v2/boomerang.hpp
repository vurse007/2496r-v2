#pragma once
#include "motion.hpp"
#include "odom.hpp"

namespace global { extern lynx::odometry odom; }

inline void lynx::drive::boomerang(double targetx, double targety, double target_theta, 
                                   std::string glead, int timeout, double scale) {
    drive_pid.settle_timer.restart();
    turn_pid.settle_timer.restart();
    util::timer safety_timer(timeout);
    safety_timer.restart();

    point target = point(targetx, targety, target_theta);
    
    while (true) {
        global::odom.update();
        point robot = point(global::odom.current_pos.x, 
                          global::odom.current_pos.y, 
                          util::to_deg(global::odom.current_pos.theta));
        
        // Distance to actual target
        double targetDistErr = robot.distance_to(target);
        
        // Switch to direct targeting when close
        point carrot(0,0,0);
        if (targetDistErr < 9.0) {
            carrot = target;  // Aim directly at target
        } else {
            carrot = global::odom.carrotPoint(robot, target, glead);
        }

        // Linear error - distance to carrot
        double linErr = robot.distance_to(carrot);
        
        // Angular error - bearing to carrot position
        double bearing_to_carrot = util::to_deg(std::atan2(carrot.y - robot.y, 
                                                             carrot.x - robot.x));
        double angErr = bearing_to_carrot - util::to_deg(global::odom.current_pos.theta);
        // Angle wrap
        while (angErr > 180.0)  angErr -= 360.0;
        while (angErr < -180.0) angErr += 360.0;

        // Target angular error for settling check
        double targetAngErr = target_theta - util::to_deg(global::odom.current_pos.theta);
        while (targetAngErr > 180.0)  targetAngErr -= 360.0;
        while (targetAngErr < -180.0) targetAngErr += 360.0;

        // PID calculations
        double linPower  = drive_pid.calculate(linErr, 0, scale);
        double turnPower = turn_pid.calculate(angErr, 0, scale);

        double leftMotor  = linPower + turnPower;
        double rightMotor = linPower - turnPower;

        global::chassis.move(leftMotor, rightMotor);

        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"tDist", "tAng", "TURN", "LIN", "CARX"}, 
            {targetDistErr, targetAngErr, turnPower, linPower, carrot.x}
        );

        // Settling check
        // if (targetDistErr < 2.0 && std::abs(targetAngErr) < 3.0) {
        //     if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target) && 
        //         turn_pid.settle_timer.has_elapsed(turn_pid.settle_timer_target)) {
        //         break;
        //     }
        // } else {
        //     drive_pid.settle_timer.restart();
        //     turn_pid.settle_timer.restart();
        // }

        // if (safety_timer.has_elapsed()) break;
        
        pros::delay(5);
    }
    
    global::chassis.move(0, 0);
}