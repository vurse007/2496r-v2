#pragma once
#include "motion.hpp"
#include "odom.hpp"

namespace global { extern lynx::odometry odom; }

inline void lynx::drive::boomerang(double targetx, double targety, double target_theta, 
                                   double dLead, int timeout, double scale) {
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
        
        // Always use carrot point calculation
        point carrot = global::odom.carrotPoint(robot, target, dLead);

        // Linear error - distance to carrot
        double linErr = robot.distance_to(carrot);
        
        // Target angular error for settling check
        double targetAngErr = target_theta - util::to_deg(global::odom.current_pos.theta);
        while (targetAngErr > 180.0)  targetAngErr -= 360.0;
        while (targetAngErr < -180.0) targetAngErr += 360.0;
        
        // Simple switching logic - NO BLENDING
        // When far from target: turn toward carrot to follow the curve
        // When close to target: turn toward target angle
        double angErr;
        if (targetDistErr > 12.0) {
            // Far from target - turn toward carrot
            double bearingToCarrot = util::to_deg(std::atan2(carrot.y - robot.y, 
                                                              carrot.x - robot.x));
            angErr = bearingToCarrot - util::to_deg(global::odom.current_pos.theta);
            while (angErr > 180.0)  angErr -= 360.0;
            while (angErr < -180.0) angErr += 360.0;
        } else {
            // Close to target - turn toward target angle
            angErr = targetAngErr;
        }

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