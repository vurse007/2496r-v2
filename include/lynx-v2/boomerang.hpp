#pragma once
#include "motion.hpp"
#include "odom.hpp"

namespace global { extern lynx::odometry odom; }

inline void lynx::drive::boomerang(double targetx, double targety, double target_theta, std::string glead, int timeout, double scale){
    drive_pid.settle_timer.restart();
    turn_pid.settle_timer.restart();
    util::timer safety_timer(timeout);
    safety_timer.restart();

    point target = point(targetx, targety, target_theta);
    while (true){

        global::odom.update();
        point robot = point(global::odom.current_pos.x, global::odom.current_pos.y, util::to_deg(global::odom.current_pos.theta));
        point carrot = global::odom.carrotPoint(robot, target, glead);

        double linErr = robot.distance_to(carrot);
        double angErr = robot.angle_error(carrot);

        double linPower  = drive_pid.calculate(linErr, 0, scale);   // distance PID
        double turnPower = turn_pid.calculate(angErr, 0, scale);    // heading PID

        double leftMotor  = linPower + turnPower;
        double rightMotor = linPower - turnPower;

        global::chassis.move(leftMotor, rightMotor);

        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"Err", "ANG", "TURN", "LIN", "CARX", "CARY"}, 
            {linErr, angErr, turnPower, linPower, carrot.x, carrot.y}
        );
        //if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target) && turn_pid.settle_timer.has_elapsed(turn_pid.settle_timer_target)) break;
        //if (safety_timer.has_elapsed()) break;
        pros::delay(5);
    }
    global::chassis.move(0,0);
}