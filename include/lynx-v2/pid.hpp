#pragma once
#include <cmath>
#include "util.hpp"
#include "config.hpp"
#include <algorithm>

namespace lynx {

    struct constants {
        double kP;
        double kI;
        double kD;

        inline constants(double kp, double ki, double kd):
            kP(kp), kI(ki), kD(kd) {}
    };

    class PID {
        public:
        constants general_constants;
        constants refined_constants;

        double refined_range;

        int dt = 5; // in ms

        double curr;
        double tgt;
        double prev_val;
        double error = 0;
        double prev_error = 0;
        double total_error = 0;
        double derivative = 0;

        double slew;
        double prev_speed;

        double speed = 0;

        inline static double glb_tgt_heading = 0;

        double integral_threshold;
        double max_integral;
        double deadband;
        double count_range;
        double settle_timer_target;

        util::timer settle_timer;

        inline PID(constants g, constants r, double rr, double slew, double it, double mi, double db, double cr, double st):
            general_constants(g), refined_constants(r), refined_range(rr), slew(slew),integral_threshold(it), max_integral(mi), deadband(db), count_range(cr), settle_timer_target(st) {}

        inline double calculate(double target, double current, double scale=1.0){
            tgt = target;
            curr = current;

            //proportional
            error = tgt - curr;

            //integral
            if (std::fabs(error) < integral_threshold){
                total_error += ((error + prev_error) / 2.0);
                total_error = std::clamp(total_error, -max_integral, max_integral);
            }

            //derivative
            derivative = error - prev_error;

            //calculate speed
            if (std::fabs(error) < refined_range){
                speed = scale * (refined_constants.kP * error + refined_constants.kI * total_error + refined_constants.kD * derivative);
            }
            else {
                speed = scale * (general_constants.kP * error + general_constants.kI * total_error + general_constants.kD * derivative);
            }

            //capping speed
            speed = std::clamp(speed, (scale*-127), (scale*127));

            //update last values
            prev_error = error;
            prev_speed = speed;

            double delta_speed = speed - prev_speed;

            //slew
            if (delta_speed > slew){
                speed = prev_speed + slew;
            }
            else if (delta_speed < -slew){
                speed = prev_speed - slew;
            }

            return speed;
        }

        inline void straight(double target, int timeout = 2000, double scale=1.0){
            settle_timer.restart();
            util::timer safety_timer(timeout);
            double init_pos = global::chassis.get_position();
            double curr_pos;
            while (true){
                if (global::chassis.distance_pod != nullptr){ //if it is using the rotation sensor
                    curr_pos = util::pods_to_inches((global::chassis.get_position() - init_pos), "odom");
                }
                else { //using chassis enocoders
                    curr_pos = util::pods_to_inches((global::chassis.get_position() - init_pos), "motor");
                }

                //double drive_speed = calculate
            }
        }
    };
}