#pragma once
#include <cmath>
#include "util.hpp"
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

        double curr;
        double tgt;
        double prev_val;
        double error = 0;
        double prev_error = 0;
        double total_error = 0;
        double derivative = 0;

        double speed = 0;

        inline static double glb_tgt_heading = 0;

        double integral_threshold;
        double max_integral;
        double deadband;
        double count_range;
        double settle_timer_target;

        util::timer settle_timer;

        inline PID(constants g, constants r, double rr, double it, double mi, double db, double cr, double st):
            general_constants(g), refined_constants(r), refined_range(rr), integral_threshold(it), max_integral(mi), deadband(db), count_range(cr), settle_timer_target(st) {}

        inline double calculate(double target, double current){
            tgt = target;
            curr = current;
            error = tgt - curr;

            //deadband
            if (std::fabs(error) < deadband){
                error = 0;
                total_error=0;
                speed=0;
                prev_error=0;
                prev_val=curr;
                return 0;
            }

            //integral
            if (std::fabs(error) < integral_threshold){
                
            }
        }
    };
}