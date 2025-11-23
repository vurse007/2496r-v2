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

        int dt = 5; // ms, not strictly used but kept for completeness

        double curr = 0.0;
        double tgt  = 0.0;

        double error       = 0.0;
        double prev_error  = 0.0;
        double total_error = 0.0;
        double derivative  = 0.0;

        double slew        = 0.0;
        double prev_speed  = 0.0;
        double speed       = 0.0;

        inline static double glb_tgt_heading = 0; // if you use this elsewhere

        double integral_threshold = 0.0;
        double max_integral       = 0.0;
        double deadband           = 0.0;
        double settle_timer_target = 0.0;

        util::timer settle_timer;

        inline PID(constants g, constants r, double rr, double slew_val,
                   double it, double mi, double db, double st):
            general_constants(g),
            refined_constants(r),
            refined_range(rr),
            slew(slew_val),
            integral_threshold(it),
            max_integral(mi),
            deadband(db),
            settle_timer_target(st) {}

        inline double calculate(double target, double current, double scale = 1.0) {
            tgt  = target;
            curr = current;

            // --------------------------------
            // ERROR TERMS
            // --------------------------------
            error = tgt - curr;

            // Deadband: treat very small error as zero
            if (std::fabs(error) < deadband) {
                error = 0.0;
            }

            // Integral (with threshold & clamping)
            if (std::fabs(error) < integral_threshold) {
                total_error += (error + prev_error) / 2.0;  // trapezoidal-ish
                total_error = std::clamp(total_error, -max_integral, max_integral);
            } else {
                // Optional: reset integral when far from target
                total_error = 0.0;
            }

            // Derivative (on error)
            derivative = error - prev_error;

            // --------------------------------
            // PICK CONSTANT SET
            // --------------------------------
            const constants& c = (std::fabs(error) < refined_range)
                ? refined_constants
                : general_constants;

            if (std::fabs(error) < refined_range) {
                // Inside refined zone, start settle timer
                settle_timer.start();
            } else {
                // If you want: reset timer when far away
                // settle_timer.reset();
            }

            // --------------------------------
            // RAW PID OUTPUT
            // --------------------------------
            double raw_speed = scale * (c.kP * error +
                                        c.kI * total_error +
                                        c.kD * derivative);

            // --------------------------------
            // SLEW LIMITING (USE PREV_SPEED CORRECTLY)
            // --------------------------------
            double delta_speed = raw_speed - prev_speed;
            if (delta_speed > slew) {
                speed = prev_speed + slew;
            }
            else if (delta_speed < -slew) {
                speed = prev_speed - slew;
            }
            else {
                speed = raw_speed;
            }

            // Clamp final speed
            speed = std::clamp(speed, -127.0 * scale, 127.0 * scale);

            // Update history
            prev_error = error;
            prev_speed = speed;

            return speed;
        }
    };

} // namespace lynx
