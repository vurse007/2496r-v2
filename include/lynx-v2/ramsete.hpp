#pragma once
#include "motion.hpp"
#include "odom.hpp"

namespace global { extern lynx::odometry odom; }

inline void lynx::drive::ramsete(double targetx, double targety, double target_theta,
                                 double v_ref, double omega_ref,
                                 double b, double zeta, int timeout, double scale) {
    util::timer safety_timer(timeout);
    safety_timer.restart();

    point target = point(targetx, targety, target_theta);
    
    while (true) {
        global::odom.update();
        
        // Get current robot pose (in radians for calculations)
        double x = global::odom.current_pos.x;
        double y = global::odom.current_pos.y;
        double theta = global::odom.current_pos.theta;  // Already in radians
        
        // Reference pose
        double x_ref = target.x;
        double y_ref = target.y;
        double theta_ref = util::to_rad(target.theta);  // Convert to radians
        
        // Calculate pose error in global frame
        double e_x = x_ref - x;
        double e_y = y_ref - y;
        double e_theta = theta_ref - theta;
        
        // Wrap angle error to [-pi, pi]
        e_theta = util::wrap_to_pi(e_theta);
        
        // Transform error to robot's reference frame
        // This is crucial - we need errors relative to where robot is facing
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        double e_x_robot = cos_theta * e_x + sin_theta * e_y;
        double e_y_robot = -sin_theta * e_x + cos_theta * e_y;
        
        // Calculate the time-varying gain k
        // This makes the controller more aggressive at higher speeds
        double k = 2.0 * zeta * std::sqrt(omega_ref * omega_ref + b * v_ref * v_ref);
        
        // RAMSETE control law
        // Linear velocity: track reference + correction term
        double v = v_ref * std::cos(e_theta) + k * e_x_robot;
        
        // Angular velocity: track reference + corrections for both position and heading
        // Using sinc function for smooth behavior: sinc(x) = sin(x)/x
        double sinc_term;
        if (std::abs(e_theta) < 1e-6) {
            // Limit case as e_theta approaches 0 (sinc(0) = 1)
            sinc_term = 1.0;
        } else {
            sinc_term = std::sin(e_theta) / e_theta;
        }
        
        // Standard RAMSETE formula: omega = omega_ref + b*v_ref*sinc(e_theta)*e_y_robot + k*e_theta
        double omega = omega_ref + b * v_ref * sinc_term * e_y_robot + k * e_theta;
        
        // Convert (v, omega) to left/right wheel velocities using differential drive kinematics
        // v_left = v - (omega * track_width / 2)
        // v_right = v + (omega * track_width / 2)
        // Note: v is in inches/s, omega is in rad/s, track_width is in inches
        double v_left = v - (omega * track_width / 2.0);
        double v_right = v + (omega * track_width / 2.0);
        
        // Convert wheel velocities to motor commands
        // Assuming max velocity of ~50 inches/s corresponds to max motor output (127)
        // This can be tuned based on your robot's actual max speed
        const double max_velocity = 50.0;  // inches/s - tune this to match your robot
        double velocityScale = scale * 127.0 / max_velocity;
        
        double leftMotorCmd = v_left * velocityScale;
        double rightMotorCmd = v_right * velocityScale;
        
        // Clamp to motor range (-127 to 127)
        leftMotorCmd = std::clamp(leftMotorCmd, -127.0 * scale, 127.0 * scale);
        rightMotorCmd = std::clamp(rightMotorCmd, -127.0 * scale, 127.0 * scale);
        
        int leftMotor = (int)leftMotorCmd;
        int rightMotor = (int)rightMotorCmd;
        
        global::chassis.move(leftMotor, rightMotor);
        
        // Calculate errors for debugging
        double targetDistErr = std::sqrt(e_x * e_x + e_y * e_y);
        double targetAngErr = util::to_deg(e_theta);
        
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"tDist", "tAng", "V", "W", "eXr", "eYr"}, 
            {targetDistErr, targetAngErr, v, omega, e_x_robot, e_y_robot}
        );
        
        // Settling check - can be enabled if needed
        // if (targetDistErr < 2.0 && std::abs(targetAngErr) < 3.0) {
        //     break;
        // }
        
        // if (safety_timer.has_elapsed()) break;
        
        pros::delay(5);
    }
    
    global::chassis.move(0, 0);
}

