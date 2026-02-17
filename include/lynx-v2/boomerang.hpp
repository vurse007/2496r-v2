#pragma once
#include "motion.hpp"
#include "odom.hpp"
#include <cmath>
#include <algorithm>

namespace global { 
    extern lynx::odometry odom;
    extern lynx::state_drive chassis;
    extern pros::Controller con;
}

// ==========================================================================
//  BOOMERANG CONTROLLER PARAMETERS (Matching Python)
// ==========================================================================

namespace lynx::boomerang_params {
    constexpr double CARROT_DISTANCE = 20.0;       // inches - lookahead toward approach point
    constexpr double APPROACH_DISTANCE = 12.0;     // inches - point behind target
    constexpr double GLEAD_GAIN = 0.7;             // blend factor (0 = pure pursuit, 1 = always face target)
    constexpr double GLEAD_BLEND_DIST = 40.0;      // distance over which g-lead blends
    constexpr double ARRIVAL_THRESHOLD = 1.5;      // inches - position arrival
    constexpr double ANGLE_THRESHOLD_DEG = 3.0;    // degrees - heading arrival
    constexpr double SLOWDOWN_DIST = 8.0;          // inches - start slowing down
    constexpr double CLOSE_ANGULAR_DAMPING = 0.7;  // reduce turn when very close
    constexpr double CLOSE_DIST = 3.0;             // inches - "very close" threshold
}

// ==========================================================================
//  HELPER FUNCTIONS
// ==========================================================================

namespace lynx::boomerang_util {

    inline double wrap_to_180(double angle_deg) {
        while (angle_deg > 180.0) angle_deg -= 360.0;
        while (angle_deg < -180.0) angle_deg += 360.0;
        return angle_deg;
    }

    inline double wrap_to_pi(double angle_rad) {
        while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
        while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
        return angle_rad;
    }

    inline double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

    inline double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

}

// ==========================================================================
//  BOOMERANG CONTROLLER (Matching Python Logic)
// ==========================================================================

inline void lynx::drive::boomerang(double target_x, double target_y, double target_theta_deg, 
                                   double dLead, int timeout, double scale) {
    using namespace boomerang_params;
    using namespace boomerang_util;

    // Reset PID settle timers
    drive_pid.settle_timer.restart();
    turn_pid.settle_timer.restart();
    
    util::timer safety_timer(timeout);
    safety_timer.restart();

    // Convert target heading to radians for internal calculations
    double target_theta_rad = util::to_rad(target_theta_deg);

    while (true) {
        global::odom.update();

        // Get robot position (inches) and heading (radians)
        double robot_x = global::odom.current_pos.x;
        double robot_y = global::odom.current_pos.y;
        double robot_theta_rad = global::odom.current_pos.theta;

        // =====================================================================
        // STEP 1: Calculate distance to target
        // =====================================================================
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double dist_to_target = std::sqrt(dx * dx + dy * dy);

        // =====================================================================
        // STEP 2: Calculate angle error to target heading (for settling check)
        // =====================================================================
        double angle_error_to_target_rad = wrap_to_pi(target_theta_rad - robot_theta_rad);
        double angle_error_to_target_deg = util::to_deg(angle_error_to_target_rad);

        // =====================================================================
        // STEP 3: Check if arrived
        // =====================================================================
        if (dist_to_target < ARRIVAL_THRESHOLD && 
            std::abs(angle_error_to_target_deg) < ANGLE_THRESHOLD_DEG) {
            
            if (drive_pid.settle_timer.has_elapsed(drive_pid.settle_timer_target) && 
                turn_pid.settle_timer.has_elapsed(turn_pid.settle_timer_target)) {
                break;  // Arrived and settled!
            }
        } else {
            drive_pid.settle_timer.restart();
            turn_pid.settle_timer.restart();
        }

        // =====================================================================
        // STEP 4: Calculate approach point (behind target in direction of target heading)
        // Python: approach_x = target_x - approach_distance * sin(target_theta)
        //         approach_y = target_y - approach_distance * cos(target_theta)
        // This uses navigation convention (0 = North, CW positive)
        // =====================================================================
        double approach_x = target_x - APPROACH_DISTANCE * std::sin(target_theta_rad);
        double approach_y = target_y - APPROACH_DISTANCE * std::cos(target_theta_rad);

        // =====================================================================
        // STEP 5: Calculate carrot point (toward approach point from robot)
        // =====================================================================
        double dx_approach = approach_x - robot_x;
        double dy_approach = approach_y - robot_y;
        double dist_to_approach = std::sqrt(dx_approach * dx_approach + dy_approach * dy_approach);

        double carrot_x, carrot_y;
        double carrot_distance = std::min(CARROT_DISTANCE, dist_to_approach);

        if (dist_to_approach > 0.1) {
            carrot_x = robot_x + (dx_approach / dist_to_approach) * carrot_distance;
            carrot_y = robot_y + (dy_approach / dist_to_approach) * carrot_distance;
        } else {
            carrot_x = robot_x;
            carrot_y = robot_y;
        }

        // =====================================================================
        // STEP 6: Calculate angle to carrot point (navigation convention)
        // Python: angle_to_carrot = atan2(carrot_x - robot_x, carrot_y - robot_y)
        // =====================================================================
        double angle_to_carrot_rad = std::atan2(carrot_x - robot_x, carrot_y - robot_y);

        // =====================================================================
        // STEP 7: G-lead blending
        // As we get closer to target, blend from carrot-following to target-heading
        // Python: distance_factor = min(1.0, distance / 40.0)
        //         glead_factor = GLEAD_GAIN * (1.0 - distance_factor)
        // =====================================================================
        double distance_factor = std::min(1.0, dist_to_target / GLEAD_BLEND_DIST);
        double glead_factor = GLEAD_GAIN * (1.0 - distance_factor);

        // Blend: target_angle = carrot_angle + glead_factor * (target_heading - carrot_angle)
        double carrot_to_target_diff = wrap_to_pi(target_theta_rad - angle_to_carrot_rad);
        double target_angle_to_follow = angle_to_carrot_rad + glead_factor * carrot_to_target_diff;

        // =====================================================================
        // STEP 8: Calculate angular error (what we feed to turn PID)
        // =====================================================================
        double ang_err_rad = wrap_to_pi(target_angle_to_follow - robot_theta_rad);
        double ang_err_deg = util::to_deg(ang_err_rad);

        // Apply close-range angular damping
        if (dist_to_target < CLOSE_DIST) {
            ang_err_deg *= CLOSE_ANGULAR_DAMPING;
        }

        // =====================================================================
        // STEP 9: Calculate linear error (distance to carrot)
        // =====================================================================
        double lin_err = distance(robot_x, robot_y, carrot_x, carrot_y);

        // Apply distance-based slowdown
        if (dist_to_target < SLOWDOWN_DIST) {
            double slowdown_factor = dist_to_target / SLOWDOWN_DIST;
            lin_err *= slowdown_factor;
        }

        // =====================================================================
        // STEP 10: PID calculations
        // =====================================================================
        double lin_power = drive_pid.calculate(lin_err, 0, scale);
        double turn_power = turn_pid.calculate(ang_err_deg, 0, scale);

        // =====================================================================
        // STEP 11: Apply to differential drive
        // =====================================================================
        double left_motor = lin_power + turn_power;
        double right_motor = lin_power - turn_power;

        global::chassis.move(left_motor, right_motor);

        // =====================================================================
        // DEBUG OUTPUT
        // =====================================================================
        lynx::util::print_info(
            safety_timer.elapsed(), 
            &global::con, 
            {"dist", "angE", "gLd", "lin", "trn"}, 
            {dist_to_target, ang_err_deg, glead_factor, lin_power, turn_power}
        );

        // Safety timeout
        if (safety_timer.has_elapsed()) {
            break;
        }

        pros::delay(5);
    }

    global::chassis.move(0, 0);
}