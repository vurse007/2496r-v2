#pragma once
#include "motion.hpp"
#include "odom.hpp"
#include "pursuit_types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace global { 
    extern lynx::odometry odom;
    extern lynx::drive chassis;
    extern pros::Controller con;  // Add this line
}

namespace lynx {

// ============================================================================
// WAYPOINT HELPER - Convert waypoint to point (needs full point definition)
// ============================================================================
inline point Waypoint_to_point(const Waypoint& wp) {
    return point(wp.x, wp.y, wp.heading);
}

// ============================================================================
// PURE PURSUIT CONTROLLER
// ============================================================================
class PurePursuitController {
private:
    std::vector<Waypoint> path;
    PursuitParams params;
    
    int current_segment_idx = 0;
    int settle_count = 0;
    double last_curvature = 0.0;
    
    // ========================================================================
    // GEOMETRY - Line-circle intersection for lookahead point
    // ========================================================================
    
    /**
     * Find intersection between a circle (robot + lookahead radius) and a line segment
     * Returns the furthest valid intersection point along the segment
     * 
     * Math: Solve |P(t) - robot_pos| = lookahead where P(t) = start + t*(end-start)
     * This gives us a quadratic equation: at² + bt + c = 0
     */
    std::pair<bool, point> lineCircleIntersection(
        const point& start, 
        const point& end,
        const point& robot_pos,
        double lookahead) const 
    {
        // Vector from start to end of segment
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        
        // Vector from start to robot
        double fx = start.x - robot_pos.x;
        double fy = start.y - robot_pos.y;
        
        // Quadratic coefficients: at² + bt + c = 0
        double a = dx * dx + dy * dy;
        double b = 2.0 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - lookahead * lookahead;
        
        double discriminant = b * b - 4.0 * a * c;
        
        // No intersection
        if (discriminant < 0.0) {
            return {false, point(0, 0)};
        }
        
        // Calculate both intersection parameters
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2.0 * a);
        double t2 = (-b + discriminant) / (2.0 * a);
        
        // We want the furthest valid intersection (prefer t2, fallback to t1)
        double t = -1.0;
        if (t2 >= 0.0 && t2 <= 1.0) {
            t = t2;  // Furthest intersection in valid range
        } else if (t1 >= 0.0 && t1 <= 1.0) {
            t = t1;  // Closer intersection
        }
        
        // No valid intersection in segment range [0, 1]
        if (t < 0.0) {
            return {false, point(0, 0)};
        }
        
        // Calculate intersection point
        point intersection(
            start.x + t * dx,
            start.y + t * dy,
            0.0  // Theta not used for lookahead point
        );
        
        return {true, intersection};
    }
    
    // ========================================================================
    // PATH ANALYSIS - Calculate curvature for adaptive lookahead
    // ========================================================================
    
    /**
     * Calculate the curvature of the path at the current segment
     * Higher curvature = sharper turn = need smaller lookahead
     * 
     * Curvature is measured as the change in direction between segments
     */
    double calculatePathCurvature() const {
        if (current_segment_idx >= (int)path.size() - 1) {
            return 0.0;  // Last segment or beyond
        }
        
        // Look ahead at next segment to measure direction change
        int next_idx = std::min(current_segment_idx + 1, (int)path.size() - 1);
        
        const Waypoint& current = path[current_segment_idx];
        const Waypoint& next = path[next_idx];
        
        // Calculate angle of current segment
        double angle1 = std::atan2(next.y - current.y, next.x - current.x);
        
        // If there's another segment, calculate its angle
        if (next_idx < (int)path.size() - 1) {
            const Waypoint& after_next = path[next_idx + 1];
            double angle2 = std::atan2(after_next.y - next.y, after_next.x - next.x);
            
            // Angular difference (wrapped to [-π, π])
            double delta_angle = angle2 - angle1;
            delta_angle = util::wrap_to_pi(delta_angle);
            
            // Curvature = angular change / distance
            double segment_length = next.distance_to(after_next);
            if (segment_length > 0.1) {  // Avoid division by zero
                return std::abs(delta_angle) / segment_length;
            }
        }
        
        return 0.0;
    }
    
    /**
     * Adaptive lookahead distance based on path curvature
     * Straight sections → large lookahead (smooth, fast)
     * Sharp turns → small lookahead (precise, no corner cutting)
     */
    double getAdaptiveLookahead() const {
        double curvature = calculatePathCurvature();
        
        // Exponential decay: lookahead = base / (1 + k*curvature)
        double adaptive = params.base_lookahead / (1.0 + params.curvature_scale * curvature);
        
        // Clamp to reasonable range
        return std::clamp(adaptive, params.min_lookahead, params.max_lookahead);
    }
    
    // ========================================================================
    // LOOKAHEAD POINT - Find the point to chase on the path
    // ========================================================================
    
    /**
     * Find the lookahead point by searching through path segments
     * Returns the intersection of the lookahead circle with the path
     */
    std::pair<bool, point> findLookaheadPoint(const point& robot_pos, double lookahead) {
        // Start from current segment and search forward
        for (int i = current_segment_idx; i < (int)path.size() - 1; i++) {
            point start = Waypoint_to_point(path[i]);
            point end = Waypoint_to_point(path[i + 1]);
            
            auto [found, intersection] = lineCircleIntersection(start, end, robot_pos, lookahead);
            
            if (found) {
                // Update current segment if we've moved forward
                current_segment_idx = i;
                return {true, intersection};
            }
        }
        
        // No intersection found - return the last point
        return {true, Waypoint_to_point(path.back())};
    }
    
    // ========================================================================
    // CURVATURE CALCULATION - Pure Pursuit control law
    // ========================================================================
    
    /**
     * Calculate curvature to reach the lookahead point
     * 
     * Pure Pursuit equation: curvature = (2 * sin(α)) / L
     * where α = angle between robot heading and line to lookahead point
     *       L = lookahead distance
     * 
     * This curvature defines the arc the robot should follow
     */
    double calculatePursuitCurvature(const point& robot_pos, const point& lookahead_point, double lookahead) {
        // Vector from robot to lookahead point
        double dx = lookahead_point.x - robot_pos.x;
        double dy = lookahead_point.y - robot_pos.y;
        
        // Angle to lookahead point (global frame)
        double angle_to_point = std::atan2(dy, dx);
        
        // α = difference from robot heading (in radians)
        double alpha = angle_to_point - robot_pos.theta;
        alpha = util::wrap_to_pi(alpha);  // Wrap to [-π, π]
        
        // Pure Pursuit curvature formula
        double curvature = (2.0 * std::sin(alpha)) / lookahead;
        
        return curvature;
    }
    
    // ========================================================================
    // HEADING CONTROL - Blend in target heading correction
    // ========================================================================
    
    /**
     * Calculate heading correction to blend with pure pursuit
     * Uses distance-based blending - more correction near waypoints
     */
    double calculateHeadingCorrection(const point& robot_pos) {
        // Find the target heading from the closest upcoming waypoint
        double target_heading_deg = path[current_segment_idx].heading;
        if (current_segment_idx < (int)path.size() - 1) {
            target_heading_deg = path[current_segment_idx + 1].heading;
        }
        
        // Convert to radians and calculate error
        double target_heading_rad = util::to_rad(target_heading_deg);
        double heading_error = target_heading_rad - robot_pos.theta;
        heading_error = util::wrap_to_pi(heading_error);
        
        // Distance to the target waypoint
        point target_waypoint = Waypoint_to_point(path[std::min(current_segment_idx + 1, (int)path.size() - 1)]);
        double distance_to_target = robot_pos.distance_to(target_waypoint);
        
        // Blending weight: increases as we get closer to waypoint
        // Uses exponential function for smooth transition
        double blend_weight = 1.0 - std::exp(-params.heading_blend_power * 
                                             (params.heading_blend_dist - distance_to_target) / 
                                             params.heading_blend_dist);
        blend_weight = std::clamp(blend_weight, 0.0, 1.0);
        
        // Proportional heading correction
        double omega_correction = params.heading_kp * heading_error;
        
        // Apply blending
        return blend_weight * omega_correction;
    }
    
    // ========================================================================
    // SETTLING - Check if we've reached the end of the path
    // ========================================================================
    
    /**
     * Check if the robot has settled at the final position
     * Requires both position and heading to be within tolerance
     */
    bool isSettled(const point& robot_pos) {
        // Must be on the last segment
        if (current_segment_idx < (int)path.size() - 2) {
            settle_count = 0;
            return false;
        }
        
        // Distance to final waypoint
        point final_point = Waypoint_to_point(path.back());
        double distance_error = robot_pos.distance_to(final_point);
        
        // Heading error to final waypoint
        double target_heading_rad = util::to_rad(path.back().heading);
        double heading_error = target_heading_rad - robot_pos.theta;
        heading_error = util::wrap_to_pi(heading_error);
        double heading_error_deg = std::abs(util::to_deg(heading_error));
        
        // Check if within tolerances
        bool position_settled = distance_error < params.path_completion_dist;
        bool heading_settled = heading_error_deg < params.final_heading_tolerance;
        
        if (position_settled && heading_settled) {
            settle_count++;
        } else {
            settle_count = 0;
        }
        
        return settle_count >= params.settle_count_target;
    }

public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    
    PurePursuitController(const std::vector<Waypoint>& path, const PursuitParams& params = PursuitParams())
        : path(path), params(params) {}
    
    // ========================================================================
    // MAIN EXECUTION LOOP
    // ========================================================================
    
    /**
     * Execute pure pursuit path following
     * This is the main control loop that ties everything together
     */
    void execute(int timeout = 5000) {
        // Safety checks
        if (path.size() < 2) {
            return;  // Need at least 2 points
        }
        
        // Reset state
        current_segment_idx = 0;
        settle_count = 0;
        
        util::timer safety_timer(timeout);
        safety_timer.restart();
        
        global::con.clear();
        
        while (true) {
            // Update odometry
            global::odom.update();
            point robot_pos = global::odom.current_pos;
            
            // Calculate adaptive lookahead
            double lookahead = getAdaptiveLookahead();
            
            // Find lookahead point on path
            auto [found, lookahead_point] = findLookaheadPoint(robot_pos, lookahead);
            
            if (!found) {
                // Shouldn't happen, but safety first
                global::chassis.move(0, 0);
                break;
            }
            
            // Calculate pure pursuit curvature
            double curvature = calculatePursuitCurvature(robot_pos, lookahead_point, lookahead);
            
            // Get target velocity from current segment
            double target_velocity = path[current_segment_idx].velocity;
            
            // Calculate base velocities from pure pursuit
            double v = target_velocity;
            double omega_pursuit = curvature * v;
            
            // Add heading correction
            double omega_heading = calculateHeadingCorrection(robot_pos);
            
            // Combine angular velocities
            double omega_total = omega_pursuit + omega_heading;
            
            // Convert to wheel velocities (differential drive kinematics)
            double v_left = v - (omega_total * global::chassis.track_width / 2.0);
            double v_right = v + (omega_total * global::chassis.track_width / 2.0);
            
            // Clamp to motor range
            v_left = std::clamp(v_left, -127.0, 127.0);
            v_right = std::clamp(v_right, -127.0, 127.0);
            
            // Send commands to motors
            global::chassis.move((int)v_left, (int)v_right);
            
            // Debug output
            point target_waypoint = Waypoint_to_point(path[std::min(current_segment_idx + 1, (int)path.size() - 1)]);
            double dist_to_target = robot_pos.distance_to(target_waypoint);
            
            lynx::util::print_info(
                safety_timer.elapsed(),
                &global::con,
                {"Seg", "Dist", "Look", "Curv"},
                {(double)current_segment_idx, dist_to_target, lookahead, curvature}
            );
            
            // Check settling
            if (isSettled(robot_pos)) {
                break;
            }
            
            // Safety timeout
            if (safety_timer.has_elapsed()) {
                break;
            }
            
            pros::delay(5);
        }
        
        // Stop motors
        global::chassis.move(0, 0);
    }
};

// ============================================================================
// CHASSIS INTEGRATION - Add pure pursuit to the drive class
// ============================================================================

inline void drive::purePursuit(const std::vector<Waypoint>& path, 
                               const PursuitParams& params,
                               int timeout) {
    PurePursuitController controller(path, params);
    controller.execute(timeout);
}

} // namespace lynx