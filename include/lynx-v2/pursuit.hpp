#pragma once
#include "motion.hpp"
#include "odom.hpp"
#include "pursuit_types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace global { 
    extern lynx::odometry odom;
    extern lynx::state_drive chassis;
    extern pros::Controller con;
}

namespace lynx {

// ============================================================================
// WAYPOINT HELPER - Convert waypoint to point (needs full point definition)
// NOTE: Waypoint.heading stays in degrees inside point.theta, but theta is NOT
// used for geometry in pursuit except where explicitly read from path[].
// ============================================================================
inline point Waypoint_to_point(const Waypoint& wp) {
    return point(wp.x, wp.y, wp.heading);
}

// ============================================================================
// PURE PURSUIT CONTROLLER
// GLOBAL FRAME: +X East, +Y North (IMU-native)
// HEADING: IMU heading (0° North, CW positive) stored in robot_pos.theta (radians)
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
    // (Pure geometry; frame choice doesn't matter here)
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
        
        // Angle of current segment in global XY (still fine)
        double angle1 = std::atan2(next.y - current.y, next.x - current.x);
        
        if (next_idx < (int)path.size() - 1) {
            const Waypoint& after_next = path[next_idx + 1];
            double angle2 = std::atan2(after_next.y - next.y, after_next.x - next.x);
            
            double delta_angle = angle2 - angle1;
            delta_angle = util::wrap_to_pi(delta_angle);
            
            double segment_length = next.distance_to(after_next);
            if (segment_length > 0.1) {
                return std::abs(delta_angle) / segment_length;
            }
        }
        
        return 0.0;
    }
    
    /**
     * Adaptive lookahead distance based on path curvature
     */
    double getAdaptiveLookahead() const {
        double curvature = calculatePathCurvature();
        double adaptive = params.base_lookahead / (1.0 + params.curvature_scale * curvature);
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
        bool found_intersection = false;
        point best_intersection(0, 0);
        int best_segment = current_segment_idx;
        double best_t = -1.0;
        
        for (int i = current_segment_idx; i < (int)path.size() - 1; i++) {
            point start = Waypoint_to_point(path[i]);
            point end = Waypoint_to_point(path[i + 1]);
            
            auto [found, intersection] = lineCircleIntersection(start, end, robot_pos, lookahead);
            
            if (found) {
                double dx = end.x - start.x;
                double dy = end.y - start.y;
                double segment_length = std::sqrt(dx * dx + dy * dy);
                
                if (segment_length > 0.01) {
                    double ix_dx = intersection.x - start.x;
                    double ix_dy = intersection.y - start.y;
                    double t = (ix_dx * dx + ix_dy * dy) / (segment_length * segment_length);
                    
                    if (!found_intersection || i > best_segment || (i == best_segment && t > best_t)) {
                        best_intersection = intersection;
                        best_segment = i;
                        best_t = t;
                        found_intersection = true;
                    }
                }
            }
        }
        
        if (found_intersection) {
            current_segment_idx = best_segment;
            
           // --- Robust segment progression ---
            if (best_segment < (int)path.size()-1) {

                point curr_wp = Waypoint_to_point(path[best_segment]);
                point next_wp = Waypoint_to_point(path[best_segment + 1]);

                double dist_curr = robot_pos.distance_to(curr_wp);
                double dist_next = robot_pos.distance_to(next_wp);

                // Advance when robot is closer to next waypoint
                if (dist_next < dist_curr) {
                    current_segment_idx = best_segment + 1;
                }
            }

            
            return {true, best_intersection};
        }
        
        return {true, Waypoint_to_point(path.back())};
    }
    
    // ========================================================================
    // CURVATURE CALCULATION - Pure Pursuit control law
    // FIXED FOR IMU-NATIVE FRAME
    // ========================================================================
    
    /**
     * Pure Pursuit curvature:
     * curvature = (2 * sin(alpha)) / L
     * 
     * GLOBAL FRAME (IMU-native):
     * - +X East, +Y North
     * - Robot heading theta is IMU heading (CW positive)
     * - 0 rad = North (+Y)
     */
    double calculatePursuitCurvature(const point& robot_pos,
                                     const point& lookahead_point,
                                     double lookahead) 
    {
        // Vector from robot to lookahead in GLOBAL frame
        double dx = lookahead_point.x - robot_pos.x; // east offset
        double dy = lookahead_point.y - robot_pos.y; // north offset
        
        // IMU-style angle to point:
        // 0 rad when pointing North (+Y), CW positive
        // => atan2(dx, dy)
        double angle_to_point = std::atan2(dx, dy);

        // alpha = target - robot heading (both IMU radians)
        double alpha = angle_to_point - robot_pos.theta;
        alpha = util::wrap_to_pi(alpha);

        double curvature = (2.0 * std::sin(alpha)) / lookahead;
        return curvature;
    }
    
    // ========================================================================
    // HEADING CONTROL - IMU FRAME
    // ========================================================================
    
    /**
     * Heading correction:
     * waypoint headings are IMU degrees (0=N, 90=E, CW+)
     * robot_pos.theta is IMU radians (CW+)
     */
    double calculateHeadingCorrection(const point& robot_pos) {
        double target_heading_deg = path[current_segment_idx].heading;
        if (current_segment_idx < (int)path.size() - 1) {
            target_heading_deg = path[current_segment_idx + 1].heading;
        }
        
        double target_heading_rad = util::to_rad(target_heading_deg);

        // Error in IMU frame (CW+)
        double heading_error = target_heading_rad - robot_pos.theta;
        heading_error = util::wrap_to_pi(heading_error);
        
        point target_waypoint = Waypoint_to_point(
            path[std::min(current_segment_idx + 1, (int)path.size() - 1)]
        );
        double distance_to_target = robot_pos.distance_to(target_waypoint);
        
        double blend_weight = 1.0 - std::exp(
            -params.heading_blend_power *
            (params.heading_blend_dist - distance_to_target) /
             params.heading_blend_dist
        );
        blend_weight = std::clamp(blend_weight, 0.0, 1.0);
        
        double omega_correction = params.heading_kp * heading_error;
        return blend_weight * omega_correction;
    }
    
    // ========================================================================
    // SETTLING
    // ========================================================================
    
    bool isSettled(const point& robot_pos) {
        if (current_segment_idx < (int)path.size() - 2) {
            settle_count = 0;
            return false;
        }
        
        point final_point = Waypoint_to_point(path.back());
        double distance_error = robot_pos.distance_to(final_point);
        
        double target_heading_rad = util::to_rad(path.back().heading);
        double heading_error = target_heading_rad - robot_pos.theta;
        heading_error = util::wrap_to_pi(heading_error);
        double heading_error_deg = std::abs(util::to_deg(heading_error));
        
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
    
    PurePursuitController(const std::vector<Waypoint>& path,
                          const PursuitParams& params = PursuitParams())
        : path(path), params(params) {}
    
    // ========================================================================
    // MAIN EXECUTION LOOP
    // ========================================================================
    
    /**
     * GLOBAL FRAME (IMU-native):
     * - +X East, +Y North
     * - Headings CW positive, 0° North
     */
    void execute(int timeout = 5000) {
    if (path.size() < 2) return;

    current_segment_idx = 0;
    settle_count = 0;

    util::timer safety_timer(timeout);
    safety_timer.restart();
    global::con.clear();

    // --- terminal handoff state ---
    bool do_terminal_handoff = false;
    double terminal_signed_forward = 0.0;
    double terminal_heading_deg = path.back().heading; // IMU degrees

    while (true) {
        global::odom.update();
        point robot_pos = global::odom.current_pos;

        point final_point = Waypoint_to_point(path.back());
        double dist_to_final = robot_pos.distance_to(final_point);

        // ============================================================
        // TERMINAL HANDOFF CHECK (uses your PID settle code)
        // ============================================================
        // Robot forward vector in IMU-native frame:
        // 0 rad = +Y (north), CW+  => forward = (sinθ, cosθ)
        double dxF = final_point.x - robot_pos.x;
        double dyF = final_point.y - robot_pos.y;

        double fwd_x = std::sin(robot_pos.theta);
        double fwd_y = std::cos(robot_pos.theta);

        // signed distance along robot forward axis
        double forward_proj = dxF * fwd_x + dyF * fwd_y;

        // Tune this (inches). Should be a bit larger than your settle dist.
        const double TERMINAL_DIST = std::max(5.0, params.path_completion_dist * 1.5);

        // If we're close enough OR we've passed the final plane (forward_proj < 0),
        // stop pursuit and let PID finish cleanly.
        if (dist_to_final < TERMINAL_DIST || forward_proj < -1.0) {
            do_terminal_handoff = true;
            terminal_signed_forward = forward_proj;     // can be negative -> reverse
            terminal_heading_deg = path.back().heading; // IMU degrees
            break;
        }

        // ============================================================
        // NORMAL PURE PURSUIT
        // ============================================================
        double lookahead = getAdaptiveLookahead();
        if (dist_to_final < lookahead * 1.5) {
            lookahead = std::max(params.min_lookahead, dist_to_final * 0.7);
        }

        auto [found, lookahead_point] = findLookaheadPoint(robot_pos, lookahead);
        if (!found) {
            global::chassis.move(0, 0);
            break;
        }

        double curvature = calculatePursuitCurvature(robot_pos, lookahead_point, lookahead);
        double target_velocity = path[current_segment_idx].velocity;

        if (dist_to_final < 12.0) {
            double slowdown_factor = std::max(0.2, dist_to_final / 12.0);
            target_velocity *= slowdown_factor;
        }

        double v = target_velocity;

        double omega_pursuit = curvature * v;
        double omega_heading = calculateHeadingCorrection(robot_pos);

        double omega_total_imu;
        if (dist_to_final < 4.0) {
            omega_total_imu = omega_heading * 2.0 + omega_pursuit * 0.2;
        } else if (dist_to_final < 8.0) {
            omega_total_imu = omega_heading * 1.5 + omega_pursuit * 0.5;
        } else {
            omega_total_imu = omega_pursuit + omega_heading;
        }

        // IMU CW+ -> chassis CCW+
        double omega_total_ccw = -omega_total_imu;

        double v_left  = v - (omega_total_ccw * global::chassis.track_width / 2.0);
        double v_right = v + (omega_total_ccw * global::chassis.track_width / 2.0);

        v_left  = std::clamp(v_left,  -127.0, 127.0);
        v_right = std::clamp(v_right, -127.0, 127.0);

        global::chassis.move((int)v_left, (int)v_right);

        // Debug print (keeping your style)
        point target_waypoint = Waypoint_to_point(
            path[std::min(current_segment_idx + 1, (int)path.size() - 1)]
        );
        double dist_to_target = robot_pos.distance_to(target_waypoint);

        lynx::util::print_info(
            safety_timer.elapsed(),
            &global::con,
            {"X", "Y", "TH", "DT", "DF", "S"},
            {robot_pos.x, robot_pos.y, util::to_deg(robot_pos.theta),
             dist_to_target, dist_to_final, (double)current_segment_idx}
        );

        if (isSettled(robot_pos)) break;
        if (safety_timer.has_elapsed()) break;

        pros::delay(5);
    }

    global::chassis.move(0, 0);

    // ============================================================
    // TERMINAL PID SETTLE (your trusted code)
    // ============================================================
    if (do_terminal_handoff) {
        // Drive signed distance to final (negative -> reverse)
        global::chassis.straight(terminal_signed_forward, 1500, 1.0);

        // Then lock final heading
        global::chassis.turn_abs(terminal_heading_deg, 1200, 1.0);
    }
}

};

// ============================================================================
// CHASSIS INTEGRATION - Add pure pursuit to the drive class
// ============================================================================

inline void drive::purePursuit(const std::vector<Waypoint>& path, 
                               int timeout, const PursuitParams& params) {
    PurePursuitController controller(path, params);
    controller.execute(timeout);
}

} // namespace lynx
