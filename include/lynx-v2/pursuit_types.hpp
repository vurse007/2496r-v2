#pragma once
#include <cmath>

namespace lynx {

// Forward declaration - we only need to know point exists, not its full definition
class point;

// ============================================================================
// WAYPOINT - Rich path point with position, heading, and velocity
// ============================================================================
struct Waypoint {
    double x;              // X position (inches)
    double y;              // Y position (inches)
    double heading;        // Target heading at this point (degrees)
    double velocity;       // Target velocity at this point (0-127)
    
    Waypoint(double x, double y, double heading = 0.0, double velocity = 100.0)
        : x(x), y(y), heading(heading), velocity(velocity) {}
    
    // Distance to another waypoint
    double distance_to(const Waypoint& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// ============================================================================
// PURE PURSUIT PARAMETERS - Centralized tuning
// ============================================================================
struct PursuitParams {
    // Lookahead parameters
    double base_lookahead = 14.5;        // Base lookahead distance (inches)
    double min_lookahead = 4.5;          // Minimum lookahead (sharp turns)
    double max_lookahead = 17.0;         // Maximum lookahead (straight paths)
    double curvature_scale = 0.4;        // How much curvature affects lookahead
    
    // Heading control parameters
    double heading_kp = 2.2;             // Heading correction gain
    double heading_blend_dist = 8.0;     // Distance to start blending heading
    double heading_blend_power = 4.1;    // Exponential blend curve (higher = later blend)
    
    // Path following parameters
    double path_completion_dist = 3.0;   // Distance to consider path complete (inches)
    double final_heading_tolerance = 5.0; // Final heading tolerance (degrees)
    
    // Settling parameters
    int settle_count_target = 8;         // Number of cycles to be settled (8 * 5ms = 40ms)
    
    PursuitParams() = default;
};

} // namespace lynx