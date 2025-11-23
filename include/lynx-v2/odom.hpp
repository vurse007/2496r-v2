

#pragma once
#include "main.h" 
#include "util.hpp"

#include <optional> // Forward declaration of wrap_to_pi 

namespace global { extern lynx::drive chassis; }

namespace lynx {

    class point {
    public:
        double x;
        double y;
        double theta;

        point(double x, double y, double theta = std::numeric_limits<double>::quiet_NaN())
            : x(x), y(y), theta(theta) {}

        inline double distance_to(const point& other) const {
            double dx = other.x - x, dy = other.y - y;
            return std::sqrt(dx * dx + dy * dy);
        }

        // In your point class:
        inline double angle_to(const point& other) const {
            double bearing = std::atan2(other.y - y, other.x - x);
            bearing = util::to_deg(bearing);  // Convert to degrees
            
            double error = bearing - theta;
            
            // Angle wrap
            while (error > 180.0)  error -= 360.0;
            while (error < -180.0) error += 360.0;
            
            return error;
        }

    };

    class odometry {
    public:
        // --- Configuration ---
        double odom_wheel_diameter;
        double odom_horizontal_offset;
        double odom_vertical_offset;

        pros::Rotation* horizontal_pod;
        pros::Rotation* vertical_pod;

        // --- Runtime state ---
        double theta_deg = 0.0;
        double prev_horizontal_in = 0.0;
        double prev_vertical_in   = 0.0;
        double prev_theta         = 0.0;

        double delta_distance = 0.0;
        double total_distance = 0.0;
        double imu_heading_offset_deg = 0.0;

        double horizontal_encoder_raw = 0.0;
        double vertical_encoder_raw   = 0.0;

        point current_pos{0.0, 0.0, 0.0};

        // --- Constructor ---
        odometry(double odwheeldiam, double h_offset, double v_offset,
                pros::Rotation* h_pod, pros::Rotation* v_pod)
            : odom_wheel_diameter(odwheeldiam),
            odom_horizontal_offset(h_offset),
            odom_vertical_offset(v_offset),
            horizontal_pod(h_pod),
            vertical_pod(v_pod) {}

        // --- Utility conversions ---
        inline double pod_ticks_to_inches(double ticks) const {
            // adjust if pods are 360 instead of 36000 per rev
            return (ticks / 36000.0) * (M_PI * odom_wheel_diameter);
        }

        inline double degrees_to_radians(double deg) const { return deg * M_PI / 180.0; }
        inline double radians_to_degrees(double rad) const { return rad * 180.0 / M_PI; }

        // --- Sensor read ---
        inline void read_sensors(pros::Rotation& horizontal, pros::Rotation& vertical, pros::Imu& inertial) {
            horizontal_encoder_raw = horizontal.get_position();
            vertical_encoder_raw   = vertical.get_position();
            theta_deg              = inertial.get_heading();
        }

        // --- Reset ---
        inline void reset(double new_x = 0.0, double new_y = 0.0, std::optional<double> new_theta = std::nullopt) {
            // zero rotation sensors to eliminate huge initial deltas
            horizontal_pod->reset();
            vertical_pod->reset();

            current_pos.x = new_x;
            current_pos.y = new_y;

            const double desired_deg = new_theta.has_value()
                ? radians_to_degrees(new_theta.value())
                : 0.0;

            read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu);
            imu_heading_offset_deg = theta_deg - desired_deg;
            current_pos.theta = degrees_to_radians(theta_deg - imu_heading_offset_deg);

            prev_horizontal_in = pod_ticks_to_inches(horizontal_encoder_raw);
            prev_vertical_in   = pod_ticks_to_inches(vertical_encoder_raw);
            prev_theta         = current_pos.theta;

            delta_distance = 0.0;
            total_distance = 0.0;
        }

        // --- Update ---
        inline void update() {
            read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu);

            const double theta_prev = current_pos.theta;
            double theta_now = theta_now = util::wrap_to_pi(degrees_to_radians(theta_deg - imu_heading_offset_deg));
            if (std::isnan(theta_now) || std::isinf(theta_now)) theta_now = 0.0;  // safe seed
            theta_now = util::wrap_to_pi(theta_now);
            current_pos.theta = theta_now;

            const double h_in_now = pod_ticks_to_inches(horizontal_encoder_raw);
            const double v_in_now = pod_ticks_to_inches(vertical_encoder_raw);

                        // 1. Compute raw deltas
            double dS = h_in_now - prev_horizontal_in;   // sideways (robot-right positive)
            double dF = v_in_now - prev_vertical_in;     // forward (robot-forward positive)

            // 2. Apply rotation offsets due to turning
            // double dF = dV - odom_vertical_offset * dTheta;    // forward corrected
            // double dS = dH - odom_horizontal_offset * dTheta;  // sideways corrected

            // 3. Convert IMU heading to radians (no rotation!)
            double heading = degrees_to_radians(theta_deg - imu_heading_offset_deg);

            // 4. Rotate robot-local motion into IMU field frame:
            // IMU FRAME: 0° north, 90° east, clockwise positive
            double cosH = std::cos(heading);
            double sinH = std::sin(heading);

            // GLOBAL FRAME MATH MATCHING IMU:
            double global_dx =  dS * cosH + dF * sinH;   // EAST/WEST   (+X east)
            double global_dy =  dF * cosH - dS * sinH;   // NORTH/SOUTH (+Y north)

            // 5. Integrate global position
            current_pos.x += global_dx;
            current_pos.y += global_dy;


            prev_horizontal_in = h_in_now;
            prev_vertical_in   = v_in_now;
            prev_theta         = current_pos.theta;
        }

        inline point carrotPoint(const point& robot, const point& target, double dLead) {
            double distance = robot.distance_to(target);
            
            // Use target's heading angle (in radians) for offset direction
            double targetThetaRad = util::to_rad(target.theta);
            
            // Offset from target in opposite direction of target angle
            // Offset distance scales with distance to target
            return point(
                target.x - distance * std::cos(targetThetaRad) * dLead,
                target.y - distance * std::sin(targetThetaRad) * dLead,
                target.theta
            );
        }


        // --- Accessors ---
        inline double get_delta_distance() const { return delta_distance; }
        inline double get_total_distance() const { return total_distance; }
        inline void reset_distance() { delta_distance = 0.0; total_distance = 0.0; }

        // --- Debug helper ---
    
    }; // class odometry

} // namespace lynx


// #pragma once
// #include "main.h" 
// #include "util.hpp"

// #include <optional>

// namespace global { extern lynx::drive chassis; }

// namespace lynx {

//     class point {
//     public:
//         double x;
//         double y;
//         double theta;

//         point(double x, double y, double theta = std::numeric_limits<double>::quiet_NaN())
//             : x(x), y(y), theta(theta) {}

//         inline double distance_to(const point& other) const {
//             double dx = other.x - x, dy = other.y - y;
//             return std::sqrt(dx * dx + dy * dy);
//         }

//         inline double angle_to(const point& other) const {
//             double bearing = std::atan2(other.y - y, other.x - x);
//             bearing = util::to_deg(bearing);
            
//             double error = bearing - theta;
            
//             // Angle wrap
//             while (error > 180.0)  error -= 360.0;
//             while (error < -180.0) error += 360.0;
            
//             return error;
//         }
//     };

//     class odometry {
//     public:
//         // --- Configuration ---
//         double odom_wheel_diameter;
//         double odom_horizontal_offset;
//         double odom_vertical_offset;

//         pros::Rotation* horizontal_pod;
//         pros::Rotation* vertical_pod;

//         // --- Runtime state ---
//         double theta_deg = 0.0;
//         double prev_horizontal_in = 0.0;
//         double prev_vertical_in   = 0.0;
//         double prev_theta         = 0.0;

//         double delta_distance = 0.0;
//         double total_distance = 0.0;
//         double imu_heading_offset_deg = 0.0;

//         double horizontal_encoder_raw = 0.0;
//         double vertical_encoder_raw   = 0.0;

//         point current_pos{0.0, 0.0, 0.0};

//         // --- Constructor ---
//         odometry(double odwheeldiam, double h_offset, double v_offset,
//                 pros::Rotation* h_pod, pros::Rotation* v_pod)
//             : odom_wheel_diameter(odwheeldiam),
//             odom_horizontal_offset(h_offset),
//             odom_vertical_offset(v_offset),
//             horizontal_pod(h_pod),
//             vertical_pod(v_pod) {}

//         // --- Utility conversions ---
//         inline double pod_ticks_to_inches(double ticks) const {
//             return (ticks / 36000.0) * (M_PI * odom_wheel_diameter);
//         }

//         inline double degrees_to_radians(double deg) const { return deg * M_PI / 180.0; }
//         inline double radians_to_degrees(double rad) const { return rad * 180.0 / M_PI; }

//         // --- Sensor read with error handling ---
//         inline bool read_sensors(pros::Rotation& horizontal, pros::Rotation& vertical, pros::Imu& inertial) {
//             // Try to read each sensor, return false if any fail
//             try {
//                 horizontal_encoder_raw = horizontal.get_position();
//                 vertical_encoder_raw   = vertical.get_position();
//                 theta_deg              = inertial.get_heading();
                
//                 // Check for invalid readings
//                 if (std::isnan(horizontal_encoder_raw) || std::isinf(horizontal_encoder_raw) ||
//                     std::isnan(vertical_encoder_raw) || std::isinf(vertical_encoder_raw) ||
//                     std::isnan(theta_deg) || std::isinf(theta_deg)) {
//                     return false;
//                 }
                
//                 return true;
//             } catch (...) {
//                 return false;
//             }
//         }

//         // --- Reset ---
//         inline void reset(double new_x = 0.0, double new_y = 0.0, std::optional<double> new_theta = std::nullopt) {
//             // Zero rotation sensors to eliminate huge initial deltas
//             horizontal_pod->reset();
//             vertical_pod->reset();

//             current_pos.x = new_x;
//             current_pos.y = new_y;

//             const double desired_deg = new_theta.has_value()
//                 ? radians_to_degrees(new_theta.value())
//                 : 0.0;

//             read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu);
//             imu_heading_offset_deg = theta_deg - desired_deg;
//             current_pos.theta = degrees_to_radians(theta_deg - imu_heading_offset_deg);

//             prev_horizontal_in = pod_ticks_to_inches(horizontal_encoder_raw);
//             prev_vertical_in   = pod_ticks_to_inches(vertical_encoder_raw);
//             prev_theta         = current_pos.theta;

//             delta_distance = 0.0;
//             total_distance = 0.0;
//         }

//         // --- Update with arc correction ---
//         inline void update() {
//             // Read sensors with error checking
//             if (!read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu)) {
//                 // Sensor read failed, skip this update
//                 return;
//             }

//             // Get current heading in radians
//             double theta_now = degrees_to_radians(theta_deg - imu_heading_offset_deg);
//             theta_now = util::wrap_to_pi(theta_now);

//             // Calculate change in heading
//             double delta_theta = theta_now - prev_theta;
//             delta_theta = util::wrap_to_pi(delta_theta);

//             // Get current encoder positions in inches
//             const double h_in_now = pod_ticks_to_inches(horizontal_encoder_raw);
//             const double v_in_now = pod_ticks_to_inches(vertical_encoder_raw);

//             // Calculate raw deltas
//             double dH = h_in_now - prev_horizontal_in;  // horizontal (sideways)
//             double dV = v_in_now - prev_vertical_in;    // vertical (forward)

//             // --- ARC CORRECTION ---
//             // When robot rotates, tracking wheels follow arcs, not straight lines
//             // This corrects for that curved motion
//             double local_dx, local_dy;
            
//             if (std::abs(delta_theta) < 0.001) {
//                 // Small angle - use straight line approximation (avoids divide by zero)
//                 local_dx = dV - odom_vertical_offset * delta_theta;
//                 local_dy = dH - odom_horizontal_offset * delta_theta;
//             } else {
//                 // Large angle - use full arc correction formula
//                 // This is based on the formula: 2 * sin(dθ/2) * (distance/dθ + offset)
//                 double sin_half = std::sin(delta_theta / 2.0);
                
//                 // Forward component (vertical pod)
//                 local_dx = 2.0 * sin_half * (dV / delta_theta + odom_vertical_offset);
                
//                 // Sideways component (horizontal pod)
//                 local_dy = 2.0 * sin_half * (dH / delta_theta + odom_horizontal_offset);
//             }

//             // --- ROTATE TO GLOBAL FRAME ---
//             // Use midpoint orientation for more accurate integration
//             double mid_orientation = prev_theta + delta_theta / 2.0;
            
//             double cos_mid = std::cos(mid_orientation);
//             double sin_mid = std::sin(mid_orientation);

//             // Transform from robot frame to global frame
//             // Global frame: +X East, +Y North (IMU-native)
//             double global_dx = local_dx * sin_mid + local_dy * cos_mid;
//             double global_dy = local_dx * cos_mid - local_dy * sin_mid;

//             // Update global position
//             current_pos.x += global_dx;
//             current_pos.y += global_dy;
//             current_pos.theta = theta_now;

//             // Track total distance traveled
//             double step_distance = std::sqrt(global_dx * global_dx + global_dy * global_dy);
//             delta_distance = step_distance;
//             total_distance += step_distance;

//             // Update previous values for next iteration
//             prev_horizontal_in = h_in_now;
//             prev_vertical_in   = v_in_now;
//             prev_theta         = theta_now;
//         }

//         inline point carrotPoint(const point& robot, const point& target, double dLead) {
//             double distance = robot.distance_to(target);
            
//             // Use target's heading angle (in radians) for offset direction
//             double targetThetaRad = util::to_rad(target.theta);
            
//             // Offset from target in opposite direction of target angle
//             // Offset distance scales with distance to target
//             return point(
//                 target.x - distance * std::cos(targetThetaRad) * dLead,
//                 target.y - distance * std::sin(targetThetaRad) * dLead,
//                 target.theta
//             );
//         }

//         // --- Accessors ---
//         inline double get_delta_distance() const { return delta_distance; }
//         inline double get_total_distance() const { return total_distance; }
//         inline void reset_distance() { delta_distance = 0.0; total_distance = 0.0; }
    
//     }; // class odometry

// } // namespace lynx