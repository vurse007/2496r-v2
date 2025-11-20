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
            double theta_now = degrees_to_radians(theta_deg - imu_heading_offset_deg);
            if (std::isnan(theta_now) || std::isinf(theta_now)) theta_now = 0.0;  // safe seed
            theta_now = util::wrap_to_pi(theta_now);
            current_pos.theta = theta_now;

            const double h_in_now = pod_ticks_to_inches(horizontal_encoder_raw);
            const double v_in_now = pod_ticks_to_inches(vertical_encoder_raw);

            double dH = h_in_now - prev_horizontal_in;
            double dV = v_in_now - prev_vertical_in;

            // normalize delta theta robustly
            double dTheta = current_pos.theta - theta_prev;
            if (std::fabs(dTheta) > M_PI)
                dTheta = std::fmod(dTheta + M_PI, 2 * M_PI) - M_PI;

            const double corrected_vertical   = dV - (odom_vertical_offset   * dTheta);
            const double corrected_horizontal = dH - (odom_horizontal_offset * dTheta);

            const double avg_theta = (theta_prev + current_pos.theta) / 2.0;
            const double cos_a = std::cos(avg_theta);
            const double sin_a = std::sin(avg_theta);

            const double global_dx = corrected_vertical * cos_a - corrected_horizontal * sin_a;
            const double global_dy = corrected_vertical * sin_a + corrected_horizontal * cos_a;

            delta_distance  = std::sqrt(global_dx * global_dx + global_dy * global_dy);
            total_distance += delta_distance;
            current_pos.x  += global_dx;
            current_pos.y  += global_dy;

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
