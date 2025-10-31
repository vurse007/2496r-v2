#pragma once
#include <cmath>
#include <optional>
#include "main.h"
#include "config.hpp"
#include "util.hpp"

namespace lynx { 
    // ===================== Point Class =====================
    class point {
    public:
        double x;
        double y;
        double theta;

        point(double x, double y, double theta = std::numeric_limits<double>::quiet_NaN())
            : x(x), y(y), theta(theta) {}

        double distance_to(const point& other) const {
            double dx = other.x - x;
            double dy = other.y - y;
            return std::sqrt(dx * dx + dy * dy);
        }

        double angle_error(const point& other) const {
            return other.theta - theta;
        }
    };
    class odometry {
    public:

        double odom_wheel_diameter;
        double odom_horizontal_offset;
        double odom_vertical_offset;

        pros::Rotation* horizontal_pod;
        pros::Rotation* vertical_pod;

        //constructor
        odometry(double odwheeldiam, double h_offset, double v_offset, pros::Rotation* h_pod, pros::Rotation* v_pod):
            odom_wheel_diameter(odwheeldiam),
            odom_horizontal_offset(h_offset),
            odom_vertical_offset(v_offset),
            horizontal_pod(h_pod),
            vertical_pod(v_pod)
        {}

        // ===================== Internal Variables =====================
        double theta_deg = 0.0;

        double prev_horizontal_in = 0.0;
        double prev_vertical_in   = 0.0;
        double prev_theta         = 0.0;

        double delta_distance = 0.0;
        double total_distance = 0.0;

        double imu_heading_offset_deg = 0.0;

        double horizontal_encoder_raw = 0.0;
        double vertical_encoder_raw   = 0.0;

        // ===================== Utility Conversions =====================
        inline double pod_ticks_to_inches(double ticks) {
            return (ticks / 36000.0) * (M_PI * odom_wheel_diameter);
        }

        inline double degrees_to_radians(double degrees) { return degrees * M_PI / 180.0; }
        inline double radians_to_degrees(double radians) { return radians * 180.0 / M_PI; }

        point current_pos{0.0, 0.0, 0.0}; // Field X,Y (in), θ (rad)

        // ===================== Core Functions =====================

        inline void read_sensors(pros::Rotation& horizontal, pros::Rotation& vertical, pros::Imu inertial) {
            horizontal_encoder_raw = horizontal.get_position();
            vertical_encoder_raw   = vertical.get_position();
            theta_deg              = inertial.get_heading();
        }

        inline void reset(double new_x = 0.0, double new_y = 0.0, std::optional<double> new_theta = std::nullopt) {
            current_pos.x = new_x;
            current_pos.y = new_y;

            // Desired heading in degrees after reset
            const double desired_deg = new_theta.has_value()
                ? radians_to_degrees(new_theta.value())
                : 0.0;

            // Read current sensor values
            read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu);

            // Compute offset so that θ = desired_deg now
            imu_heading_offset_deg = theta_deg - desired_deg;
            current_pos.theta = degrees_to_radians(theta_deg - imu_heading_offset_deg);

            // Prime encoder readings
            prev_horizontal_in = pod_ticks_to_inches(horizontal_encoder_raw);
            prev_vertical_in   = pod_ticks_to_inches(vertical_encoder_raw);
            prev_theta         = current_pos.theta;

            delta_distance = 0.0;
            total_distance = 0.0;
        }

        inline void update() {
            // 1) Read sensors
            read_sensors(*horizontal_pod, *vertical_pod, *global::chassis.imu);

            // 2) Heading with offset (in field frame)
            const double theta_prev = current_pos.theta;
            double theta_now = degrees_to_radians(theta_deg - imu_heading_offset_deg);
            theta_now = util::wrap_to_pi(theta_now);
            current_pos.theta = theta_now;

            // 3) Convert encoder ticks to inches
            const double h_in_now = pod_ticks_to_inches(horizontal_encoder_raw); // lateral
            const double v_in_now = pod_ticks_to_inches(vertical_encoder_raw);   // forward

            // 4) Local deltas
            double dH = h_in_now - prev_horizontal_in; // lateral (right +)
            double dV = v_in_now - prev_vertical_in;   // forward (+up)

            // Uncomment to flip direction if necessary:
            // dH = -dH;
            // dV = -dV;

            // 5) Heading change
            const double dTheta = util::wrap_to_pi(current_pos.theta - theta_prev);

            // 6) Apply off-center corrections
            const double corrected_vertical   = dV - (odom_vertical_offset   * dTheta);
            const double corrected_horizontal = dH - (odom_horizontal_offset * dTheta);

            // 7) Rotate local deltas → field frame (X,Y)
            const double avg_theta = (theta_prev + current_pos.theta) / 2.0;
            const double cos_a = std::cos(avg_theta);
            const double sin_a = std::sin(avg_theta);

            const double global_dx = corrected_vertical * cos_a - corrected_horizontal * sin_a;
            const double global_dy = corrected_vertical * sin_a + corrected_horizontal * cos_a;

            // 8) Integrate position
            delta_distance  = std::sqrt(global_dx * global_dx + global_dy * global_dy);
            total_distance += delta_distance;
            current_pos.x  += global_dx;
            current_pos.y  += global_dy;

            // 9) Save previous values
            prev_horizontal_in = h_in_now;
            prev_vertical_in   = v_in_now;
            prev_theta         = current_pos.theta;
        }

        // accessors
        inline double get_delta_distance() { return delta_distance; }
        inline double get_total_distance() { return total_distance; }
        inline void reset_distance() { total_distance = 0.0; delta_distance = 0.0; }

    }; // class odom
} // namespace lynx