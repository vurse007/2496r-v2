#pragma once
#include "main.h"
#include "pid.hpp"
#include "pursuit_types.hpp"


namespace lynx {
    struct motor_specs {
        int port;
        pros::v5::MotorGears gearset;

        motor_specs(int p, pros::v5::MotorGears g)
            : port(p), gearset(g) {}
    };

    class group {
        private:
            std::vector<std::shared_ptr<pros::Motor>> motors;
        
        public:
            group(const std::vector<motor_specs>& motor_specs) {
                for (const auto& spec : motor_specs){
                    auto motor = std::make_shared<pros::Motor>(spec.port, spec.gearset);
                    motors.push_back(motor);
                }
            }

            void set_brake_mode(pros::motor_brake_mode_e_t mode) {
                for (auto& motor : motors) {
                    motor->set_brake_mode(mode);
                }
            }

            void move(int voltage) {
                for (auto& motor : motors) {
                    motor->move(voltage);
                }
            }

            void tare(){
                for (auto& motor : motors) {
                    motor->tare_position();
                }
            }

            std::shared_ptr<pros::Motor> get_motor(int index) {
                if (index >=0 && index < motors.size()) {
                    return motors[index];
                } else{
                    return nullptr;
                }
            }

            double get_avg_pos() const {
                double total_pos = 0.0;
                int count = 0;
                for (auto& motor : motors) {
                    if (motor) {
                        total_pos += motor->get_position();
                        count++;
                    }
                }
                if (count == 0) return 0.0;
                return total_pos/count;
            }

            const std::vector<std::shared_ptr<pros::Motor>>& get_motors() const {
                return motors;
            }
    };

    //for every function that doesn't have a helper above
    inline void apply_to_group(group& motor_group, const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
        for (const auto& motor : motor_group.get_motors()) {
            if (motor) func(motor);
        }
    }

    //chassis class encapsulating left and right sides
    class drive {
        public:
            group left;
            group right;

            const double wheel_diameter;
            const double external_gear_ratio;
            const double track_width;

            pros::Imu* imu;
            pros::Rotation* distance_pod;        

            PID turn_pid{
                {1,1,1},
                {0.5,1,1},
                0,
                127,
                0,
                1000,
                0,
                0
            };

            PID drive_pid{
                {10, 0.005, 1},      // general_constants: kp, ki, kd
                {3, 0.005, 1},      // refined_constants
                1.4,                // refined_range
                0.35,               // slew
                30,                 // integral_threshold
                200,                // max_integral
                0,                  // deadband
                40                  // settle_timer_target (8 checks * 5ms)
            };

            drive(const std::vector<motor_specs>& ls, const std::vector<motor_specs>& rs, const double wd, const double egr, const double tw, pros::Imu* imu, pros::Rotation* distance_pod):
                left(ls), right(rs), wheel_diameter(wd), external_gear_ratio(egr), track_width(tw), imu(imu), distance_pod(distance_pod) {}

            // helper functions to do common tasks to motors
            void set_brake_mode(pros::motor_brake_mode_e_t mode) {
                left.set_brake_mode(mode);
                right.set_brake_mode(mode);
            }

            void move(int left_velocity, int right_velocity) {
                left.move(left_velocity);
                right.move(right_velocity);
            }

            void tare() {
                left.tare();
                right.tare();
            }

            double get_position() const{
                if (distance_pod == nullptr){
                    return (left.get_avg_pos() + right.get_avg_pos()) / 2.0;
                } else {
                    return distance_pod->get_position();
                }
            }

            double get_left_pos(){
                return left.get_avg_pos();
            }

            double get_right_pos(){
                return right.get_avg_pos();
            }

            void apply_to_chassis(const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
                for (const auto& motor : left.get_motors()) {
                    if (motor) func(motor);
                }
                for (const auto& motor : right.get_motors()) {
                    if (motor) func(motor);
                }
            }

            //sample use case:
            // my_chassis.apply_to_group(my_chassis.left, [](std::shared_ptr<pros::Motor> motor){ motor->set_voltage_limit(12000); });
            //get rid of my_chassis.left to apply to whole chassis
            //second parameter is a lambda function, needs to have a motor pointer as parameter, to use when applying the function

            void straight(double target, int timeout = 2000, double scale=1.0);
            void turn_rel(double target, int timeout = 1000, double scale=1.0);
            void turn_abs(double target, int timeout = 1000, double scale=1.0);
            void boomerang(double targetx, double targety, double target_theta, double dLead = 0.5, int timeout = 2000, double scale=1.0);
            void ramsete(double targetx, double targety, double target_theta, double v_ref, double omega_ref, double b = 2.0, double zeta = 0.7, int timeout = 2000, double scale = 1.0);
            void purePursuit(const std::vector<Waypoint>& path, const PursuitParams& params = PursuitParams(), int timeout = 5000);
    };

}