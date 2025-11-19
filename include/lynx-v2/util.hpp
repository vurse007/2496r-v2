#pragma once
#include "main.h"
#include <chrono>
#include <vector>
#include <cmath>
#include <type_traits>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm> 

namespace lynx {
    namespace util {

        class timer {
            public:
                u_int32_t start_time = pros::millis();
                u_int32_t target_time;
                bool running = false;

                //constructor
                timer(u_int32_t t = 0): target_time(t) {}

                void start(){
                    start_time = pros::millis();
                    running = true;
                }

                u_int32_t elapsed() const{
                    if (!running) return 0;
                    return pros::millis() - start_time;
                }

                void reset(){
                    start_time = 0;
                    running = false;
                }

                void restart(){
                    start();
                }

                bool has_elapsed(u_int32_t ms=0){
                    if (ms==0) ms = target_time;
                    return elapsed() >= ms;
                }

                void stop(){
                    running = false;
                }
        };

        class poly{
            private:
                std::vector<long double> coefficients;

            public:
                poly(const std::vector<long double>& coeffs){
                    this->coefficients = coeffs;
                    std::reverse(this->coefficients.begin(), this->coefficients.end()); //reverse the coefficients internally so it works with the horner's method
                }

                void update_coefficients(const std::vector<long double>& coeffs){
                    this->coefficients = coeffs;
                    std::reverse(this->coefficients.begin(), this->coefficients.end());
                }

                long double evaluate(long double x) const{
                    long double y = 0.00;
                    for (const double& coeff : this->coefficients){
                        y = (y*x) + coeff; //using const to prevent changing of coeffs and & for preventing copy of the vector every time the for loop runs
                    }
                    
                    return y;

                }

                long double scientific_notation(long double number, double exponent){
                    return number * pow(10, exponent);
                }
        };

        inline double wrap_to_pi(double angle) {
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            return angle - M_PI;
        }

        inline double wrap_to_deg(double angle) {
            angle = std::fmod(angle + 180, 360);
            if (angle < 0) angle += 360;
            return angle - 180;
        }

        inline double absolute_logic(double init_heading, pros::Imu *inertial){
            return fmod((init_heading - inertial->get_heading() + 540), 360) - 180;
        }

        inline void print_info(int time, pros::Controller *controller, const std::vector<std::string>& labels, const std::vector<double>& values) {
            if (!controller || labels.empty() || labels.size() != values.size()) return;

            const int pairs_per_line = std::ceil(labels.size() / 3.0);

            auto build_line = [&](int start, int end) -> std::string {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2); // two decimal places
                for (int i = start; i < end && i < (int)labels.size(); ++i) {
                    oss << labels[i] << ": " << values[i];
                    if (i < end - 1 && i < (int)labels.size() - 1) oss << " | ";
                }
                return oss.str();
            };

            if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0) {
                std::string line0 = build_line(0, pairs_per_line);
                controller->print(0, 0, "%s   ", line0.c_str());
            }

            if (time % 100 == 0 && time % 150 != 0) {
                std::string line1 = build_line(pairs_per_line, pairs_per_line * 2);
                controller->print(1, 0, "%s   ", line1.c_str());
            }

            if (time % 150 == 0 && time % 300 != 0) {
                std::string line2 = build_line(pairs_per_line * 2, pairs_per_line * 3);
                controller->print(2, 0, "%s   ", line2.c_str());
            }
        }

        inline float to_rad(float angle){
            return (angle/(180/M_PI));
        }

        inline float to_deg(float angle){
            return(angle*(180/M_PI));
        }

        inline double gLeadExp(float distance, float k = 10.0, float lambda = 0.1) {
            return k * (1 - std::exp(-lambda * distance));
        }

        // Polynomial GLead function
        inline double gLeadPoly(float distance, const std::vector<long double>& coeffs = {0.1, 0.5, 2.0}) { //coefficients in normal order (highest degree first)
            poly p(coeffs);
            return p.evaluate(distance);
        }

        inline float deadband(float input, float range){
            if (std::fabs(input) < range) return 0;
            return input;
        }

        inline bool is_line_settled(float desired_x, float desired_y, float desired_angle_deg, float current_x, float current_y){
            return( (desired_y-current_y) * cos(to_rad(desired_angle_deg)) <= -(desired_x-current_x) * sin(to_rad(desired_angle_deg)) );
        }

        //enum class for passing flags into a function to change the way it behaves
        enum class flags {
            none = 0, //0000
            prt_error = 1 << 0, //0001
            prt_time = 1 << 1, //0010
            hc_off = 1 << 2 //0100
        };
        //since we are using enum class for safety we must do some operator functions since the types need to be casted
        inline flags operator|(flags a, flags b){
            return static_cast<flags>(
                static_cast<std::underlying_type_t<flags>>(a) | static_cast<std::underlying_type_t<flags>>(b)
            );
        }
        inline flags operator|=(flags& a, flags b){
            a = a | b;
            return a;
        }
        inline bool has_flag(flags flag, flags flags_to_check){
            return (static_cast<std::underlying_type_t<flags>>(flag) & static_cast<std::underlying_type_t<flags>>(flags_to_check)) != 0;
        }

        inline timer drive_timer; //both are initialized with default values of target=0 - update later based on the user's needs
        inline timer turn_timer;
        inline timer arc_timer;
    }
}