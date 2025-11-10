#pragma once
#include "main.h"
#include <chrono>
#include <vector>
#include <cmath>
#include <type_traits>

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

                u_int32_t elapsed() {
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
                }

                void update_coefficients(const std::vector<long double>& coeffs){
                    this->coefficients = coeffs;
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

        float to_rad(float angle){
            return (angle/(180/M_PI));
        }

        float to_deg(float angle){
            return(angle*(180/M_PI));
        }

        float deadband(float input, float range){
            if (std::fabs(input) < range) return 0;
            return input;
        }

        bool is_line_settled(float desired_x, float desired_y, float desired_angle_deg, float current_x, float current_y){
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

        timer drive_timer; //both are initialized with default values of target=0 - update later based on the user's needs
        timer turn_timer;
        timer arc_timer;

    }
}