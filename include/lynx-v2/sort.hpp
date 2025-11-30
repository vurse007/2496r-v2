#pragma once
#include "main.h"

namespace lynx {

class ColorSort {
private:

    pros::Optical* optical = nullptr;
    pros::adi::Pneumatics* piston = nullptr;

    std::string alliance_color = "red";     // default

    int eject_open_time     = 165;          // default
    int eject_buffer        = 100;          // default
    int proximity_threshold = 180;          // default

    double red_low  = 0,   red_high  = 20;  // default
    double blue_low = 210, blue_high = 230; // default

    bool eject_requested = false;
    pros::Task* worker_task = nullptr;

    bool colorToggle = true;

public:

    // -------------------------------------------------------------------
    // CONSTRUCTOR — takes int* and char* (nullable)
    // -------------------------------------------------------------------
    ColorSort(int* optical_port,
              char* piston_port,
              int open_ms = 165,    // default
              int buffer_ms = 310)  // default
        : eject_open_time(open_ms),
          eject_buffer(buffer_ms)
    {
        if (optical_port)
        {
            optical = new pros::Optical(*optical_port);
            optical->set_led_pwm(100);          // defaults
            optical->set_integration_time(25);  // default
        }

        if (piston_port) piston = new pros::adi::Pneumatics(*piston_port, false);

        // -------- START SINGULAR BACKGROUND TASK --------
        worker_task = new pros::Task([this] { this->task_loop(); });
    }

    // -------------------------------------------------------------------
    // SETTERS and GETTERS
    // -------------------------------------------------------------------
    void set_alliance_color(const std::string& c)   { alliance_color = c; }

    void set_red_range(double low, double high)     { red_low = low; red_high = high; }
    void set_blue_range(double low, double high)    { blue_low = low; blue_high = high; }

    void set_proximity_threshold(int prox)          { proximity_threshold = prox; }
    void set_led_pwm(int pwm)                       { if (optical) optical->set_led_pwm(pwm); }
    void set_integration_time(int ms)               { if (optical) optical->set_integration_time(ms); }

    void set_all(int proxy, int led_pwm, int integration, const std::string& alliance) {
        set_proximity_threshold(proxy);
        set_led_pwm(led_pwm);
        set_integration_time(integration);
        set_alliance_color(alliance);
    }

    void enable()  { colorToggle = true; }
    void disable() { colorToggle = false; }
    void toggle()  { colorToggle = !colorToggle; }
    bool is_enabled() const { return colorToggle; }

    // -------------------------------------------------------------------
    // UPDATE LOOP
    // -------------------------------------------------------------------
    void update() {

        if (!colorToggle) return;
        if (!optical || !piston) return;

        int prox   = optical->get_proximity();
        if (prox < proximity_threshold) return;

        double hue = optical->get_hue();

        bool isRed  = (hue > red_low  && hue < red_high);
        bool isBlue = (hue > blue_low && hue < blue_high);

        if (!isRed && !isBlue) return;

        if (isRed  && alliance_color == "blue") eject_requested = true;
        if (isBlue && alliance_color == "red")  eject_requested = true;
    }

private:

    // -------------------------------------------------------------------
    // BACKGROUND TASK — runs forever — ONLY ONE TASK
    // -------------------------------------------------------------------
    void task_loop() {
        while (true) {
            if (eject_requested) {
                eject_requested = false;

                // eject sequence (blocking inside this task, not main)
                piston->set_value(true);
                pros::delay(eject_open_time);
                piston->set_value(false);
                pros::delay(eject_buffer);
            }

            pros::delay(10);
        }
    }
};

} // namespace lynx