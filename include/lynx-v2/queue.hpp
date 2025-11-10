#pragma once
#include "main.h"
#include "util.hpp"
#include <vector>
#include <functional>
#include <atomic>
#include <memory>
#include <algorithm>

namespace lynx {

class Queue {
    private:
        struct Action {
            uint32_t execute_time_ms;       // absolute millis when it should run
            std::function<void()> callback; // lambda to run
            bool executed = false;
        };

        std::vector<Action> actions;
        std::atomic<bool> running{false};
        std::unique_ptr<pros::Task> worker;
        lynx::util::timer auton_timer; // internal timer that starts with scheduler

    public:
        Queue() = default;

        void start() {
            if (running) return;
            running = true;
            auton_timer.restart(); // start internal timer at t=0

            worker = std::make_unique<pros::Task>([this] {
                while (running) {
                    uint32_t now = pros::millis();
                    for (auto &a : actions) {
                        if (!a.executed && now >= a.execute_time_ms) {
                            a.callback();
                            a.executed = true;
                        }
                    }
                    pros::delay(10);
                }
            });
        }

        void stop() {
            running = false;
            if (worker) worker.reset();
            actions.clear();
        }

        // Schedule a relative delay from NOW
        void schedule_delay(uint32_t delay_ms, std::function<void()> func) {
            actions.push_back({
                pros::millis() + delay_ms,
                std::move(func),
                false
            });
        }

        // Schedule a specific timestamp
        void schedule_at(uint32_t target_ms, std::function<void()> func) {
            uint32_t now = pros::millis();
            uint32_t elapsed = auton_timer.elapsed();
            uint32_t execute_time = now + (target_ms > elapsed ? (target_ms - elapsed) : 0);

            actions.push_back({
                execute_time,
                std::move(func),
                false
            });
        }

        bool is_running() const { return running; }

        void clear_executed() {
            actions.erase(
                std::remove_if(actions.begin(), actions.end(),
                            [](const Action &a) { return a.executed; }),
                actions.end()
            );
        }

        uint32_t elapsed() const { return auton_timer.elapsed(); }
    };

} // namespace lynx
