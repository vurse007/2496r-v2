#pragma once
#include "main.h"
#include "lynx-v2/chassis.hpp"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/util.hpp"

namespace global {

    pros::Rotation horizontal_pod(10);
    pros::Rotation vertical_pod(16);
    pros::Imu imu(7);

    pros::Controller con(pros::E_CONTROLLER_MASTER);

    inline lynx::drive chassis {
        {{-11, pros::v5::MotorGears::blue}, {12, pros::v5::MotorGears::blue}, {-13, pros::v5::MotorGears::blue}},//left motors
        {{18, pros::v5::MotorGears::blue}, {-19, pros::v5::MotorGears::blue}, {20, pros::v5::MotorGears::blue}},//right motors
        3.25, //wheel diameter in inches
        0.75, //external gear ratio
        12.0, //track width in inches
        &imu,
        &vertical_pod
    };

    inline lynx::odometry odom{
        2,
        0,
        0,
        &horizontal_pod,
        &vertical_pod
    };

}