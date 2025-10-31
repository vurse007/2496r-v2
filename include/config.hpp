#pragma once
#include "main.h"
#include "lynx-v2/odom.hpp"
#include "lynx-v2/util.hpp"
#include "lynx-v2/chassis.hpp"

namespace global {

    pros::Rotation horizontal_pod(9);
    pros::Rotation vertical_pod(10);
    pros::Imu imu(1);

    inline lynx::drive chassis {
        {{-11, pros::E_MOTOR_GEARSET_06}, {12, pros::E_MOTOR_GEARSET_06}, {-13, pros::E_MOTOR_GEARSET_06} },//left motors
        {{18, pros::E_MOTOR_GEARSET_06}, {-19, pros::E_MOTOR_GEARSET_06}, {20, pros::E_MOTOR_GEARSET_06} },//right motors
        3.25, //wheel diameter in inches
        0.75, //external gear ratio
        12.0, //track width in inches
        &imu,
        &vertical_pod
    };

    inline lynx::odometry odom{
        1,
        0,
        0,
        &horizontal_pod,
        &vertical_pod
    };

}