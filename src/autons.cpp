#include "lynx.hpp"

lynx::Queue queue;
using global::chassis;
using global::odom;
using global::con;

void solo_awp_right(){
    queue.start();
    odom.reset(0, 0, 0);

    chassis.purePursuit(lynx::path({
        {0, 0, 0.0, 100},
        {5.26, 19.75, 34.1, 100},
        {30.01, 30.25, 93.2, 100}
    }, 0.4, 0.5), 5000);
}

void half_left_red(){
    queue.start();
    global::chassis.move_intake(127);
    chassis.purePursuit(lynx::path({
    {0, 0, 0.0, 75},
    // {2, 5, -15, 75},
    {-2, 28, -90, 45}
    }, 0.4, 0.5), 2000);
    global::matchLoaderP.set_value(true);
    delay(600);
    chassis.turn_abs(-90);
    chassis.move_intake(127);
    chassis.straight(10.67, 700, 0.75);
    delay(350);
    chassis.turn_abs(-91, 300);
    chassis.move_intake(0);
    chassis.straight(-31, 900, 0.75);
    chassis.move_intake(-127);
    delay(50);
    chassis.move_intake(127);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(1400);
    chassis.purePursuit(lynx::path({
        {7, 28, -90, 55},
        {8, 10, -180, 45},
        {17, 3, 135, 35}
    }, 0.4, 0.5), 3000);


}

void half_left_blue(){
    queue.start();
    odom.reset(0, 0, 0);
    chassis.purePursuit(lynx::path({
        {0, 0, 357.5, 65},
        {23.51, 24.25, 90, 65},
        {48.01, 48, 26.9, 65}
    }, 0.4, 0.5), 5000);
}

void half_right_red(){
    queue.start();
    global::chassis.move_intake(127);
    chassis.purePursuit(lynx::path({
    {0, 0, 0.0, 75},
    // {2, 5, -15, 75},
    {-2, 28, 90, 45}
    }, 0.4, 0.5), 2000);
    global::matchLoaderP.set_value(true);
    delay(600);
    chassis.turn_abs(90);
    chassis.move_intake(127);
    chassis.straight(10.67, 700, 0.75);
    delay(500);
    chassis.turn_abs(91, 300);
    chassis.move_intake(0);
    chassis.straight(-31, 900, 0.75);
    chassis.move_intake(-127);
    delay(50);
    chassis.move_intake(127);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(1400);
    chassis.purePursuit(lynx::path({
        {-7, 28, -90, 55},
        {-8, 10, -180, 45},
        {-17, 3, 135, 35}
    }, 0.4, 0.5), 3000);
}

void half_right_blue(){
    queue.start();
}

void skills_auton(){
    queue.start();
    odom.reset(0, 0, 0);
    global::wingPiston.set_value(true);
    chassis.purePursuit(lynx::path({
        {0, 0, 0, 65},
        {7, 19, -60, 45},
        {-2, 29.5, -90, 50}
    }, 0.4, 0.5), 3500);
    global::matchLoaderP.set_value(true);
    delay(400);
    //chassis.move_intake(127);
    chassis.straight(16, 900, 0.35);
    delay(2500);

    chassis.straight(-5, 900, 0.95);
    global::matchLoaderP.set_value(false);

    chassis.turn_abs(15);
    chassis.purePursuit(lynx::path({
        {-13, 39, 90, 45},
        {3, 40, 70, 35},
        {20, 42.5, 70, 45},
        {85, 44.5, 70, 45},
        {86, 33.5, 120, 50},
        {86, 32.5, 120, 50},
        {89, 28, 180, 50}
    }), 5000);
    chassis.turn_abs(91, 600);
    chassis.straight(-16, 900, 0.75);
    chassis.turn_abs(91, 200);

    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(88, 400);
    delay(10);
    chassis.straight(22, 800, 0.45);
    global::set_intake(global::intakeState::STORAGE);
    delay(2500);

    chassis.turn_abs(93.25, 300);
    chassis.straight(-27, 1000, 0.55);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    chassis.straight(10, 900, 0.65);
    chassis.turn_abs(180, 600);
    global::matchLoaderP.set_value(false);
    chassis.purePursuit(lynx::path({
        {97, 0, -180, 65},
        //{92, -45, -140, 65},
        {95, -61, -180, 45},
    }, 0.8), 6000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(90, 600);
    //chassis.move_intake(127);
    chassis.straight(20, 900, 0.35);
    delay(2500);

    // ~

    chassis.straight(-5, 900, 0.95);
    global::matchLoaderP.set_value(false);
    global::wingPiston.set_value(true);

    chassis.turn_abs(-175);

    chassis.purePursuit(lynx::path({
        {80, -90, -90, 55},
        {70, -95, -90, 50},
        {20, -76, -90, 50},
        {10, -60.5, -40, 65},
        {0, -60.5, -90, 45}
    }));

    chassis.turn_abs(-91, 200);
    chassis.straight(-15, 900, 0.95);
    chassis.turn_abs(-91, 200);

    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(-89, 400);
    chassis.straight(23, 1000, 0.65);
    global::set_intake(global::intakeState::STORAGE);
    delay(2500);

    chassis.turn_abs(-93.5, 300);
    chassis.straight(-23, 1000, 0.65);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    chassis.straight(5, 800, 0.65);

    chassis.purePursuit(lynx::path({
        {0, -60.5, -90.0, 60},
        {-9.3, -45.4, 0.0, 45},
        {-16.3, -22.6, 0.0, 75}
    }, 0.2, 0.5), 5000);
}

void blank(){
    queue.start();
    odom.reset(0, 0, 0);

}


Auton soloAwp        ("Solo Awp     ", "Red   ", solo_awp_right,   "red");
Auton autonHalfLRed  ("Half Left    ", "Red   ", half_left_red,    "red");
Auton autonHalfLBlue ("Half Left    ", "Blue  ", half_left_blue,   "blue");
Auton autonHalfRRed  ("Half Right   ", "Red   ", half_right_red,   "red");
Auton autonHalfRBlue ("Half Right   ", "Blue  ", half_right_blue,  "blue");
Auton skills         ("Skills       ", "Red   ", skills_auton,     "red");
Auton noAuto         ("BLANK        ", "Red   ", blank,            "red");

std::vector<Auton> autons = {
    skills, autonHalfLBlue, soloAwp, autonHalfLRed, 
    autonHalfRRed, autonHalfRBlue, 
    noAuto
}; 

