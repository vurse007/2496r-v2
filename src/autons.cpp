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
    delay(300);
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
        {3, 39, 75, 35},
        {20, 39, 75, 50},
        {73, 41.5, 75, 45},
        {82, 29.25, 110, 35},
        {85, 29, 90, 35}
    }), 5000);
    chassis.turn_abs(91, 200);
    chassis.straight(-14, 900, 0.95);
    chassis.turn_abs(91, 200);

    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(88, 400);
    delay(10);
    chassis.straight(16, 600, 0.65);
    global::set_intake(global::intakeState::STORAGE);
    delay(2500);

    chassis.turn_abs(94, 300);
    chassis.straight(-22, 1000, 0.55);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    chassis.straight(5, 800, 0.65);
    chassis.turn_abs(180, 600);
    global::matchLoaderP.set_value(false);
    chassis.purePursuit(lynx::path({
        {83, 0, -180, 65},
        {83, -63, -180, 65},
    }));
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(90, 600);
    //chassis.move_intake(127);
    chassis.straight(17, 900, 0.35);
    delay(2500);

    chassis.straight(-5, 900, 0.95);
    global::matchLoaderP.set_value(false);
    global::wingPiston.set_value(true);

    chassis.turn_abs(-165);

    chassis.purePursuit(lynx::path({
        {93, -101, -90, 65},
        {80, -101, -90, 35},
        {70, -101, -86, 50},
        {20, -100, -86, 50},
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

