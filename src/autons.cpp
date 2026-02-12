#include "lynx.hpp"

lynx::Queue queue;
using global::chassis;
using global::odom;
using global::con;

void solo_awp_right(){
    queue.start();
    odom.reset(0, 0, 0);

    std::vector<lynx::Waypoint> path1 = {
        lynx::Waypoint(0, 0, 0, 100),
        lynx::Waypoint(0, 24, 0, 100),      // Start
        lynx::Waypoint(24, 24, 90, 100),
        lynx::Waypoint(0, 24, -90, 100)

    };

    

    chassis.purePursuit(path1);
}

void half_left_red(){
    queue.start();
    delay(300);
    //global::chassis.turn_abs(90, 3000);
    global::chassis.straight(24);
}

void half_left_blue(){
    delay(1);
    global::intakeRamp.set_value(true);
    //global::chassis.move_intake(127);
    global::chassis.straight(14);
    global::chassis.turn_abs(-42);
    global::chassis.straight(17.5, 1500, 0.4);
    delay(700);
    global::chassis.turn_abs(-135);
    global::chassis.straight(-13.5);
    global::intakeRamp.set_value(false);
    delay(1500);
    global::intakeRamp.set_value(true);
    global::chassis.straight(47);
    delay(200);
    global::chassis.turn_abs(180, 600);
    global::chassis.straight(-10, 200);
    global::matchLoaderP.set_value(true);

    global::chassis.straight(12, 600, 0.5);
    delay(1000);
    global::fourBarPiston.set_value(true);
    global::chassis.straight(-20, 1500, 0.8);
    global::matchLoaderP.set_value(false);
    global::intakeRamp.set_value(false);
    delay(1000);
    global::chassis.straight(7, 300);
    // global::chassis.move(-60, 127);
    // delay(500);
    // global::chassis.turn_abs(0);
    // global::wingPiston.set_value(false);
    // global::chassis.straight(15, 900);
    global::chassis.set_brake_mode(MOTOR_BRAKE_HOLD);


}

void half_right_red(){
    queue.start();
}

void half_right_blue(){
    queue.start();
}

void skills_auton(){
    queue.start();
    global::chassis.move(67,67);
    delay(150);
    global::chassis.move(0,0);
}

void blank(){
    queue.start();
}


Auton soloAwp        ("Solo Awp     ", "Red   ", solo_awp_right,   "red");
Auton autonHalfLRed  ("Half Left    ", "Red   ", half_left_red,    "red");
Auton autonHalfLBlue ("Half Left    ", "Blue  ", half_left_blue,   "blue");
Auton autonHalfRRed  ("Half Right   ", "Red   ", half_right_red,   "red");
Auton autonHalfRBlue ("Half Right   ", "Blue  ", half_right_blue,  "blue");
Auton skills         ("Skills       ", "Red   ", skills_auton,     "red");
Auton noAuto         ("BLANK        ", "Red   ", blank,            "red");

std::vector<Auton> autons = {
    autonHalfLBlue, soloAwp, autonHalfLRed, 
    autonHalfRRed, autonHalfRBlue, skills, 
    noAuto
}; 

