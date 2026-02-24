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
    //global::chassis.turn_abs(90, 3000);
    global::chassis.straight(24);
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

