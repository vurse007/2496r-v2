#include "lynx.hpp"

lynx::Queue queue;
using namespace global;

void solo_awp_right(){
    queue.start();
    // queue.schedule_delay(500, [] {matchLoaderP.set_value(true);});
    chassis.boomerang(20, 1, 90);
    // chassis.move(127, 127);
    // delay(300);
    // chassis.move(0, 0);
    // delay(1000);
    //chassis.turn_abs(90);
    //chassis.straight(30);
}

void half_left_red(){
    queue.start();
}

void half_left_blue(){
    queue.start();
}

void half_right_red(){
    queue.start();
}

void half_right_blue(){
    queue.start();
}

void skills_auton(){
    queue.start();
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
    soloAwp, autonHalfLRed, autonHalfLBlue,
    autonHalfRRed, autonHalfRBlue, skills, 
    noAuto
}; 

