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

    //chassis.turn_abs(90, 10000);
    
    // queue.schedule_delay(500, [] {matchLoaderP.set_value(true);});
    //chassis.boomerang(20, 20, 90, 0.1);

    // chassis.ramsete(20, 20, 90, 2.0, 0.5, 2.0, 0.7, 2000, 1.0);
    // chassis.move(127, 127);
    // delay(300);
    // chassis.move(0, 0);
    // delay(1000);
    //chassis.turn_abs(90);
    //chassis.straight(30);
}

// void solo_awp_right(){
//     queue.start();
    
//     // Reset odometry at origin, facing 0° (forward along X axis)
//     odom.reset(0, 0, 0);
    
//     // Print initial state
//     con.clear();
//     con.print(0, 0, "X:%.1f Y:%.1f", odom.current_pos.x, odom.current_pos.y);
//     con.print(1, 0, "Th:%.1f IMU:%.1f", 
//               lynx::util::to_deg(odom.current_pos.theta), 
//               imu.get_heading());
//     delay(2000);
    
//     // Drive forward with both motors
//     chassis.move(60, 60);
//     delay(1000);
//     chassis.move(0, 0);
    
//     // Update and check if Y increased (should increase if going "forward" in your coordinate system)
//     odom.update();
//     con.print(2, 0, "After: Y=%.1f", odom.current_pos.y);
//     delay(3000);
// }

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

