#include "lynx.hpp"

lynx::Queue queue;
using global::chassis;
using global::odom;
using global::con;


void solo_awp_right(){
    queue.start();
    odom.reset(0, 0, 0);
    chassis.straight(32, 1200, 0.6);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(90, 600);
    chassis.move_intake(127);
    chassis.straight(22.5, 800, 0.35);
    delay(150);
    queue.schedule_delay(800, []() {
        chassis.move_intake(0);
    });
    queue.schedule_delay(400, []() {
        global::matchLoaderP.set_value(false);
    });
    chassis.straight(-29.5, 900, 0.6);
    
    chassis.move_intake(-127);
    queue.schedule_delay(50, []() {
        chassis.turn_abs(90, 500);
    });
    queue.schedule_delay(300, []() {
        chassis.straight(-4, 300);
    });
    delay(10);
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.move_intake(127);
    delay(1100);
    global::matchLoaderP.set_value(false);
    chassis.straight(11, 800, 0.65);
    delay(50);
    chassis.turn_abs(-145, 800);
    queue.schedule_delay(300, []() {
        global::set_intake(global::intakeState::STORAGE);
    });
    // queue.schedule_delay(700, []() {
    //     global::matchLoaderP.set_value(true);
    // });
    // queue.schedule_delay(900, []() {
    //     global::matchLoaderP.set_value(false);
    // });
    chassis.straight(28, 900, 0.65);
    delay(25);
    chassis.turn_abs(185, 800);
    delay(25);
    queue.schedule_delay(900, []() {
        global::matchLoaderP.set_value(true);
    });
    queue.schedule_delay(1250, []() {
        global::matchLoaderP.set_value(false);
    });
    
    chassis.straight(38, 1200, 0.7);
    // delay(50);
    chassis.turn_abs(133, 900);
    // delay(50);
    chassis.straight(-18.75, 800, 0.70);
    chassis.move_intake(-127);
    global::set_intake(global::intakeState::MID_GOAL);
    chassis.move_intake(127);
    delay(175);
    chassis.turn_abs(139, 300);
    chassis.move_intake(0);
    global::set_intake(global::intakeState::STORAGE);

    chassis.straight(39.5, 1100, 0.55);
    delay(75);
    global::matchLoaderP.set_value(true);

    chassis.turn_abs(90, 800);
    chassis.move_intake(127);
    chassis.straight(27, 900, 0.35);
    delay(185);
    queue.schedule_delay(800, []() {
        chassis.move_intake(0);
    });
    chassis.straight(-31, 900, 0.55);
    
    chassis.move_intake(-127);
    queue.schedule_delay(50, []() {
        chassis.turn_abs(90, 300);
    });
    queue.schedule_delay(300, []() {
        chassis.straight(-3, 300);
    });
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.move_intake(127);
    global::matchLoaderP.set_value(false);

    delay(1350);

    





}
// void solo_awp_right(){
//     queue.start();
//     odom.reset(0, 0, 0);
//     chassis.straight(31.25, 1200, 0.6);
//     global::matchLoaderP.set_value(true);
//     chassis.turn_abs(90, 600);
//     chassis.move_intake(127);
//     chassis.straight(22.5, 900, 0.35);
//     delay(215);
//     queue.schedule_delay(800, []() {
//         chassis.move_intake(0);
//     });
//     queue.schedule_delay(400, []() {
//         global::matchLoaderP.set_value(false);
//     });
//     chassis.straight(-31, 900, 0.65);
    
//     chassis.move_intake(-127);
//     queue.schedule_delay(50, []() {
//         chassis.turn_abs(90, 500);
//     });
//     queue.schedule_delay(300, []() {
//         chassis.straight(-4, 300);
//     });
//     delay(10);
//     global::set_intake(global::intakeState::HIGH_GOAL);
//     chassis.move_intake(127);
//     delay(1100);
//     global::matchLoaderP.set_value(false);
//     chassis.straight(16, 800, 0.65);
//     delay(50);
//     chassis.turn_abs(-145, 800);
//     queue.schedule_delay(300, []() {
//         global::set_intake(global::intakeState::STORAGE);
//     });
//     // queue.schedule_delay(700, []() {
//     //     global::matchLoaderP.set_value(true);
//     // });
//     // queue.schedule_delay(900, []() {
//     //     global::matchLoaderP.set_value(false);
//     // });
//     chassis.straight(32, 900, 0.65);
//     delay(25);
//     chassis.turn_abs(180, 400);
//     delay(25);
//     queue.schedule_delay(1000, []() {
//         global::matchLoaderP.set_value(true);
//     });
//     queue.schedule_delay(1250, []() {
//         global::matchLoaderP.set_value(false);
//     });
//     chassis.straight(42.25, 1200, 0.7);
//     // delay(50);
//     chassis.turn_abs(133, 900);
//     // delay(50);
//     chassis.straight(-17.15, 800, 0.70);
//     chassis.move_intake(-127);
//     global::set_intake(global::intakeState::MID_GOAL);
//     chassis.move_intake(127);
//     delay(175);
//     chassis.turn_abs(139, 300);
//     chassis.move_intake(0);
//     global::set_intake(global::intakeState::STORAGE);

//     chassis.straight(46.5, 1100, 0.55);
//     delay(50);
//     global::matchLoaderP.set_value(true);

//     chassis.turn_abs(90, 800);
//     chassis.move_intake(0);
//     chassis.move_intake(127);
//     chassis.straight(25, 900, 0.35);
//     delay(185);
//     queue.schedule_delay(800, []() {
//         chassis.move_intake(0);
//     });
//     chassis.straight(-31, 900, 0.55);
    
//     chassis.move_intake(-127);
//     queue.schedule_delay(50, []() {
//         chassis.turn_abs(90, 300);
//     });
//     queue.schedule_delay(300, []() {
//         chassis.straight(-3, 300);
//     });
//     global::set_intake(global::intakeState::HIGH_GOAL);
//     chassis.move_intake(127);
//     global::matchLoaderP.set_value(false);

//     delay(1350);

    





// }

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
    chassis.straight(31.75, 1200, 0.7);
    delay(50);
    chassis.turn_abs(-90, 600);
    global::matchLoaderP.set_value(true);
    delay(400);
    chassis.move_intake(127);
    chassis.straight(22.5, 900, 0.35);
    delay(350);
    queue.schedule_delay(300, []() {
        chassis.move_intake(0);
    });
    chassis.straight(-31, 900, 0.55);
    chassis.move_intake(-127);
    queue.schedule_delay(300, []() {
        chassis.straight(-3, 300);
    });
    delay(50);
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.move_intake(127);
    delay(1350);
    global::matchLoaderP.set_value(false);
    chassis.straight(18.5, 800, 0.65);
    delay(100);
    chassis.turn_abs(136, 800);
    global::set_intake(global::intakeState::STORAGE);
    delay(200);

    chassis.straight(30, 900, 0.65);
    global::matchLoaderP.set_value(true);
    delay(1000);
    global::matchLoaderP.set_value(false);

    chassis.turn_abs(-43, 900);
    delay(100);
    chassis.straight(-24, 800, 0.70);
    chassis.move_intake(-127);
    delay(25);    
    global::set_intake(global::intakeState::MID_GOAL);
    chassis.move_intake(127);
    delay(1000);
    chassis.turn_abs(-41, 300);
    chassis.straight(32, 1000, 0.65);
    delay(300);
    chassis.turn_abs(-90, 800, 0.95);
    chassis.move_intake(0);

    global::wingPiston.set_value(false);
    queue.schedule_delay(500, []() {
        chassis.set_state(DriveState::CHASSIS_8);
    });
    chassis.set_brake_mode(MOTOR_BRAKE_HOLD);

    chassis.straight(-25, 2500, 0.55);

    // queue.start();
    // odom.reset(0, 0, 0);
    // chassis.straight(31, 1200, 0.6);
    // delay(50);
    // chassis.turn_abs(-90, 600);
    // global::matchLoaderP.set_value(true);
    // delay(200);
    // chassis.move_intake(127);
    // chassis.straight(22, 900, 0.35);
    // delay(200);
    // chassis.turn_abs(-89, 600);

    // queue.schedule_delay(300, []() {
    //     chassis.move_intake(0);
    // });
    // chassis.straight(-31, 900, 0.45);
    // chassis.move_intake(-127);
    // chassis.turn_abs(-90, 600);

    // queue.schedule_delay(300, []() {
    //     chassis.straight(-3, 300);
    // });
    // delay(50);
    // global::set_intake(global::intakeState::HIGH_GOAL);
    // chassis.move_intake(127);
    // delay(1350);
    // global::matchLoaderP.set_value(false);
    // chassis.straight(13, 800, 0.65);
    // delay(100);
    // chassis.turn_abs(136, 800);
    // global::set_intake(global::intakeState::STORAGE);
    // delay(200);
    // queue.schedule_delay(700, []() {
    //             global::matchLoaderP.set_value(true);
    // });
    // chassis.straight(28, 900, 0.65);
    // chassis.turn_abs(136, 400);
    // delay(1000);
    // global::matchLoaderP.set_value(false);

    // chassis.turn_abs(-43, 900);
    // delay(100);
    // chassis.straight(-17, 900, 0.60);
    // chassis.turn_abs(-41, 200);

    // chassis.straight(-6, 400, 0.70);
    // chassis.move_intake(-127);
    // delay(25);    
    // global::set_intake(global::intakeState::MID_GOAL);
    // chassis.move_intake(127);
    // delay(1000);
    // chassis.turn_abs(-41, 300);
    // chassis.straight(31.5, 1000, 0.65);
    // delay(300);
    // chassis.turn_abs(-90, 800, 0.95);
    // chassis.move_intake(0);

    // global::wingPiston.set_value(false);
    // queue.schedule_delay(500, []() {
    //     chassis.set_state(DriveState::CHASSIS_8);
    // });
    // chassis.set_brake_mode(MOTOR_BRAKE_HOLD);

    // chassis.straight(-25, 2500, 0.55);





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
    odom.reset(0, 0, 0);
    chassis.straight(31, 1200, 0.7);
    delay(50);
    chassis.turn_abs(90, 600);
    global::matchLoaderP.set_value(true);
    delay(200);
    chassis.move_intake(127);
    chassis.straight(22.5, 900, 0.35);
    delay(400);
    queue.schedule_delay(300, []() {
        chassis.move_intake(0);
    });
    chassis.straight(-31, 900, 0.55);
    chassis.move_intake(-127);
    queue.schedule_delay(300, []() {
        chassis.straight(-3, 300);
    });
    delay(50);
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.move_intake(127);
    delay(1350);
    chassis.turn_abs(90);
    global::matchLoaderP.set_value(false);
    chassis.straight(15, 800, 0.65);
    delay(100);
    chassis.turn_abs(-136, 800);
    global::set_intake(global::intakeState::STORAGE);
    delay(200);

    chassis.straight(30, 900, 0.65);
    global::matchLoaderP.set_value(true);
    delay(1000);
    global::matchLoaderP.set_value(false);

    //chassis.turn_abs(-142, 900);
    delay(100);
    chassis.straight(17.5, 800, 0.60);
    // chassis.turn_abs(-149, 300);
    chassis.move_intake(-127);
    delay(1000);
    chassis.turn_abs(-136, 300);
    global::matchLoaderP.set_value(false);

    chassis.straight(-31, 1000, 0.65);
    delay(400);
    chassis.turn_abs(-90, 800, 0.95);
    chassis.move_intake(0);

    global::wingPiston.set_value(false);
    queue.schedule_delay(500, []() {
        chassis.set_state(DriveState::CHASSIS_8);
    });
    chassis.set_brake_mode(MOTOR_BRAKE_HOLD);

    chassis.straight(25, 2500, 0.55);




}

void skills_auton(){
    queue.start();
    odom.reset(-3, 0, 0);
    global::wingPiston.set_value(true);
    chassis.purePursuit(lynx::path({
        {0, 0, 0, 65},
        {7, 19, -60, 45},
        {-2, 29.5, -90, 50}
    }, 0.4, 0.5), 3500);
    global::matchLoaderP.set_value(true);
    delay(400);
    chassis.move_intake(127);
    chassis.straight(16, 900, 0.35);
    delay(2500);

    chassis.straight(-5, 900, 0.95);
    global::matchLoaderP.set_value(false);

    chassis.turn_abs(15);
    chassis.move_intake(0);

    chassis.purePursuit(lynx::path({
        {-13, 39, 90, 45},
        {3, 40, 70, 35},
        {20, 42.5, 70, 45},
        {84, 44.5, 70, 45},
        {85, 36, 120, 50},
        {86, 31, 140, 50},
        {89, 27, 0, 50}
    }), 5000);
    chassis.turn_abs(0, 1000);
    delay(100);
    chassis.straight(35, 1000, 0.55);
    chassis.turn_abs(0, 600);
    chassis.straight(5, 900, 0.55);
    chassis.move_intake(0);

    chassis.straight(-12.75, 800, 0.65);
    chassis.turn_abs(90, 800);
    global::set_intake(global::intakeState::HIGH_GOAL);

    chassis.straight(-22.5, 900, 0.70);
    delay(100);
    chassis.turn_abs(90, 800, 3);
    chassis.straight(-3, 900, 1);


    delay(50);
    chassis.move_intake(127);

    delay(2000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(90, 600);
    delay(10);
    chassis.straight(35.5, 1000, 0.375);
    global::set_intake(global::intakeState::STORAGE);
    delay(100);
    chassis.turn_abs(90, 600);

    chassis.straight(5, 800, 0.75);
    delay(2000);


    chassis.turn_abs(93, 300);
    chassis.straight(-28, 1000, 0.65);
    chassis.move_intake(-127);
    delay(25);
    
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.move_intake(127);
    chassis.turn_abs(90, 300);

    delay(2000);
    chassis.straight(12, 900, 0.65);
    chassis.turn_abs(178.25, 600);
    global::matchLoaderP.set_value(false);
    // chassis.purePursuit(lynx::path({
    //     {93, 27.65, 190, 50},
    //     {89, 0, 190, 55},
    //     {88, -45, 180, 55},
    //     {92, -63, 180, 55},
    // }, 0.2, 0.1), 6000);

    chassis.straight(96, 3000, 0.45);
    global::set_intake(global::intakeState::STORAGE);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(90, 900, 0.75);
    chassis.move_intake(127);
    chassis.straight(25, 900, 0.40);
    delay(100);
    chassis.straight(5, 800, 0.75);
    delay(2500);

    chassis.straight(-5, 900, 0.95);
    global::matchLoaderP.set_value(false);
    global::wingPiston.set_value(true);

    chassis.turn_abs(-180);
    chassis.move_intake(0);


    chassis.straight(18, 800, 0.55);
    chassis.straight(-1.5, 700);
    chassis.turn_abs(-90, 1500);
    chassis.straight(92, 3000, 0.55);
    chassis.turn_abs(180, 800);
    chassis.straight(11, 900, 0.65);
    global::set_intake(global::intakeState::HIGH_GOAL);
    chassis.straight(5, 900, 0.55);
    chassis.move_intake(0);

    chassis.straight(-12.75, 800, 0.65);
    chassis.turn_abs(-90, 800);
    global::set_intake(global::intakeState::HIGH_GOAL);

    chassis.straight(-22.5, 900, 0.70);
    delay(100);
    chassis.turn_abs(-90, 800, 3);
    chassis.straight(-3, 900, 1);


    delay(50);
    chassis.move_intake(127);


    


    // chassis.purePursuit(lynx::path({
    //     {90, -140, -90, 55},
    //     {70, -137, -90, 50},
    //     {20, -90, -90, 50},
    //     {10, -80.5, -s40, 65},
    //     {0, -70.5, -90, 45}
    // }));

    delay(2000);
    global::matchLoaderP.set_value(true);
    chassis.turn_abs(-91, 400);
    queue.schedule_delay(500, [](){
           global::set_intake(global::intakeState::STORAGE);

    });

    chassis.straight(28, 1000, 0.45);
    delay(50);
    chassis.straight(5, 800, 0.65);

    delay(2500);

    chassis.turn_abs(-87.5, 800);
    chassis.straight(-27, 1000, 0.45);
    global::set_intake(global::intakeState::HIGH_GOAL);
    delay(2000);
    chassis.straight(5, 800, 0.65);

    global::matchLoaderP.set_value(false);

    // queue.schedule_delay(900, [](){
    //     global::matchLoaderP.set_value(true);
    // });
    // chassis.purePursuit(lynx::path({
    //     {0, -60.5, -90.0, 127},
    //     {-9.3, -45.4, 0.0, 127},
    //     {-16.3, -10.6, 0.0, 127}
    // }, 0.2, 0.5), 5000);

    chassis.straight(10, 900, 0.75);
    chassis.turn_abs(-45, 800);
    queue.schedule_delay(200, [](){
        chassis.set_state(DriveState::CHASSIS_8);
    });
    chassis.straight(24, 1000, 0.55);
    chassis.turn_abs(-2, 800);
    queue.schedule_delay(800, [](){
        global::matchLoaderP.set_value(true);
    });
    chassis.straight(27, 2000);
    delay(1000);
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
    soloAwp, autonHalfRBlue, autonHalfLBlue, skills, autonHalfLRed, 
    autonHalfRRed, 
    noAuto
}; 

