#include "robot_setup.hpp"

//change these as needed 

pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);


//drivetrain
 pros::MotorGroup rightMg({-1,-2,-3});
 pros::MotorGroup leftMg({8,9,10});

//odomertry
 pros::Rotation verticalWheel(6);
 pros::Rotation horizontaleWheel(-5);
 pros::Imu inert(7);

//distence tracking

 pros::Distance Fl(0);
 pros::Distance Fr(0);
 pros::Distance R(0);
 pros::Distance L(0);

//ai vision sensor
pros::AIVision vis(4);

//add anything else down here

pros::Motor contentiouslift(0);