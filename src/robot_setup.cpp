#include "robot_setup.hpp"

//change these as needed 

//drivetrain
 pros::MotorGroup rightMg({1,2,3});
 pros::MotorGroup leftMg({4,5,6});

//odomertry
 pros::Rotation verticalWheel(7);
 pros::Rotation horizontaleWheel(8);
 pros::Imu inert(9);

//distence tracking

 pros::Distance Fl(10);
 pros::Distance Fr(11);
 pros::Distance R(12);
 pros::Distance L(13);

//add anything else down here
