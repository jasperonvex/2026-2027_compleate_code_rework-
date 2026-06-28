#include "main.h"
#include "robot_setup.hpp"
#include "mainFunctionsH/tracking.hpp"
#include "mainFunctionsH/functions.hpp"

void initialize() {
	pros::lcd::initialize();

	
}



void disabled() {}


void competition_initialize() {}


void autonomous() {}


void opcontrol() {

	inert.reset();
	pros::delay(500);
		
	track.cameraTrack.Start_ATLA();



	while (true) {
		track.RoboPosition.a = function.DegToRad(inert.get_heading());

		pros::lcd::set_text(1,"testing camera tracking");
		pros::lcd::set_text(2,"robot X: " + std::to_string(track.RoboPosition.x));
		pros::lcd::set_text(3,"robot Y: " + std::to_string(track.RoboPosition.y));
		pros::lcd::set_text(4,"robot A: " + std::to_string(track.RoboPosition.a));


		pros::delay(20);                               
	}
}