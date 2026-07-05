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

	
	

	while(true){
		 rightMg.move(master.get_analog(ANALOG_LEFT_Y));
		 leftMg.move(master.get_analog(ANALOG_RIGHT_Y));


		 pros::delay(20);
	}
		
	/*

	inert.reset();
	pros::delay(2000);
	track.RoboPosition.x = -48;
	track.RoboPosition.y = 48;
	track.cameraTrack.Start_ATLA();



	while (true) {
		track.RoboPosition.a = function.roundNearistThous(function.DegToRad(inert.get_heading()));

		pros::lcd::set_text(0,track.cameraTrack.camerastatus);
		pros::lcd::set_text(2,"robot X: " + std::to_string(track.RoboPosition.x));
		pros::lcd::set_text(3,"robot Y: " + std::to_string(track.RoboPosition.y));
		pros::lcd::set_text(4,"robot A: " + std::to_string(function.RadToDeg(track.RoboPosition.a)));
		
		//pros::lcd::set_text(4, "tag position x" + std::to_string(track.cameraTrack.curentAprilTag.TagPosition.x));
		//pros::lcd::set_text(5, "tag position y: " + std::to_string(track.cameraTrack.curentAprilTag.TagPosition.y));

		pros::lcd::set_text(6, "offset x: " + std::to_string(track.cameraTrack.CamXOffset));
		pros::lcd::set_text(7, "offset y:  " + std::to_string(track.cameraTrack.CamYoffset));

		pros::delay(100);                               
	}
*/
	
}