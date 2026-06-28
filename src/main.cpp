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
	pros::delay(2000);
	track.RoboPosition.x = -48;
	track.RoboPosition.y = 48;
	track.cameraTrack.Start_ATLA();



	while (true) {
		track.RoboPosition.a = function.roundNearistThous(function.DegToRad(inert.get_heading()));

		
		pros::lcd::set_text(1,"robot X: " + std::to_string(track.RoboPosition.x));
		pros::lcd::set_text(2,"robot Y: " + std::to_string(track.RoboPosition.y));
		pros::lcd::set_text(3,"robot A: " + std::to_string(function.RadToDeg(track.RoboPosition.a)));
		
		pros::lcd::set_text(4, "pixel height: " + std::to_string(track.cameraTrack.pixelHieght));
		pros::lcd::set_text(5, "pexel width: " + std::to_string(track.cameraTrack.pixleWidth));

		pros::lcd::set_text(6, "pixleX" + std::to_string(track.cameraTrack.TagPixlePosition[0]));
		pros::lcd::set_text(7, "pixleY" + std::to_string(track.cameraTrack.TagPixlePosition[1]));

		pros::delay(100);                               
	}
}