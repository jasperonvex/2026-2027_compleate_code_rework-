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

	/*
	pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);
	pros::Motor lift(10);

	bool isbuttonpressed = true;
	while (true){
		pros::delay(10);

		if(master.get_digital(DIGITAL_R1)){
			lift.move_voltage(12000);
			isbuttonpressed = true;
		}
		if(master.get_digital(DIGITAL_R2)){
			lift.move_voltage(-12000);
			isbuttonpressed = true;
		}

		if(isbuttonpressed == false){
			lift.move_voltage(0);
		}

		isbuttonpressed = false;

		
		pros::delay(10);
		
	}
		*/
	

	inert.reset();
	pros::delay(2000);
	track.RoboPosition.x = -48;
	track.RoboPosition.y = 48;
	track.cameraTrack.Start_ATLA();



	while (true) {
		track.RoboPosition.a = function.roundNearistThous(function.DegToRad(inert.get_heading()));

		pros::lcd::set_text(0,"pixel Width: " + std::to_string(track.cameraTrack.pixleWidth));
		pros::lcd::set_text(1,"robot X: " + std::to_string(track.RoboPosition.x));
		pros::lcd::set_text(2,"robot Y: " + std::to_string(track.RoboPosition.y));
		pros::lcd::set_text(3,"robot A: " + std::to_string(function.RadToDeg(track.RoboPosition.a)));
		
		pros::lcd::set_text(4, "tag position x" + std::to_string(track.cameraTrack.curentAprilTag.TagPosition.x));
		pros::lcd::set_text(5, "tag position y: " + std::to_string(track.cameraTrack.curentAprilTag.TagPosition.y));

		pros::lcd::set_text(6, "offset x: " + std::to_string(track.cameraTrack.CamXOffset));
		pros::lcd::set_text(7, "offset y:  " + std::to_string(track.cameraTrack.CamYoffset));

		pros::delay(100);                               
	}

	
}