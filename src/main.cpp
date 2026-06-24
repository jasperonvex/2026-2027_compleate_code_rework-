#include "main.h"
#include "robot_setup.hpp"
#include "mainFunctionsH/functions.hpp"

pros::AIVision vis(5);

void initialize() {
	pros::lcd::initialize();

	
	
	
}



void disabled() {}


void competition_initialize() {}


void autonomous() {}

struct Position {
    double x;
    double y;
};

void opcontrol() {
    vis.reset();
    pros::delay(500);

    vis.enable_detection_types(pros::AivisionModeType::tags);
    vis.set_tag_family(pros::AivisionTagFamily::tag_21H7, true);

	double image_width = 320;
	double imagecenterx = image_width/2;

	
	double horosontle_fov_rad = 74 * (M_PI/180);

	double  imageheight = 240;
	double imagecentery = imageheight/2;
	
	double vertical_fov_rad = 63*(M_PI / 180);

    double focal_lenght_pixles = 157.3605;

    Position tag[] = {{0,0},{-48,24}};

    while (true) {
        auto objects = vis.get_all_objects();

        if (objects.empty()) {
            pros::lcd::set_text(1, "No tag detected");
            pros::lcd::clear_line(2);
            pros::lcd::clear_line(3);
            pros::lcd::clear_line(4);
            pros::lcd::clear_line(5);
            pros::lcd::clear_line(6);
            pros::lcd::clear_line(7);
        } else {
            auto object = objects[0];
            double cx = (
                object.object.tag.x0 +
                object.object.tag.x1 +
                object.object.tag.x2 +
                object.object.tag.x3
            ) / 4.0;

            double cy = (
                object.object.tag.y0 +
                object.object.tag.y1 +
                object.object.tag.y2 +
                object.object.tag.y3
            ) / 4.0;

			double fx = imagecenterx / tan(horosontle_fov_rad/2.0);
			double pixleoffset = cx - imagecenterx;

			double horosontle_bearing = atan(pixleoffset/fx);

			double fy = imagecentery/ tan(vertical_fov_rad/2);
			double pixleoffsety = imagecentery - cy;

			double verticalbearijng = atan(pixleoffsety/fy);

            double topWidth = function.GetDistence(object.object.tag.x0,object.object.tag.y0,object.object.tag.x1,object.object.tag.y1);
            double bottemWidth = function.GetDistence(object.object.tag.x3,object.object.tag.y3,object.object.tag.x2,object.object.tag.y2);

            double pixleWidht = (topWidth + bottemWidth)/2;

           //double rightHieght = function.GetDistence(object.object.tag.x0,object.object.tag.y0,object.object.tag.x3,object.object.tag.y3);
           //double leftHiehgt = function.GetDistence(object.object.tag.x1,object.object.tag.y1,object.object.tag.x2,object.object.tag.y2);
           
           //double picleHieght = (rightHieght + leftHiehgt)/2;

            double distence = (1 * focal_lenght_pixles)/pixleWidht;

            double groundDis = distence * cos(verticalbearijng);

            double x_position = tag[object.id].x - (groundDis * cos(horosontle_bearing));
            double y_position = tag[object.id].y - (groundDis * sin(horosontle_bearing));

            

            pros::lcd::set_text(1, "id: " + std::to_string(object.id));
            pros::lcd::set_text(2, "x0: " + std::to_string(object.object.tag.x0) +
                                   " || y0: " + std::to_string(object.object.tag.y0) +
    								" || x1: " + std::to_string(object.object.tag.x1) +
                                   " || y1: " + std::to_string(object.object.tag.y1));
            pros::lcd::set_text(3, "x2: " + std::to_string(object.object.tag.x2) +
                                   " || y2: " + std::to_string(object.object.tag.y2) +
           							" || x3: " + std::to_string(object.object.tag.x3) +
                                   " || y3: " + std::to_string(object.object.tag.y3));
            pros::lcd::set_text(4, "cx: " + std::to_string(cx) +
                                   " || cy: " + std::to_string(cy));
			//pros::lcd::set_text(5, "horsontledeg: " + std::to_string(horosontle_bearing * (180/M_PI)));
			//pros::lcd::set_text(6, " verticalbearing: " + std::to_string(verticalbearijng * (180 / M_PI)));
            pros::lcd::set_text(5,"pixleWidht: " + std::to_string(pixleWidht));
            pros::lcd::set_text(6,"x: " + std::to_string(x_position) + " || y: " +std::to_string(y_position));
            pros::lcd::set_text(7,"groundDis: " + std::to_string(groundDis));
        }

		

        pros::delay(100);
    }
}