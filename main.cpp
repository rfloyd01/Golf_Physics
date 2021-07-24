#include <iostream>
#include <cmath>
#include "Physics.h"

int main()
{
	//Input info on the club being used
	Golf_Club seven_iron;
	seven_iron.mass = .272; //weight of the club head in kg
	seven_iron.loft = 33; //the loft of the club in degrees
	seven_iron.clubhead_length = 0.064; //length of the clubhead in meters
	seven_iron.clubhead_height = 0.054; //height of the clubhead in meters
	seven_iron.clubhead_width = 0.011; //thickness of the clubhead in meters
	seven_iron.clubhead_cm_height = 0.25; //ratio of the height of the clubs center of mass vs. the total height of the club
	seven_iron.Moment_of_Inertia = { 0.00256167, 0.00256167, 0.00256167 };

	//Input info on the ball being used
	Golf_Ball ball;
	ball.mass = 0.0459; //the mass of the golfball in kilograms
	ball.radius = 0.02135; //radius of the ball in meters
	ball.Moment_of_Inertia = 2.0 / 5.0 * ball.mass * ball.radius * ball.radius; //Moment of Inertia about CM of golf ball (same as for a standard sphere)

	//Input info on the aspect of the swing
	Golf_Swing swing;
	swing.swing_speed = 40.0; //The velocity of the clubhead at impact with the golfball
	swing.angle_of_attack = 7; //The angle at which the club head moves into the ball relative to the ground. A positive value means the club is moving downwards and a negative value means it's moving upwards
	swing.target_line_angle = 0.0; //The angle at which the clubhead is approaching the target line (positive angle, towards right of target )or moving away from the target line (negative angle, to the left of target). A value of 0 means the club head is traveling parallel to target line at impact
	swing.clubhead_dynamic_loft = 5.5; //how many degrees delofted (positive value) or lofted (negative value) the clubhead is at impact from it's standard loft. A value of zero means the club hits the ball with it's standard loft.
	swing.clubhead_openness = 0.0; //how many degrees open (positive value) or closed (negative value) the clubhead is at impact from the original target line. A value of zero means that clubhead is aimed right at the target.
	swing.clubhead_tilt = 0.0; //how many degrees towards the heel (positive value) or towards the toe (negative value) the clubhead is tilted towards the ground at impact with the ball. A value of zero means the bottom of the club head is flush with the ground.
	swing.contact_height = -.25 * seven_iron.clubhead_height; //how high up on the clubface contact is made with the ball. Values are limited to -clubface_height/2 and +clubface_height/2
	swing.contact_length = 0; //how far off center towards the toe (positive number) or heel (negative number) that the ball strikes the clubface. Input limited to -clubface_length/2 and clubface_length/2

	//Calculate velocity, direction and spin of golf ball based on impact form above characteristics
	std::pair<Vector_3d, Vector_3d> ball_flight_characteristics = getBallVelocities(seven_iron, ball, swing);

	std::cout << "Final Ball Velocity [X, Y, Z] = {" << ball_flight_characteristics.first.x << ", " << ball_flight_characteristics.first.y << ", " << ball_flight_characteristics.first.z << "}" << std::endl;
	std::cout << "Final Ball Angular Velocity [X, Y, Z] = {" << ball_flight_characteristics.second.x << ", " << ball_flight_characteristics.second.y << ", " << ball_flight_characteristics.second.z << "}" << std::endl << std::endl;
	
	//Now that we have the initial velocity and spin of the golfball we can figure out the ultimate trajectory for the ball

	return 1;
} 
