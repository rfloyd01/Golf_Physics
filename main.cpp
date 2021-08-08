#include <iostream>
#include <cmath>
#include "Physics.h"
#include "gnuplot.h"

#define ms_to_mph 2.23694

int main()
{
	//Generate a bag full of clubs to use
	std::map<std::string, Golf_Club> Golf_Bag = generateClubs();
	
	//Select the club to use by changing the value of the gc string
	//Iron names are of the form #i, so a 4 iron would be "4i", wedges are the first letter of the wedge followed by a w (sand wedge = "sw", pitching wedge = "pw", etc.)
	//Wood names are the number of the wood followed by a w (driver = "1w", 5 wood = "5w"). If the selected club isn't in the bag then a 7 iron will be automatically chosen
	std::string club_name = "7i";
	Golf_Club club = Golf_Bag[club_name];

	if (club.loft == 0)
	{
		std::cout << "Sorry, the " << club_name << " isn't in the bag, setting club to a 7i." << std::endl << std::endl;
		club = Golf_Bag["7i"];
	}

	//Input info on the ball being used
	Golf_Ball ball;
	ball.mass = 0.0459; //the mass of the golfball in kilograms
	ball.radius = 0.02135; //radius of the ball in meters
	ball.Moment_of_Inertia = 2.0 / 5.0 * ball.mass * ball.radius * ball.radius; //Moment of Inertia about CM of golf ball (same as for a standard sphere)

	//Here's a link with average stats for PGA pros on a per club basis https://blog.trackmangolf.com/trackman-average-tour-stats/

	//Input info on the aspect of the swing
	Golf_Swing swing;
	swing.swing_speed = 50.5 * club.speed_modifier; //The velocity of the clubhead at impact with the golfball in m/s (conversion from mph to m/s is mph * 0.44704 = m/s)
	swing.angle_of_attack = 2.3; //The angle at which the club head moves into the ball relative to the ground. A positive value means the club is moving downwards and a negative value means it's moving upwards
	swing.target_line_angle = 0.0; //The angle at which the clubhead is approaching the target line (positive angle, towards right of target )or moving away from the target line (negative angle, to the left of target). A value of 0 means the club head is traveling parallel to target line at impact
	swing.clubhead_dynamic_loft = 7.0; //how many degrees delofted (positive value) or lofted (negative value) the clubhead is at impact from it's standard loft. A value of zero means the club hits the ball with it's standard loft.
	swing.clubhead_openness = 0.0; //how many degrees open (positive value) or closed (negative value) the clubhead is at impact from the original target line. A value of zero means that clubhead is aimed right at the target.
	swing.clubhead_tilt = 0.0; //how many degrees towards the heel (positive value) or towards the toe (negative value) the clubhead is tilted towards the ground at impact with the ball. A value of zero means the bottom of the club head is flush with the ground.
	swing.contact_height = club.clubhead_height / 2.0 - club.clubhead_height / 4.0; //how high up on the clubface contact is made with the ball. Values are limited to -clubface_height/2 and +clubface_height/2
	swing.contact_length = 0; //how far off center towards the toe (positive number) or heel (negative number) that the ball strikes the clubface. Input limited to -clubface_length/2 and clubface_length/2

	//Calculate velocity, direction and spin of golf ball based on impact form above characteristics
	std::pair<Vector_3d, Vector_3d> ball_flight_characteristics = getBallVelocities(club, ball, swing);

	std::cout << "Final Ball Velocity [X, Y, Z] (m/s) = {" << ball_flight_characteristics.first.x << ", " << ball_flight_characteristics.first.y << ", " << ball_flight_characteristics.first.z << "}" << std::endl;
	std::cout << "Final Ball Speed = " << VectorMagnitude(ball_flight_characteristics.first) * ms_to_mph << " mph" << std::endl << std::endl;
	//std::cout << "Smash Factor (ball speed / clubhead speed) = " << VectorMagnitude(ball_flight_characteristics.first) / swing.swing_speed << std::endl << std::endl;
	std::cout << "Final Ball Angular Velocity [X, Y, Z] = {" << ball_flight_characteristics.second.x << ", " << ball_flight_characteristics.second.y << ", " << ball_flight_characteristics.second.z << "}" << std::endl;
	std::cout << "Final Angular Speed = " << VectorMagnitude(ball_flight_characteristics.second) << " RPM" << std::endl << std::endl;

	//Now that we have the initial velocity and spin of the golfball we can figure out the ultimate trajectory for the ball
	calculateBallFlight(ball_flight_characteristics, ball, 0.001);
	graphBallFlight();

	return 1;
} 
