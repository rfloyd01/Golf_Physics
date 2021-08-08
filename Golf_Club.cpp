#include "Golf_Club.h"

std::map<std::string, Golf_Club> generateClubs()
{
	//this function just generates a pre-made set of clubs for use in the main function
	//info on standard clubhead weights http://clubmaker-online.com/swingweight.factors.html
	//info on standard clubhead lofts http://www.leaderboard.com/loftinfo.htm

	std::map<std::string, Golf_Club> clubs;

	//make a golf club variable and load up info on a standard 7 iron
	Golf_Club club;
	club.mass = .272; //weight of the club head in kg
	club.loft = 30; //the loft of the club in degrees
	club.clubhead_length = 0.064; //length of the clubhead in meters
	club.clubhead_height = 0.054; //height of the clubhead in meters
	club.clubhead_width = 0.011; //thickness of the clubhead in meters
	club.clubhead_cm_height = 0.25; //ratio of the height of the clubs center of mass vs. the total height of the club
	club.Moment_of_Inertia = { 0.00256167, 0.00256167, 0.00256167 }; //the moment of inertia about the N, P and T axes in kg/m^2
	club.speed_modifier = .796; //ratio of clubhead speed for a 7 iron vs. a driver with the same hand speed (this happens because the driver is longer)
	clubs["7i"] = club; //load club to bag

	//load info on a standard pitching wedge
	club.mass = .293;
	club.loft = 45;
	club.speed_modifier = .735;
	clubs["pw"] = club; //load club to bag

	//load info on a standard sand wedge
	club.mass = .3;
	club.loft = 56;
	club.speed_modifier = .717;
	clubs["sw"] = club; //load club to bag

	//load info for a driver
	club.mass = .2;
	club.loft = 9;
	club.clubhead_cm_height = 0.5; //a driver's cm will be closer to the middle of the clubhead
	club.Moment_of_Inertia = { 0.00456167, 0.00456167, 0.00456167 }; //driver's have higher moment's of inertia
	club.speed_modifier = 1.0;
	clubs["1w"] = club; //load club to bag

	return clubs;
}
