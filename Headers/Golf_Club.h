#pragma once
#include "Math.h"

struct Golf_Club
{
	double mass; //weight of the club head in kg
	double loft; //the loft of the club in degrees

	double clubhead_length; //length of the clubhead in meters
	double clubhead_height; //height of the clubhead in meters
	double clubhead_width; //thickness of the clubhead in meters
	double clubhead_cm_height; //ratio of the height of the clubs center of mass vs. the total height of the club

	//The moments of inertia for the clubhead about 3 axes (all passing through the CM of the club).
	//First axis is through the normal of the clubface, second axis runs straight up the clubface and third axis runs along the length of the clubface
	//All three of these axes are mutually perpendicular to each other
	Vector_3d Moment_of_Inertia; 
};

struct Golf_Swing
{
	//velocity variables
	double swing_speed; //The velocity of the clubhead at impact with the golfball
	double angle_of_attack; //The angle at which the club head moves into the ball relative to the ground. A positive value means the club is moving downwards and a negative value means it's moving upwards
	double target_line_angle; //The angle at which the clubhead is approaching the target line (positive angle, towards right of target )or moving away from the target line (negative angle, to the left of target). A value of 0 means the club head is traveling parallel to target line at impact

	//clubface angles
	double clubhead_dynamic_loft; //how many degrees delofted (positive value) or lofted (negative value) the clubhead is at impact from it's standard loft. A value of zero means the club hits the ball with it's standard loft.
	double clubhead_openness; //how many degrees open (positive value) or closed (negative value) the clubhead is at impact from the original target line. A value of zero means that clubhead is aimed right at the target.
	double clubhead_tilt; //how many degrees towards the heel (positive value) or towards the toe (negative value) the clubhead is tilted towards the ground at impact with the ball. A value of zero means the bottom of the club head is flush with the ground.

	//contact variables
	double contact_height; //how high up on the clubface contact is made with the ball. Values are limited to -clubface_height/2 and +clubface_height/2
	double contact_length; //how far off center of the clubface that contact is made with the ball. Values are limited to -clubface_length/2 and +clubface_length/2
};

struct Golf_Ball
{
	double mass; //the weight of the golfball in kg
	double radius; //the radius of the golfball in m
	double Moment_of_Inertia; //the moment of inertia of the golfball in kg/m^2. This value is the same about any axis
};
