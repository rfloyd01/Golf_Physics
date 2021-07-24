#include "Physics.h"
#include <iostream>

std::pair<Vector_3d, Vector_3d> getBallVelocities(Golf_Club& club, Golf_Ball& ball, Golf_Swing& swing)
{
	//This function takes club properties, ball properties and swing properties as input and returns two Vector_3d types
	//The first contains the velocity of the golf ball after impact and the second contains the angular velocity of the ball after impact

	//The +x axis is defined as from the golf ball to the intended targer
	//The +y axis is defined as from the golfer to the golf ball
	//The +z axis is defined as from the golf ball to the sky

	//convert any value given in degrees to radians
	double pi = 3.14159;
	double loft = club.loft * pi / 180.0;
	double dynamic_loft = swing.clubhead_dynamic_loft * pi / 180.0;
	double tilt = swing.clubhead_tilt * pi / 180.0;
	double openness = swing.clubhead_openness * pi / 180.0;
	double angle_of_attack = swing.angle_of_attack * pi / 180.0;
	double target_angle = swing.target_line_angle * pi / 180.0;


	//for now, assume that the contact point chosen on the clubface will be the origin and that the clubface doesn't have any loft or tilt applied to it yet and is just a rectangle sitting flat on the ground
	//unit vectors describing the direction of the clubface normal, clubface tangent (side to side of face) and clubface parallel (top to bottom) directions will be defined as:
	Vector_3d clubface_normal = { cos(loft), 0, sin(loft) };
	Vector_3d clubface_tangent = { 0, 1, 0 };
	Vector_3d clubface_parallel = { -sin(loft), 0, cos(loft) };

	//define rotation quaternion that will translate the clubface normal, tangent and parallel vectors from initial orientation to orientation with dynamic loft, openess and tilt applied to clubhead
	double q0 = cos(tilt / 2.0) * cos(dynamic_loft / 2.0) * cos(openness / 2.0) + sin(tilt / 2.0) * sin(dynamic_loft / 2.0) * sin(openness / 2.0);
	double q1 = sin(tilt / 2.0) * cos(dynamic_loft / 2.0) * cos(openness / 2.0) - cos(tilt / 2.0) * sin(dynamic_loft / 2.0) * sin(openness / 2.0);
	double q2 = cos(tilt / 2.0) * sin(dynamic_loft / 2.0) * cos(openness / 2.0) + sin(tilt / 2.0) * cos(dynamic_loft / 2.0) * sin(openness / 2.0);
	double q3 = cos(tilt / 2.0) * cos(dynamic_loft / 2.0) * sin(openness / 2.0) - sin(tilt / 2.0) * sin(dynamic_loft / 2.0) * cos(openness / 2.0);
	glm::dquat rotation_quaternion = { q0, q1, q2, q3 };

	//rotate the clubface normal, parallel and tangent vectors so that they now point in the correct x, y and z directions
	QuatRotate(rotation_quaternion, clubface_normal);
	QuatRotate(rotation_quaternion, clubface_tangent);
	QuatRotate(rotation_quaternion, clubface_parallel);

	//Create a vector describing the initial velocity of the clubhead in [X, Y, Z] coordinates
	Vector_3d initial_clubhead_velocity = { swing.swing_speed * cos(angle_of_attack) * cos(target_angle) , swing.swing_speed * cos(angle_of_attack) * sin(target_angle) , -swing.swing_speed * sin(angle_of_attack) };

	//Project the [X, Y, Z] initial velocity vector onto the rotated normal, parallel and tangent vectors of the club face. This gives the [X, Y, Z] components of the velocity in [N, P, T] space
	Vector_3d clubface_normal_velocity = VectorProjection(initial_clubhead_velocity, clubface_normal); //the projection of the clubhead velocity onto the clubface normal vector
	Vector_3d clubface_parallel_velocity = VectorProjection(initial_clubhead_velocity, clubface_parallel); //the projection of the clubhead velocity onto the clubface parallel vector
	Vector_3d clubface_tangent_velocity = VectorProjection(initial_clubhead_velocity, clubface_tangent); //the projection of the clubhead velocity onto the clubface tangent vector
	Vector_3d Vci; //This vector represents the initial velocity of the clubhead in [N, P, T] coordinates

	//Check whether the components of the initial velocity vector are pointing in the positive or negative N, P and T directions
	//Need to check if the magnitude of any component of the velocity is 0 to avoid division by 0.
	if (VectorMagnitude(clubface_normal_velocity) != 0)
		Vci.x = VectorMagnitude(clubface_normal_velocity) * DotProduct(clubface_normal_velocity, clubface_normal) / VectorMagnitude(clubface_normal_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	if (VectorMagnitude(clubface_parallel_velocity) != 0)
		Vci.y = VectorMagnitude(clubface_parallel_velocity) * DotProduct(clubface_parallel_velocity, clubface_parallel) / VectorMagnitude(clubface_parallel_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	if (VectorMagnitude(clubface_tangent_velocity) != 0)
		Vci.z = VectorMagnitude(clubface_tangent_velocity) * DotProduct(clubface_tangent_velocity, clubface_tangent) / VectorMagnitude(clubface_tangent_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	//Calculate the coefficient of restitution for the collision based on the initial velocity of the clubhead along the clubface normal vector. This equation for e is based on research found online
	//TODO: The research I read for the calculation of e was only based on driver swings, need to do some more research to see if it differs by club at all
	double coefficient_of_restitution;
	coefficient_of_restitution = 0.86 - 0.0029 * Vci.x; //this value taken from a research paper, the less directly the clubface impacts the ball the less the ball will compress so more energy is conserved

	//Since ball contact is always made on the vector from the clubface normal to the center of the ball, the normal component of the r vector is rb and the tangent and parallel components are 0
	//Likewise, no matter how the clubface is rotated in the cartesian frame the contact point on the club face will never move away from the center of mass in the normal, parallel and tangent from
	///i.e. the normal component from contact point to center of mass will always be -clubhead_width / 2, along the parallel will be - impact_height and along the tangent will always be -impact_length.
	Vector_3d r = { -ball.radius, 0, 0 }; //these are [N, P, T] coordinates
	Vector_3d R = { club.clubhead_width / 2.0, swing.contact_height + club.clubhead_height * club.clubhead_cm_height, swing.contact_length };

	//We currently have 15 unknown variables to solve for and 15 different equations of motion that govern the impact so the values can be obtained using a linear equation module named Eigen
	//The unknown variables are: Pn, Pp, Pt, Vcfn, Vcfp, Vcft, Vbfn, Vbfp, Vbft, Wcfn, Wcfp, Wcft, Wbfn, Wbfp and Wbft where P is Impulse, V is velocity and W is angular velocity (c and b mean club and ball)
	Eigen::MatrixXf Values(15, 15);
	Eigen::MatrixXf Answers(15, 1);

	Values <<
		-1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, -1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0,

		1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0,

		0, R.z, -R.y, 0, -R.z * club.mass, R.y * club.mass, 0, 0, 0, club.Moment_of_Inertia.x, 0, 0, 0, 0, 0,
		-R.z, 0, R.x, R.z * club.mass, 0, -R.x * club.mass, 0, 0, 0, 0, club.Moment_of_Inertia.y, 0, 0, 0, 0,
		R.y, -R.x, 0, -R.y * club.mass, R.x* club.mass, 0, 0, 0, 0, 0, 0, club.Moment_of_Inertia.z, 0, 0, 0,

		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ball.Moment_of_Inertia, 0, 0,
		0, 0, -r.x, 0, 0, 0, 0, 0, r.x* ball.mass, 0, 0, 0, 0, ball.Moment_of_Inertia, 0,
		0, r.x, 0, 0, 0, 0, 0, -r.x * ball.mass, 0, 0, 0, 0, 0, 0, ball.Moment_of_Inertia,

		0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -R.z, R.y, 0, 0, 0,
		0, 0, 0, 0, -1, 0, 0, 1, 0, R.z, 0, -R.x, 0, 0, r.x,
		0, 0, 0, 0, 0, -1, 0, 0, 1, -R.y, R.x, 0, 0, -r.x, 0;

	Answers <<
		club.mass * Vci.x, club.mass * Vci.y, club.mass * Vci.z,
		R.y * club.mass * Vci.z - R.z * club.mass * Vci.y,
		R.z * club.mass * Vci.x - R.x * club.mass * Vci.z,
		R.x * club.mass * Vci.y - R.y * club.mass * Vci.x,
		0, 0, 0, 0, 0, 0, coefficient_of_restitution* Vci.x, 0, 0;

	Eigen::MatrixXf Unknowns = Values.colPivHouseholderQr().solve(Answers);

	//The unknown collision variables have now been acquired in [N, P, T] coordinates
	Vector_3d Impulse = { Unknowns(0), Unknowns(1), Unknowns(2) };
	Vector_3d Vfc = { Unknowns(3), Unknowns(4), Unknowns(5) };
	Vector_3d Vfb = { Unknowns(6), Unknowns(7), Unknowns(8) };
	Vector_3d Wfc = { Unknowns(9), Unknowns(10), Unknowns(11) };
	Vector_3d Wfb = { Unknowns(12), Unknowns(13), Unknowns(14) };

	//Convert Ball linear and angular velocity back into [X, Y, Z] coordinates
	Vector_3d Ball_Velocity, Ball_Spin;

	Ball_Velocity.x = clubface_normal.x * Vfb.x + clubface_parallel.x * Vfb.y + clubface_tangent.x * Vfb.z;
	Ball_Velocity.y = clubface_normal.y * Vfb.x + clubface_parallel.y * Vfb.y + clubface_tangent.y * Vfb.z;
	Ball_Velocity.z = clubface_normal.z * Vfb.x + clubface_parallel.z * Vfb.y + clubface_tangent.z * Vfb.z;

	Ball_Spin.x = (clubface_normal.x * Wfb.x + clubface_parallel.x * Wfb.y + clubface_tangent.x * Wfb.z) / (2.0 * pi) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.y = (clubface_normal.y * Wfb.x + clubface_parallel.y * Wfb.y + clubface_tangent.y * Wfb.z) / (2.0 * pi) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.z = (clubface_normal.z * Wfb.x + clubface_parallel.z * Wfb.y + clubface_tangent.z * Wfb.z) / (2.0 * pi) * 60; //convert spin rate from rad/s to RPM

	return { Ball_Velocity, Ball_Spin };
}
