#include "Physics.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#define PI 3.141592654
#define g 9.80665
#define MtoY 1.09361

std::pair<Vector_3d, Vector_3d> getBallVelocities(Golf_Club& club, Golf_Ball& ball, Golf_Swing& swing)
{
	//This function takes club properties, ball properties and swing properties as input and returns two Vector_3d types
	//The first contains the velocity of the golf ball after impact and the second contains the angular velocity of the ball after impact

	//The +x axis is defined as from the golf ball to the intended targer
	//The +y axis is defined as from the golfer to the golf ball
	//The +z axis is defined as from the golf ball to the sky

	//convert any value given in degrees to radians
	//double pi = 3.14159;
	double loft = club.loft * PI / 180.0;
	double dynamic_loft = swing.clubhead_dynamic_loft * PI / 180.0;
	double tilt = swing.clubhead_tilt * PI / 180.0;
	double openness = swing.clubhead_openness * PI / 180.0;
	double angle_of_attack = swing.angle_of_attack * PI / 180.0;
	double target_angle = swing.target_line_angle * PI / 180.0;

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
	//TODO: The research I read for the calculation of e was only based on driver swings, need to do some more research to see if it differs by club at all. Not getting nearly enough velocity on a sandwedge
	double coefficient_of_restitution;
	coefficient_of_restitution = 0.86 - 0.0029 * Vci.x; //this value taken from a research paper, the less directly the clubface impacts the ball the less the ball will compress so more energy is conserved

	//Since ball contact is always made on the vector from the clubface normal to the center of the ball, the normal component of the r vector is rb and the tangent and parallel components are 0
	//Likewise, no matter how the clubface is rotated in the cartesian frame the contact point on the club face will never move away from the center of mass in the normal, parallel and tangent from
	///i.e. the normal component from contact point to center of mass will always be -clubhead_width / 2, along the parallel will be - impact_height and along the tangent will always be -impact_length.
	Vector_3d r = { -ball.radius, 0, 0 }; //these are [N, P, T] coordinates
	Vector_3d R = { club.clubhead_width / 2.0, swing.contact_height + club.clubhead_height * club.clubhead_cm_height, swing.contact_length };

	//TODO: Altering the contact height isn't currently having an effect on final ball velocity or spin rate which seems wrong, take a look into this at some point

	//We currently have 15 unknown variables to solve for and 15 different equations of motion that govern the impact so the values can be obtained using a linear equation module named Eigen
	//The unknown variables are: Pn, Pp, Pt, Vcfn, Vcfp, Vcft, Vbfn, Vbfp, Vbft, Wcfn, Wcfp, Wcft, Wbfn, Wbfp and Wbft where P is Impulse, V is velocity and W is angular velocity (c and b mean club and ball)
	Eigen::MatrixXf Values(15, 15);
	Eigen::MatrixXf Answers(15, 1);

	//My own equations which include orbital momentum
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
		0, 0, 0,
		R.y * club.mass * Vci.z - R.z * club.mass * Vci.y,
		R.z * club.mass * Vci.x - R.x * club.mass * Vci.z,
		R.x * club.mass * Vci.y - R.y * club.mass * Vci.x,
		0, 0, 0, coefficient_of_restitution * Vci.x, 0, 0;

	//Equations from Research Paper, orbital momentum not included
	/*
	Values <<
		-1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, -1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, club.mass, 0, 0, 0, 0, 0, 0, 0, 0, 0,

		1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, ball.mass, 0, 0, 0, 0, 0, 0,

		0, -R.z, R.y, 0, 0, 0, 0, 0, 0, -club.Moment_of_Inertia.x, 0, 0, 0, 0, 0,
		R.z, 0, -R.x, 0, 0, 0, 0, 0, 0, 0, -club.Moment_of_Inertia.y, 0, 0, 0, 0,
		-R.y, R.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, -club.Moment_of_Inertia.z, 0, 0, 0,

		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -ball.Moment_of_Inertia, 0, 0,
		0, 0, r.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -ball.Moment_of_Inertia, 0,
		0, -r.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -ball.Moment_of_Inertia,

		0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -R.z, R.y, 0, 0, 0,
		0, 0, 0, 0, -1, 0, 0, 1, 0, R.z, 0, -R.x, 0, 0, r.x,
		0, 0, 0, 0, 0, -1, 0, 0, 1, -R.y, R.x, 0, 0, -r.x, 0;

	Answers <<
		club.mass * Vci.x, club.mass * Vci.y, club.mass * Vci.z,
		0, 0, 0, 0, 0, 0, 0, 0, 0, coefficient_of_restitution * Vci.x, 0, 0;
	*/

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

	Ball_Spin.x = (clubface_normal.x * Wfb.x + clubface_parallel.x * Wfb.y + clubface_tangent.x * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.y = (clubface_normal.y * Wfb.x + clubface_parallel.y * Wfb.y + clubface_tangent.y * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.z = (clubface_normal.z * Wfb.x + clubface_parallel.z * Wfb.y + clubface_tangent.z * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM

	//Uncomment this section to see details on the final velocity and spin of the club, useful when debugging
	/*
	Vector_3d Club_Velocity, Club_Spin;
	Club_Velocity.x = clubface_normal.x * Vfc.x + clubface_parallel.x * Vfc.y + clubface_tangent.x * Vfc.z;
	Club_Velocity.y = clubface_normal.y * Vfc.x + clubface_parallel.y * Vfc.y + clubface_tangent.y * Vfc.z;
	Club_Velocity.z = clubface_normal.z * Vfc.x + clubface_parallel.z * Vfc.y + clubface_tangent.z * Vfc.z;

	Club_Spin.x = (clubface_normal.x * Wfc.x + clubface_parallel.x * Wfc.y + clubface_tangent.x * Wfc.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Club_Spin.y = (clubface_normal.y * Wfc.x + clubface_parallel.y * Wfc.y + clubface_tangent.y * Wfc.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Club_Spin.z = (clubface_normal.z * Wfc.x + clubface_parallel.z * Wfc.y + clubface_tangent.z * Wfc.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM

	std::cout << "Final Club Velocity [X, Y, Z] = {" << Club_Velocity.x << ", " << Club_Velocity.y << ", " << Club_Velocity.z << "}" << std::endl;
	std::cout << "Final Club Speed = " << VectorMagnitude(Club_Velocity) << " m/s" << std::endl << std::endl;

	std::cout << "Final Club Angular Velocity [X, Y, Z] = {" << Club_Spin.x << ", " << Club_Spin.y << ", " << Club_Spin.z << "}" << std::endl;
	std::cout << "Final Club Angular Speed = " << VectorMagnitude(Club_Spin) << " RPM" << std::endl << std::endl;
	*/

	return { Ball_Velocity, Ball_Spin };
}

void calculateBallFlight(std::pair<Vector_3d, Vector_3d> bi, Golf_Ball &ball, double delta_t)
{
	//takes the initial velocity and angular velocity of ball after impact in vector form and calculates the trajectory
	//the position data will be written to an external file so it can be graphed at a later point
	//TODO: Currently doesn't take wind into account, add this functionality eventually
	//TODO: Something seems a little off with the acceleration caused from air and magnus, they seem too high, look into this. Can't get decent ball flight without really lowering Cd value

	//Need to convert ball angular velocity from RPM back to rad/s
	bi.second.x *= (2 * PI / 60.0);
	bi.second.y *= (2 * PI / 60.0);
	bi.second.z *= (2 * PI / 60.0);

	double acceleration[3][2], velocity[3][2]; //arrays to hold most recent and current values for acceleration and velocity in x, y, z directions for integration
	Vector_3d S = CrossProduct(bi.first, bi.second);
	Normalize(S); //S is a unit vector representing the direction of the Magnus force

	Vector_3d B = bi.first, W = bi.second;
	Normalize(B); //B is a unit vector representing the current direction of the ball's velocity
	Normalize(W); //W is a unit vector representing the ball's angular velocity, it's assumed that the direction of the angular velocity won't change during flight

	//std::cout << W.x << ", " << W.y << ", " << W.z << std::endl;
	//std::cout << B.x << ", " << B.y << ", " << B.z << std::endl;
	//std::cout << S.x << ", " << S.y << ", " << S.z << std::endl;

	//Info on drag coefficient of golfball found here https://www.scirp.org/journal/paperinformation.aspx?paperid=85529
	//Info on lift coefficient of golfball found here https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiwktq7tv7xAhXzRTABHcKYASkQFjABegQIFRAD&url=https%3A%2F%2Fwww.mdpi.com%2F2504-3900%2F2%2F6%2F238%2Fpdf%23%3A~%3Atext%3DThe%2520coefficient%2520of%2520drag%2520for%2Ccarry%2520distance%2520(18%2520m).&usg=AOvVaw13OhhShGEQ25dm9SYdc8wW
	double rho = 1.205; //density of air in kg/m^3 TODO: wan't to include a weather conditions struct that will incorporate this values as well as wind conditions separately
	double Cd = 0.15; //Drag coefficient for golf ball obtained form experimental values TODO: would like to tie this into the golfball struct at some point as this value should change with dimple condition
	double spin_factor = VectorMagnitude(bi.second) * ball.radius / VectorMagnitude(bi.first);
	double Cl = -3.25 * spin_factor * spin_factor + 1.99 * spin_factor; //Lift coefficient for golf ball obtained form experimental values TODO: would like to tie this into the golfball struct at some point as this value should change with dimple condition
	double ball_area_factor = 0.5 * rho * PI * ball.radius * ball.radius; //define this variable to avoid repeat multiplication
	double angular_decel = 0.0;
	double Reynolds_Number; //Gives an idea of how turbulent the airflow is around the ball, more turbulent air will reduce the amount of drag the ball experiences

	//std::cout << "Coefficient of lift: " << Cl << std::endl;

	//set initial velocity and acceleration values from input velocity of ball
	velocity[0][1] = bi.first.x;
	velocity[1][1] = bi.first.y;
	velocity[2][1] = bi.first.z;

	velocity[0][0] = 0;
	velocity[1][0] = 0;
	velocity[2][0] = 0;

	double magnitude_velocity_squared = velocity[0][1] * velocity[0][1] + velocity[1][1] * velocity[1][1] + velocity[2][1] * velocity[2][1];
	double air_force = ball_area_factor * magnitude_velocity_squared;

	acceleration[0][1] = air_force * (Cl * S.x - Cd * B.x) / ball.mass; //acceleration caused by magnus affect and drag in the x direction
	acceleration[1][1] = air_force * (Cl * S.y - Cd * B.y) / ball.mass; //acceleration caused by magnus affect and drag in the y direction
	acceleration[2][1] = (air_force * (Cl * S.z - Cd * B.z) - ball.mass * g) / ball.mass; //acceleration caused by magnus affect, drag and gravity in the z direction

	//vector to hold position data in x, y and z dimensions
	std::vector<std::vector<double> > position;
	for (int i = 0; i < 3; i++)
	{
		std::vector<double> yo;
		position.push_back(yo);
	}
	//ball starts at the origin and moves along the +x direction
	position[0].push_back(0);
	position[1].push_back(0);
	position[2].push_back(0);

	double flight_time = 0.0;

	//Simulate the flight of the golf ball by stepping forward in increments of delta_t until the z value is less than 0
	while (position[2].back() >= 0)
	{
		//calculate the current Reynold's number based on the ball's velocity
		Reynolds_Number = sqrt(magnitude_velocity_squared) * 2 * ball.radius / 0.00001527; //TODO: this is just down and dirty for now, work in better velocity magnitude, kinematic air  viscocity and ball diameter numbers

		//recalculate the drag coefficient based on the new Reynold's Number
		if (Reynolds_Number <= 7500) Cd = 0.000000000129 * Reynolds_Number * Reynolds_Number - .0000259 * Reynolds_Number + 1.50;
		else Cd = 0.0000000000191 * Reynolds_Number * Reynolds_Number - 0.0000054 * Reynolds_Number + 0.56;

		flight_time += delta_t;
		//std::cout << "Current Ball Location: {" << position[0].back() << ", " << position[1].back() << ", " << position[2].back() << "}" << std::endl;
		//std::cout << "Current Ball Angular Velocity [X, Y, Z] = {" << bi.second.x << ", " << bi.second.y << ", " << bi.second.z << "}" << std::endl;

		//first move the position of the current velocity and acceleration (element 1 of the arrays) to the position of past velocity and acceleration (element 0)
		velocity[0][0] = velocity[0][1];
		velocity[1][0] = velocity[1][1];
		velocity[2][0] = velocity[2][1];
		acceleration[0][0] = acceleration[0][1];
		acceleration[1][0] = acceleration[1][1];
		acceleration[2][0] = acceleration[2][1];

		//next calculate the angular deceleration of the ball based on the current angular and linear velocity over time interval delta_t
		angular_decel = -0.00002 / ball.radius * sqrt(magnitude_velocity_squared) * VectorMagnitude(bi.second) * delta_t;

		//apply the angular deceleration to the current angular velocity
		bi.second.x += angular_decel * W.x;
		bi.second.y += angular_decel * W.y;
		bi.second.z += angular_decel * W.z;

		//calculate the new direction of S based on new velocity vector obtained in last iteration of loop
		//S = CrossProduct(bi.first, bi.second);
		S = CrossProduct(bi.first, bi.second);
		Normalize(S);

		//std::cout << "t = " << flight_time << ", height = " << position[2].back() << std::endl;
		//std::cout << S.x << ", " << S.y << ", " << S.z << " current x = " << position[0].back() << std::endl << std::endl;

		//calculate new accceleration values
		acceleration[0][1] = air_force * (Cl * S.x - Cd * B.x) / ball.mass; //acceleration caused by magnus affect and drag in the x direction
		acceleration[1][1] = air_force * (Cl * S.y - Cd * B.y) / ball.mass; //acceleration caused by magnus affect and drag in the y direction
		acceleration[2][1] = (air_force * (Cl * S.z - Cd * B.z) - ball.mass * g) / ball.mass; //acceleration caused by magnus affect, drag and gravity in the z direction
		//std::cout << acceleration[0][1] << ", " << acceleration[1][1] << ", " << acceleration[2][1] << ", " << flight_time << std::endl;

		//std::cout << "X Accelerations: " << air_force * Cl * S.x / ball.mass << " N lift force, " << air_force * -Cd * B.x / ball.mass << " N drag force" << std::endl;
		//std::cout << "Z Accelerations: " << air_force * Cl * S.z / ball.mass << " N lift acc., " << air_force * -Cd * B.z / ball.mass << " N drag acc., " << -g << "grav. acc." << std::endl << std::endl;

		//integrate current and previous acceleration values to get current velocity values
		velocity[0][1] = velocity[0][0] + Integrate(acceleration[0][1], acceleration[0][0], delta_t);
		velocity[1][1] = velocity[1][0] + Integrate(acceleration[1][1], acceleration[1][0], delta_t);
		velocity[2][1] = velocity[2][0] + Integrate(acceleration[2][1], acceleration[2][0], delta_t);
		
		//std::cout << flight_time << std::endl;
		//std::cout << velocity[0][0] << ", " << velocity[1][0] << ", " << velocity[2][0] << std::endl;
		//std::cout << velocity[0][1] << ", " << velocity[1][1] << ", " << velocity[2][1] << std::endl << std::endl;

		//integrate current and previous velocity values to get current position values
		position[0].push_back(position[0].back() + Integrate(velocity[0][1], velocity[0][0], delta_t)); //convert meter readings to yards for better golfer readability
		position[1].push_back(position[1].back() + Integrate(velocity[1][1], velocity[1][0], delta_t)); //convert meter readings to yards for better golfer readability
		position[2].push_back(position[2].back() + Integrate(velocity[2][1], velocity[2][0], delta_t)); //convert meter readings to yards for better golfer readability
		
		//std::cout << flight_time << std::endl;
		//std::cout << position[2][position[2].size() - 2] << std::endl;
		//std::cout << position[2][position[2].size() - 1] << std::endl << std::endl;

		//update velocity vector, velocity magnitude squared value, Lift Coefficient and air_force effect based on current velocity
		bi.first.x = velocity[0][1];
		bi.first.y = velocity[1][1];
		bi.first.z = velocity[2][1];

		spin_factor = VectorMagnitude(bi.second) * ball.radius / VectorMagnitude(bi.first);
		Cl = -3.25 * spin_factor * spin_factor + 1.99 * spin_factor;

		//std::cout << Cl << std::endl;

		magnitude_velocity_squared = velocity[0][1] * velocity[0][1] + velocity[1][1] * velocity[1][1] + velocity[2][1] * velocity[2][1];
		air_force = ball_area_factor * magnitude_velocity_squared;
		B = bi.first;
		Normalize(B);

	}

	//std::cout << "Final Final Ball Angular Velocity [X, Y, Z] = {" << bi.second.x << ", " << bi.second.y << ", " << bi.second.z << "}" << std::endl;
	//std::cout << "Final Final Angular Speed = " << VectorMagnitude(bi.second) << " rad/s" << std::endl << std::endl;
	//std::cout << "Final Final Ball Velocity [X, Y, Z] = {" << bi.first.x << ", " << bi.first.y << ", " << bi.first.z << "}" << std::endl;
	//std::cout << "Final Final Speed = " << VectorMagnitude(bi.first) << " m/s" << std::endl << std::endl;
	
	std::cout << "Final Ball Location (Yards): {" << position[0].back() << ", " << position[1].back() << ", " << position[2].back() << "}" << std::endl;
	std::cout << "Total flight time = " << flight_time << " seconds." << std::endl;

	//having issues with the y-axis being inverted in gnuplot so just manually invert all y-position dimensions
	for (int i = 0; i < position[1].size(); i++) position[1][i] *= -1;

	//write all position data to an external file
	std::ofstream myFile;
	myFile.open("ball_flight.dat");
	for (int i = 0; i < position[0].size(); i++)
	{
		myFile << position[0][i] << "    " << position[1][i] << "    " << position[2][i] << '\n';
	}
	myFile.close();
}
