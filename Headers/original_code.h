#pragma once

//this is the code that I originally used and it didn't quite work but I didn't want to just delete it

/*
void oldCode()
{
    //another vector will be defined that points from the contact point to the center of mass of the club head, to find it we create two vectors from the origin, one to the center of mass and one to
	//the contact point. These vectors will undergo rotation from the rotation quaternion and will still be located at the origin. The vector from the contact point to the center of mass will be the
	//addition of the negative of the origin to contact point vector (which is the same as the contact point to origin) and the origin to center of mass vector after they have been rotated
	Vector_3d contact_point_to_center_of_mass; //this will be calculated later
	Vector_3d origin_to_contact_point = { -(contact_point_height + .054 / 4.0) * sin(clubhead_standard_loft), contact_point_length, (contact_point_height + .054 / 4.0) * cos(clubhead_standard_loft) };
	Vector_3d origin_to_center_of_mass = { -.0055 * cos(clubhead_standard_loft), 0, -.0055 * sin(clubhead_standard_loft) };

    QuatRotate(rotation_quaternion, origin_to_contact_point);
	QuatRotate(rotation_quaternion, origin_to_center_of_mass);

    //we can now calculate the vector from the contact point to the center of mass of the club in the new rotated frame
	contact_point_to_center_of_mass.x = origin_to_center_of_mass.x - origin_to_contact_point.x;
	contact_point_to_center_of_mass.y = origin_to_center_of_mass.y - origin_to_contact_point.y;
	contact_point_to_center_of_mass.z = origin_to_center_of_mass.z - origin_to_contact_point.z;

	//std::cout << "Contact Point to Center of Mass Vector: {" << contact_point_to_center_of_mass[0] << ", " << contact_point_to_center_of_mass[1] << ", " << contact_point_to_center_of_mass[2] << "}" << std::endl;

	//With all of the rotation work done and our coordinate system set its time to define a last few variables

	//from this point on, we change the origin from the location on the clubface that lies on the normal vector to the clubface through the center of mass to the center of the golf ball
	//this means in x, y, x space the contact point of the golf club can be found by adding the negative normal vector to the point at the center of the golf ball [0, 0, 0]
	std::vector<double> contact_point = { -clubface_normal.x * golfball_radius , -clubface_normal.y * golfball_radius, -clubface_normal.z * golfball_radius };
	std::vector<double> center_of_mass_club = { contact_point[0] + contact_point_to_center_of_mass.x, contact_point[1] + contact_point_to_center_of_mass.y, contact_point[2] + contact_point_to_center_of_mass.z };
	//std::cout << "Contact Point (XYZ) = {" << contact_point[0] << ", " << contact_point[1] << ", " << contact_point[2] << "}" << std::endl;

    //Solve for the magnituide of velocity for the club and ball after collision in the clubface normal direction
	double ball_velocity_normal = (clubhead_mass / (clubhead_mass + golfball_mass)) * (1 + coefficient_of_restitution) * VectorMagnitude(clubface_normal_velocity);
	double club_velocity_normal_final = (clubhead_mass * VectorMagnitude(clubface_normal_velocity) - golfball_mass * ball_velocity_normal) / clubhead_mass; //TODO might need a separate vector to keep track of direction here for negative values

    double A, B, C, D, E; //variables to make math a little easier
	A = R.z * club_velocity_normal_initial - R.x * club_velocity_tangent_initial;
	B = R.z * club_velocity_normal_final;
	C = R.x * club_velocity_tangent_initial;
	D = 0.4 * golfball_mass * golfball_radius * club_velocity_tangent_initial;
	E = R.x * golfball_mass / clubhead_mass - r.x + 0.4 * golfball_mass * golfball_radius + 2 * golfball_mass * golfball_mass * golfball_radius / (5 * clubhead_mass);

	double ball_velocity_tangent = (A - B + C + D) / E;

	//with the final tangent velocity for the ball found we can now calculate both the final club tangent velocity and the angular velocity about the parallel axis
	double club_velocity_tangent_final = (clubhead_mass * club_velocity_tangent_initial - golfball_mass * ball_velocity_tangent) / clubhead_mass;
	double ball_angular_velocity_parallel = (ball_velocity_tangent - club_velocity_tangent_final) / golfball_radius;

	//solve for the final parallel ball velocity in the same maner as we did for the tangential velocity
	A = R.x * club_velocity_parallel_initial - R.y * club_velocity_normal_initial;
	B = R.y * club_velocity_normal_final;
	C = R.x * club_velocity_parallel_initial;
	D = 0.4 * golfball_mass * golfball_radius * club_velocity_parallel_initial;
	E = -R.x * golfball_mass / clubhead_mass + r.x + 0.4 * golfball_mass * golfball_radius + 2 * golfball_mass * golfball_mass * golfball_radius / (5 * clubhead_mass);

	double ball_velocity_parallel = (A + B - C + D) / E;
	//can use the final parallel velocity to get the final club parallel velocity as well as the angular velocity about the tangent axis
	double club_velocity_parallel_final = (clubhead_mass * club_velocity_parallel_initial - golfball_mass * ball_velocity_parallel) / clubhead_mass;
	double ball_angular_velocity_tangent = (ball_velocity_parallel - club_velocity_parallel_final) / golfball_radius;

	//lastly we can solve for the angular velocity about the normal axis of the ball (which is caused by the center of mass of the club being in a different spot than the contact point)
	double ball_angular_velocity_normal = (R.y * club_velocity_tangent_initial - R.z * club_velocity_parallel_initial - R.y * club_velocity_tangent_final - R.z * club_velocity_parallel_final) / golfball_moment_of_inertia;

	//Now to plug back in the values obtained into the equations of motion to double check that everything checks out
	//Linear momentum equations
	//std::cout << clubhead_mass * club_velocity_normal_initial << " = " << clubhead_mass * club_velocity_normal_final + golfball_mass * ball_velocity_normal << std::endl;
	//std::cout << clubhead_mass * club_velocity_parallel_initial << " = " << clubhead_mass * club_velocity_parallel_final + golfball_mass * ball_velocity_parallel << std::endl;
	//std::cout << clubhead_mass * club_velocity_tangent_initial << " = " << clubhead_mass * club_velocity_tangent_final + golfball_mass * ball_velocity_tangent << std::endl << std::endl;

	//Angular Momentum Equations
	//std::cout << R.y * club_velocity_tangent_initial - R.z * club_velocity_parallel_initial << " = " << R.y * club_velocity_tangent_final - R.z * club_velocity_parallel_final  + golfball_moment_of_inertia * ball_angular_velocity_normal << std::endl;
	//std::cout << R.z * club_velocity_normal_initial - R.x * club_velocity_tangent_initial << " = " << R.z * club_velocity_normal_final - R.x * club_velocity_tangent_final - r.x * ball_velocity_tangent + golfball_moment_of_inertia * ball_angular_velocity_parallel << std::endl;
	//std::cout << R.x * club_velocity_parallel_initial - R.y * club_velocity_normal_initial << " = " << R.x * club_velocity_parallel_final - R.y * club_velocity_normal_final + r.x * ball_velocity_parallel + golfball_moment_of_inertia * ball_angular_velocity_tangent << std::endl << std::endl;

	Values <<
		-1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, -1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0,

		1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0,

		0, -R.z, R.y, 0, 0, 0, 0, 0, 0, -clubhead_moment_of_inertia_n, 0, 0, 0, 0, 0,
		R.z, 0, -R.x, 0, 0, 0, 0, 0, 0, 0, -clubhead_moment_of_inertia_p, 0, 0, 0, 0,
		-R.y, R.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, -clubhead_moment_of_inertia_t, 0, 0, 0,

		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -golfball_moment_of_inertia, 0, 0,
		0, 0, r.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -golfball_moment_of_inertia, 0,
		0, -r.x, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -golfball_moment_of_inertia,

		0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -R.z, R.y, 0, 0, 0,
		0, 0, 0, 0, -1, 0, 0, 1, 0, R.z, 0, -R.x, 0, 0, r.x,
		0, 0, 0, 0, 0, -1, 0, 0, 1, -R.y, R.x, 0, 0, -r.x, 0;

		Answers <<
		clubhead_mass * club_velocity_normal_initial, clubhead_mass * club_velocity_parallel_initial, clubhead_mass * club_velocity_tangent_initial,
		0, 0, 0, 0, 0, 0, 0, 0, 0, coefficient_of_restitution* club_velocity_normal_initial, 0, 0;

	//comparison values for testing
//std::cout << "Linear Momentum  (club, n): " << clubhead_mass * (Vfc.x - club_velocity_normal_initial) << " = " << Impulse.x << std::endl;
//std::cout << "Linear Momentum  (club, p): " << clubhead_mass * (Vfc.y - club_velocity_parallel_initial) << " = " << Impulse.y << std::endl;
//std::cout << "Linear Momentum  (club, t): " << clubhead_mass * (Vfc.z - club_velocity_tangent_initial) << " = " << Impulse.z << std::endl;
//std::cout << "Linear Momentum  (ball, n): " << golfball_mass * Vfb.x << " = " << -Impulse.x << std::endl;
//std::cout << "Linear Momentum  (ball, p): " << golfball_mass * Vfb.y << " = " << -Impulse.y << std::endl;
//std::cout << "Linear Momentum  (ball, t): " << golfball_mass * Vfb.z << " = " << -Impulse.z << std::endl;
//std::cout << "Angular Momentum (club, n): " << CrossProduct(R, Impulse).x << " = " << clubhead_moment_of_inertia_n * Wfc.x << std::endl;
//std::cout << "Angular Momentum (club, p): " << CrossProduct(R, Impulse).y << " = " << clubhead_moment_of_inertia_p * Wfc.y << std::endl;
//std::cout << "Angular Momentum (club, t): " << CrossProduct(R, Impulse).z << " = " << clubhead_moment_of_inertia_t * Wfc.z << std::endl;
//std::cout << "Angular Momentum (ball, n): " << CrossProduct(r, { -Impulse.x, -Impulse.y, -Impulse.z }).x << " = " << golfball_moment_of_inertia * Wfb.x << std::endl;
//std::cout << "Angular Momentum (ball, p): " << CrossProduct(r, { -Impulse.x, -Impulse.y, -Impulse.z }).y << " = " << golfball_moment_of_inertia * Wfb.y << std::endl;
//std::cout << "Angular Momentum (ball, t): " << CrossProduct(r, { -Impulse.x, -Impulse.y, -Impulse.z }).z << " = " << golfball_moment_of_inertia * Wfb.z << std::endl;
	//no slip conditions
	//std::cout << ball_velocity_parallel - club_velocity_parallel_final << " = " << golfball_radius * ball_angular_velocity_tangent << std::endl;
	//std::cout << ball_velocity_tangent - club_velocity_tangent_final << " = " << golfball_radius * ball_angular_velocity_parallel << std::endl << std::endl;

	//std::cout << "Initial Clubhead Velocity (Normal): " << club_velocity_normal_initial << std::endl;
	//std::cout << "Initial Clubhead Velocity (Parallel): " << club_velocity_parallel_initial<< std::endl;
	//std::cout << "Initial Clubhead Velocity (Tangent): " << club_velocity_tangent_initial << std::endl << std::endl;

	//std::cout << "Final Clubhead Velocity (Normal): " << club_velocity_normal_final << std::endl;
	//std::cout << "Final Clubhead Velocity (Parallel): " << club_velocity_parallel_final << std::endl;
	//std::cout << "Final Clubhead Velocity (Tangent): " << club_velocity_tangent_final << std::endl << std::endl;

	//std::cout << "Final Ball Velocity (Normal): " << ball_velocity_normal << std::endl;
	//std::cout << "Final Ball Velocity (Parallel): " << ball_velocity_parallel << std::endl;
	//std::cout << "Final Ball Velocity (Tangent): " << ball_velocity_tangent << std::endl << std::endl;

	//std::cout << "Final Ball Velocity: " << sqrt(ball_velocity_parallel * ball_velocity_parallel + ball_velocity_tangent * ball_velocity_tangent + ball_velocity_normal * ball_velocity_normal) << std::endl << std::endl;

	//std::cout << "Final Ball Angular Velocity (Normal): " << ball_angular_velocity_normal << std::endl;
	//std::cout << "Final Ball Angular Velocity (Parallel): " << ball_angular_velocity_parallel << std::endl;
	//std::cout << "Final Ball Angular Velocity (Tangent): " << ball_angular_velocity_tangent << std::endl << std::endl;
}
*/

/*
	//for now, assume that the contact point chosen on the clubface will be the origin and that the clubface doesn't have any loft or tilt applied to it yet and is just a rectangle sitting flat on the ground
	//unit vectors describing the direction of the clubface normal, clubface tangent (side to side of face) and clubface parallel (top to bottom) directions will be defined as:
	Vector_3d clubface_normal   = { cos(clubhead_standard_loft), 0, sin(clubhead_standard_loft) };
	Vector_3d clubface_tangent  = { 0, 1, 0 };
	Vector_3d clubface_parallel = { -sin(clubhead_standard_loft), 0, cos(clubhead_standard_loft) };

	//define rotation quaternion that will translate the clubface normal, tangent and parallel vectors from initial orientation to orientation with dynamic loft, openess and tilt applied to clubhead
	//the vector from the contact point to the clubhead center of mass will also be rotated
	double q0 = cos(clubhead_tilt / 2.0) * cos((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * cos(clubhead_openess / 2.0) + sin(clubhead_tilt / 2.0) * sin((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * sin(clubhead_openess / 2.0);
	double q1 = sin(clubhead_tilt / 2.0) * cos((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * cos(clubhead_openess / 2.0) - cos(clubhead_tilt / 2.0) * sin((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * sin(clubhead_openess / 2.0);
	double q2 = cos(clubhead_tilt / 2.0) * sin((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * cos(clubhead_openess / 2.0) + sin(clubhead_tilt / 2.0) * cos((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * sin(clubhead_openess / 2.0);
	double q3 = cos(clubhead_tilt / 2.0) * cos((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * sin(clubhead_openess / 2.0) - sin(clubhead_tilt / 2.0) * sin((clubhead_standard_loft - clubhead_dynamic_loft) / 2.0) * cos(clubhead_openess / 2.0);
	glm::dquat rotation_quaternion = { q0, q1, q2, q3 };

	//rotate the vectors so that they now point in the correct directions
	//std::cout << "q: {" << rotation_quaternion.w << ", " << rotation_quaternion.x << ", " << rotation_quaternion.y << ", " << rotation_quaternion.z << "}" << std::endl;
	QuatRotate(rotation_quaternion, clubface_normal);
	QuatRotate(rotation_quaternion, clubface_tangent);
	QuatRotate(rotation_quaternion, clubface_parallel);

	//std::cout << "Clubface Normal Vector: {" << clubface_normal.x << ", " << clubface_normal.y << ", " << clubface_normal.z << "}" << std::endl;
	//std::cout << "Clubface Parallel Vector: {" << clubface_parallel.x << ", " << clubface_parallel.y << ", " << clubface_parallel.z << "}" << std::endl;
	//std::cout << "Clubface Tangent Vector: {" << clubface_tangent.x << ", " << clubface_tangent.y << ", " << clubface_tangent.z << "}" << std::endl;
	//std::cout << std::endl;

	//Create vectors describing motion of golf club right before impact
	Vector_3d initial_clubhead_velocity = { initial_clubhead_speed * cos(angle_of_attack) * cos(target_line_angle) , initial_clubhead_speed * cos(angle_of_attack) * sin(target_line_angle) , -initial_clubhead_speed * sin(angle_of_attack) }; //the projection
	//std::cout << "Initial clubhead velocity = {" << initial_clubhead_velocity.x << ", " << initial_clubhead_velocity.y << ", " << initial_clubhead_velocity.z << "}" << std::endl;

	Vector_3d clubface_normal_velocity = VectorProjection(initial_clubhead_velocity, clubface_normal); //the projection of the clubhead velocity onto the clubface normal vector
	Vector_3d clubface_parallel_velocity = VectorProjection(initial_clubhead_velocity, clubface_parallel); //the projection of the clubhead velocity onto the clubface parallel vector
	Vector_3d clubface_tangent_velocity = VectorProjection(initial_clubhead_velocity, clubface_tangent); //the projection of the clubhead velocity onto the clubface tangent vector

	//std::cout << "Initial clubhead velocity (Normal) = {" << clubface_normal_velocity.x << ", " << clubface_normal_velocity.y << ", " << clubface_normal_velocity.z << "}" << std::endl;
	//std::cout << "Initial clubhead velocity (Parallel) = {" << clubface_parallel_velocity.x << ", " << clubface_parallel_velocity.y << ", " << clubface_parallel_velocity.z << "}" << std::endl;
	//std::cout << "Initial clubhead velocity (Tangent) = {" << clubface_tangent_velocity.x << ", " << clubface_tangent_velocity.y << ", " << clubface_tangent_velocity.z << "}" << std::endl;

	//TODO: The calculation for e is based on a driver, need to see if it differs by club
	double coefficient_of_restitution; //the coefficient of restitution for the golf ball, which has been proven to change slightly based on impact speed
	coefficient_of_restitution = 0.86 - 0.0029 * VectorMagnitude(clubface_normal_velocity); //this value taken from a research paper, the less directly the clubface impacts the ball the less the ball will compress so more energy is conserved
	//std::cout << "e = " << coefficient_of_restitution_ball << std::endl;

	double club_velocity_normal_initial = 0;
	double club_velocity_parallel_initial = 0;
	double club_velocity_tangent_initial = 0;
	//convert initial clubhead velocity vectors into values along n, p and t axes, the values need to be negative if pointing along the negative of the axes so multiply by dot product

	if (VectorMagnitude(clubface_normal_velocity) != 0)
		club_velocity_normal_initial = VectorMagnitude(clubface_normal_velocity) * DotProduct(clubface_normal_velocity, clubface_normal) / VectorMagnitude(clubface_normal_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	if (VectorMagnitude(clubface_parallel_velocity) != 0)
		club_velocity_parallel_initial = VectorMagnitude(clubface_parallel_velocity) * DotProduct(clubface_parallel_velocity, clubface_parallel) / VectorMagnitude(clubface_parallel_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	if (VectorMagnitude(clubface_tangent_velocity) != 0)
		club_velocity_tangent_initial = VectorMagnitude(clubface_tangent_velocity) * DotProduct(clubface_tangent_velocity, clubface_tangent) / VectorMagnitude(clubface_tangent_velocity); //the dot product will be -1 if the values are 180 degrees from eachother

	Vector_3d initial_clubhead_velocity_npt = { club_velocity_normal_initial, club_velocity_parallel_initial, club_velocity_tangent_initial };
	//std::cout << "Initial clubhead velocity [X, Y, Z] = {" << initial_clubhead_velocity.x << ", " << initial_clubhead_velocity.y << ", " << initial_clubhead_velocity.z << "}" << std::endl;
	//std::cout << "Initial clubhead velocity [N, P, T] = {" << initial_clubhead_velocity_npt.x << ", " << initial_clubhead_velocity_npt.y << ", " << initial_clubhead_velocity_npt.z << "}" << std::endl;

	//Knowing the normal velocity of the ball and club will allow us to solve for the tangential velocity of the ball and club
	//the equations here are quite cumbersome and explained in detail elsewhere. Need to express the vectors from contact point to center of mass and contact point to center of ball in terms of the
	//clubface normal, parallel and tangent axes

	//Since ball contact as always made on the vector from the clubface normal to the center of the ball, the normal component of the rball vector is rb and the tangent and parallel components are 0
	//Likewise, no matter how the clubface is rotated in the cartesian frame the contact point on the club face will never move away from the center of mass in the normal, parallel and tangent from
	///i.e. the normal component from contact point to center of mass will always be -clubhead_width / 2, along the parallel will be - impact_height and along the tangent will always be -impact_length.
	Vector_3d r = { -golfball_radius, 0, 0 }; //these are [N, P, T] coordinates
	Vector_3d R = { clubhead_width / 2.0, contact_point_height + clubhead_height * clubhead_cm_height, contact_point_length };

	//std::cout << "r: {" << r.x << ", " << r.y << ", " << r.z << "}" << std::endl;
	//std::cout << "R: {" << R.x << ", " << R.y << ", " << R.z << "}" << std::endl;

	//Solve for unknowns using Linear Algebra module Eigen::
	Eigen::MatrixXf Values(15, 15);
	Eigen::MatrixXf Answers(15, 1);

	//Establish the values matrix
	//Pn, Pp, Pt, Vcfn, Vcfp, Vcft, Vbfn, Vbfp, Vbft, Wcfn, Wcfp, Wcft, Wbfn, Wbfp, Wbft
	Values <<
		-1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, -1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, -1, 0, 0, clubhead_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0,

		1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, golfball_mass, 0, 0, 0, 0, 0, 0,

		0, R.z, -R.y, 0, -R.z * clubhead_mass, R.y * clubhead_mass, 0, 0, 0, clubhead_moment_of_inertia_n, 0, 0, 0, 0, 0,
		-R.z, 0, R.x, R.z * clubhead_mass, 0, -R.x * clubhead_mass, 0, 0, 0, 0, clubhead_moment_of_inertia_p, 0, 0, 0, 0,
		R.y, -R.x, 0, -R.y * clubhead_mass, R.x * clubhead_mass, 0, 0, 0, 0, 0, 0, clubhead_moment_of_inertia_t, 0, 0, 0,

		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, golfball_moment_of_inertia, 0, 0,
		0, 0, -r.x, 0, 0, 0, 0, 0, r.x* golfball_mass, 0, 0, 0, 0, golfball_moment_of_inertia, 0,
		0, r.x, 0, 0, 0, 0, 0, -r.x * golfball_mass, 0, 0, 0, 0, 0, 0, golfball_moment_of_inertia,

		0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -R.z, R.y, 0, 0, 0,
		0, 0, 0, 0, -1, 0, 0, 1, 0, R.z, 0, -R.x, 0, 0, r.x,
		0, 0, 0, 0, 0, -1, 0, 0, 1, -R.y, R.x, 0, 0, -r.x, 0;

	Answers <<
		clubhead_mass * initial_clubhead_velocity_npt.x, clubhead_mass * initial_clubhead_velocity_npt.y, clubhead_mass * initial_clubhead_velocity_npt.z,
		R.y * clubhead_mass * initial_clubhead_velocity_npt.z - R.z * clubhead_mass * initial_clubhead_velocity_npt.y,
		R.z * clubhead_mass * initial_clubhead_velocity_npt.x - R.x * clubhead_mass * initial_clubhead_velocity_npt.z,
		R.x * clubhead_mass * initial_clubhead_velocity_npt.y - R.y * clubhead_mass * initial_clubhead_velocity_npt.x,
		0, 0, 0, 0, 0, 0, coefficient_of_restitution* club_velocity_normal_initial, 0, 0;

	Eigen::MatrixXf Unknowns = Values.colPivHouseholderQr().solve(Answers);

	Vector_3d Impulse = { Unknowns(0), Unknowns(1), Unknowns(2) };
	Vector_3d Vfc = { Unknowns(3), Unknowns(4), Unknowns(5) };
	Vector_3d Vfb = { Unknowns(6), Unknowns(7), Unknowns(8) };
	Vector_3d Wfc = { Unknowns(9), Unknowns(10), Unknowns(11) };
	Vector_3d Wfb = { Unknowns(12), Unknowns(13), Unknowns(14) };

	//std::cout << "Impulse [n, p, t] = {" << Impulse.x << ", " << Impulse.y << ", " << Impulse.z << "}" << std::endl;
	//std::cout << "Final CLubhead Velocity [n, p, t] = {" << Vfc.x << ", " << Vfc.y << ", " << Vfc.z << "}" << std::endl;
	//std::cout << "Final Ball Velocity [n, p, t] = {" << Vfb.x << ", " << Vfb.y << ", " << Vfb.z << "}" << std::endl;
	//std::cout << "Final Clubhead Angular Velocity [n, p, t] = {" << Wfc.x << ", " << Wfc.y << ", " << Wfc.z << "}" << std::endl;
	//std::cout << "Final Ball Angular Velocity [n, p, t] = {" << Wfb.x << ", " << Wfb.y << ", " << Wfb.z << "}" << std::endl << std::endl;

	//Convert Ball linear and angular velocity back into x, y, z coordinates
	Vector_3d Ball_Velocity, Ball_Spin;
	Ball_Velocity.x = clubface_normal.x * Vfb.x + clubface_parallel.x * Vfb.y + clubface_tangent.x * Vfb.z;
	Ball_Velocity.y = clubface_normal.y * Vfb.x + clubface_parallel.y * Vfb.y + clubface_tangent.y * Vfb.z;
	Ball_Velocity.z = clubface_normal.z * Vfb.x + clubface_parallel.z * Vfb.y + clubface_tangent.z * Vfb.z;

	Ball_Spin.x = (clubface_normal.x * Wfb.x + clubface_parallel.x * Wfb.y + clubface_tangent.x * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.y = (clubface_normal.y * Wfb.x + clubface_parallel.y * Wfb.y + clubface_tangent.y * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	Ball_Spin.z = (clubface_normal.z * Wfb.x + clubface_parallel.z * Wfb.y + clubface_tangent.z * Wfb.z) / (2.0 * PI) * 60; //convert spin rate from rad/s to RPM
	
		//The +x axis is defined as from the golf ball to the intended targer
	//The +y axis is defined as from the golfer to the golf ball
	//The +z axis is defined as from the golf ball to the sky

	//Define some basic info about the ball and club
	double clubhead_mass = .272; //the mass of the clubhead in kilograms
	double clubhead_length = 0.064; //length of the clubhead in meters
	double clubhead_height = 0.054; //height of the clubhead in meters
	double clubhead_width = 0.011; //thickness of the clubhead in meters
	double clubhead_cm_height = 0.25; //the ratio of the height of the clubhead center of mass vs. the total height of the clubface. Maximum value is .55 and minimum value is .05
	double golfball_mass = 0.0459; //the mass of the golfball in kilograms
	double golfball_radius = 0.02135; //the radius of the golfball in meters
	double golfball_moment_of_inertia = 2.0 / 5.0 * golfball_mass * golfball_radius * golfball_radius; //moment of inertia for the golball in kg * m^2 (this value is the same about any axis through the golfball)
	double clubhead_moment_of_inertia_n, clubhead_moment_of_inertia_p, clubhead_moment_of_inertia_t; //the moments of inertia for the club head about axes normal, parallel and tangent to the clubface
	
	//double check that center of mass height for club is correct
	if (clubhead_cm_height > 0.55) clubhead_cm_height = 0.55;
	else if (clubhead_cm_height < 0.05) clubhead_cm_height = 0.05;

	//calculate the moments of inertia for the different axis of the club
	//the clubface is assumed to be made up of two rectangular prisms that are connected. Each prism has the same length and width but the heights differ. The height of the first prism is 9 * clubface height / 10
	//and the height of the second prism is clubface_height / 10. The ratio of masses of the two prisms are calculated based on the specified height for the center of mass of the golf club. Once this ratio is found
	//the mass of each prism is calculated by using the clubhead_mass variable. The moments of inertia for the clubhead is the sum of the moments of inertia for the two prisms utilizing the parallel axis theorem.

	double prism_one_mass;
	//too much bad math here but it should work out
	if (clubhead_cm_height >= 0.1) prism_one_mass = clubhead_mass / (20.0 / 9.0 * (1 - clubhead_cm_height));
	else prism_one_mass = clubhead_mass * (1 - 1.0 / (20 * clubhead_cm_height));
	double prism_two_mass = clubhead_mass - prism_one_mass;

	//TODO: these values should be correct but if things seem off double check these
	clubhead_moment_of_inertia_p = (clubhead_width * clubhead_width + clubhead_length * clubhead_length) / 12.0 * (prism_one_mass + prism_two_mass); //the p axis goes through the cm of both prisms so the total moment of inertia is just the sum of the two
	clubhead_moment_of_inertia_n = prism_one_mass * (81.0 / 100.0 * clubhead_height * clubhead_height + clubhead_length * clubhead_length) / 12.0 + prism_one_mass * (11 * clubhead_height / 20 - clubhead_height * clubhead_cm_height) * (11 * clubhead_height / 20 - clubhead_height * clubhead_cm_height); //component of In from prism 1
	clubhead_moment_of_inertia_n += prism_two_mass * (1.0 / 100.0 * clubhead_height * clubhead_height + clubhead_length * clubhead_length) / 12.0 + prism_two_mass * (clubhead_height * clubhead_cm_height - clubhead_height / 20) * (clubhead_height * clubhead_cm_height - clubhead_height / 20); //component of In from prism 2
	clubhead_moment_of_inertia_t = prism_one_mass * (81.0 / 100.0 * clubhead_height * clubhead_height + clubhead_width * clubhead_width) / 12.0 + prism_one_mass * (11 * clubhead_height / 20 - clubhead_height * clubhead_cm_height) * (11 * clubhead_height / 20 - clubhead_height * clubhead_cm_height); //component of It from prism 1
	clubhead_moment_of_inertia_t += prism_two_mass * (1.0 / 100.0 * clubhead_height * clubhead_height + clubhead_width * clubhead_width) / 12.0 + prism_two_mass * (clubhead_height * clubhead_cm_height - clubhead_height / 20) * (clubhead_height * clubhead_cm_height - clubhead_height / 20); //component of It from prism 2

	//TODO: The USGA has set a limit on MOI for clubheads at 5900 g/cm^2 so it stands to reason that my own values for I of the clubhead should be at least near this value
	//currently I'm closer to 500 g/cm^2 using my flat rectangular club face. Consider changing up the shape or the calculation for clubhead MOI. This lower MOI will make the club head want to spin
	//way more than it needs to
	clubhead_moment_of_inertia_n = 0.00256167;
	clubhead_moment_of_inertia_p = 0.00256167;
	clubhead_moment_of_inertia_t = 0.00256167;

	//std::cout << "Clubhead Moment of Inertia (normal): " << clubhead_moment_of_inertia_n << " kg * m^2" << std::endl;
	//std::cout << "Clubhead Moment of Inertia (parallel): " << clubhead_moment_of_inertia_p << " kg * m^2" << std::endl;
	//std::cout << "Clubhead Moment of Inertia (tangent): " << clubhead_moment_of_inertia_t << " kg * m^2" << std::endl << std::endl;

	//info about clubhead orientation
	double clubhead_standard_loft = 33.0 * PI / 180.0; //the standard loft of the club
	double clubhead_dynamic_loft = 27.5 * PI / 180.0; //the loft of the club relative to the vertical at impact with the ball (in degrees), equivalent to pitch euler angle, a higher dynamic loft means that face is gets more lofted, lower means it delofts
	double clubhead_tilt = -10.0 * PI / 180.0; //how many degrees the clubhead is tilted towards the heel (positive angle) or the toe (negative angle), 0 means the bottom of the clubhead is flush with the ground, equicalent to roll euler angle
	double clubhead_openess = 0.0 * PI / 180.0; //how many degrees open (positive angle) or closed (negative angle) the club head is to the target line at impact, equivalent to yaw euler angle

	//info about clubhead velocity direction into the ball
	double initial_clubhead_speed = 40.23; //the magnitude of the velocity vector (m/s) of the clubhead right before impact with the ball
	double angle_of_attack = 4.3 * PI / 180.0; //The angle at which the club is traveling relative to the ground when the ball is struck, a negative value means the club is traveling upwards at impact
	double target_line_angle = 0.0 * PI / 180.0; //the angle at which the clubhead is approaching the target line (positive angle, towards right of target )or moving away from the target line (negative angle, to the left of target). A value of 0 means the club head is traveling parallel to target line at impact
	
	//for the purposes of this code the clubhead will be assumed to be a rectangular prism with with dimensions length = .064 m, width = .011 m and height = .054 m.
	//the loft of the clubhead will not effect the clubface dimensions or weight. The center of mass of the club head will be located in the center of the length and width and 1/4 of the hieght
	//the user will be able to choose where on the clubface the ball first makes contact. Choose a width length location between -.032 and + .032 and a height location between -.027 and +.027
	double contact_point_length = 0;
	double contact_point_height = -.25 * clubhead_height;

	*/
