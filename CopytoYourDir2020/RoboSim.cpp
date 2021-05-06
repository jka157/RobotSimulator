// ProgrammingDemo.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>

//------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#include <math.h>		//includes sin, cos, atan2 function
#include <cmath>		//includes trig calculations
#include <vector>		//includes vector functions
#include <iomanip>		//used for setw function
#include <tuple>		//used for tuples
#include <fstream>		//open/close files

#include <chrono>
#include <thread>

using namespace std;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//Robot Dimensions
double l1 = 195;
double l2 = 142;
double d1 = 405;
double d2 = 70;
double d_3 = 410;
double d4 = 80;
double d_5 = 55;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------//

//------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// D-H Parameter Values:
double alpha[5] = { 0, 0, 0, PI, 0 };
double a_dist[5] = { 0, 195, 142, 30, 30 };
double d_dist[5] = { 405, 70, -70, 220, 60 };    // d3 will be variable
//------------------------------------------------------------------------------------------------------------------------------------------------------------------//


//WHERE function: Given joint vectors, output the corresponding position and orientation in terms of (x,y,z,theta)
vector<double> WHERE(JOINT joint_vector) {

	//JOINT joint_vector;
	vector<double> position_vector;

	// Getting the coordinates:
	position_vector.push_back(round(l1 * cos(joint_vector[0]) + l2 * cos(joint_vector[0] + joint_vector[1])));
	position_vector.push_back(round(l1 * sin(joint_vector[0]) + l2 * sin(joint_vector[0] + joint_vector[1])));
	position_vector.push_back(round(d1 + d2 - d_3 - d4 - d_5 - joint_vector[2]));
	position_vector.push_back(round((joint_vector[0] + joint_vector[1] - joint_vector[3]) * 180 / PI));

	cout << "----------Corresponding Position and Orientation----------" << endl << endl;
	cout << "X: " << position_vector[0] << " mm" << endl;
	cout << "Y: " << position_vector[1] << " mm" << endl;
	cout << "Z: " << position_vector[2] << " mm" << endl;
	cout << "Phi: " << position_vector[3] << " deg" << endl;
	cout << "----------------------------------------------------------" << endl << endl;

	return (position_vector);
}

//SOLVE function: Given the target position, outputs the nearest corresponding joint vectors (theta1, theta2, d3, theta4)
vector<double> SOLVE(double target_position[4], int int_flag, int index, vector<double> previous_config) {

	vector<double> target_joint_vectors(4);
	//vector<double> target_joint_vectors = { 1000,1000,1000,1000 };//return the corresponding joint vectors in vectors
	double theta1[2], theta2[2], d3, theta4[2]; // the correspoding joint vector
	double solution1[4], solution2[4]; // two possible solutions
	double x, y, z, phi; // target positions
	double c2, s2; // cos(theta2) and sin(theta2) 
	double a, b; //temp variables
	JOINT current_joint_config; // current joint configuration
	double distance1, distance2; // distance between the possible solutions to the current joint configuration
	bool solution1_exceed = false;
	bool solution2_exceed = false;


	x = target_position[0];
	y = target_position[1];
	z = target_position[2];
	phi = target_position[3];

	//Calculating Theta2
	c2 = (pow(x, 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2);
	s2 = sqrt(1 - pow(c2, 2));

	//there are two solutions for theta2
	theta2[0] = acos(c2);
	theta2[1] = theta2[0] * -1;

	//Calculating Theta1 using Theta2[0]
	c2 = cos(theta2[0]);
	s2 = sin(theta2[0]);
	a = (l1 + l2 * c2) * y - (l2 * s2) * x;
	b = (l1 + l2 * c2) * x + l2 * s2 * y;
	theta1[0] = atan2(a, b);

	//Calculating Theta1 using Theta2[1]
	c2 = cos(theta2[1]);
	s2 = sin(theta2[1]);
	a = (l1 + l2 * c2) * y - (l2 * s2) * x;
	b = (l1 + l2 * c2) * x + l2 * s2 * y;
	theta1[1] = atan2(a, b);

	//Calculating d3
	d3 = (d1 + d2 - d_3 - d4 - d_5 - z);

	//Calculating Theta4 using Theta2[0]
	theta4[0] = theta1[0] + theta2[0];
	theta4[1] = theta1[1] + theta2[1];


	//Two solutions
	solution1[0] = theta1[0] * 180 / PI;
	solution1[1] = theta2[0] * 180 / PI;
	solution1[2] = d3;
	solution1[3] = theta4[0] * 180 / PI - phi;// "-" because the z3-direction is the opposite from z2-direction 

	solution2[0] = theta1[1] * 180 / PI;
	solution2[1] = theta2[1] * 180 / PI;
	solution2[2] = d3;
	solution2[3] = theta4[1] * 180 / PI - phi;// "-" because the z3-direction is the opposite from z2-direction 

	//round
	for (int i = 0; i < 4; i++) {
		solution1[i] = round(solution1[i]);
		solution2[i] = round(solution2[i]);
	}


	if (int_flag == 3) {
		// Compare with current configuration
		if (index == 0) {
			GetConfiguration(current_joint_config);
		}

		// Compare with one of the possible solutions from inv kin
		else {
			for (int i = 0; i < 4; i++)
			{
				current_joint_config[i] = previous_config[i];
			}
			//current_joint_config = previous_config;
		}
	}

	else {
		GetConfiguration(current_joint_config);
	}

	cout << "Current Joint Values: {";
	for (int i = 0; i < 4; i++) {
		cout << current_joint_config[i] << " ";
	}
	cout << "}" << endl;

	cout << "solution1: { ";
	for (int i = 0; i < 4; i++) {
		//cout << round(solution1[i])<< " ";
		cout << solution1[i] << " ";
	}
	cout << "}" << endl;

	cout << "solution2: { ";
	for (int i = 0; i < 4; i++) {
		//cout << round(solution2[i])<< " ";
		cout << solution2[i] << " ";
	}
	cout << "}" << endl;

	//Rounding may affect the precision (floating point numbers are rounded)
	if (solution1[0] < -150 || solution1[0] > 150 || solution1[1] < -100 || solution1[1] > 100 || solution1[2] < -200 || solution1[2] > -100 || solution1[3] < -160 || solution1[3] >160) {
		cout << "Reject Solution 1 because the Joint Limit Exceed!" << endl;
		solution1_exceed = true;
	}
	if (solution2[0] < -150 || solution2[0] > 150 || solution2[1] < -100 || solution2[1] > 100 || solution2[2] < -200 || solution2[2] > -100 || solution2[3] < -160 || solution2[3] >160) {
		cout << "Reject Solution 2 because the Joint Limit Exceed!" << endl;
		solution2_exceed = true;
	}
	//Find the joint configuration closet to the current joint configuration


	distance1 = 0;
	distance2 = 0;
	for (int i = 0; i < 4; i++) {
		distance1 += abs(solution1[i] - current_joint_config[i]);
		distance2 += abs(solution2[i] - current_joint_config[i]);
	}
	if (!solution1_exceed && !solution2_exceed) {
		if (distance1 < distance2) {
			cout << "First solution is closer to the current joint configuration." << endl << endl;
			for (int i = 0; i < 4; i++) {
				target_joint_vectors.at(i) = solution1[i];
			}
		}
		else {
			cout << "Second solution is closer to the current joint configuration." << endl << endl;
			for (int i = 0; i < 4; i++) {
				target_joint_vectors.at(i) = solution2[i];
			}
		}
	}
	else if (!solution1_exceed && solution2_exceed) {
		cout << "First solution is chosen." << endl << endl;
		for (int i = 0; i < 4; i++) {
			target_joint_vectors.at(i) = solution1[i];
		}
	}
	else if (solution1_exceed && !solution2_exceed) {
		cout << "Second solution is chosen." << endl << endl;
		for (int i = 0; i < 4; i++) {
			target_joint_vectors.at(i) = solution2[i];
		}
	}
	else {
		cout << "Both solutions exceed the Joint Limits" << endl;
	}

	if (!(solution1_exceed && solution2_exceed)) {
		cout << "----------Corresponding Joint Vectors----------" << endl;
		cout << "Theta1: " << target_joint_vectors.at(0) << " deg" << endl;
		cout << "Theta2: " << target_joint_vectors.at(1) << " deg" << endl;
		cout << "D3: " << target_joint_vectors.at(2) << " mm" << endl;
		cout << "Theta4: " << target_joint_vectors.at(3) << " deg" << endl;
		cout << "-----------------------------------------------" << endl << endl;
	}
	return target_joint_vectors;
}


// Homogeneous Matrix Transformations
void matrixTransform(double angle, double distance, int index) {

	//  _____                                                                                                                     ____
	// |                                                                                                                              |
	// |       cos(theta)                        -sin(theta)                          0                          a_(i-1)              |
	// | sin(theta)cos(alpha_(i-1))        cos(theta)cos(alpha_(i-1))          -sin(alpha_(i-1))          -sin(alpha_(i-1))d_i        | 
	// | sin(theta)sin(alpha_(i-1))        cos(theta)sin(alpha_(i-1))          cos(alpha_(i-1))            cos(alpha_(i-1))d_i        | 
	// |             0                                 0                              0                             1                 |
	// |_____                                                                                                                     ____|

	double transformMatrix[4][4];

	//Output variables for debugging:
	/*cout << "\n///////////////////////////////";
	cout << "\nMatrix variables for debugging:";
	cout << "\ntheta[" << index - 1 << "]: " << angle;
	cout << "\nalpha[" << index - 1 << "]: " << alpha[index - 1];
	cout << "\nd_dist[" << index - 1 << "]: " << distance;
	cout << "\n///////////////////////////////";*/

	//First Row:
	transformMatrix[0][0] = cos(angle);
	transformMatrix[0][1] = -sin(angle);
	transformMatrix[0][2] = 0;
	transformMatrix[0][3] = a_dist[index - 1];

	//Second Row:
	transformMatrix[1][0] = sin(angle) * cos(alpha[index - 1]);
	transformMatrix[1][1] = cos(angle) * cos(alpha[index - 1]);
	transformMatrix[1][2] = -sin(alpha[index - 1]);
	transformMatrix[1][3] = -sin(alpha[index - 1]) * distance;

	//Third Row:
	transformMatrix[2][0] = sin(angle) * sin(alpha[index - 1]);
	transformMatrix[2][1] = cos(angle) * sin(alpha[index - 1]);
	transformMatrix[2][2] = cos(alpha[index - 1]);
	transformMatrix[2][3] = cos(alpha[index - 1]) * distance;

	//Fourth Row:
	transformMatrix[3][0] = 0;
	transformMatrix[3][1] = 0;
	transformMatrix[3][2] = 0;
	transformMatrix[3][3] = 1;

	// Outputs the Transform Matrix for Function Validation
	cout << "\nMatrix Output: " << endl;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			cout << right << setw(12) << setprecision(3) << std::fixed << transformMatrix[row][col];
		}
		cout << endl;
	}
}

// Forward Kinematics Function - asks for user inputs
vector<double> FwdKIN(int flag)
{
	double theta1, theta2, d3, theta4;
	vector<double> parameters;
	bool bool_theta1 = false;
	bool bool_theta2 = false;
	bool bool_d3 = false;
	bool bool_theta4 = false;

	double t1, t2, t4;
	double rad = PI / 180;

	bool arm_position = false;

	JOINT T_0_deg = { 0, 0, -200, 0 };
	JOINT T_0_rad = { 0, 0, 0, 0 };
	JOINT current_joint_config;

	cout << "You've chosen Forward Kinematics: " << endl;
	cout << "Configure the initial robot configuration T_0" << endl;

	// Theta1 values
	while (!bool_theta1) {
		cout << "Enter a value for theta1 in the range of [-150, 150]" << endl;
		cout << "Theta1 = ";
		cin >> theta1;
		cout << endl;

		if (theta1 <= 150 && theta1 >= -150) {
			bool_theta1 = true;
		}
		else {
			cout << "Please enter a value for theta1 in the correct range of [-150, 150]" << endl;
		}
	}

	// Theta2 values
	while (!bool_theta2) {
		cout << "Enter a value for theta2 in the range of [-100, 100]" << endl;
		cout << "Theta2 = ";
		cin >> theta2;
		cout << endl;

		if (theta2 <= 100 && theta2 >= -100) {
			bool_theta2 = true;
		}

		else {
			cout << "Please enter a value for theta2 in the correct range of [-100, 100]" << endl;
		}
	}

	// d3 values
	while (!bool_d3) {

		cout << "Enter a value for d3 in the range of [-200mm, -100mm]" << endl;
		cout << "D3 = ";
		cin >> d3;
		cout << endl;

		if (d3 >= -200 && d3 <= -100) {
			bool_d3 = true;
		}

		else {
			cout << "Please enter a value of d3 in the correct range of [-200mm, -100mm]" << endl;
		}
	}

	// Theta4 values
	while (!bool_theta4) {

		cout << "Enter a value for theta4 in the range of [-160, 160]" << endl;
		cout << "Theta4 = ";
		cin >> theta4;
		cout << endl;

		if (theta4 <= 160 && theta4 >= -160) {
			bool_theta4 = true;
		}

		else {
			cout << "Please enter a value of theta4 in the correct range of [-160, 160]" << endl;
		}
	}


	// Changing angles to radians:
	t1 = theta1 * rad;
	t2 = theta2 * rad;
	t4 = theta4 * rad;
	T_0_rad[0] = t1;
	T_0_rad[1] = t2;
	T_0_rad[2] = d3;
	T_0_rad[3] = t4;


	// Function for returning the coordinates (x, y, z, phi):
	parameters = WHERE(T_0_rad);



	// Joint values in degree for MoveToConfiguration function
	T_0_deg[0] = theta1;
	T_0_deg[1] = theta2;
	T_0_deg[2] = d3; //keep d3 as -200 to locate the arm directly above the desired location
	T_0_deg[3] = theta4;

	// Update emulator with specified parameters:
	MoveToConfiguration(T_0_deg); //This function needs joint values in degrees
	cout << "--> Positioning to desired joint configuration." << endl;
	while (!arm_position) {
		GetConfiguration(current_joint_config);
		if (current_joint_config[0] == T_0_deg[0] && current_joint_config[1] == T_0_deg[1] && current_joint_config[2] == T_0_deg[2] && current_joint_config[3] == T_0_deg[3]) {
			arm_position = true;
		}
	}
	arm_position = false;

	// If we want Trajectory Planning -- we don't want to lift up 
	if (flag == 1) {

		cout << "--> Arrived above the box. Start moving down." << endl; //Stop here as we don't need to move the box down?
		T_0_deg[2] = d3;
		MoveToConfiguration(T_0_deg);

		while (!arm_position) {
			GetConfiguration(current_joint_config);
			if (current_joint_config[0] == T_0_deg[0] && current_joint_config[1] == T_0_deg[1] && current_joint_config[2] == T_0_deg[2] && current_joint_config[3] == T_0_deg[3]) {
				arm_position = true;
			}
		}

		cout << "--> Box is picked up. Start lifting up." << endl;
		T_0_deg[2] = -200;
		MoveToConfiguration(T_0_deg);
	}

	// Calculate the Transform Matrices:
	/*for (int i = 1; i < 6; i++) {

		if (i == 3) {
			d_dist[i - 1] += T_0_rad[i - 1];
			matrixTransform(0, d_dist[i - 1], i);
		}

		else if (i == 5) {
			matrixTransform(0, d_dist[i - 1], i);
		}

		else {
			matrixTransform(T_0_rad[i - 1], d_dist[i - 1], i);
		}

	}*/

	return parameters;

}

// Inverse Kinematics Function - asks for user inputs or use passed in values
tuple<vector<double>, vector<double>> InvKIN(vector<double> positions, int set_flag, int incrementer, vector<double> target_angles) {

	//vector<double> target_angles;
	double x_position, y_position, z_position, phi;
	bool condition_bool = false;
	double condition;
	double target_position[4];
	JOINT q;
	//vector<double> target_angles = { 1000, 1000, 1000, 1000 };
	//vector<double> position_vector;
	double theta2_1, theta2_2;

	bool arm_position = false;


	cout << "Doing the Inverse Kinematics:" << endl;

	while (!condition_bool) {

		// If case = 2, (just doing the inverse kinematics)
		if (set_flag != 3) {

			// x_position values
			cout << "Enter x value:" << endl;
			cin >> x_position;
			cout << endl;
			// y_position values
			cout << "Enter y value:" << endl;
			cin >> y_position;
			cout << endl;
			// z_position values
			cout << "Enter z value in the range of [30mm, 130mm]" << endl;
			cin >> z_position;
			cout << endl;
			// phi values
			cout << "Enter phi value:" << endl;
			cin >> phi;
			cout << endl;
		}

		else {
			x_position = positions[0];
			y_position = positions[1];
			z_position = positions[2];
			phi = positions[3];
		}

		// Solution 2 from catalogue of solutions: 
		condition = (pow(x_position, 2) + pow(y_position, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2);

		if (condition > 1 || condition < -1) {
			cout << "Limit Exceed: Out of the workspace" << endl;
			cout << "Please re-enter position vectors" << endl;
			cout << endl;

			// To triger an error checking case for limit exceeded value 
			if (set_flag == 3) {
				/*positions[0] = 0;*/ //Position[0] = 0 can't be an error case because in some cases position is equal to 0
				positions[2] = 0; //z never be 0 in valid cases. So changed to position[2] from position[0]
				return { positions, target_angles };
			}

		}

		else {
			target_position[0] = x_position;
			target_position[1] = y_position;
			target_position[2] = z_position;
			target_position[3] = phi;

			target_angles = SOLVE(target_position, set_flag, incrementer, target_angles);

			for (int i = 0; i < 4; i++) {
				q[i] = target_angles.at(i);
			}
			if (q[0] < -150 || q[0] > 150 || q[1] < -100 || q[1] > 100 || q[2] < -200 || q[2] > -100 || q[3] < -160 || q[3] >160) {
				cout << "Joint Limit Exceed!" << endl;
				cout << "Please re-enter position vectors" << endl;
				cout << endl;

				// Case 3 - Joint limits exceeded:
				if (set_flag == 3) {
					//positions[0] = 0;
					positions[2] = 0;
					return { positions, target_angles };
				}

			}
			else {

				// Case 3 - Joint limits 
				if (set_flag == 3) {
					positions[0] = q[0];
					positions[1] = q[1];
					positions[2] = q[2];
					positions[3] = q[3];

					return { positions, target_angles };
				}


				condition_bool = true;
			}
		}
	}

	// if statement here for flag
	if (set_flag != 3) {
		q[2] = -200;
		JOINT current_joint_config;

		MoveToConfiguration(q);
		cout << "--> Positioning to desired joint configuration." << endl;

		while (!arm_position) {
			GetConfiguration(current_joint_config);
			if (current_joint_config[0] == q[0] && current_joint_config[1] == q[1] && current_joint_config[2] == q[2] && current_joint_config[3] == q[3]) {
				arm_position = true;
			}
		}
		arm_position = false;
		cout << "--> Arrived above the box. Start moving down." << endl;
		q[2] = target_angles.at(2);
		MoveToConfiguration(q);

		while (!arm_position) {
			GetConfiguration(current_joint_config);
			if (current_joint_config[0] == q[0] && current_joint_config[1] == q[1] && current_joint_config[2] == q[2] && current_joint_config[3] == q[3]) {
				arm_position = true;
			}
		}
		cout << "--> Box is picked up. Start lifting up." << endl;
		cout << "-------------------------------------------------------------" << endl << endl;
		q[2] = -200;

		//// Update emulator with specified parameters:
		MoveToConfiguration(q);
	}

	if (set_flag == 3) {
		for (int k = 0; k < 4; k++) {
			positions[k] = q[0];
		}
	}

	return { positions, target_angles };

}

// Work in Progress 
vector<vector<double>> CubicCoefficients(vector<double> thetaO, vector<double> thetaF, vector<double> velO, vector<double> velF, double tF)
{
	vector<vector<double>> cubicCoeff;
	vector<double> a_constants(4);
	double a0, a1, a2, a3;

	for (int i = 0; i < 4; i++) {

		a0 = thetaO[i];
		a1 = velO[i];
		a2 = (3 / pow(tF, 2)) * (thetaF[i] - thetaO[i]) - (2 / tF) * velO[i] - (1 / tF) * velF[i];
		a3 = (-2 / pow(tF, 3)) * (thetaF[i] - thetaO[i]) + (1 / pow(tF, 2)) * (velF[i] + velO[i]);

		a_constants[0] = a0;
		a_constants[1] = a1;
		a_constants[2] = a2;
		a_constants[3] = a3;

		cubicCoeff.push_back(a_constants);

	}


	return cubicCoeff;
}

//Computes average velocity for each interval
vector<double> get_avg_vel(vector<double> joints, vector<double> time_intervals) {
	vector<double> average_velocity;
	double temp;
	//Error case
	/*if (sizeof(joints) != sizeof(time_intervals)) {
		cout << "Error in computing average velocity for each time interval. Invalid Index" << endl;
	}*/

	for (int i = 0; i < 4; i++) {
		temp = (joints[i + 1] - joints[i]) / (time_intervals[i + 1] - time_intervals[i]);
		average_velocity.push_back(temp);
	}

	return average_velocity;
}
//Computes time intervals
//***Note: For now, it only divides them into equal time segments***
vector<double> get_time_intervals(double total_time) {

	vector<double> time_intervals;
	double time_segment = total_time / 4;
	//double temp;
	for (int i = 0; i < 5; i++) {
		//temp = i * time_segment;
		//time_intervals.push_back(temp);
		time_intervals.push_back(i * time_segment);
	}
	return time_intervals;
}

// Calculates the Path for Position Using the Coefficients:
vector<vector<double>> PositionEQ(vector<vector<double>> coefficients, double time_value, double sampling_rate, double sample_count) {

	// position equation: theta = a_0 + a_1t + a_2(t)^2 + a_3(t)^3

	// (spline_coeff1, time_intervals[0], sampling_rate, samples)

	vector<vector<double>> theta_T;
	vector<double> temp;

	//cout << "\nSampling rate is: " << sampling_rate << endl; // = 0.6
	//cout << "Time value is :" << time_value << endl; // = 3
	//cout << "Entered Position function\n" << endl;
	for (int i = 0; i < 4; i++) {
		for (double j = 0; j <= time_value + 1; j = j + sampling_rate) {
			// Loop for all theta1, theta2, d3, theta4
			temp.push_back(coefficients[i][0] + coefficients[i][1] * j + coefficients[i][2] * (pow(j, 2)) + coefficients[i][3] * (pow(j, 3)));
			//cout << "This is j: " << j << endl;

		}
		//cout << "Finished loop " << i << endl;
		theta_T.push_back(temp);
		temp.clear();

	}
	//cout << "Finished all loops" << endl;
	/*

	theta_T{
				Theta_T(theta1){y(t=0), y(t=0.001), y(t=0.002), ........ y(t=3)},
				Theta_T(theta2){ },
				Theta_T(d3){ },
				Theta_T(theta4){ }}

	*/
	//cout << "Size of Position Vector is: " << theta_T.size() << endl;

	return theta_T;
}

// Calculates the Path for Velocity Using the Coefficients:
vector<vector<double>> VelocityEQ(vector<vector<double>> coefficients, double time_value, double sampling_rate, double sample_count) {

	// velocity equation: theta' = a_1 + 2a_2t + 3a_3(t)^2

	vector<vector<double>> vel_T;
	vector<double> temp;

	//cout << "\nSampling rate is: " << sampling_rate << endl; // = 0.6
	//cout << "Time value is :" << time_value << endl; // = 3
	//cout << "Entered Velocity function:\n" << endl;
	for (int i = 0; i < 4; i++) {
		for (double j = 0; j <= time_value + 1; j = j + sampling_rate) {
			temp.push_back(coefficients[i][1] + 2 * coefficients[i][2] * j + 3 * coefficients[i][3] * (pow(j, 2)));
			//cout << "Velocity Value: " << coefficients[i][1] + 2 * coefficients[i][2] * j + 3 * coefficients[i][3] * (pow(j, 2)) << endl;
			//cout << "This is value of J: " << j << endl;
		}
		vel_T.push_back(temp);
		temp.clear();
	}

	return vel_T;
}

// Calculates the Path for Acceleration Using the Coefficients:
vector<vector<double>> AccelerationEQ(vector<vector<double>> coefficients, double time_value, double sampling_rate, double sample_count) {

	// acceleration equation: theta'' = 2a_2 + 6a_3t

	vector<vector<double>> accel_T;
	vector<double> temp;

	//cout << "\nSampling rate is: " << sampling_rate << endl; // = 0.6
	// cout << "Time value is :" << time_value << endl; // = 3
	//cout << "Entered Acceleration function:\n" << endl;
	for (int i = 0; i < 4; i++) {
		for (double j = 0; j <= time_value + 1; j = j + sampling_rate) {
			temp.push_back(2 * coefficients[i][2] + 6 * coefficients[i][3] * j);
			//cout << "Accleration Value: " << 2 * coefficients[i][2] + 6 * coefficients[i][3] * j << endl;
			//cout << "This is value of J: "  << j << endl;
		}
		accel_T.push_back(temp);
		temp.clear();
	}

	return accel_T;
}

// Write to CSV File:
void write_csv(vector<double> time, vector<double> joint1, vector<double> joint2, vector<double> joint3, vector<double> joint4) {
	//void write_csv(vector<double> time){

		// Write to csv file:
std:ofstream myFile("Export_Data.csv");
	myFile << "Exported Data from SCARA Robot" << endl;
	myFile << "Time(s): " << "," << "Joint1" << "," << "Joint2" << "," << "Joint3" << "," << "Joint4" << endl;

	for (int i = 0; i < time.size(); i++) {
		myFile << time.at(i) << "," << joint1.at(i) << "," << joint2.at(i) << "," << joint3.at(i) << "," << joint4.at(i) << endl;
	}

	myFile.close();

}


// Demo 2: Trajectory Projection Function
vector<double> TrajProjection(vector<double> position, int input_flag) {

	// User specifies three intermediate tool frames as well as the desired goal frame
	// User also specifies the obstacles in the enviornment

	double x, y, z, phi;
	double time;
	double samples = 50;
	double time_segment;
	double sampling_rate;

	vector<double> temp(4);
	vector<double> time_vector;
	vector<double> joints;
	vector<double> compare_joint;
	vector<double> final_position;
	vector<double> time_intervals;
	vector<double> average_velocity_1, average_velocity_2, average_velocity_3, average_velocity_4;
	vector<double> joint1_values, joint2_values, joint3_values, joint4_values;
	vector<double> empty_vector = { 0, 0, 0, 0 };

	vector<vector<double>> spline_coeff1, spline_coeff2, spline_coeff3, spline_coeff4;

	vector<double> joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint1_vel, joint2_vel, joint3_vel, joint4_vel, joint1_acc, joint2_acc, joint3_acc, joint4_acc;
	vector<vector<double>> joint_pos;
	vector<double> joint_temp;

	vector<vector<double>> pos1, pos2, pos3, pos4;
	vector<vector<double>> vel1, vel2, vel3, vel4;
	vector<vector<double>> accel1, accel2, accel3, accel4;

	vector<vector<double>> intermediate_frames;

	JOINT configurations = { 0,0,0,0 };
	JOINT velocities = { 0,0,0,0 };
	JOINT accelerations = { 0,0,0,0 };

	bool is_velocity_limit_exceed = true;
	bool valid_option_chosen = false;
	int limit_option = 0;



	// Ask the user to input the intermediate tool frame positions in x, y, z, phi:
	cout << "User enters three intermediate Tool Frames and the Goal Frame" << endl;

	for (int i = 0; i < 5; i++) {


		if (i == 0) {

			cout << "Current Position and Orientation of the Robot: " << endl;
			cout << "X: " << position[0] << endl;
			cout << "Y: " << position[1] << endl;
			cout << "Z: " << position[2] << endl;
			cout << "Phi: " << position[3] << endl;

			temp[0] = position[0];
			temp[1] = position[1];
			temp[2] = position[2];
			temp[3] = position[3];

			cout << "-------------------------------------------------------------" << endl << endl;
			cout << "Corresponding Joint Values Calculated from Inverse Kinematics: " << endl;


		}
		else {

			if (i != 4) {
				cout << "Specify the Tool Frame " << i << " [x, y, z, phi]: " << endl;
				cout << "For intermediate frame: " << i << endl;
			}

			else {
				// Ask the user to input the goal frame position in x, y, z, phi:
				cout << "Specify the Goal Frame for the robot to reach: " << endl;
			}

			cout << "x: ";
			cin >> x;

			cout << "y: ";
			cin >> y;

			cout << "z: ";
			cin >> z;

			cout << "phi: ";
			cin >> phi;
			cout << endl;

			temp[0] = x;
			temp[1] = y;
			temp[2] = z;
			temp[3] = phi;
		}

		//-----------------------------------------------
		//Testing purposes only
		/*if (i == 0) {
			temp[0] = position[0];
			temp[1] = position[1];
			temp[2] = position[2];
			temp[3] = position[3];
		}
		else if (i == 1) {
			temp[0] = -167;
			temp[1] = 143;
			temp[2] = 30;
			temp[3] = 100;
		}
		else if (i == 2) {
			temp[0] = 337;
			temp[1] = 0;
			temp[2] = 130;
			temp[3] = 0;
		}
		else if (i == 3) {
			temp[0] = 100;
			temp[1] = -150;
			temp[2] = 30;
			temp[3] = 90;
		}
		else {
			temp[0] = 0;
			temp[1] = 337;
			temp[2] = 30;
			temp[3] = 90;
		}*/
		//-----------------------------------------------


		// Returns Current Joints and Newly Calculated Joints: - also checks for out of range
		tie(joints, compare_joint) = InvKIN(temp, input_flag, i, compare_joint);

		if (joints[2] == 0) {
			i--;
			// not a valid tool frame
		}

		else {
			intermediate_frames.push_back(joints);

		}

	}




	//------------------------------------------
	//***test output***
	cout << "Intermediate Frames: " << endl;
	/*cout << "[ ";*/
	//cout << "row = " << sizeof(intermediate_frames) << endl;
	//cout << "col = " << sizeof(intermediate_frames[0]) << endl;
	for (int a = 0; a < 5; a++) {
		/*cout << "a is" << a << endl;*/

		cout << "Frame: " << a + 1 << endl;

		for (int b = 0; b < 4; b++) {
			/*cout << "b is" << b << endl;*/
			cout << intermediate_frames[a][b] << " ";
		}
		cout << endl;
	}
	//------------------------------------------



	// User specifies the time duration (in seconds) for the trajectory
	cout << "Specify the duration: " << endl;
	cin >> time;
	cout << "-------------------------------------------------------------" << endl << endl;

	//-------------------------------END OF USER INPUTS-------------------------------------------------

	while (is_velocity_limit_exceed) {
		is_velocity_limit_exceed = false;
		valid_option_chosen = false;
		cout << "Total time: " << time << endl;
		//***Note: For now, it only divides them into equal time segments***
		time_intervals = get_time_intervals(time);
		time_segment = time / 4;



		//------------------------------------------
		//***test output***
		//cout << "size of time_intervals " << sizeof(time_intervals) << endl;
		/*cout << "Time Intervals: ";
		for (int ti = 0; ti < 5; ti++) {
			cout << time_intervals[ti] << " ";
		}
		cout << endl;*/
		//------------------------------------------



		//grouping each joint values into one vector
		for (int i = 0; i < 5; i++) {

			joint1_values.push_back(intermediate_frames[i][0]);
			joint2_values.push_back(intermediate_frames[i][1]);
			joint3_values.push_back(intermediate_frames[i][2]);
			joint4_values.push_back(intermediate_frames[i][3]);
		}

		//Calculating average_velocity for each joints at each interval

		average_velocity_1 = get_avg_vel(joint1_values, time_intervals);
		average_velocity_2 = get_avg_vel(joint2_values, time_intervals);
		average_velocity_3 = get_avg_vel(joint3_values, time_intervals);
		average_velocity_4 = get_avg_vel(joint4_values, time_intervals);




		//------------------------------------------
		//***test output***
		/*cout << "joint1_values are ";
		for (int i = 0; i < 5; i++) {
			cout << joint1_values[i] << " ";
		}
		cout << endl;

		cout << "joint2_values are ";
		for (int i = 0; i < 5; i++) {
			cout << joint2_values[i] << " ";
		}
		cout << endl;

		cout << "joint3_values are ";
		for (int i = 0; i < 5; i++) {
			cout << joint3_values[i] << " ";
		}
		cout << endl;

		cout << "joint4_values are ";
		for (int i = 0; i < 5; i++) {
			cout << joint4_values[i] << " ";
		}
		cout << endl;

		*/

		//cout << "average_velocity_1 are ";
		//for (int i = 0; i < 4; i++) {
		//	cout << average_velocity_1[i] << " ";
		//}
		//cout << endl;

		//cout << "average_velocity_2 are ";
		//for (int i = 0; i < 4; i++) {
		//	cout << average_velocity_2[i] << " ";
		//}
		//cout << endl;

		//cout << "average_velocity_3 are ";
		//for (int i = 0; i < 4; i++) {
		//	cout << average_velocity_3[i] << " ";
		//}
		//cout << endl;

		//cout << "average_velocity_4 are ";
		//for (int i = 0; i < 4; i++) {
		//	cout << average_velocity_4[i] << " ";
		//}
		//cout << endl << endl;


		/*for (int k = 0; k < 5; k++) {
			cout << "Average velocity: " << k + 1 << endl;
			cout << "Joint Values: " << k + 1 << endl;

			for (int l = 0; l < 5; l++) {
				cout << average_velocity[k][l] << endl;
				cout << joint_values[k][l] << endl;
			}

		}*/


		//***test output***
		//------------------------------------------

		// Current Frame to First Intermediate Frame:
		spline_coeff1 = CubicCoefficients(intermediate_frames[0], intermediate_frames[1], empty_vector, average_velocity_1, time_segment); // velocity initial should be 0?

		//spline_coeff1 = {a0, a1, a2, a3}   == theta 1
		//				  {a0, a1, a2, a3}   == theta 2
		//				  {a0, a1, a2, a3}   == d3
		//				  {a0, a1, a2, a3}}  == theta4

		/*cout << "For Starting Position to A:" << endl;
		for (int i = 0; i < 4; i++) {

			cout << "Theta: " << i + 1 << endl;

			for (int j = 0; j < 4; j++) {

				cout << spline_coeff1[i][j] << " ";
			}
			cout << endl;

		}
		cout << "-------------------------------------------------------------" << endl << endl;*/


		// First Intermediate Frame to Second Intermediate Frame:
		spline_coeff2 = CubicCoefficients(intermediate_frames[1], intermediate_frames[2], average_velocity_1, average_velocity_2, time_segment);

		/*cout << "For A to B:" ;
		for (int i = 0; i < 4; i++) {

			cout << "Theta: " << i + 1 << endl;

			for (int j = 0; j < 4; j++) {

				cout << spline_coeff2[i][j] << " ";
			}

		}

		cout << endl <<"-------------------------------------------------------------" << endl << endl;*/

		// Second Intermediate Frame to Third Intermediate Frame:
		spline_coeff3 = CubicCoefficients(intermediate_frames[2], intermediate_frames[3], average_velocity_2, average_velocity_3, time_segment);

		// Third Intermediate Frame to Goal Frame:
		spline_coeff4 = CubicCoefficients(intermediate_frames[3], intermediate_frames[4], average_velocity_3, empty_vector, time_segment); // velocity final should be 0?

		sampling_rate = time_segment / samples;

		pos1 = PositionEQ(spline_coeff1, time_segment, sampling_rate, samples);
		pos2 = PositionEQ(spline_coeff2, time_segment, sampling_rate, samples);
		pos3 = PositionEQ(spline_coeff3, time_segment, sampling_rate, samples);
		pos4 = PositionEQ(spline_coeff4, time_segment, sampling_rate, samples);

		vel1 = VelocityEQ(spline_coeff1, time_segment, sampling_rate, samples);
		vel2 = VelocityEQ(spline_coeff2, time_segment, sampling_rate, samples);
		vel3 = VelocityEQ(spline_coeff3, time_segment, sampling_rate, samples);
		vel4 = VelocityEQ(spline_coeff4, time_segment, sampling_rate, samples);

		accel1 = AccelerationEQ(spline_coeff1, time_segment, sampling_rate, samples);
		accel2 = AccelerationEQ(spline_coeff2, time_segment, sampling_rate, samples);
		accel3 = AccelerationEQ(spline_coeff3, time_segment, sampling_rate, samples);
		accel4 = AccelerationEQ(spline_coeff4, time_segment, sampling_rate, samples);


		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < vel1[0].size(); i++) {
				if (j != 2) {
					if (abs(vel1[j][i]) > 150) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed at Start Frame and Frame 1" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel2[j][i]) > 150) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 1 and Frame 2" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel3[j][i]) > 150) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 2 and Frame 3" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel4[j][i]) > 150) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 3 and Goal Frame" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
				}
				else {
					if (abs(vel1[j][i]) > 50) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed at Start Frame and Frame 1" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel2[j][i]) > 50) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 1 and Frame 2" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel3[j][i]) > 50) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 2 and Frame 3" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
					if (abs(vel4[j][i]) > 150) {
						cout << "ERROR: Joint " << j + 1 << " velocity limit exceed between Frame 3 and Goal Frame" << endl;
						is_velocity_limit_exceed = true;
						break;
					}
				}




			}
		}

		while (!(valid_option_chosen)) {
			valid_option_chosen = true;
			if (is_velocity_limit_exceed) {

				cout << "1. Continue with exceed velocity limit" << endl << "2. Double the total time" << endl << "3. Go to the main menu" << endl;

				cin >> limit_option;

				if (limit_option == 1) {
					cout << "continuing with exceed velocity limit";
					is_velocity_limit_exceed = false;
				}
				else if (limit_option == 2) {
					cout << "Total time is doubled" << endl;;
					time = time * 2;
				}
				else if (limit_option == 3) {
					return joints;
				}
				else {
					cout << "Not a valid option. Please select from the provided options." << endl;
					valid_option_chosen = false;
				}





			}
		}
		cout << endl << "-------------------------------------------------------------" << endl << endl;
	}


	//velocity limit doesn't exceed or user wants to proceed with limit exceeded





	//move from starting postion to frame1 
	long long sleep_time = 1000 * time / pos1[0].size() / 4;
	cout << "Moving from the starting position to frame 1" << endl;
	for (int j = 0; j < pos1[0].size(); j++) {
		for (int i = 0; i < 4; i++) {
			configurations[i] = pos1[i][j];
			velocities[i] = vel1[i][j];
			accelerations[i] = accel1[i][j];
		}
		/*cout << "Configuration = [ " << configurations[0] << " " << configurations[1] << " " << configurations[2] << " " << configurations[3] << "]" << endl;
		cout << "velocities = [ " << velocities[0] << " " << velocities[1] << " " << velocities[2] << " " << velocities[3] << "]" << endl;
		cout << "accelerations = [ " << accelerations[0] << " " << accelerations[1] << " " << accelerations[2] << " " << accelerations[3] << "]" << endl;*/

		MoveWithConfVelAcc(configurations, velocities, accelerations);
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		//cout << "Next time interval" << endl;

	}
	/*StopRobot();
	ResetRobot();*/
	cout << "Moving from the frame 1 to frame 2" << endl;
	for (int j = 0; j < pos2[0].size(); j++) {
		for (int i = 0; i < 4; i++) {
			configurations[i] = pos2[i][j];
			velocities[i] = vel2[i][j];
			accelerations[i] = accel2[i][j];
		}
		/*cout << "Configuration = [ " << configurations[0] << " " << configurations[1] << " " << configurations[2] << " " << configurations[3] << "]" << endl;
		cout << "velocities = [ " << velocities[0] << " " << velocities[1] << " " << velocities[2] << " " << velocities[3] << "]" << endl;
		cout << "accelerations = [ " << accelerations[0] << " " << accelerations[1] << " " << accelerations[2] << " " << accelerations[3] << "]" << endl;*/

		MoveWithConfVelAcc(configurations, velocities, accelerations);
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		//cout << "Next time interval" << endl;

	}
	/*StopRobot();
	ResetRobot();*/
	cout << "Moving from the frame 2 to frame 3" << endl;
	for (int j = 0; j < pos3[0].size(); j++) {
		for (int i = 0; i < 4; i++) {
			configurations[i] = pos3[i][j];
			velocities[i] = vel3[i][j];
			accelerations[i] = accel3[i][j];
		}
		/*cout << "Configuration = [ " << configurations[0] << " " << configurations[1] << " " << configurations[2] << " " << configurations[3] << "]" << endl;
		cout << "velocities = [ " << velocities[0] << " " << velocities[1] << " " << velocities[2] << " " << velocities[3] << "]" << endl;
		cout << "accelerations = [ " << accelerations[0] << " " << accelerations[1] << " " << accelerations[2] << " " << accelerations[3] << "]" << endl;*/

		MoveWithConfVelAcc(configurations, velocities, accelerations);
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		//cout << "Next time interval" << endl;

	}
	/*StopRobot();
	ResetRobot();*/
	cout << "Moving from frame 3 to goal frame" << endl;
	for (int j = 0; j < pos4[0].size(); j++) {
		for (int i = 0; i < 4; i++) {
			configurations[i] = pos4[i][j];
			velocities[i] = vel4[i][j];
			accelerations[i] = accel4[i][j];
		}
		/*cout << "Configuration = [ " << configurations[0] << " " << configurations[1] << " " << configurations[2] << " " << configurations[3] << "]" << endl;
		cout << "velocities = [ " << velocities[0] << " " << velocities[1] << " " << velocities[2] << " " << velocities[3] << "]" << endl;
		cout << "accelerations = [ " << accelerations[0] << " " << accelerations[1] << " " << accelerations[2] << " " << accelerations[3] << "]" << endl;*/

		MoveWithConfVelAcc(configurations, velocities, accelerations);
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		//cout << "Next time interval" << endl;

	}
	StopRobot();
	ResetRobot();



	// To export to csv (this is our t axis)
	for (double t = 0; t <= time; t += sampling_rate) {
		time_vector.push_back(t);
	}


	//joint1_pos.push_back(pos1[0]);

	for (int i = 0; i < 4; i++) {

		joint_temp = {};

		for (int j = 0; j < pos1[i].size(); j++) {
			joint_temp.push_back(pos1[i][j]);

		}

		for (int k = 0; k < pos2[i].size(); k++) {
			joint_temp.push_back(pos2[i][k]);
		}

		for (int l = 0; l < pos3[i].size(); l++) {
			joint_temp.push_back(pos3[i][l]);
		}

		for (int m = 0; m < pos4[i].size(); m++) {
			joint_temp.push_back(pos4[i][m]);
		}

		if (i == 0) {
			joint1_pos = joint_temp;
		}

		else if (i == 1) {
			joint2_pos = joint_temp;
		}

		else if (i == 2) {
			joint3_pos = joint_temp;
		}

		else {
			joint4_pos = joint_temp;
		}


	}


	// Function to Write to CSV:
	write_csv(time_vector, joint1_pos, joint2_pos, joint3_pos, joint4_pos);

	cout << endl;



	return joints;
}
//-----------------------------------------------------------------------------
//bool MoveWithConfVelAcc(JOINT &conf, JOINT &vel, JOINT &acc);
//
//Description:
//      This function moves the robot with the desired profile, namely to 
//         acheive the desired velocity/acceleration at the desired configuration.
//
//Input Parameter:
//      conf: Desired configuration
//      vel : Desired velocity
//      acc : Desired acceleration 
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------


// Main function: runs our UI
int main(int argc, char* argv[])
{
	int input = 0;															//User input variable
	int option;																// User option for Trajectory Planning
	vector <double> starting_position;										//Starting position used for Trajectory Planning
	printf("Starting Scara Robot Simulation \n\n");							//Running UI

	//Degenerate condition Check
	cout << "-----------------Degenerate Condition Check------------------" << endl;
	if (l1 != l2) {
		cout << "L1 and L2 are not equal. Degenerate condition doesn't apply." << endl;
	}
	else {
		cout << "L1 and L2 are equal. Degenerate condition applies." << endl << endl;
	}
	cout << "-------------------------------------------------------------" << endl << endl;



	while (1) {
		cout << "-------------------------------------------------------------" << endl;
		printf("Choose from the following options: \n");						//Provide user with menu options
		printf("0. Exit Program \n");
		printf("1. Forward Kinematics \n");
		printf("2. Inverse Kinematics \n");
		printf("3. Trajectory Projection \n");


		cin >> input;															//Ask for user input (menu option)
		cout << endl;
		switch (input) {

		case 0: // Exits program

			return 0;

		case 1: // Forward Kinematics

			FwdKIN(input);
			break;

		case 2: // Inverse Kinematics

			InvKIN(starting_position, input, NULL, vector<double>());			// Passing in NULL vector
			break;

		case 3: // Forward Kinemeatics first, then trajectory Projection 

			cout << "Start Trajectory Projection - Select from the following options: " << endl;

			
			starting_position = FwdKIN(input);
			starting_position = TrajProjection(starting_position, input);

			break;

		default:

			cout << "Not a valid option. Please select from the provided options." << endl;

		}
	}
	cout << "THIS IS THE END OF THE LINE" << endl;

	return 0;
}