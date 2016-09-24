#ifndef CURVATURE_APPROACH_H
#define CURVATURE_APPROACH_H

#include <vector>

using namespace std;

#include "Vector2.h"
#include "predefined_variables.h"  // contains variable that we need to calibrate
#include "motor_command.h"
#include "best_fit_circle_code.h"

#define NUM_OF_POINTS_PER_ARC 3   // minimum three

// This function calculates how long the motor command should be executed
double calculateTimeForMotorCommand (Circle circle, Vector2 startpoint, Vector2 endpoint)
{
	Vector2 centerOfCircle (circle.h, circle.k);
	Vector2 Os = startpoint = centerOfCircle;
	Vector2 Oe = endpoint - centerOfCircle;   // from center of circle to the point by whose vinicity robot want to change motor behavior
	
	double angle = GetAngle (Os, Oe);
	if (angle < 0)
	{
		angle = - angle;
	}
	double t = PI * (circle.r) / SPEED;	
	
	return t;
}

pair<double, double> calculateRatioOfSpeedBetweenWheels (Circle circle, Vector2 currentPosition, Vector2 nextPosition)
{
	pair<double, double> ratio (0, 0);   // ratio.first is vl, ratio.second is vr
	double r = circle.r;
	Vector2 centerOfCircle (circle.h, circle.k);
	Vector2 face = nextPosition - currentPosition;
	Vector2 accelerationDirection = centerOfCircle - currentPosition;
	double crossProduct = Vector2_Cross (face, accelerationDirection);

	if (crossProduct > 0)   // when positive, turn left
	{
		
		ratio.first = (r - WHEELGAP/2) / (r + WHEELGAP/2);
		ratio.second = 1;
	}
	else					// when negative, turn right
	{
		ratio.first = 1;
		ratio.second = (r - WHEELGAP/2) / (r + WHEELGAP/2);
	}

	return ratio;
}

// this function conclude all calculate made so far to generate a final motor command
MotorCommand calculateFinalCommand (pair<double, double> ratio, double timeInterval)
{
	MotorCommand command = {0, 0, 0};
	if (ratio.first == 0)
	{
		command.leftSpeed = 0;
		command.rightSpeed = SPEED;
	}
	else if (ratio.second == 0)
	{
		command.leftSpeed = SPEED;
		command.rightSpeed = 0;
	}
	else
	{
		command.leftSpeed = 2 * SPEED * ratio.first / (ratio.first + ratio.second);
		command.rightSpeed = 2 * SPEED * ratio.second / (ratio.first + ratio.second);
	}
	
	command.timeInterval = timeInterval;
	
	return command;
}

// Circle fit_with_circle (vector<Vector2> path)
vector<MotorCommand> calculate_command_base_on_best_fit_circle (vector<Vector2> path)
{
	vector<MotorCommand> command;
	
	if (path.size () < NUM_OF_POINTS_PER_ARC)
	{
		cout << "Too few points in input!" << endl;
		return command;
	}
	
	MotorCommand temp_command;
	int loopCount = path.size() - NUM_OF_POINTS_PER_ARC + 1;
	
	for (int i=0; i<loopCount; i++)
	{
		vector<Vector2> delta_displacement;
		for (int j=0; j<NUM_OF_POINTS_PER_ARC; j++)
		{
			delta_displacement.push_back (path[i+j]);
		}
		
		Circle circle = fit_with_circle (delta_displacement);
		
		double timeInterval = calculateTimeForMotorCommand (circle, path[i], path[i+1]);
		
		pair<double, double> ratio = calculateRatioOfSpeedBetweenWheels (circle, path[i], path[i+1]);    // ratio = Vl / Vr;
		
		temp_command = calculateFinalCommand (ratio, timeInterval);
		
		command.push_back (temp_command);
	}
	
	return command;
}



#endif





























