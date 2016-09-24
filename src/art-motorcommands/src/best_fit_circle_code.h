#ifndef BEST_FIT_CIRCLE_CODE_H
#define BEST_FIT_CIRCLE_CODE_H

using namespace std;
#include <iostream>

#include <vector>
#include <math.h>
#include "Vector2.h"

// this function is to sum every element in an array
double Sigma_sum (vector<double> array)
{
	double sum;
	int num = array.size();
	for (int i=0; i<num; i++)
	{
		sum += array[i];
	}
	return sum;
}

double calculate_determinant (double (&matrix)[3][3])
{
	double determinant;
	double term1 = matrix[0][0] * matrix[1][1] * matrix[2][2];
	double term2 = matrix[0][1] * matrix[1][2] * matrix[2][0];
	double term3 = matrix[0][2] * matrix[1][0] * matrix[2][1];

	double term4 = matrix[0][0] * matrix[1][2] * matrix[2][1];
	double term5 = matrix[0][1] * matrix[1][0] * matrix[2][2];
	double term6 = matrix[0][2] * matrix[1][1] * matrix[2][0];

	determinant = term1 + term2 + term3 - term4 - term5 - term6;
	return determinant;
}

double calculate_determinant_1 (double element1, double element2, double element3, double element4, double element5, double element6, double element7, double element8, double element9)
{
	double determinant;
	double term1 = element1 * element5 * element9;
	double term2 = element2 * element6 * element7;
	double term3 = element3 * element4 * element8;

	double term4 = element1 * element6 * element8;
	double term5 = element2 * element4 * element9;
	double term6 = element3 * element5 * element7;

	determinant = term1 + term2 + term3 - term4 - term5 - term6;
	return determinant;
}

// This function solve a three variable equation with matrix operation (Ax = y) based on Cramer's Rule: xi = det(Ai) / det(A)
vector<double> solve_3_variable_equations (double (&mc)[3][3], double* mr)   // [mc] * [s] = [mr]
{
	vector<double> solution;
	double determinant = calculate_determinant (mc);

	if (determinant == 0)   // no solution
	{
		cout << "determine is zero, no solution" << endl;
		return solution;    // return empty vector
	}

	double temp_solution;
	double temp_matrix [3][3];

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			if (j == 0)
			{
				temp_matrix[i][j] = mr[i];
			}
			else
			{
				temp_matrix[i][j] = mc[i][j];
			}
		}
	}

	temp_solution = (calculate_determinant (temp_matrix)) / determinant;
	solution.push_back (temp_solution);

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			if (j == 1)
			{
				temp_matrix[i][j] = mr[i];
			}
			else
			{
				temp_matrix[i][j] = mc[i][j];
			}
		}
	}
	temp_solution = (calculate_determinant (temp_matrix)) / determinant;
	solution.push_back (temp_solution);

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			if (j == 2)
			{
				temp_matrix[i][j] = mr[i];
			}
			else
			{
				temp_matrix[i][j] = mc[i][j];
			}
		}
	}
	temp_solution = (calculate_determinant (temp_matrix)) / determinant;
	solution.push_back (temp_solution);

	cout << "executing" << endl;
	return solution;
}

typedef struct circle   //  (x-h)^2 + (y-k)^2 = r^2
{
	double h;
	double k;
	double r;
} Circle;

// This function takes some points and return a best fit circle to given points 
// if the number of points given is three, then the circle returned will pass through all given points
// reference : http://www.had2know.com/academics/best-fit-circle-least-squares.html
Circle fit_with_circle (vector<Vector2> path)     //  (x-h)^2 + (y-k)^2 = r^2
{
	Circle circle;

	double A, B, C;              // A = 2 * h; B = 2 * k; C = r^2 - h^2 - k^2;

	double matrix_coef[3][3];
	double matrix_right[3];

	vector<double> x;
	vector<double> y;
	vector<double> xy;    // for storing terms x*y
	vector<double> x2;    // for storing terms x^2
	vector<double> y2;    // for storing terms y^2
	// for storing right hand side
	vector<double> right0; 
	vector<double> right1;
	vector<double> right2;

	double product, squareX, squareY;
	// fill the vectors so that they can passed to get their sums
	for (int i=0; i<path.size(); i++)
	{
		product = (path[i]).x * (path[i]).y;
		squareX = (path[i]).x * (path[i]).x;
		squareY = (path[i]).y * (path[i]).y;
		
		x.push_back ((path[i]).x);
		y.push_back ((path[i]).y);
		xy.push_back (product);
		x2.push_back (squareX);
		y2.push_back (squareY);
		
		right0.push_back (path[i].x * (squareX + squareY)); 
		right1.push_back (path[i].y * (squareX + squareY));
		right2.push_back (squareX + squareY);
		
	}

	// taking sum and define each element, see my lab book forth page for reference
	matrix_coef[0][0] = Sigma_sum (x2);
	matrix_coef[0][1] = Sigma_sum (xy);
	matrix_coef[0][2] = Sigma_sum (x);

	matrix_coef[1][0] = Sigma_sum (xy);
	matrix_coef[1][1] = Sigma_sum (y2);
	matrix_coef[1][2] = Sigma_sum (y);

	matrix_coef[2][0] = Sigma_sum (x);
	matrix_coef[2][1] = Sigma_sum (y);
	matrix_coef[2][2] = path.size();

	matrix_right[0] = Sigma_sum (right0);
	matrix_right[1] = Sigma_sum (right1);
	matrix_right[2] = Sigma_sum (right2);
	
	// generate output
	vector<double> abc;
	abc = solve_3_variable_equations(matrix_coef, matrix_right);
	
	if (abc.size() == 0)
	{
		cout << "program failed" << endl;
		return circle;
	}
	
	A = abc[0];
	B = abc[1];
	C = abc[2];
	cout << A << " , " << B << " , " << C << endl;

	// conversion
	double h, k, r;
	h = - A / 2;
	k = - B / 2;
	r = (sqrt (4*C + A*A + B*B)) / 2;

	// output circle
	circle.h = h;
	circle.k = k;
	circle.r = r;
	cout << h << " , " << k << " , " << r << endl;
	return circle;
}


#endif




























