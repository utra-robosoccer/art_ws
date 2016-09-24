#ifndef VECTOR2_H
#define VECTOR2_H

#include <math.h>

class Vector2
{
public:
    // members
    double x;
    double y;

    // functions
    Vector2 ();

    Vector2 (double x_coor, double y_coor);

    Vector2 (int x_coor, int y_coor);

    Vector2 operator+ (const Vector2 & add) const;

    Vector2 operator- (const Vector2 & substract) const;

    Vector2 Scale (double scalingFactor);
    
    Vector2 Normalize ();
    
    double Size ();
};

Vector2::Vector2 ()
{
    x = 0.0;
    y = 0.0;
}

Vector2::Vector2 (double x_coor, double y_coor)
{
    x = x_coor;
    y = y_coor;
}

Vector2::Vector2 (int x_coor, int y_coor)
{
    x = x_coor/1.000;
    y = y_coor/1.000;
}

Vector2 Vector2::operator+ (const Vector2 & add) const
{
    double x_new;
    double y_new;
    x_new = x + add.x;
    y_new = y + add.y;
    Vector2 sum (x_new, y_new);
    return sum;
}

Vector2 Vector2::operator- (const Vector2 & substract) const
{
    double x_new;
    double y_new;
    x_new = x - substract.x;
    y_new = y - substract.y;
    Vector2 difference (x_new, y_new);
    return difference;
}

double Vector2_Dot (const Vector2 u, const Vector2 v)
{
    double dotProduct;
    dotProduct= u.x * v.x + u.y * v.y;
    return dotProduct;
}

double Vector2_Cross (const Vector2 u, const Vector2 v)      // u x v, result in a vector pointing in z direction.
{
    double crossProduct = u.x * v.y - u.y * v.x;
    return crossProduct;
}

Vector2 Vector2::Scale (double scalingFactor)
{
    double x_new = x;
    double y_new = y;
    x_new *= scalingFactor;
    y_new *= scalingFactor;
    Vector2 scaledVector (x_new, y_new);
    return scaledVector;
}

double Vector2::Size ()
{
	double size;
	size = x*x + y*y;
	size = sqrt (size);
	return size;
}

Vector2 Vector2::Normalize ()
{
	Vector2 u;
	if ((*this).Size () == 0)
	{
		u = Vector2 (0,0);
		return u;
	}
	u.x = x / ((*this).Size ());
	u.y = y / ((*this).Size ());
	return u;
}

bool inRange (Vector2 startingPoint, Vector2 point, Vector2 path, double safeRange)
{
	Vector2 Vref, Vp, Up;
	Vp = path - startingPoint;
	Vref = point - startingPoint;
	Up = Vp.Normalize();
	double dotProduct = Vector2_Dot (Up, Vref);
	
	if (dotProduct < 0)
	{
		return false;
	}
	
	Up = Up.Scale(dotProduct);
	Vector2 difference;
	difference = Vref - Up;
	double distance = difference.Size();
	
	// checking
	if (distance >= safeRange)
	{
		return false;
	}
	else
	{
		return true;
	}
}

#define PI 3.141592353
double GetAngle (Vector2 v1, Vector2 v2)    // return angle in radian
{
	double crossProduct, dotProduct, magProduct;       		
	crossProduct = v1.x * v2.y - v1.y * v2.x;  	// z-compoonent of |v1 x v2|
	dotProduct = v1.x*v2.x + v1.y * v2.y;       // v1 .* v2
	magProduct = v1.Size() * v2.Size();         // |v1| * |v2|
	double Theta = 0;
	// bool counterClockwise = false;
	double sinTheta, cosTheta;
	sinTheta = crossProduct / magProduct;
	/// cosTheta = dotProduct / magProduct;
	
	Theta = asin (sinTheta);
	if (dotProduct < 0)
	{
		if (crossProduct > 0)
		{
			Theta = PI - Theta;
		}
		else
		{
			Theta = -PI - Theta;
		}
	}
	
	return Theta;
}

#endif

