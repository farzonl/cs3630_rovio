#include "orientation.h"
//#include <Core\Matrix.h>

CvPoint chordPoint;
CvPoint chordMP;
CvPoint robot;
CvPoint dest;
RovioDirection side;
double Angle;

//Matrix3d a;
//Vector3d temp;

CvPoint midPoint(CvPoint* a, CvPoint* b)
{
	double mx = ( b->x + a->x)/2.0;
    double my =( b->y  + a->y)/2.0;
	return cvPoint(mx,my);
}

double distance(CvPoint* a, CvPoint* b)
{
    assert(a);
	assert(b);
    int dx = abs(b->x - a->x);
    int dy = abs(b->y - a->y);
    return sqrt((double)(dx*dx + dy*dy));
}

CvPoint getCPoint(CvPoint* robot, CvPoint* orientation, double radius)
{
		
	if((robot->x == orientation->x) &&(robot->y != orientation->y))
	{
		if(robot->y > orientation->y)
			return cvPoint(robot->x,robot->y-radius);
		else
			return cvPoint(robot->x,robot->y+radius);
	}
	else if((robot->y == orientation->y) &&(robot->x != orientation->x))
	{
		if(robot->x > orientation->x)
			return cvPoint(robot->x-radius,robot->y);
		else
			return cvPoint(robot->x+radius,robot->y);
	}
	// only case where you have a slope
	else
	{
		//distance is radius
		double dX = (orientation->x-robot->x);
		double dY = (orientation->y-robot->y);
		double slope = dY/dX;
		double theta = atan(slope);

		if ((dX<0)&&(dY<0) ||(dX<0)&&(dY>0))
		{
			theta = theta + PI;
		}
		double newX = (radius*cos(theta))+(robot->x);
		double newY = (radius*sin(theta))+(robot->y);
		return cvPoint(newX,newY);
		

			/*double slope = (orientation->y-robot->y)/(orientation->x-robot->x);
			double theta = atan(slope);
			//double theta = atan(((double)robot->y/robot->x));
			double newX = (radius*cos(theta))+(robot->x);
			double newY = -(radius*sin(theta))+(robot->y);
			return cvPoint(newX,newY);*/
			
			/*double slope = (orientation->y-robot->y)/(orientation->x-robot->x);
			double newX = radius/sqrt((slope*slope)+1);
			double newY = (radius/sqrt((slope*slope)+1))*slope;
			return cvPoint(newX,newY);*/
		
		
		//printf("FUCK");
	}
		

}



double getDir(double angle)
{
	if(robot.x > dest.x)
		side = DirLeft;
	//else if (R->getX() == L->getX()) // <--- this should be fuzzy
	else if ((angle == 0)||((angle < 20)&&(angle > 0))||((angle > -20)&&(angle < 0)))
		side = DirCenter;
	else
		side = DirRight;

	if((dest.y < robot.y)&&(dest.x < robot.x)&& (side == DirLeft))
		angle = 180 - angle; 

	if((dest.y < robot.y)&&(dest.x >= robot.x)&& (side == DirRight))
		angle = 180 - angle; 

	return angle;
}



void TriangleAlgorithm(CvPoint* Robot, CvPoint* orientation, CvPoint* Dest)
{
	assert(Robot);
	assert(orientation);
	assert(Dest);
	
	robot               =  *Robot;
    dest                =  *Dest;
	double radius       =  distance(Robot, Dest);
	chordPoint          =  getCPoint(Robot,orientation,radius);
	double chord        =  distance(&chordPoint, Dest);
	chordMP             =   midPoint(&chordPoint, Dest);
	double half         =  distance(Robot, &chordMP);
	
	double angle        = 2*asin((chord/2.0)/radius);
	if ((radius<0)&&((chord/2.0)<0) ||(radius<0)&&((chord/2.0)>0))
	{
		angle = angle + PI;
	}
    
	angle = angle* 180.0 / PI
	
	assert(isfinite(angle));
	printf("Angle: %f\n",angle);
	Angle = getDir(angle);
	assert(isfinite(angle));
	printf("Angle: %f\n",Angle);


}