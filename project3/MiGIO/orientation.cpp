#include "orientation.h"
#include "RobotInterfaceRovio.h"
//#include <Core\Matrix.h>

CvPoint chordPoint;
CvPoint chordMP;
CvPoint robot;
CvPoint dest;
RovioDirection side;
double Angle;

int running = 0;
char moveMessage;
CvPoint savePoint;

CvPoint midPoint(CvPoint* a, CvPoint* b)
{
	double mx = ( b->x + a->x)/2.0;
    double my =( b->y  + a->y)/2.0;
	return cvPoint((int)mx,(int)my);
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
			return cvPoint((int)(robot->x),(int)(robot->y-radius));
		else
			return cvPoint((int)(robot->x),(int)(robot->y+radius));
	}
	else if((robot->y == orientation->y) &&(robot->x != orientation->x))
	{
		if(robot->x > orientation->x)
			return cvPoint((int)(robot->x-radius),(int)robot->y);
		else
			return cvPoint((int)(robot->x+radius),robot->y);
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
		return cvPoint((int)newX,(int)newY);
		

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
	chordMP             =  midPoint(&chordPoint, Dest);
	double half         =  distance(Robot, &chordMP);
	
	double slope        =  (chord/2.0)/half;
	double angle        =  atan(slope);
	
	if ((half<0) && ((chord/2.0)<0) || (half<0) && ((chord/2.0)>0))
	{
		angle = angle + PI;
	}

	angle = 2*angle* 180.0 / PI;
	
	/*double angle        = 2*asin((chord/2.0)/radius);
	if ((radius<0)&&((chord/2.0)<0) ||(radius<0)&&((chord/2.0)>0))
	{
		angle = angle + PI;
	}
    
	angle = angle* 180.0 / PI;*/
	
	//Add some ifdef win32 stuff here
	//assert(isfinite(angle));
	//printf("Angle: %f\n",angle);
	//Angle = getDir(angle);
	Angle = angle;
	//assert(isfinite(angle));
	printf("Angle: %f\n",Angle);
}

static bool angle_close(double test_angle, double want_angle)
{
	if (abs(test_angle - want_angle) <= 5) return true;
	if (abs((test_angle + 360) - want_angle) <= 5) return true;
	if (abs((test_angle - 360) - want_angle) <= 5) return true;
	return false;
}

void giveOrders(CvPoint* point) 
{
	if(!running)
	{
		savePoint = *point;
		running = 1;
	}
	if( !(point->x >= savePoint.x+100)||!(point->x <= savePoint.x-100) &&(!(point->y >= savePoint.y+100)||!(point->y <= savePoint.y-100)))
	{
		if(moveMessage == 'f')
		{
			rovio_backward(2);
			rovio_driveRight(2);
			moveMessage ='b';
		}
		
		else if(moveMessage == 'b')
		{
			rovio_forward(2);
			rovio_driveLeft(2);
			moveMessage ='f';
		}
		
		else if(moveMessage == 'e')
		{
			rovio_DiagBackLeft(2);
			rovio_forward(2);
			moveMessage ='l';
		}
		
		else if(moveMessage == 'k')
		{
			rovio_DiagBackRight(2);
			rovio_forward(2);
			moveMessage ='r';
		}
		
		else if(moveMessage == 'r')
		{
			rovio_driveLeft(2);
			rovio_backward(2);
			moveMessage ='l';
		}
		
		else if(moveMessage == 'l')
		{
			rovio_driveRight(2);
			rovio_forward(2);
			moveMessage ='r';
		}
		//Angle case
		else
		{
			rovio_driveLeft(4);
		}
		
		savePoint = *point;
	}
	
    int counter = 0;
    double temp_angle = Angle;
	
	// Forward - 0
	if(angle_close(temp_angle, 0))
	{
		rovio_forward(4);
		moveMessage = 'f'; printf("moveMessage: %c\n", moveMessage);
		return;
	}
	
	// Backward - 180
	if(angle_close(temp_angle, 180))
	{
		rovio_backward(4);
		moveMessage = 'b'; printf("moveMessage: %c\n", moveMessage);
		return;
	}
	
	// Forward Right - 360 minus 45
	if(angle_close(temp_angle, 360 - 45))
	{
		rovio_DiagForRight(4);
		moveMessage = 'e'; printf("moveMessage: %c\n", moveMessage);
		return;
	}
	
	// Forward Left - 0 plus 45
	if(angle_close(temp_angle, 45))
	{
		rovio_DiagForLeft(4);
		moveMessage = 'k'; printf("moveMessage: %c\n", moveMessage);
		return;
	}
	
	// Right - 270
	if(angle_close(temp_angle, 270))
	{
		rovio_driveRight(4);
		moveMessage = 'r'; printf("moveMessage: %c\n", moveMessage);
		return;
	}
	
	// Left - 90
	if(angle_close(temp_angle, 90))
	{		
		rovio_driveLeft(4);
		moveMessage = 'l';
		printf("moveMessage: %c\n", moveMessage);
		return;
	}
    temp_angle = abs(temp_angle); 
	
	while(temp_angle >=0) 
    {
        temp_angle-=20;
        counter++;
    }
	
	if ((side == DirLeft) || ((Angle <= 90) && (Angle >= -90)) || ((Angle <=  450) && (Angle >= 270)))
    {
		rovio_turnLeftByDegree(counter);
		moveMessage = 'T';
		rovio_forward(4);
	}
	
	else
    {
		rovio_turnRightByDegree(counter); 
		moveMessage = 'Y';
		rovio_forward(4);
	}
	
	savePoint = *point;
}

