#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include "RobotInterfaceRovio.h"

#define PI 3.14159265

extern CvPoint chordPoint;
extern CvPoint chordMP;
extern CvPoint robot;
extern CvPoint dest;
extern RovioDirection side;
extern double Angle;

CvPoint getCPoint(CvPoint* robot, CvPoint* orientation, double radius);

CvPoint midPoint(CvPoint* a, CvPoint* b);

double distance(CvPoint* a, CvPoint* b);

double getDir(double angle);

void TriangleAlgorithm(CvPoint* Robot, CvPoint* orientation, CvPoint* Dest);