#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>

#define PI 3.14159265

enum Direction {
    left = 0,
    center,
    right
};

extern CvPoint chordPoint;
extern CvPoint chordMP;
extern CvPoint robot;
extern CvPoint dest;
extern Direction side;
extern double Angle;

CvPoint getCPoint(CvPoint* robot, CvPoint* orientation, double radius);

CvPoint midPoint(CvPoint* a, CvPoint* b);

double distance(CvPoint* a, CvPoint* b);

double getDir(double angle);

void TriangleAlgorithm(CvPoint* Robot, CvPoint* orientation, CvPoint* Dest);