#include "CameraInterface.h"
#include "HTTPInterface.h"
#include "RobotInterfaceRovio.h"

#include <stdio.h>
//#include <tchar.h>
#include "Blob.h"
#include "BlobResult.h"
#include "visilibity.hpp"
#include "orientation.h"

#include <fstream>
#ifndef _WIN32
#include <unistd.h>
#endif

#define dprintf if (DEBUG) printf

bool escape = true;
CvMemStorage *gStorage = NULL;
IplImage* background = NULL;
IplImage* ObstacleBackground = NULL;
IplImage* visibility_graph_image = NULL;
IplImage* GoalObstacleBackground = NULL;

int numCameras=0;
char cameraName[20][1000];
char cameraURL[20][50000];
CvPoint goal1;
CvPoint goal2;
CvPoint enemyPos;
CvPoint goalPoint; // for goal line
bool enemyFound;
vector<CvPoint> enemyPositions;

struct ObjectPos {
	CvPoint fruitPos, robotPos;
	int robotOrientation;
	bool found;
};

// try once to find the fruit and robot
// result.found is true if found
static ObjectPos find_objects(bool find_fruit);

// loop until find_objects finds the robot
static ObjectPos find_robot();

#pragma mark -- movement

static int distance(int x1, int y1, int x2, int y2)
{
    int xd = x1-x2;
    int yd = y1-y2;
    return xd*xd+yd*yd;
}

static int pdistance(CvPoint a, CvPoint b)
{
    return distance(a.x, a.y, b.x, b.y);
}

namespace movement {
    static float angle_towards(int x1, int y1, int x2, int y2)
    {
        int xv = x2 - x1;
        int yv = y2 - y1;
        
        float res = atan2f(yv, xv);
        
        res = (2*M_PI) - res;
        res = fmodf(res, 2*M_PI);
        return res * (180./M_PI);
    }
    
    // 1 if b is CCW of a
    // 0 if b is CW of a
    static int compare_angle(float a, float b)
    {
        int ret;
        
        if ((a <= 180 && b <= 180) || (a >= 180 && b >= 180)) {
            // b, a both top
            // ccw - b > a
            ret = b > a;
        } else {
            if (b >= a) {
                ret = (360 + b) >= a;
            } else
                ret = (360 + a) >= b;
        }
        
        dprintf("a %f b %f ccw %d\n", a, b, ret);
        
        return ret;
    }
    
    static void _turn_to(float curAngle, float wantAngle)
    {
        float maxA, minA;
        RovioTurn turn;
        float diff, fn;
        int n;
        float lastCurAngle = curAngle;
        
        if (compare_angle(curAngle, wantAngle)) {
            turn = TurnLeft;
        } else {
            turn = TurnRight;
        }
        
        if (curAngle > wantAngle) {
            maxA = curAngle;
            minA = wantAngle;
        } else {
            maxA = wantAngle;
            minA = curAngle;
        }
        diff = maxA - minA;
        
        fn = diff / 15.;
        n = fn + .5;
        
        dprintf(">> turn %f -> %f (%f degrees, dir %d = %f turns = cmd %d)\n", curAngle, wantAngle, diff, (int)turn, fn, n);
        rovio_turn(turn, n);
        
        do {
            RovioTurn cturn;
            ObjectPos pos;
            
            // wait to find the robot
            pos = find_robot();
            lastCurAngle = curAngle;
            curAngle = pos.robotOrientation;
            if (compare_angle(curAngle, wantAngle)) {
                cturn = TurnLeft;
            } else {
                cturn = TurnRight;
            }
            
            if (abs(lastCurAngle - curAngle) > 60.) {
                dprintf("-- noisy orientation reading, retry\n");
                continue;
            }
            
            if (turn != cturn) {
                dprintf("-- overturned (turn = %d cturn = %d) (%f -> %f), break\n", turn, cturn, curAngle, wantAngle);
                break;
            }
            
            if ((abs(curAngle - wantAngle) < 10 || abs((360+curAngle) - wantAngle) < 10)) {
                dprintf("-- can't improve turn, break (cur = %f want = %f)\n", curAngle, wantAngle);
                break;
            }
            
            dprintf("correction turn %f -> %f (%f degrees, dir %d)\n", curAngle, wantAngle, diff, (int)turn);
            rovio_turn_small(turn);
        } while (1);
        
        dprintf("<< turned, final angle %f wanted %f\n", curAngle, wantAngle);
    }
    
    // we're already turned as best we can
    // just drive forwards until past it
    static void _drive_to(int wantX, int wantY)
    {
        int cdistance, last_distance = INT_MAX;
        ObjectPos pos = find_robot();
        
        printf(">> drive from %d,%d to %d,%d\n", pos.robotPos.x, pos.robotPos.y, wantX, wantY);
        
        cdistance = distance(pos.robotPos.x, pos.robotPos.y, wantX, wantY);
        
		
        do {
            dprintf("distance = %f, driving forward\n", sqrt((double)cdistance));
            rovio_drive(5, DirForward);
            
            pos = find_robot();
            
            // avoid iterating forever due to noise
            if (abs(last_distance - cdistance) > 10)
                last_distance = cdistance;
            
            cdistance = distance(pos.robotPos.x, pos.robotPos.y, wantX, wantY);
            dprintf("distance now = %f\n", sqrt((double)cdistance));
        } while ((last_distance - cdistance) > 10);
        
        printf("<< stopped, distance increased to %f (pos %d, %d)\n", sqrt((double)cdistance), pos.robotPos.x, pos.robotPos.y);
    }
    
    static void _drive_to_point(VisiLibity::Point p)
    {
        int did_drive;
        int should_redo_turn = 1;
        
        do {
            int next_x = p.x(), next_y = p.y();
            ObjectPos pos = find_robot();
            // FIXME this find_robot() is just for termination checks which could be merged into sub-functions
            
            int cur_x = pos.robotPos.x, cur_y = pos.robotPos.y;
            float angle;
            
            did_drive = 0;
            
            do {
                angle = angle_towards(cur_x, cur_y, next_x, next_y);
                float curAngle = pos.robotOrientation;
                
                if (!should_redo_turn)
                    break;
                
                if ((abs(curAngle - angle) < 10 || abs((360+curAngle) - angle) < 10))
                    break;
                
                _turn_to(curAngle, angle);
            } while (0);
            
            do {
                if (distance(cur_x,cur_y,next_x,next_y) <= 60*60)
                    break;
                
                did_drive = 1;
                
                _drive_to(next_x, next_y);
            } while (0);
            
            if (!did_drive) break;
            
            // check the angle of the distance
            // if it flipped, we crossed the center point
            // if it didn't, we drove off somewhere - turn around and try again
            float new_angle = angle_towards(pos.robotPos.x, pos.robotPos.y, next_x, next_y);
            float diff = new_angle - angle;
            
            // diff should be ~= 180 degrees if we can stop
            should_redo_turn = abs(diff - 180.) >= 10. && abs(diff) >= 10.;
            dprintf("-- after stopping, old angle = %f, new angle = %f, should be inverse, retry %d, didDrive %d\n", angle, new_angle, should_redo_turn, did_drive);
        } while (should_redo_turn);
    }
    
    // drives robot (robotPos) to the goal
    static void drive_to_goal(std::vector<VisiLibity::Point> path)
    {
        using namespace VisiLibity;
        for (int i = 1; i < path.size(); i++) {
            Point p = path[i];
            
            _drive_to_point(p);
            return;
        }
    }
}

#pragma mark -- visibility graph

static IplImage *same_size_image_8bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
}

static IplImage *same_size_image_24bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 3);
}

static void draw_rect(IplImage *frame, CvRect r, int color)
{
    // drawing
    CvPoint pt1, pt2;
    const CvScalar colors[] = {
        CV_RGB(255, 0, 0), // face red
        CV_RGB(0, 0, 255), // blob blue
        CV_RGB(255, 255, 255), // chosen white
    };
    
    pt1.x = r.x;
    pt2.x = r.x + r.width;
    pt1.y = r.y;
    pt2.y = r.y + r.height;
    
    //printf("rect: x %d y %d w %d h %d\n", r.x, r.y, r.width, r.height);
    
    cvRectangle(frame, pt1, pt2, colors[color], 3, 8, 0);
}

static void draw_point(IplImage *frame, int x, int y, int color)
{
    // drawing
    CvPoint pt1, pt2;
    const CvScalar colors[] = {
        CV_RGB(255, 0, 0), // face red
        CV_RGB(0, 0, 255), // chosen white
    };
    
    pt1.x = x - 5;
    pt2.x = x + 5;
    pt1.y = y - 5;
    pt2.y = y + 5;
    
    cvRectangle(frame, pt1, pt2, colors[color], 3, 8, 0);
}

namespace visibility {
    // private
	std::vector<CvBox2D> permanent_obstacles;
	VisiLibity::Environment *environment;
	
    // returns input image (obstacles highlighted)
    // fills out global variable obstacleboxes
    static void mark_obstacle_boxes(IplImage *obstacles)
    {
        IplImage *grey = same_size_image_8bit(obstacles);
        
        cvCvtColor(obstacles, grey, CV_BGR2GRAY);    
        
        CBlobResult blobs = CBlobResult(grey, NULL, 10, true);
        
        blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
        blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 250000);
        
        if (DEBUG) blobs.PrintBlobs("/dev/stdout");
        
        for (int i = 0; i < blobs.GetNumBlobs(); i++)
        {
            CBlob blob = blobs.GetBlob(i);
            CvBox2D box = blob.GetEllipse();
            CvPoint2D32f pt[4];
            
            {
                CvRect fr;
                fr.x      = blob.MinX();
                fr.width  = blob.MaxX() - fr.x;
                fr.y      = blob.MinY();
                fr.height = blob.MaxY() - fr.y;
                
                if (box.size.width > fr.width)
                    box.size.width = fr.width; // some opencv bug
                
                if (box.size.height > fr.height)
                    box.size.height = fr.height;
                
                if (fr.width >= obstacles->width && fr.height >= obstacles->height)
                    continue;
                
                //draw_rect(obstacles, fr, 1);
            }
            
            blob.FillBlob(obstacles, CV_RGB(0,255,0));
            dprintf("blob %d has %d edges\n", i, blob.edges->total);
            dprintf("%d - x %f to %f, y %f to %f\n", i, blob.MinX(), blob.MaxX(), blob.MinY(), blob.MaxY());        
            
            cvBoxPoints(box, pt);
            permanent_obstacles.push_back(box);
            
            //for (int j = 0; j < 4; j++) {
            //    cvLine(obstacles, cvPointFrom32f(pt[j]), cvPointFrom32f(pt[(j+1)%4]), CV_RGB(0, 0, 255));
            //}
        }
        
        //return obstacles;
    }
    
    static VisiLibity::Point move_point_away(VisiLibity::Point p)
    {
        // find the obstacle whose corner this point is on
        // then move the point away from its center
        // FIXME does not handle temporary obstacles - maybe put those in the list
        float cx=0, cy=0;
        
        for (int i = 0; i < permanent_obstacles.size(); i++) {
            CvBox2D box = permanent_obstacles[i];
            CvPoint2D32f pt[4];
            
            cvBoxPoints(box, pt);
            
            for (int j = 0; j < 4; j++) {
                float px = pt[j].x, py = pt[j].y;
                
                if (abs(px - p.x()) < 2. && abs(py - p.y()) < 2.) {
                    cx = box.center.x;
                    cy = box.center.y;
                    goto done;
                }
            }
        }
        
        return p;
        
    done:
        float cxv = p.x() - cx, cyv = p.y() - cy;
        float r = sqrtf(cxv * cxv + cyv * cyv);
        cxv /= r;
        cyv /= r;
        
        cxv *= 100;
        cyv *= 100;
        
        return VisiLibity::Point(p.x() + cxv, p.y() + cyv);
    }
    
    // input - visibility_mark_obstacle_boxes
    // output - visibility graph drawn over image
    // fills out global variable visibility
    static void make_visibility_graph(int width, int height)
    {
        using namespace VisiLibity;
        
        {
            Point rp[4] = {
                Point(0, 0),
                Point(width, 0),
                Point(width, height),
                Point(0, height)};
            std::vector<Point> rpv(rp, rp+4);
            VisiLibity::Polygon boundary(rpv);
            
            environment = new Environment(boundary);
        }
        
        visibility_graph_image = same_size_image_24bit(ObstacleBackground);
        cvSet(visibility_graph_image, cvScalar(0,0,0));

        for (int i = 0; i < permanent_obstacles.size(); i++) {
            CvBox2D box = permanent_obstacles[i];
            CvPoint2D32f pt[4];
            Point pp[4];
            
            cvBoxPoints(box, pt);
            
            for (int j = 0; j < 4; j++) {
                pp[j] = Point(pt[j].x, pt[j].y);
            }
            
            std::vector<Point> pv(pp, pp+4);
            VisiLibity::Polygon hole(pv);
            environment->add_hole(hole);
            
            for (int j = 0; j < 4; j++) {
                cvLine(visibility_graph_image, cvPointFrom32f(pt[j]), cvPointFrom32f(pt[(j+1)%4]), CV_RGB(0, 0, 255));
            }
        }
        
        environment->enforce_standard_form();
        
        //printf("vis valid %d area %f dia %f\n", visibility->is_valid(), visibility->area(), visibility->diameter());
        
        //printf("drawing visibility graph\n");
        
        int nlines = 0, nvertices = 0;
        
        {
            Visibility_Graph graph(*environment);
            nvertices = graph.n();
            
            for (int i = 0; i < nvertices; i++) {
                Point &p1 = (*environment)(i);
                for (int j = 0; j < nvertices; j++) {
                    Point &p2 = (*environment)(j);
                    if (!graph(i, j) || (j > i && graph(j, i))) continue;
                    
                    nlines++;
                    
                    CvPoint cp1, cp2;
                    cp1.x = p1.x();cp1.y = p1.y();
                    cp2.x = p2.x();cp2.y = p2.y();
                    
                    cvLine(visibility_graph_image, cp1, cp2, CV_RGB(255, 0, 255));
                }
            }
            
        }
        
        cvShowImage("visibility graph", visibility_graph_image);
        //printf("%d vertices, %d edges in graph\n", nvertices, nlines);        
    }
    
    // before - call make_visibility_graph
    // output - image with path drawn on it
    // global variable path_to_goal
    static std::vector<VisiLibity::Point> find_robot_path(CvPoint robotPos, CvPoint targetPos)
    {
        using namespace VisiLibity;
        
        std::vector<Point> path_to_goal;
        
        //printf("drawing shortest path\n");
        Point robotp(robotPos.x, robotPos.y);
        Point goalp(targetPos.x, targetPos.y);
        VisiLibity::Polyline path = environment->shortest_path(robotp, goalp, 5);
        
        for (int i = 0; i < path.size(); i++) {
            Point p = move_point_away(path[i]);
            path_to_goal.push_back(p);
        }
        
        for (int i = 0; i < (path.size()-1); i++) {
            Point p  = path_to_goal[i];
            Point p1 = path_to_goal[i+1];
            CvPoint cp1, cp2;
            cp1.x = p.x(); cp1.y = p.y();
            cp2.x = p1.x();cp2.y = p1.y();
            
            dprintf("p #%d: x %d y %d -> x %d y %d\n", i, cp1.x, cp1.y, cp2.x, cp2.y);
            
            cvLine(visibility_graph_image, cp1, cp2, CV_RGB(255, 255, 255), 4, 8, 0);
        }
        
        cvShowImage("visibility graph", visibility_graph_image);
        
        //return visibility_graph_image;
        return path_to_goal;
    }
}

#pragma mark -- vision

static IplImage *fetch_camera_image()
{
    IplImage *img;
#if 1
    http_fetch(cameraURL[0],"Data/Camera0.jpg");
    img=cvLoadImage("Data/Camera0.jpg");
    
    cvReleaseImage(&img);
    http_fetch(cameraURL[0],"Data/Camera0.jpg");
    img=cvLoadImage("Data/Camera0.jpg");
#else
    static CvCapture *cap = cvCaptureFromCAM(1);
    cvGrabFrame(cap);
    IplImage *img2 = cvQueryFrame(cap);
    img = cvCloneImage(img2);
    cvReleaseImage(&img2);
#endif
    
    return img;
}

static void setBackground(){
	int key;
	IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        img = fetch_camera_image();
        
        cvShowImage("Display", img);
        if(key == '1')
            break;
        cvReleaseImage(&img);
	}
	
	printf("1: Set background\n");
	
    background = cvCloneImage(img);
	cvSmooth(img, background, CV_BILATERAL,5,5,50,50);
	cvReleaseImage(&img);
}

#define pixRGB(img, x, y, c) ((uchar *)(img->imageData + y*img->widthStep))[x*img->nChannels + c]

static void resetGoalObstacleBackground(CvPoint robot, CvPoint enemy, CvPoint goal1, CvPoint goal2){
    int key, backr, backg, backb;
    IplImage* img = NULL;
	img = fetch_camera_image();
    
	//while(1){
    //  key = cvWaitKey(30);
    
    //cvShowImage("Display", img);
    //if(key == '2')
    //   break;
    //cvReleaseImage(&img);
	//}
    IplImage* temp = cvCloneImage(GoalObstacleBackground);
    GoalObstacleBackground = cvCloneImage(img);
	cvSmooth(img, GoalObstacleBackground, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&img);
    img = cvCloneImage(GoalObstacleBackground);
    
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r, g, b;
			r = pixRGB(img, j, i, 2);
			g = pixRGB(img, j, i, 1);
			b = pixRGB(img, j, i, 0);
			
			backr = pixRGB(background, j, i, 2);
			backg = pixRGB(background, j, i, 1);
			backb = pixRGB(background, j, i, 0);			
            
            //now we have pulled out the rgb values in the original background and the current image.
			if((i>goal1.x-25)&&(i<goal1.x+25)&&(i>goal1.y-25)&&(i<goal1.y+25)||
               (i>goal2.x-25)&&(i<goal2.x+25)&&(i>goal2.y-25)&&(i<goal2.y+25)){
				//if the pixel is near the goal, we just copy in our old goal images
                pixRGB(GoalObstacleBackground, j, i, 2) = pixRGB(temp, j, i, 2);
                pixRGB(GoalObstacleBackground, j, i, 1) = pixRGB(temp, j, i, 1);
                pixRGB(GoalObstacleBackground, j, i, 0) = pixRGB(temp, j, i, 0);
			}	
			else if((i>robot.x-25)&&(i<robot.x+25)&&(i>robot.y-25)&&(i<robot.y+25)||
                    (i>enemy.x-25)&&(i<enemy.x+25)&&(i>enemy.y-25)&&(i<enemy.y+25))
				//(i>goal1.x-25)&&(i<goal1.x+25)&&(i>goal1.y-25)&&(i<goal1.y+25)||
				//(i>goal2.x-25)&&(i<goal2.x+25)&&(i>goal2.y-25)&&(i<goal2.y+25))
            {
                pixRGB(GoalObstacleBackground, j, i, 2) = backr;
                pixRGB(GoalObstacleBackground, j, i, 1) = backg;
                pixRGB(GoalObstacleBackground, j, i, 0) = backb;
            }
			else
			{
				int diff = (r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb);
                
				diff = sqrt((double)diff);
				if (diff > 255) diff = 255;
				if (diff < 0) diff = 0;
				
                if (diff <= 100) diff = 0;
                if (diff >= 120) diff = 255;
                
                pixRGB(img, j, i, 2) = pixRGB(img, j, i, 1) = pixRGB(img, j, i, 0) = diff;
			}
		}
	}
    
    cvErode(img, img);
    cvDilate(img, img);
    
    visibility::mark_obstacle_boxes(img);
    visibility::make_visibility_graph(img->width, img->height);    
}

static void setGoalObstacleBackground(){
	int key, backr, backg, backb;
	IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        img = fetch_camera_image();
        
        cvShowImage("Display", img);
        if(key == '3')
            break;
		cvReleaseImage(&img);
	}
	
	printf("3: Set goals\n");
	
    GoalObstacleBackground = cvCloneImage(img);
	cvSmooth(img, GoalObstacleBackground, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&img);
    img = cvCloneImage(GoalObstacleBackground);
    
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
			int r, g, b;
			r = pixRGB(img, j, i, 2);
			g = pixRGB(img, j, i, 1);
			b = pixRGB(img, j, i, 0);
			
			backr = pixRGB(ObstacleBackground, j, i, 2);
			backg = pixRGB(ObstacleBackground, j, i, 1);
			backb = pixRGB(ObstacleBackground, j, i, 0);			
            
            int diff = (r-backr)*(r-backr) + (g-backg)*(g-backg) + (b-backb)*(b-backb);
            
            diff = sqrt((double)diff);
            if (diff > 255) diff = 255;
            if (diff < 0) diff = 0;
            
            if (diff <= 100) diff = 0;
			if (diff >= 120) diff = 255;
            
			pixRGB(img, j, i, 2) = pixRGB(img, j, i, 1) = pixRGB(img, j, i, 0) = diff;
		}
	} // we have now set our images to be pure red wherever the goals are. We need to perform blob detection
	
	IplImage *gray = same_size_image_8bit(img);
	
	cvCvtColor(img, gray, CV_BGR2GRAY);
	
	//and use the the two blobs as the locations of our goals.
	
	CBlobResult goals = CBlobResult(gray, NULL,0,true);
	
	//goals.PrintBlobs("/dev/stdout");
	
	goals.Filter(goals, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
	goals.Filter(goals, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
	
	//goals.PrintBlobs("/dev/stdout");
	
	for(int i = 0; i < goals.GetNumBlobs(); i ++){
        CBlob goalArea = goals.GetBlob(i);
        CvBox2D goalEllipse = goalArea.GetEllipse();
		CvPoint gp = cvPoint(goalEllipse.center.x, goalEllipse.center.y);
		int n = 0;
		
        if(goalArea.Area() > 50)
        { 
			if( i == 0){
				goal1.x = goalEllipse.center.x;
				goal1.y = goalEllipse.center.y;
				n = 1;
			}
			if(i == 1){
				goal2.x = goalEllipse.center.x;
				goal2.y = goalEllipse.center.y;
				n = 2;
			}
		}
		
		if (n == 0)
		    cvCircle(GoalObstacleBackground, gp, 20, CV_RGB(0, 250, 0), 2, 1);
		else if (n == 1)
		    cvCircle(GoalObstacleBackground, gp, 20, CV_RGB(250, 0, 0), 2, 1);
        else
            cvCircle(GoalObstacleBackground, gp, 20, CV_RGB(0, 0, 250), 2, 1);
        
		cvShowImage("goals", GoalObstacleBackground);
	}
}

static void setObstacleBackground(){
    int key, backr, backg, backb;
    IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        img = fetch_camera_image();
        
        cvShowImage("Display", img);
        if(key == '2')
            break;
		cvReleaseImage(&img);
	}
    
	printf("2: Set obstacles\n");

    ObstacleBackground = cvCloneImage(img);
	cvSmooth(img, ObstacleBackground, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&img);
    img = cvCloneImage(ObstacleBackground);
    
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
            int r, g, b;
			r = pixRGB(img, j, i, 2);
			g = pixRGB(img, j, i, 1);
			b = pixRGB(img, j, i, 0);
			
			backr = pixRGB(background, j, i, 2);
			backg = pixRGB(background, j, i, 1);
			backb = pixRGB(background, j, i, 0);
            
            int diff = (r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb);
            
            diff = sqrt((double)diff);
            if (diff > 255) diff = 255;
            if (diff < 0) diff = 0;
            
            if (diff <= 100) diff = 0;
			if (diff >= 120) diff = 255;
            
			pixRGB(img, j, i, 2) = pixRGB(img, j, i, 1) = pixRGB(img, j, i, 0) = diff;
		}
	}
    
    cvErode(img, img);
    cvDilate(img, img);
    
    visibility::mark_obstacle_boxes(img);
    visibility::make_visibility_graph(img->width, img->height);    
}

static int HSV_filter1(int h, int s, int v, int threshold) {
    //	printf("H: %d S: %d V: %d \n", h, s, v);
    //	int FilteredColor[3] = {200, 250, 10}; // This one is Orange HSV
	int FilteredColor[3] = {200, 20,200}; //this works for pink, but not white.
	int diff =  (FilteredColor[0]-h)*(FilteredColor[0]-h) +
    (FilteredColor[1]-s)*(FilteredColor[1]-s) + 
    (FilteredColor[2]-v)*(FilteredColor[2]-v);
	
	if(diff < threshold) return abs(diff-threshold); /** If here, it has passed! */
	return 0; /** With 0 this is discarded */
}

static int RGB_filter1(int r, int g, int b, int threshold){
	//int FilteredColor[3] = {190, 190, 75}; //the RGB values for bright yellow.
	int FilteredColor[3] = {10, 110, 60};//the filter for green
    int black = (r+g+b) < 25;
    if (black) return 0;
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
    (FilteredColor[1] - g)*(FilteredColor[1] - g)+
    (FilteredColor[2] - b)*(FilteredColor[2] - b);
    
    diff = sqrt((double)diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    
    if (diff <= 180) diff = 0;
    
    return diff;
}

static int RGB_filter2(int r, int g, int b, int threshold){
	int FilteredColor[3] = {40, 60, 160}; //the RGB values for the blue felt.
    
    int black = (r+g+b) < 25;
    if (black) return 0;
    
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
    (FilteredColor[1] - g)*(FilteredColor[1] - g)+
    (FilteredColor[2] - b)*(FilteredColor[2] - b);
    
    diff = sqrt((double)diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    
    if (diff <= 180) diff = 0;
    
    return diff;
}

static int RGB_filter3(int r, int g, int b, int threshold){
	//int FilteredColor[3] = {240, 70, 120}; //the RGB values for teh apple.
	int FilteredColor[3] = {253, 249, 149}; // the RGB values for teh lemon
    
    int black = (r+g+b) < 25;
    if (black) return 0;
    
	int diff = (FilteredColor[0] - r)*(FilteredColor[0]-r)+
    (FilteredColor[1] - g)*(FilteredColor[1] - g)+
    (FilteredColor[2] - b)*(FilteredColor[2] - b);
    
    diff = sqrt((double)diff);
    if (diff > 255) diff = 255;
    if (diff < 0) diff = 0;
    diff = 255 - diff;
    return diff;
}

void newEnemyPos(CvPoint place){
	vector<CvPoint> newplaces;
	if(enemyPositions.size() < 5){
		enemyPositions.push_back(place);
		enemyPos = place;
		enemyFound = true;
	}
	else{
		enemyFound = false;
		if((enemyPositions.at(4).x > (place.x - 300))&&(enemyPositions.at(4).x < (place.x + 300))&&
           (enemyPositions.at(4).y > (place.y - 300))&&(enemyPositions.at(4).y < (place.y + 300))){
            enemyPos = place;
            enemyFound = true;
		}
        
		for(int i = 1; i < 5; i ++){
			newplaces.push_back(enemyPositions.at(i));
		}
		newplaces.push_back(place);
		enemyPositions = newplaces;
	}
    
}
// finds robot and fruit
static ObjectPos find_objects(bool find_fruit)
{
	IplImage* input;
    IplImage* img, *imgCopy;
    IplImage* i1;
	IplImage* rob;
	IplImage* enemy;
	//IplImage* fruit;
	
	ObjectPos pos;
    CvPoint Left, Right;
	pos.found = 0;
    
    input = fetch_camera_image();
    //cvDestroyWindow("Display");
    cvWaitKey(1);
    
	img = cvCloneImage(input);
	//hsv_img = cvCloneImage(img);
	//bw_img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	rob = cvCreateImage(cvSize(img->width, img->height), 8,1);
	enemy = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	i1 = cvCreateImage(cvSize(img->width, img->height), 8, 1);
    cvZero(i1);
	cvZero(rob);
	cvZero(enemy);
    // Smooth input image using a Gaussian filter, assign HSV, BW image
    imgCopy = cvCloneImage(img);
    cvSmooth(imgCopy, img, CV_BILATERAL,5,5,50,50);
    cvReleaseImage(&imgCopy);
    
	for(int i = 0; i < img->height; i ++){
		for(int j = 0; j < img->width; j++){
            int r, g, b, backr, backg, backb;
			r = pixRGB(img, j, i, 2);
			g = pixRGB(img, j, i, 1);
			b = pixRGB(img, j, i, 0);
			
			backr = pixRGB(GoalObstacleBackground, j, i, 2);
			backg = pixRGB(GoalObstacleBackground, j, i, 1);
			backb = pixRGB(GoalObstacleBackground, j, i, 0);
            
			if(((r-backr)*(r-backr) + (g-backg)*(g-backg)+ (b-backb)*(b-backb)) < 1000){
                pixRGB(img, j, i, 2) = pixRGB(img, j, i, 1) = pixRGB(img, j, i, 0) = 0;
			}
		}
	}
    
	//cvShowImage("Background subtracted", img);
	for(int i = 0; i < img->height; i++) {
		for(int j = 0; j < img->width; j++) {
            int r, g, b;
			r = pixRGB(img, j, i, 2);
			g = pixRGB(img, j, i, 1);
			b = pixRGB(img, j, i, 0);
            
            ((uchar *)(i1->imageData + i*i1->widthStep))[j] = RGB_filter1(r, g, b, 6000);
			((uchar *)(rob->imageData + i*rob->widthStep))[j] = RGB_filter2(r, g, b, 6000);
            //((uchar *)(fruit->imageData + i*fruit->widthStep))[j] = RGB_filter3(r,g,b, 6000);
			if(((r > 10)||(g > 10)||(b > 10)) && (RGB_filter1(r,g,b, 6000) == 0) && (RGB_filter2(r,g,b, 6000) ==0)){
				((uchar *)(enemy->imageData + i*enemy->widthStep))[j] = 255;
			}
		}
    }
    
    int found_robot=0, found_fruit=0;
    
    cvErode(i1, i1, NULL, 2);
    cvErode(rob, rob, NULL, 2);
    cvErode(enemy, enemy);
    
    /*
     cvShowImage("robotLeft", rob);
     cvShowImage("robotRight", i1);
     
     cvShowImage("fruit", fruit);
     */
    
    CBlobResult robBlobs = CBlobResult(rob, NULL, 0, true);
    CBlobResult blobs = CBlobResult(i1, NULL, 0, true);
    //CBlobResult fruitBlob = CBlobResult(fruit, NULL, 0, true);
    CBlobResult enemys = CBlobResult(enemy, NULL, 0, true);
    
    //fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetMean(), B_GREATER, 120);
    //fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    //fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
    
    blobs.Filter(blobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 130);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
    
	enemys.Filter(enemys, B_INCLUDE, CBlobGetMean(), B_GREATER, 130);
	enemys.Filter(enemys, B_INCLUDE, CBlobGetArea(), B_GREATER, 50);
	enemys.Filter(enemys, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
    
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 130);
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    robBlobs.Filter(robBlobs, B_INCLUDE, CBlobGetArea(), B_LESS, 6000);
	
	CBlob test;
    // find right side of robot
    for (int i = 0; i < blobs.GetNumBlobs(); i++ )
    {
		test = blobs.GetBlob(i); // just used to initialize the variable.
		CBlob blobArea = blobs.GetBlob(i);
        CvBox2D BlobEllipse = blobArea.GetEllipse();
        CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
        
        Right = centrum;
        found_robot = 1;
        if(blobArea.Area() > 10)
        { blobArea.FillBlob(input, cvScalar(255, 0, 0));
            cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
    }
	
	enemys.GetNthBlob(CBlobGetArea(), 0, test);
	CvBox2D testEllipse = test.GetEllipse();
	newEnemyPos(cvPoint(testEllipse.center.x, testEllipse.center.y));
    // find fruit
    /*for (int i = 0; i < fruitBlob.GetNumBlobs(); i++ )
     {
     CBlob fruitBlobArea = fruitBlob.GetBlob(i);
     CvBox2D BlobEllipse = fruitBlobArea.GetEllipse();
     CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
     
     pos.fruitPos = centrum;
     found_fruit = 1;
     if(fruitBlobArea.Area() > 10)
     { fruitBlobArea.FillBlob(input, cvScalar(255, 0, 250));
     cvCircle(input, centrum, 20, CV_RGB(250, 0, 250),2, 1);}
     }*/
    
    //robotPos.clear();
    //find other side of robot
    for (int i = 0; i < robBlobs.GetNumBlobs(); i++ )
    {
        //printf("blobbled");
        CBlob robBlobArea = robBlobs.GetBlob(i);
        CvBox2D BlobEllipse = robBlobArea.GetEllipse();
        CvPoint centrum = cvPoint(BlobEllipse.center.x, BlobEllipse.center.y);
        
        Left = centrum;
        if (found_robot==1)
            found_robot++;
        
        if(robBlobArea.Area() > 4)
        { robBlobArea.FillBlob(input, cvScalar(0, 255, 0));
            cvCircle(input, centrum, 20, CV_RGB(250, 250, 0),2, 1);}
    }
    
    if(found_robot == 2){
        cvCircle(input, Right, 20, CV_RGB(250, 250, 250),2,1);
        cvCircle(input, Left, 20, CV_RGB(127, 127, 127), 2, 1);
        cvCircle(input, pos.fruitPos, 20, CV_RGB(0, 0, 0), 2, 1);
        double temp = sqrt((double)distance(Left.x,Left.y,Right.x,Right.y));
        temp = acos((Right.x-Left.x)/temp);
        temp = (temp/M_PI)*180;
        
        if(((Right.y - Left.y) < 0) && ((Right.x-Left.x) > 0)){
            temp = 360 -temp;
        }
        else if(((Right.y - Left.y < 0) && ((Right.x - Left.x) < 0))){
            float rho = 180 - temp;
            temp = temp + 2*rho;
        }
        temp = temp + 90;
        temp = fmod(temp,360);
        if(temp > 180){
            temp = (180 + 360 - temp);
        }
        if(temp < 180){
            temp = 180 - temp;
        }
        pos.robotOrientation = temp;
        
        pos.robotPos.x = (int)(Right.x + Left.x)/2;
        pos.robotPos.y = (int)(Right.y + Left.y)/2;
    }
    
	 int drawx = pos.robotPos.x + cos(pos.robotOrientation*(PI/180))*100;
     int drawy = pos.robotPos.y - sin(pos.robotOrientation*(PI/180))*100;
     cvLine(input, pos.robotPos, cvPoint(drawx,drawy),CV_RGB(0, 250,250));
     cvLine(input, pos.robotPos, goalPoint,CV_RGB(250, 0,250));

	 /*if((goal1.x != 0)&&(goal1.y != 0))
	 {
		CvPoint p = cvPoint(drawx,drawy);
		 TriangleAlgorithm(&pos.robotPos,&p,&goal1);
		 cvLine(input,pos.robotPos, chordPoint, CV_RGB(0,0,255));
		 cvLine(input,pos.robotPos, cvPoint(drawx,drawy), CV_RGB(0,255,0));
		 cvLine(input,pos.robotPos, goal1, CV_RGB(0,255,255));
	 }*/


     cvShowImage("orientations!!!", input);

    blobs.ClearBlobs();
    cvReleaseImage(&img);
    
    //cvReleaseImage(&fruit);
    cvReleaseImage(&rob);
    robBlobs.ClearBlobs();
    cvReleaseImage(&i1);
    
    CvScalar cyan = CV_RGB(0, 250,250);
    //CvScalar lightGreen = CV_RGB(127,255,0);
    //CvScalar purple = CV_RGB(138,43,226);
    
    // here is where we are going to draw in our squares.
    /*
     if(path.size() > 0){
     int xdiff, ydiff;
     if(path.size() > 20){
     CvPoint newest, older;
     newest = path.back();
     older = path.at(path.size()-10);
     xdiff= newest.x - older.x;
     ydiff = newest.y -older.y;
     }
     else{
     xdiff = 0;
     ydiff = 0;
     }
     robotPoint = path.back();
     
     //int shapeSize = 100;
     for(int i = 0; i < path.size(); i ++){
     cvCircle(input, path.at(i),1, cyan, 2);
     }
     }
     */
    
    // cvNamedWindow("Output Image - Blob Demo", 1);
    
	{
		static int imageCount = 0;
		if(1){
			string imageName;
			std::stringstream out;
			out << "trace/trace" << imageCount++ <<".jpg";
			imageName = out.str();
			cvSaveImage(imageName.c_str(), input);
		}
	}
     
    cvShowImage("Output Image - Blob Demo", input);
    cvWaitKey(1);
    
    cvReleaseImage(&input);
    
    int sdistance = pdistance(pos.fruitPos, pos.robotPos);
    
    dprintf("found robot %d (%d,%d), angle %d, fruit %d (%d,%d), dist %f\n", found_robot, pos.robotPos.x, pos.robotPos.y, pos.robotOrientation, found_fruit, pos.fruitPos.x, pos.fruitPos.y, sqrt((double)sdistance));
    
    pos.found = found_robot==2;
	return pos;
}

static ObjectPos find_robot()
{
	ObjectPos pos;
	
	while ((pos = find_objects(false)).found == false)
	    ;
	
	return pos;	
}

static void idleAwaitingObjects()
{
 	int key;
	IplImage* img = NULL;
    
	while(1){
        key = cvWaitKey(30);
        img = fetch_camera_image();
        
        cvShowImage("Display", img);
        cvReleaseImage(&img);
        if(key == '4')
            break;
	} 
	
	printf("4: Set robot\n");
}

#pragma mark -- main loop


int selectGoal(ObjectPos pos, int foundEnemy){
	if(!foundEnemy){
		int goal1dist;
		int goal2dist;
        
		goal1dist = pdistance(pos.robotPos, goal1);
		goal2dist = pdistance(pos.robotPos, goal2);
		printf("distances 1: %d 2: %d\n", goal1dist, goal2dist);
		if(goal2dist < goal1dist){
			return 2;
		}
		else{
			return 1;
		}
	}
	else{
		//if we found the enemy, we need to find the goal that the enemy is closer to, and select the opposite.
		int goal1dist;
		int goal2dist;
		int goal1enemy;
		int goal2enemy;
		goal1dist = pdistance(pos.robotPos, goal1);
		goal2dist = pdistance(pos.robotPos, goal2);
		goal1enemy = pdistance(enemyPos, goal1);
		goal2enemy = pdistance(enemyPos, goal2);
		if(abs(goal1enemy - goal2enemy) < 50*50){
			if(goal2dist < goal1dist){
				return 2;
			}
			else{
				return 1;
			}
		}
		else{
			if(goal1dist < goal1enemy){
				return 1;
			}
			else{
				return 2;
			}
		}
	}
}

void giveOrders() 
{
    int counter = 0;
    double temp_angle = Angle;
    
	if((temp_angle >=-45) && (temp_angle <45))
	{
		printf("- forward, angle %f\n", temp_angle);
		rovio_forward(3); 
		return;
	}
    
	if((temp_angle >=-135) && (temp_angle <-45))
	{
		printf("- left, angle %f\n", temp_angle);
		rovio_driveLeft(3);
		return;
	}
	if((temp_angle >=45) &&(temp_angle < 135)){
		printf("- right, angle %f\n", temp_angle);
		rovio_driveRight(3);
		return;
	}
	else{
		printf("- back, angle %f\n", temp_angle);
		rovio_backward(3);
		return;
	}
    
	while(temp_angle >=0) 
    {
        temp_angle-=20;
        counter++;
    }
    
	if (side == DirLeft)
    {
		if(counter != 0)
		{
			rovio_turnLeftByDegree(counter);
			rovio_forward(3);
        }
        else if (side == DirCenter)
            rovio_forward(3);
        else
        {
			if(counter != 0)
			{
				rovio_turnRightByDegree(counter); 
				rovio_forward(3);
			}
        }    
	}
}

void moveToPoint(CvPoint p, ObjectPos pos){
	CvPoint arbitrary;
	
	while(pdistance(pos.robotPos, p) >= 20*20){
		arbitrary.x = (int)(cos(pos.robotOrientation*(PI/180))*20 + pos.robotPos.x);
		arbitrary.y = (int)(pos.robotPos.y - sin(pos.robotOrientation*(PI/180))*20);
		CvPoint Robot = pos.robotPos;
		CvPoint orientation = arbitrary;
		CvPoint Dest = p;
		TriangleAlgorithm(&Robot,&orientation,&Dest);
		giveOrders();
		goalPoint = p;
		pos = find_robot();
		
		printf("distance = %f\n", sqrt(pdistance(pos.robotPos, p)));
	}
}

void processCamera()
{
    static int first = 1, placed = 0, found=0, goal = -1;
	ObjectPos pos;
	
    if(first){
        first = 0;
        setBackground();
        setObstacleBackground();
		setGoalObstacleBackground();
        
        //cvShowImage("visibility graph", visibility_image);
        cvWaitKey(3);
        
        if (!placed) {
            idleAwaitingObjects();
            placed = 1;
        }
		pos = find_robot();
		int misses = 0;
		while(((pos.found != true) || (enemyFound !=true)) && (misses < 5)){
			misses ++;
			pos = find_robot();
		}
		if(misses == 5){
			while(pos.found != true){
				pos = find_robot();
			}
		}
		goal = selectGoal(pos, enemyFound);
		
		printf("returned from first\n");
	}
	
    try {
	if(escape){
		if (!found && (pos = find_objects(true)).found) {
			found = 1;
			std::vector<VisiLibity::Point> path;
			CvPoint originalPos = pos.robotPos;
            path = visibility::find_robot_path(pos.robotPos, goal==1 ? goal1 : goal2);
			//path = visibility::find_robot_path(pos.robotPos, pos.fruitPos);
			//cvShowImage("visibility graph", visibility_image);
			cvWaitKey(3);
			//rovio_camera_height(middle);
			for(int i = 0; i < path.size(); i ++){
				//int px = (int)(path.at(i)).x();
				//int py = (int)(path.at(i)).y();
                
				moveToPoint((cvPoint(path.at(i).x(), path.at(i).y())),pos);
			}
			
			printf("escaped\n");
			
			//dprintf("--\ndrive to fruit\n\n");
			//movement::drive_to_goal(path);
            
			//dprintf("--\ndrive back\n\n");
			//rovio_camera_height(low);
			//rovio_turn(TurnLeft, 10);
            
			//path = visibility::find_robot_path(pos.robotPos, originalPos);
			//movement::drive_to_goal(path);   
		}
	}
	else{
		if (!found && (pos = find_objects(true)).found) {
			found = 1;
			std::vector<VisiLibity::Point> path;
			CvPoint originalPos = pos.robotPos;
			//if(goal == 1){
			printf("nothing to do\n");
			path = visibility::find_robot_path(pos.robotPos, enemyPos);
			//movement::drive_to_goal(path);
            //of couse, the enemy will move, so we will recalculate the path.
            
			//}
			//else{
            //path = visibility::find_robot_path(pos.robotPos, goal2);
			//}
			//path = visibility::find_robot_path(pos.robotPos, pos.fruitPos);
			//cvShowImage("visibility graph", visibility_image);
			cvWaitKey(3);
			//rovio_camera_height(middle);
            
			//dprintf("--\ndrive to fruit\n\n");
			//movement::drive_to_goal(path);
            
			//dprintf("--\ndrive back\n\n");
			//rovio_camera_height(low);
			//rovio_turn(TurnLeft, 10);
            
			//path = visibility::find_robot_path(pos.robotPos, originalPos);
			//movement::drive_to_goal(path);   
		}
	}
    } catch (std::exception &e) {
        
        printf("%s", e.what());
        
        throw;
	}
}

void initCameras(){
	// Initialize all the cameras from infile
	char cname[100];
	fstream infile("Data/config-cam.txt",ios::in);
	while(!infile.eof()){
		infile >> cameraURL[numCameras];
		sprintf(cname, "Data/Camera%d.jpg", numCameras);
		sprintf(cameraName[numCameras], "Data/Camera%d.jpg", numCameras);
        
        /*	cvNamedWindow(cname);
         http_fetch(cameraURL[numCameras],cname);
         images[numCameras] = cvLoadImage(cname);
         cvShowImage(cameraName[numCameras], images[numCameras]);
         */
		numCameras++;
	}
	infile.close();
}

IplImage* images[20];

#ifdef _WIN32
void cameraThread(void *arg)
#else 
void *cameraThread(void *arg)
#endif
{
	while(true){
		for(int i=0; i<numCameras; i++){
			http_fetch(cameraURL[i],cameraName[i]);
			cvReleaseImage(&images[i]);
			images[i] = cvLoadImage(cameraName[i]);
			if (!images[i]) continue;
			cvShowImage(cameraName[i], images[i]);
		}
#ifdef _WIN32
		Sleep(100);
#else
		usleep(100);
#endif
	}
	
#ifndef _WIN32
	return NULL;
#endif
}

