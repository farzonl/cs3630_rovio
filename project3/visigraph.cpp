#include <cv.h>
#include <highgui.h>
#include "visilibity.hpp"
#include "blob.h"
#include "BlobResult.h"

/* 
 steps
 
 - creation of map
 <creation of obstacle image goes here from video>
 1. load obstacle image from disk (hack)
 2. run blob detection on red things
 3. create corner image
 4. (however this works) mark corners inside blobs, create polygons with that
 
 5. create visibility environment with empty space = everything not an obstacle
 
 6. get a graph or something
 
 - motion along map
 */

CvMemStorage *gStorage = NULL;
static std::vector<CvBox2D> obstacleboxes;
static VisiLibity::Environment *visibility;

static int robot_x = 360, robot_y = 235;
static int goal_x  = 685, goal_y  = 225;

static IplImage *same_size_image_8bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
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

// returns input image (obstacled-highlighted)
// with the visibility graph drawn on top
static IplImage *get_obstacle_image()
{
    CvCapture *filecapture = cvCaptureFromFile("robot_image_obstacles.png");
    IplImage *obstacles;
    
    cvGrabFrame(filecapture);
    obstacles = cvCloneImage(cvRetrieveFrame(filecapture, 0));
    cvReleaseCapture(&filecapture);
    
    IplImage *grey = same_size_image_8bit(obstacles);
    
    cvCvtColor(obstacles, grey, CV_BGR2GRAY);
    
    CBlobResult blobs = CBlobResult(grey, NULL, 0, true);
    //blobs.Filter(blobs, B_INCLUDE, CBlobGetMean(), B_GREATER, 10);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 20);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_LESS, 250000);

    blobs.PrintBlobs("/dev/stdout");
    
    for (int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob blob = blobs.GetBlob(i);
        CvBox2D box = blob.GetEllipse();
        CvPoint2D32f pt[4];
        
        blob.FillBlob(obstacles, CV_RGB(0,255,0));
        printf("blob %d has %d edges\n", i, blob.edges->total);
        printf("%d - x %f to %f, y %f to %f\n", i, blob.MinX(), blob.MaxX(), blob.MinY(), blob.MaxY());
        
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
                        
            draw_rect(obstacles, fr, 1);
        }
        
        cvBoxPoints(box, pt);
        obstacleboxes.push_back(box);
        
        for (int j = 0; j < 4; j++) {
            cvLine(obstacles, cvPointFrom32f(pt[j]), cvPointFrom32f(pt[(j+1)%4]), CV_RGB(0, 0, 255));
        }
    }
    
    draw_point(obstacles, robot_x, robot_y, 0);
    draw_point(obstacles, goal_x, goal_y, 1);
    
    return obstacles;
}

static void add_rect(int x, int y)
{
    using namespace VisiLibity;

    Point pp[4];
    Point p1(x-1, y-1), p2(x+1, y+1);
    
    pp[0] = p1;
    pp[1] = p1;
    pp[2] = p2;
    pp[3] = p2;
    
    pp[1].set_x(p2.x());
    pp[3].set_x(p1.x());
    
    std::vector<Point> pv(pp, pp+4);
    Polygon hole(pv);
    visibility->add_hole(hole);
}

static IplImage *get_visibility_image(IplImage *obstacle_image)
{
    using namespace VisiLibity;

    {
        Point rp[4] = {
            Point(0, 0),
            Point(obstacle_image->width, 0),
            Point(obstacle_image->width, obstacle_image->height),
            Point(0, obstacle_image->height)};
        std::vector<Point> rpv(rp, rp+4);
        Polygon boundary(rpv);
        
        visibility = new Environment(boundary);
    }
    
    for (int i = 0; i < obstacleboxes.size(); i++) {
        CvBox2D box = obstacleboxes[i];
        CvPoint2D32f pt[4];
        Point pp[4];
        
        cvBoxPoints(box, pt);
        
        for (int j = 0; j < 4; j++) {
            pp[j] = Point(pt[j].x, pt[j].y);
        }

        std::vector<Point> pv(pp, pp+4);
        Polygon hole(pv);
        visibility->add_hole(hole);
    }
    
    //add_rect(robot_x, robot_y);
    //add_rect(goal_x, goal_y);
    
    visibility->enforce_standard_form();
    
   // printf("vis valid %d area %f dia %f\n", visibility->is_valid(), visibility->area(), visibility->diameter());
    
    printf("drawing visibility graph\n");
    
    IplImage *visibility_image = cvCloneImage(obstacle_image);

    int nlines = 0, nvertices = 0;
    
    {
        Visibility_Graph graph(*visibility);
        nvertices = graph.n();
        
        for (int i = 0; i < nvertices; i++) {
            Point &p1 = (*visibility)(i);
            for (int j = 0; j < nvertices; j++) {
                Point &p2 = (*visibility)(j);
                if (!graph(i, j) || (j > i && graph(j, i))) continue;
                
                nlines++;
                
                CvPoint cp1, cp2;
                cp1.x = p1.x();cp1.y = p1.y();
                cp2.x = p2.x();cp2.y = p2.y();
                
                cvLine(visibility_image, cp1, cp2, CV_RGB(255, 0, 255));
            }
        }
        
    }
     
    printf("%d vertices, %d edges in graph\n", nvertices, nlines);
    
    printf("drawing visibility poly of robot\n");
    
    Point robotp(robot_x, robot_y);
    Point goalp(goal_x, goal_y);
    
    {
        Visibility_Polygon rpoly(robotp, *visibility, 5);
        printf("vpoly area %f dia %f\n", rpoly.area(), rpoly.diameter());
        
        for (int i = 0; i < rpoly.n(); i++) {
            Point p = rpoly[i];
            Point p1 = rpoly[i+1];
            CvPoint cp1, cp2;
            cp1.x = p.x(); cp1.y = p.y();
            cp2.x = p1.x();cp2.y = p1.y();
            
            cvLine(visibility_image, cp1, cp2, CV_RGB(255, 255, 0), 2, 8, 0);
        }
    }
    
    printf("drawing shortest path\n");
    
    {
        Polyline path = visibility->shortest_path(robotp, goalp, 5);
        
        for (int i = 0; i < (path.size()-1); i++) {
            Point p = path[i];
            Point p1 = path[i+1];
            CvPoint cp1, cp2;
            cp1.x = p.x(); cp1.y = p.y();
            cp2.x = p1.x();cp2.y = p1.y();
            
            printf("p #%d: x %d y %d -> x %d y %d\n", i, cp1.x, cp1.y, cp2.x, cp2.y);
            
            cvLine(visibility_image, cp1, cp2, CV_RGB(255, 255, 255), 4, 8, 0);
        }
        
        printf("path size %d length %f\n", path.size(), path.length());
    }
    
    return visibility_image;
}

int main(int argc, char *argv[])
{    
    IplImage *obstacle_image, *visibility_image;
    
    gStorage    = cvCreateMemStorage(0);
    obstacle_image = get_obstacle_image();
    
    cvNamedWindow("visibility", 1);
    
    visibility_image = get_visibility_image(obstacle_image);
    
    cvShowImage("visibility", visibility_image);
    
    while (1) {
        cvWaitKey(2000);
    }
    
    return 0;
}