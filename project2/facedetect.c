#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <curl/curl.h>

static int imageWidth, imageHeight;
static CURL *gCURL;

#define ROVIO_IP "143.215.110.22"
#define ROVIO_CAM 1

static CvCapture *create_capture_from_localcam()
{
    return cvCaptureFromCAM(-1);
}

static CvCapture *create_capture_from_rovio()
{
    return cvCaptureFromFile("http://"ROVIO_IP"/Jpeg/CamImg/1.jpg");
}

typedef enum horizontal_class {
    left = 0,
    center,
    right
} horizontal_class;

typedef enum vertical_class {
    low = 0,
    middle,
    high
} vertical_class;

typedef enum RovioDirection {
    None = 0,
    Forward = 1,
    Backward,
    Left,
    Right,
    LeftForward = 7,
    RightForward,
    LeftBackward,
    RightBackward
} RovioDirection;

static vertical_class current_height = middle;

static CURLcode rovio_drive(CURL *curl, int n, RovioDirection direction)
{
    CURLcode res;
    char buf[1024];
    int i;
    
    if (direction == None) return 0;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=3", direction);
    
    //puts(buf);
    
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, buf);
        res = curl_easy_perform(curl);
    }
    return res;
}

// direction = Left or Right
// n = 3 for 45, 7 for 90, technically in increments of 20 degrees
static CURLcode rovio_turn(CURL *curl, horizontal_class direction, int n)
{
    CURLcode res;
    char buf[1024];
    int i;
    
    if (direction == center) return 0;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d", direction == left ? 17 : 18, n);
    
    //puts(buf);
    
    curl_easy_setopt(curl, CURLOPT_URL, buf);
    res = curl_easy_perform(curl);
    return res;
}

static CURLcode rovio_camera_height(CURL *curl, vertical_class height)
{
    CURLcode res;
    char buf[1024];
    int i;
    int code;
    
    switch (height) {
        case high:
            code = 11;
            break;
        case low:
            code = 12;
            break;
        default:
            code = 13;
    }
        
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d", code);
    
    //puts(buf);
    
    current_height = height;
    
    curl_easy_setopt(curl, CURLOPT_URL, buf);
    res = curl_easy_perform(curl);
    return res;
}

static horizontal_class horiz_class(int w)
{
    float ratio = w / (float)imageWidth;
    float center_limit = 1./3;
    
    if (ratio < center_limit) return left;
    if (ratio > (1. - center_limit)) return right;
    
    return center;
}

static vertical_class vert_class(int h)
{
    float ratio = h / (float)imageHeight;
    float center_limit = 2./5;
    
    if (ratio < center_limit) return high;
    if (ratio > (1. - center_limit)) return low;
    return middle;
}

static int ticks_since_plan = 0, ticks_before_replan = 0, ticks_to_delay_replan = 0; // boredom timer
static int num_deplanned_undoes = 0;
static vertical_class planned_camera_level = middle;
static RovioDirection planned_direction = None;
static horizontal_class planned_turn = center;

static void robot_deplan()
{
    int can_undo = num_deplanned_undoes-- > 0;
    ticks_since_plan = 0;
    
    planned_camera_level = can_undo ? middle : current_height;
    planned_direction = None;
    planned_turn = can_undo ? 2 - planned_turn : center;
    
    ticks_to_delay_replan = 0;
}

// r - the rectangle in the image of the detected face
// NULL if no face was detected
static void robot_update_plan(CvRect *r)
{
    /*
     simple -
     no vertical classification
     no fine adjustments
     no consideration of effect of distance on how much the turn moves the rect center
     */
    
    /*
     no face -
     don't know what to do here, boredom timer probably handles it
     */
    
    if (!r) return;
    
    if (ticks_to_delay_replan--) return;
    
    horizontal_class leftmost, rightmost;
    vertical_class   bottommost, topmost;
    
    leftmost = horiz_class(r->x);
    rightmost= horiz_class(r->x + r->width);
    
    topmost   = vert_class(r->y);
    bottommost= vert_class(r->y + r->height);
    
    ticks_since_plan = 0;
    ticks_to_delay_replan = 0;
    
    float face_ratio = r->width / (float)imageWidth;
        
    //printf("left %d right %d top %d bottom %d\n", leftmost, rightmost, topmost, bottommost);
    
    //printf("face ratio %f\n", face_ratio);
    
    if (topmost == high && topmost == bottommost) {
        // printf(", look up\n");
        // face is high up - raise camera one level, or back up the robot
        planned_camera_level = planned_camera_level + 1;
        if (planned_camera_level > 2) planned_camera_level = 2;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (topmost == low && topmost == bottommost) {
        // printf(", look down\n");
        // face is low down - lower camera one level
        planned_camera_level = planned_camera_level - 1;
        if (planned_camera_level < 0) planned_camera_level = 0;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else {
        // face is centered - raise camera to mid level
        planned_camera_level = middle;
    }
    
    planned_turn = center;
    planned_direction = None;
    
    if (leftmost == left && leftmost == rightmost) {
        // far left
        planned_turn = left;
        planned_direction = face_ratio < .3 ? LeftForward : Left;
        ticks_before_replan = 2;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (leftmost == right && leftmost == rightmost) {
        // far right
        planned_turn = right;
        planned_direction = face_ratio < .3 ? RightForward : Right;
        ticks_before_replan = 2;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (leftmost == left && rightmost == right) {
        // probably doesn't happen
        planned_direction = None;
        ticks_before_replan = 0;
        ticks_to_delay_replan = 0;
        num_deplanned_undoes = 0;
    } else if (leftmost == center || rightmost == center) {
        // centered
        float center_diff = fabs(((r->x + (r->width/2.)) / (imageWidth)) - .5);
        
        // if leftmost != rightmost, maybe turn
        // otherwise, maybe fine-adjust (by moving sideways or diagonal) if that doesn't cause the face center to just shift sides
        if (leftmost == rightmost || center_diff < (1/3.)) {
            planned_direction = Forward;
            ticks_before_replan = 10;
            ticks_to_delay_replan += 4;
        } else {
            planned_direction = (leftmost == left) ? LeftForward : RightForward;
            ticks_before_replan = 5;
            ticks_to_delay_replan += 4;
        }
    }
}

// do whatever we decided to do last time we could update
static void robot_execute_plan()
{
    if (ticks_before_replan == ticks_since_plan)
        robot_deplan();
    
    //printf("level %d turn %d direction %d\n", planned_camera_level, planned_turn, planned_direction);
    
    rovio_camera_height(gCURL, planned_camera_level);
    rovio_drive(gCURL, 5, planned_direction);
    rovio_turn(gCURL, planned_turn, 1); 
    
    ticks_since_plan++;
}

static int detect_object(IplImage *frame, CvHaarClassifierCascade *cascade, CvMemStorage *storage, CvRect *r)
{
    CvSeq *faces = cvHaarDetectObjects(frame, cascade, storage, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(20,20), cvSize(0,0));
    
    if (faces->total) {
        *r = *(CvRect*)cvGetSeqElem(faces, 1);
        return 1;
    }
    
    // blob detection goes here
    
    // failure
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        return 1;
    }
    
    char *cascade_file = argv[1];
    
    CvMemStorage *storage = NULL;
    CvHaarClassifierCascade *cascade = NULL;
    CvCapture *capture = NULL;
    IplImage *frame = NULL, *frame2 = NULL;
    
    cascade = cvLoad(cascade_file, NULL, NULL, NULL);
    
    if (!cascade)
        return 1;
    
    storage = cvCreateMemStorage(0);
#if !ROVIO_CAM
    capture = create_capture_from_localcam();
#else
    // rovio cam
#endif
    
    gCURL = curl_easy_init();
    curl_easy_setopt(gCURL, CURLOPT_WRITEDATA, stderr);
    
    cvNamedWindow("capturew", 1);
    
    while (1) {
#if ROVIO_CAM
        cvReleaseCapture(&capture);
        capture = create_capture_from_rovio();
#endif
        if (!cvGrabFrame(capture)) break;
        frame = cvRetrieveFrame(capture, 0);
        if (!frame) break;
        
        imageWidth = frame->width;
        imageHeight = frame->height;
        
        if (!frame2)
            frame2 = cvCreateImage(cvSize(frame->width, frame->height),
                                   IPL_DEPTH_8U, frame->nChannels);
        cvCopy(frame, frame2, 0);
        cvClearMemStorage(storage);
        
        CvRect r;
        
        if (detect_object(frame2, cascade, storage, &r)) {
            // decision making
            robot_update_plan(&r);
            
            // drawing
            CvPoint pt1, pt2;
            
            pt1.x = r.x;
            pt2.x = r.x + r.width;
            pt1.y = r.y;
            pt2.y = r.y + r.height;
            
            cvRectangle(frame2, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0);            
        } else {
            robot_update_plan(NULL);
        }

        robot_execute_plan();

        cvShowImage("capturew", frame2);
        //cvReleaseImage(&frame);
        cvWaitKey(67); // 1/15 sec
    }
        
    return 0;
}