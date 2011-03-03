#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <curl/curl.h>
#include "blob.h"
#include "BlobResult.h"

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
    _left = 0,
    center,
    _right
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

static int rovio_drive(CURL *curl, int n, RovioDirection direction)
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
static int rovio_turn(CURL *curl, horizontal_class direction, int n)
{
    CURLcode res;
    char buf[1024];
    
    if (direction == center) return 0;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d", direction == _left ? 17 : 18, n);
    
    //puts(buf);
    
    curl_easy_setopt(curl, CURLOPT_URL, buf);
    res = curl_easy_perform(curl);
    return res;
}

static int rovio_camera_height(CURL *curl, vertical_class height)
{
    CURLcode res;
    char buf[1024];
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
    
    if (ratio < center_limit) return _left;
    if (ratio > (1. - center_limit)) return _right;
    
    return center;
}

static vertical_class vert_class(int h)
{
    float ratio = h / (float)imageHeight;
    float high_limit = 1./5, low_limit = 2./5;
    
    if (ratio < high_limit) return high;
    if (ratio > (1. - low_limit)) return low;
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
    planned_turn = can_undo ? (horizontal_class)(2 - planned_turn) : _left;
    
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
        planned_camera_level = (vertical_class)(planned_camera_level + 1);
        if (planned_camera_level > 2) planned_camera_level = high;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (topmost == low && topmost == bottommost) {
        // printf(", look down\n");
        // face is low down - lower camera one level
        planned_camera_level = (vertical_class)(planned_camera_level - 1);
        if (planned_camera_level < 0) planned_camera_level = low;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else {
        // face is centered - raise camera to mid level
        planned_camera_level = middle;
    }
    
    planned_turn = center;
    planned_direction = None;
    
    if (leftmost == _left && leftmost == rightmost) {
        // far left
        planned_turn = _left;
        planned_direction = face_ratio < .3 ? LeftForward : Left;
        ticks_before_replan = 2;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (leftmost == _right && leftmost == rightmost) {
        // far right
        planned_turn = _right;
        planned_direction = face_ratio < .3 ? RightForward : Right;
        ticks_before_replan = 2;
        ticks_to_delay_replan += 4;
        num_deplanned_undoes = 2;
    } else if (leftmost == _left && rightmost == _right) {
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
            ticks_before_replan = 5;
            ticks_to_delay_replan += 4;
        } else {
            planned_direction = (leftmost == _left) ? LeftForward : RightForward;
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

static uint8_t clip8(int i) {if (i < 0) i=0; if (i>255) i=255; return i;}

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

enum {
    kind_face = 0,
    kind_lemon
};

static int rect_distance(CvRect a, CvRect b)
{
    int ax, ay, bx, by;
    
    ax = a.x + (a.width/2);
    ay = a.y + (a.height/2);
    
    bx = b.x + (b.width/2);
    by = b.y + (b.height/2);
    
    return (ax-bx)*(ax-bx) + (ay-by)*(ay-by);
}

CvRect last_rect; int last_kind=-1;
CvRect best_rect; int best_kind=-1; int best_score;

static void reset_possible_rect()
{
    best_kind = -1;
}

static void possible_rect(CvRect r, int kind)
{
    if (kind == kind_lemon && best_kind == kind_face) return;
    if (last_kind == -1 && best_kind != -1) return;
    if (best_kind != -1 && (r.x*r.y <= 30)) return;
    int score = rect_distance(r, last_rect) + ((r.x*r.y)-(last_rect.x*last_rect.y));
    if (best_kind == -1 || score < best_score) {
        best_rect = r;
        best_kind = kind;
        best_score = score;
    }
}

static int choose_best_rect(CvRect *r)
{
    if (best_kind == -1) return 0;
    last_rect = best_rect;
    last_kind = best_kind;
    *r = last_rect;
    return 1;
}

static IplImage *copy_image_24bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, frame->nChannels);
}

static IplImage *copy_image_8bit(IplImage *frame)
{
    return cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
}

#define image_at_24(p, x, y, c) \
    ((uchar *)((p)->imageData + (y)*(p)->widthStep))[(x)*(p)->nChannels + (c)]

#define image_at_8(p, x, y) \
    ((uchar *)((p)->imageData + (y)*(p)->widthStep))[(x)]

static int detect_object(IplImage *frame, CvHaarClassifierCascade *cascade, CvMemStorage *storage, CvRect *r)
{
    CvSeq *faces = cvHaarDetectObjects(frame, cascade, storage, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(20,20), cvSize(0,0));
    int i, success;
    
    // blob detection goes here
    IplImage *himage = copy_image_24bit(frame);
    IplImage *timage = copy_image_8bit(frame);
    CBlobResult fruitBlob;
	CBlob fruitBlobArea;
    
    cvCvtColor(frame, himage, CV_BGR2HSV_FULL);
    
    for(int y = 0; y < frame->height; y++) {
		for(int x = 0; x < frame->width; x++) {
            uint8_t h, s, v/*, b, g, r*/;
            
            if (y < (frame->height/3)) {
                image_at_8(timage, x, y) = 0;
                continue;
            }
            
            h = image_at_24(himage, x, y, 0);
            s = image_at_24(himage, x, y, 1);
            v = image_at_24(himage, x, y, 2);

            uint8_t lh = 40, ls = 255, lv = 120;
            int dist = sqrtf((lh-h)*(lh-h) + (ls-s)*(ls-s) + (lv-v)*(lv-v));
            uint8_t diff = -clip8(dist);
            
			image_at_8(timage, x, y) = (diff > 210) ? 255 : 0;
        }
    }
    
    cvErode(timage, timage, NULL, 2);
    cvDilate(timage, timage, NULL, 2);
    
    cvShowImage("filterw", himage);
    cvShowImage("threshw", timage);

    fruitBlob = CBlobResult(timage, NULL, 0, true);

    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_GREATER, 15);
    fruitBlob.Filter(fruitBlob, B_INCLUDE, CBlobGetArea(), B_LESS, 250000);

    reset_possible_rect();
    
    for (i = 0; i < faces->total; i++) {
        CvRect fr = *(CvRect*)cvGetSeqElem(faces, i);
        
        draw_rect(frame, fr, 0);
        possible_rect(fr, kind_face);
    }
    
    for (int i = 0; i < fruitBlob.GetNumBlobs(); i++)
    {
        fruitBlobArea = fruitBlob.GetBlob(i);
        CvRect fr;
        fr.x      = fruitBlobArea.MinX();
        fr.width  = fruitBlobArea.MaxX() - fr.x;
        fr.y      = fruitBlobArea.MinY();
        fr.height = fruitBlobArea.MaxY() - fr.y;
        
        if (fr.x == 0 && fr.width >= 640) continue;
        
        draw_rect(frame, fr, 1);
        possible_rect(fr, kind_lemon);
    }
    
    success = choose_best_rect(r);
    if (success) draw_rect(frame, *r, 2);
    
    //printf("-\n");
    cvReleaseImage(&timage);
    cvReleaseImage(&himage);

    return success;
}

int main(int argc, char *argv[])
{
    const char *cascade_file;
    
    if (argc < 2) {
        cascade_file = "/usr/local/opencv/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
    } else
        cascade_file = argv[1];
        
    CvMemStorage *storage = NULL;
    CvHaarClassifierCascade *cascade = NULL;
    CvCapture *capture = NULL;
    IplImage *frame = NULL, *filtered = NULL;
    
    cascade = (CvHaarClassifierCascade*)cvLoad(cascade_file, NULL, NULL, NULL);
    
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
    cvNamedWindow("filterw", 2);
    cvNamedWindow("threshw", 3);

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
        
        if (!filtered) filtered = copy_image_24bit(frame);
        cvSmooth(frame, filtered, CV_BILATERAL, 5, 5, 50, 30);
        cvClearMemStorage(storage);
        
        CvRect r;
        
        if (detect_object(filtered, cascade, storage, &r)) {
            // decision making
            robot_update_plan(&r);
        } else {
            robot_update_plan(NULL);
        }

        cvShowImage("capturew", filtered);

        robot_execute_plan();

        //cvReleaseImage(&frame);
        cvWaitKey(67); // 1/15 sec
    }
        
    return 0;
}