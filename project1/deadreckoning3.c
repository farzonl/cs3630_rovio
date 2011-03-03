/* Example program to interact with rovio using libcurl
 *
 * Author: Neil T. Dantam
 * Date: 2010-01-13
 */

#include <curl/curl.h>
#include <unistd.h>

#define ROVIO_IP "143.215.97.77"
//97.77 110.22

enum RovioDirection {
    Forward = 1,
    Backward,
    Left,
    Right,
    LeftForward = 7,
    RightForward,
    LeftBackward,
    RightBackward
};

static CURLcode rovio_drive(CURL *curl, int n, enum RovioDirection direction)
{
    CURLcode res;
    char buf[1024];
    int i;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=3", direction);
    
    puts(buf);

    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, buf);
        res = curl_easy_perform(curl);
    }
    return res;
}

// direction = Left or Right
// n = 3 for 45, 7 for 90, technically in increments of 20 degrees
static CURLcode rovio_turn(CURL *curl, enum RovioDirection direction, int n)
{
    CURLcode res;
    char buf[1024];
    int i;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_IP ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d", direction == Left ? 17 : 18, n);
    
    puts(buf);
    
    curl_easy_setopt(curl, CURLOPT_URL, buf);
    res = curl_easy_perform(curl);
    return res;
}

int main( int argc, char **argv ) {
    CURL *curl;
    CURLcode res;
    int i;
    
    curl = curl_easy_init();
    if(curl) {
        
        // drive in an hourglass shape
        rovio_turn(curl, Right, 3);
        sleep(1);
        rovio_drive(curl, 34, Forward);
        sleep(1);
        rovio_turn(curl, Right, 3);
        sleep(1);
        rovio_drive(curl, 24, Backward);
        sleep(1);
        rovio_turn(curl, Right, 3);
        sleep(1);
        rovio_drive(curl, 34, Forward);
        sleep(1);
        rovio_turn(curl, Right, 3);
        sleep(1);
        rovio_drive(curl, 24, Right);
        sleep(1);
        
        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }
    return 0;
} 
