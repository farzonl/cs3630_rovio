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
    
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, buf);
        res = curl_easy_perform(curl);
    }
    return res;
}

static CURLcode rovio_turnright90(CURL *curl)
{
    CURLcode res;
    int i;
    
    curl_easy_setopt(curl, CURLOPT_URL, 
                     "http://admin:admin1@" ROVIO_IP ":80"
                     "/rev.cgi?Cmd=nav&action=18&drive=18&speed=5&angle=7");
    res = curl_easy_perform(curl);
    return res;
}

int main( int argc, char **argv ) {
    CURL *curl;
    CURLcode res;
    int i;
    
    curl = curl_easy_init();
    if(curl) {
        
        for (i = 0; i <= 3; i++) {
            rovio_drive(curl, 24, Forward);
            sleep(1);
            rovio_turnright90(curl);
            sleep(1);
        }
        
        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }
    return 0;
} 
