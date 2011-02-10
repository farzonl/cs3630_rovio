/* Example program to interact with rovio using libcurl
 *
 * Author: Neil T. Dantam
 * Date: 2010-01-13
 */

#include <curl/curl.h>
#include <unistd.h>

#define ROVIO_IP "143.215.110.22"

static CURLcode rovio_forward(CURL *curl, int n)
{
    CURLcode res;
    int i;
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://admin:admin1@" ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=1&speed=5");
        res = curl_easy_perform(curl);
    }
    return res;
}

static CURLcode rovio_turnright(CURL *curl, int n)
{
    CURLcode res;
    int i;
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://admin:admin1@" ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=6&speed=5");
        res = curl_easy_perform(curl);
    }
    return res;
}

int main( int argc, char **argv ) {
    CURL *curl;
    CURLcode res;
    int i;
 
    curl = curl_easy_init();
    if(curl) {
        //go forwards
  
        for (i = 0; i <= 3; i++) {
        rovio_forward(curl, 21);
            sleep(1);
        rovio_turnright(curl, 4);
            sleep(1);
        }
 
        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }
    return 0;
} 
