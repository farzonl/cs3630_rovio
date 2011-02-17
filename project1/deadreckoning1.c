/* Example program to interact with rovio using libcurl
 *
 * Author: Neil T. Dantam
 * Date: 2010-01-13
 */

#include <curl/curl.h>
#include <unistd.h>
// my Rovio ip 143.215.97.77
// their robot 143.215.110.22
#define ROVIO_IP "143.215.97.77"
#define USER_N "admin:"
#define PWD  "foobar1@"


static CURLcode rovio_forward(CURL *curl, int n)
{
    CURLcode res;
    int i;
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://admin:admin1@" ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=1&speed=3");
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
                         "http://"ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=6&speed=5");
        res = curl_easy_perform(curl);
    }
    return res;
}

static CURLcode rovio_turnRightByDegree(CURL *curl, int n)
{

    CURLcode res;
    int i;
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://"ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=18&speed=5");
        res = curl_easy_perform(curl);
    }
    return res;
}

static CURLcode rovio_turnRightByRange(CURL *curl, int bool)
{
    int n;
    if (bool == 0)
      n = 4;
    else
      n = 5;
    
    CURLcode res;
    int i;
    for(i = 0; i < n; i++){
        curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://"ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=18&speed=5");
        res = curl_easy_perform(curl);
    }
    return res;
}

static CURLcode rovio_driveLeft(CURL *curl, int n) {
  
        CURLcode res;
	int i;
  
	curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://"ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=6&speed=5");
        for( i = 0; i<n; i++)
            res = curl_easy_perform(curl);
	
	return res;
  
}

static CURLcode rovio_driveRight(CURL *curl, int n) {
  
        CURLcode res;
	int i;
  
	curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://"ROVIO_IP ":80"
                         "/rev.cgi?Cmd=nav&action=18&drive=4&speed=5");
        for( i = 0; i<n; i++)
            res = curl_easy_perform(curl);
	
	return res;
  
}

int main( int argc, char **argv ) {
    CURL *curl;
    CURLcode res;
    int i;
 
    curl = curl_easy_init();
    if(curl) {
        //go forwards
	int bool = 0;
        for (i = 0; i <= 3; i++) {
        rovio_forward(curl, 18);
	//rovio_turnRightByDegree(curl,6);
	rovio_turnRightByRange(curl,bool);
	sleep(1);
        if (bool == 0)
	  bool = 1;
	else
	  bool = 0;
	
	
	/*rovio_driveRight(curl,2);
	    sleep(1);*/
	}
        
 
        /* always cleanup */ 
        curl_easy_cleanup(curl);
	
	
    }
    return 0;
} 
