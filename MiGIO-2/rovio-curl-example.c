/* Example program to interact with rovio using libcurl
 *
 * Author: Neil T. Dantam
 * Date: 2010-01-13
 */

#include "curl\curl.h"
#include <stdio.h>
//#include <unistd.h>





int main( int argc, char **argv ) {
    CURL *curl;
    CURLcode res;
    int i;
 
    curl = curl_easy_init();
    if(curl) {
        //go forwards
		for(i = 0; i < 4; i ++){
			curl_easy_setopt(curl, CURLOPT_URL,
							 "http://143.215.108.60:80"
							 "/rev.cgi?Cmd=nav&action=18&drive=1&speed=5");
			res = curl_easy_perform(curl);
			Sleep(1);
			//turn right 
			curl_easy_setopt(curl, CURLOPT_URL,
							 "http://143.215.108.60:80"
							 "/rev.cgi?Cmd=nav&action=18&drive=6&speed=5");
			res = curl_easy_perform(curl);
			Sleep(1);
			}
 
        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }
    return 0;
} 
