#include "Configuration.h"
#ifdef ROVIO

#include <cstdio>
#include <cstdlib>
#include "RobotInterfaceRovio.h"
#include "HTTPInterface.h"
#include "curl/curl.h"



void robotReadSensors(){

    float x=0, y=0, theta=0;
    int room = 0;
    int beacon = 0;
    int beacon_x = 0;
    int next_room = 0;
    int next_room_ss = 0;
    int state = 0;
    int pp = 0;
    int flags = 0;
    int brightness = 0;
    int resolution = 0;
    int video_compression = 0;
    int frame_rate = 0;
    int privelege = 0;
    int user_check = 0;
    int speaker_volume = 0;
    int mic_volume = 0;
    int wifi_ss = 0;
    int show_time = 0;
    int ddns_state = 0;
    int email_state = 0;
    int battery = 0;
    int charging = 0;
    int head_position = 0;
    int ac_freq = 0;
    int responses = 0;
    int ss;
    int ui_status = 0;
    int resistance = 0;
    int sm = 0;
    
    char *report = http_strdup( "http://" ROVIO_HOST ":" ROVIO_PORT 
                         "/rev.cgi?Cmd=nav&action=1" );
    puts( report );
	

    free( report );
}

void robotController(){
    static long long x = 0;
    x++;
    actuatorValues[0] = ((x/10)%2) ? 1 : 2;
    actuatorValues[1] = 5;
    
}

void robotSendActuators(){
    static char buf[1024];
//    sprintf( buf, 
//            "http://" ROVIO_HOST ":" ROVIO_PORT "/rev.cgi?"
//             "Cmd=nav&"
//             "action=18&"
//             "drive=%d&"
//             "speed=%d",
//             actuatorValues[0], actuatorValues[1]
//             );
    
	sprintf(buf,
		     "http://" ROVIO_HOST ":" ROVIO_PORT "/rev.cgi?"
             "Cmd=nav&"
             "action=18&"
             "drive=1&"
             "speed=5d"
			);
	http_fetch( buf, NULL );

}

int rovio_drive(CURL *curl, int n, RovioDirection direction)
{
    CURLcode res;
    char buf[1024];
    int i;
    
    if (direction == DirNone) return 0;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
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
int rovio_turn(CURL *curl, horizontal_class direction, int n)
{
    CURLcode res;
    char buf[1024];
    
    if (direction == center) return 0;
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d", direction == _left ? 17 : 18, n);
    
    //puts(buf);
    
    curl_easy_setopt(curl, CURLOPT_URL, buf);
    res = curl_easy_perform(curl);
    return res;
}

#endif
