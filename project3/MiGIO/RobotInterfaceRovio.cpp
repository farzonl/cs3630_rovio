#include "Configuration.h"
#ifdef ROVIO

#include <cstdio>
#include <cstdlib>
#include "RobotInterfaceRovio.h"
#include "HTTPInterface.h"

#define ROVIO_PORT "80"

static const char *rovio_hosts[] = {
	"143.215.110.131",
	"143.215.110.22"
};

static int current_rovio = 0;

void rovio_set_robot(int n)
{
	current_rovio = n;
}

void robotReadSensors(){
}

void robotController(){
    static long long x = 0;
    x++;
    //actuatorValues[0] = ((x/10)%2) ? 1 : 2;
    //actuatorValues[1] = 5;
}

void rovio_drive(int n, RovioDirection direction)
{
    char buf[1024];
    int i;
    
    if (direction == DirNone) return;
    
    sprintf(buf,
             "http://%s:80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=3", rovio_hosts[current_rovio], direction);
        
    for(i = 0; i < n; i++){
		http_fetch(buf, NULL);
    }
    
    //sleep(1);
}

void rovio_turn(RovioTurn direction, int n)
{
    char buf[1024];
        
    sprintf(buf, 
             "http://%s:80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d",
				rovio_hosts[current_rovio],
				direction == TurnLeft ? 17 : 18, n);
        
	http_fetch(buf, NULL);
    
    //sleep(1);
}

void rovio_turn_small(RovioTurn direction)
{
    char buf[1024];
    
    sprintf(buf,
             "http://%s:80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=1&angle=1", rovio_hosts[current_rovio], direction == TurnLeft ? 17 : 18);
    
	http_fetch(buf, NULL);
    
    ///sleep(1);
}

void rovio_camera_height(vertical_class height)
{
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
    
    sprintf(buf,
             "%s:80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d", rovio_hosts[current_rovio], code);
    http_fetch(buf, NULL);
}

void robotSendActuators(){
    //static char buf[1024];
//    sprintf( buf, 
//            "http://" ROVIO_HOST ":" ROVIO_PORT "/rev.cgi?"
//             "Cmd=nav&"
//             "action=18&"
//             "drive=%d&"
//             "speed=%d",
//             actuatorValues[0], actuatorValues[1]
//             );
    
	//sprintf(buf,
		     //"http://" ROVIO_HOST ":" ROVIO_PORT "/rev.cgi?"
             //"Cmd=nav&"
             //"action=18&"
            // "drive=1&"
          //   "speed=5d"
		//	);
	//http_fetch( buf, NULL );

}

//CURLcode rovio_forward(CURL *curl, int n,int s)
void rovio_forward(int n,int s)
{
    //CURLcode res;
	char buf[1024];
    int i;

	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=1&speed=%d", rovio_hosts[current_rovio], s);

    for(i = 0; i < n; i++)
	{

        /*curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://" ROVIO_HOST ":" ROVIO_PORT
                         "/rev.cgi?Cmd=nav&action=18&drive=1&speed=%d",s);*/
        ///res = curl_easy_perform(curl);

		http_fetch(buf, NULL);
	
    }
    //return res;
}

void rovio_backward(int n,int s)
{
	char buf[1024];
    int i;
	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=2&speed=%d", rovio_hosts[current_rovio], s);

    for(i = 0; i < n; i++)
	{

		http_fetch(buf, NULL);
    }
}

//CURLcode rovio_turnRightByDegree(CURL *curl, int n)
void  rovio_turnRightByDegree(int n)
{
	char buf[1024];

	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=18&speed=5", rovio_hosts[current_rovio]);

    //CURLcode res;
    int i;
    for(i = 0; i < n; i++)
	{
        /*curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://" ROVIO_HOST ":" ROVIO_PORT
                         "/rev.cgi?Cmd=nav&action=18&drive=18&speed=5");
        res = curl_easy_perform(curl);*/

		http_fetch(buf, NULL);
    }
    //return res;
}

//CURLcode rovio_turnLeftByDegree(CURL *curl, int n)
void rovio_turnLeftByDegree(int n)
{
	char buf[1024];
    //CURLcode res;
	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=17&speed=5", rovio_hosts[current_rovio]);

    int i;
    for(i = 0; i < n; i++){
        /*curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://" ROVIO_HOST ":" ROVIO_PORT
                         "/rev.cgi?Cmd=nav&action=18&drive=17&speed=5");
        res = curl_easy_perform(curl);*/
    }

    //return res;
}

//CURLcode rovio_driveLeft(CURL *curl, int n) 
void rovio_driveLeft(int n, int s) 
{  
    
	char buf[1024];
	//CURLcode res;
	int i;
	
	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=3&speed=%d", rovio_hosts[current_rovio], s);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);

            //res = curl_easy_perform(curl);
	
	//return res;
  
}

//CURLcode rovio_driveRight(CURL *curl, int n) {
void rovio_driveRight(int n, int s) 
{  
    
	char buf[1024];

	//CURLcode res;
	int i;
  
	/*curl_easy_setopt(curl, CURLOPT_URL, 
                         "http://" ROVIO_HOST ":" ROVIO_PORT
                         "/rev.cgi?Cmd=nav&action=18&drive=4&speed=5");*/
	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=4&speed=%d", rovio_hosts[current_rovio], s);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);
           // res = curl_easy_perform(curl);
	
	//return res;
  
}

void rovio_DiagForRight(int n) {  
	int i;

	char buf[1024];

	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=8&speed=5", rovio_hosts[current_rovio]);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);
          
  
}

void rovio_DiagForLeft(int n)
{  
  
	int i;
	char buf[1024];
	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=7&speed=5", rovio_hosts[current_rovio]);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);
}

void rovio_DiagBackLeft(int n) {  
  
	int i;
	char buf[1024];

	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=9&speed=5", rovio_hosts[current_rovio]);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);
}

void rovio_DiagBackRight(int n) {  
  
	int i;
	char buf[1024];

	sprintf(buf,
             "http://%s/rev.cgi?Cmd=nav&action=18&drive=10&speed=5", rovio_hosts[current_rovio]);

        for( i = 0; i<n; i++)
			http_fetch(buf, NULL);
}

void robotWait() {
}

#endif
