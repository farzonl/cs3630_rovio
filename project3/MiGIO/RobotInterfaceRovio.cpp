#include "Configuration.h"
#ifdef ROVIO

#include <cstdio>
#include <cstdlib>
#include "RobotInterfaceRovio.h"
#include "HTTPInterface.h"



void robotReadSensors(){
    char *report = http_strdup( "http://" ROVIO_HOST ":" ROVIO_PORT 
                         "/rev.cgi?Cmd=nav&action=1" );
    puts( report );
    free( report );
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
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=3", direction);
        
    for(i = 0; i < n; i++){
		http_fetch(buf, NULL);
    }
    
    sleep(1);
}

void rovio_turn(RovioTurn direction, int n)
{
    char buf[1024];
        
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=5&angle=%d", direction == TurnLeft ? 17 : 18, n);
        
	http_fetch(buf, NULL);
    
    sleep(1);
}

void rovio_turn_small(RovioTurn direction)
{
    char buf[1024];
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d&speed=1&angle=1", direction == TurnLeft ? 17 : 18);
    
	http_fetch(buf, NULL);
    
    sleep(1);
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
    
    snprintf(buf, sizeof(buf),
             "http://admin:admin1@" ROVIO_HOST ":80"
             "/rev.cgi?Cmd=nav&action=18&drive=%d", code);
    
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

void robotWait() {
}

#endif
