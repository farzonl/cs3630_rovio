#include "Configuration.h"
#ifdef PLEO

#include "RobotInterfacePleo.h"
#include "SerialTerm.h"
#define _angle(a) a+32+45

#include <math.h>

char pleo_sensor_data[30];
char pleo_joint_data[30];
int actuatorValues[30];
int binarySensors[30];
static int pleo_loop_count;
static float t = 0;

//---------------------------------------------------------------------
// Pleo functions
void pleo_wag_tail();

void pleoUpdateJoints() {
	for(int i = 0; i < numActuators; i++) {
		actuatorValues[i]=(int)pleo_joint_data[i+1]-32-45;
	}
}

void pleoUpdateSensors() {
	for(int i = 0; i < numBinarySensors; i++) {
		if(pleo_sensor_data[i] == '1') binarySensors[i] = 1;
		else binarySensors[i] = 0;
	}
}

// Initialize Joint Data Array for transmission
void pleoInitialize() {
	pleo_joint_data[0] = 'm';
	for(int i = 1; i < 1+numActuators; i++)
		pleo_joint_data[i] = _angle(0);
	pleo_joint_data[15] = '!';
	printf("Initial Joint Data is [%s]\n", pleo_joint_data);
}

// Set all Pleo Joints to 0
void pleoNeutralizeJoints() {
	Sleep(1000L);
	for(int i = 1; i < 1+numActuators; i++)
		pleo_joint_data[i] = _angle(0);

	serialterm_send(pleo_joint_data, pleo_joint_data_size);
}

// Request Sensor data from Pleo
void pleoRequestSensors() {
	char data[4] = {0};
	sprintf(data, "s!");
	serialterm_send(data, 2);
}
// End Pleo functions
//---------------------------------------------------------------------
// Generic Functions
void robotReadSensors(){
	pleoInitialize();
}

void robotController(){
	serialterm_init(PLEO_PORT);
#ifdef PLEO_COMM_SYNC 
	serialterm_receive_thread();
#endif
}

// In the loop
void robotSendActuators(){
	int sensor_size = 0;
	char sensor_data[30] ={0};
	pleo_loop_count++;
	//if(pleo_loop_count == 1)
		//pleoNeutralizeJoints();
	pleo_wag_tail();
	pleoRequestSensors();
	serialterm_read(sensor_data, &sensor_size);
#if DEBUG > 1
	printf("Received %s\n", sensor_data);
#endif
	if(sensor_size > 20) {
		memcpy(pleo_sensor_data, sensor_data, sensor_size);
	}
	pleoUpdateSensors();
	pleoUpdateJoints();
}

void robotWait(){
	serialterm_wait();
	serialterm_close();
}


//---------------------------------------------------------------------
// End Generic Functions
#define SLEEP 150L
void pleo_wag_tail() {
	int i = 0, j = 0;
	int a = 45;

	t+=.8;

	int x = (int)(sin(t)*45.0);
	int y = (int)(cos(t)*45.0);
	pleo_joint_data[JOINT_TAIL_HORIZONTAL+1] = _angle(x);
	pleo_joint_data[JOINT_TAIL_VERTICAL+1] = _angle(y);
	pleo_joint_data[JOINT_HEAD+1] = _angle(y);
	serialterm_send(pleo_joint_data, pleo_joint_data_size);

	Sleep(60);

}

#endif