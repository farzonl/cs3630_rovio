
#ifndef ROBOT_INTERFACE_R
#define ROBOT_INTERFACE_R

#include "Configuration.h"
#ifdef PLEO

static char robotImage[] = "Data/PleoHeader.jpg";

enum sensor_name 
{
    SENSOR_NONE          =    -1, /** name used to specify 'no sensor' */
    SENSOR_MIN           =     0, /** number of the lowest defined sensor (useful for loops) */
    SENSOR_BATTERY       =     2, /** Battery level (0-100) */
    SENSOR_IR            =     3, /** IR data received */
    SENSOR_IR_ACTIVITY   =     4, /** IR has received a code of. DISABLED */
    SENSOR_TOUCH_FIRST   =     6, /** Alias for the first touch sensor */ 
    SENSOR_HEAD          =     6, /** Head touch sensor 0=not touched 1=touched */
    SENSOR_CHIN          =     7, /** Chin touch sensor 0=not touched 1=touched */
    SENSOR_BACK          =     8, /** Back touch sensor 0=not touched 1=touched */
    SENSOR_LEFT_LEG      =     9, /** Left Leg touch sensor 0=not touched 1=touched */
    SENSOR_RIGHT_LEG     =    10, /** Right Leg touch sensor 0=not touched 1=touched */
    SENSOR_LEFT_ARM      =    11, /** Left Arm touch sensor 0=not touched 1=touched */
    SENSOR_RIGHT_ARM     =    12, /** Right Arm touch sensor 0=not touched 1=touched */
    SENSOR_ARSE          =    13, /** Arse touch sensor 0=not touched 1=touched */
    SENSOR_TAIL          =    13, /** Alias for arse touch sensor 0=not touched 1=touched */
    SENSOR_TOUCH_LAST    =    13, /** Alias for the last touch sensor  */
  
    SENSOR_FOOT_FIRST    =    14, /** Alias for the first foot switch */
    SENSOR_FRONT_LEFT    =    14, /** front left foot switch */
    SENSOR_FRONT_RIGHT   =    15, /** front right foot switch */
    SENSOR_BACK_LEFT     =    16, /** back left foot switch */
    SENSOR_BACK_RIGHT    =    17, /** back right foot switch */
    SENSOR_FOOT_LAST     =    17, /** Alias for the last foot switch */

    SENSOR_CARD_DETECT   =    18, /** SD Card is present */
    SENSOR_WRITE_PROTECT =    19, /** SD Cards write protect tab is set (software only)*/
    SENSOR_LEFT_LOUD     =    20, /** Absolute loudness of left microphone*/
    SENSOR_LIGHT         =    21, /** Absolute light light (0-100)*/
    SENSOR_RIGHT_LOUD    =    22, /** Absolute loudness of right microphone*/
    SENSOR_OBJECT        =    23, /** Object in front of Pleo (reflected IR)*/
    SENSOR_MOUTH         =    24, /** Something blocking mouth IR*/
    SENSOR_SOUND_DIR     =    26, /** Sound direction */
    SENSOR_LIGHT_CHANGE  =    27, /** Light level change, lighten or darken*/
    SENSOR_SOUND_LOUD    =    28, /** Absolute loudness of ambient sound (0-100)*/
    SENSOR_TILT          =    29, /** Current orientation (per tilt_name enum)*/
    SENSOR_TERMINAL      =    30, /** line of text from terminal/serial*/
    SENSOR_POWER_DETECT  =    31, /** Is the charger plugged in?*/
    SENSOR_USB_DETECT    =    32, /** Is the USB cable onnected*/
    SENSOR_WAKEUP        =    33, /** Wakeup / Mom button*/
    SENSOR_BATTERY_TEMP  =    34, /** Battery at critical temp?*/
    SENSOR_CHARGER_STATE =    35, /** charger state */
    SENSOR_SHAKE         =    36, /** Shake sensor activated */
    SENSOR_SOUND_LOUD_CHANGE =37, /** 1 = went above trig, 0 is went below aux_trig*/
    SENSOR_BEACON        =    38, /** value = ID of other Pleo*/
    SENSOR_BATTERY_CURRENT =  39, /** electrical current draw from battery*/
    SENSOR_PACKET        =    40, /** virtual sensor to get the packet data from the NXP processor*/
    SENSOR_MAX
};

enum joint_name {
  JOINT_NONE            =    -1,
  JOINT_MIN             =     0,
  JOINT_RIGHT_SHOULDER  =     0,
  JOINT_RIGHT_ELBOW     =     1,
  JOINT_LEFT_SHOULDER   =     2,
  JOINT_LEFT_ELBOW      =     3,
  JOINT_LEFT_HIP        =     4,
  JOINT_LEFT_KNEE       =     5,
  JOINT_RIGHT_HIP       =     6,
  JOINT_RIGHT_KNEE      =     7,
  JOINT_TORSO           =     8,
  JOINT_TAIL_HORIZONTAL =     9,
  JOINT_TAIL_VERTICAL   =    10,
  JOINT_NECK_HORIZONTAL =    11,
  JOINT_NECK_VERTICAL   =    12,
  JOINT_HEAD            =    13,
  JOINT_MAX_PHYSICAL    =    13,
  JOINT_TRANSITION      =    14,
  JOINT_SOUND           =    15,
  JOINT_MAX
};

extern char pleo_sensor_data[];
extern char pleo_joint_data[];
static int pleo_sensor_data_size = 42;
static int pleo_joint_data_size = 16;

static int numActuators = 14;
extern int actuatorValues[];
static const char* actuatorNames[] = {" 0)RSHOU"," 1)RELBO"," 2)LSHOU"," 3)LELBO"," 4)LHIP "," 5)LKNEE"," 6)RHIP ",
                       " 7)RKNEE"," 8)TORSO"," 9)HTAIL","10)VTAIL","12)HNECK","13)VNECK","14)HEAD "};

static int numBinarySensors = 30;
extern int binarySensors[];
static const char* binaryNames[] = {"0", "1" ,"2" ,"3", "4", "5", "6", "7", "8", "9", "10",
								"11", "12" ,"13" ,"14", "15", "16", "17", "18", "19", "20",
								"21" ,"22" ,"23", "24", "25", "26", "27", "28", "29", "30"};

static int numContSensors = 0;
static int contSensors[1];
static const char* contNames[] = {""};

void robotReadSensors();
void robotController();
void robotSendActuators();
void robotWait();

void pleoInitialize();
void pleoNeutralizeJoints();
void pleoRequestSensors();
void pleoUpdateJoints();

#endif
#endif
