// Ryan Khoo Yeap Hong | 30480183 | Year 3 Individual Project | Academic year 2021/2022
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include <Adafruit_ICM20948.h>
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_gyro, *icm_mag, *icm_accel;
#include <PGMWrap.h>
#include "Filter.h"

//Changable Parameters 0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

//Walking pattern progress tracked by:
int cycleStage ; 
int mode = 9; //<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--<--
// 0 to disable all cycles (for debugging)
// 9 to begin in dynamic neutral
// 1 for normal operation
// 10 to enable servo initialisations 
// 20 for neutral position
// 30 for custom test position

//Servo execution speed (in miliseconds between each state execution, smaller number = faster execution)
int setDelayX = 2; //Default = 2, the time between each "eating" of a pending set, minimum depends on how fast void loop can execute
int setDelay = 2;// Code usues this variable, but "X" version is to allow initialisation to go slower before main loop

//Delay between IMU measurements
int IMUDelay = 20;

// footstep set modifiers
int emptyMod = 40; // Default = 40, 0 to 100, adjusts height of empty step, mostly unchanged.
int xMod = 0; //-100 to 100, degree of forward stepping 100 = front full, set by code
int yMod = 0; //-100 to 100, degree of side stepping 100 = right full, set by code

const int gaitAngle = 10; // Default = 20, the angle that the legs will swing forward and back, smaller angles require less torque, 50 for demo
const int gaitSideAngle = -10; // Default = 20, the angle that the legs will Left and Right, smaller angles require less torque, 50 for demo

//max divisions possible, may need to be reduce depending on memeory use, the higher the better/smoother.
const int maxDivisions = 10;

//when both X and Y modifiers are below stillThreshold, robot will stand still
int stillThreshold = 30;

//Modifier Weightages (100 = 1)
int fuzzWeight = 100;
int controllerWeight = 200;

//Filter Weights
ExponentialFilter<float > filteredRoll(5, 0);
ExponentialFilter<float> filteredPitch(5, 0);
ExponentialFilter<float > filteredAngVelX(5, 0);
ExponentialFilter<float > filteredAngVelY(8, 0);

//0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
//1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
//Movement sets 
//Time Variable may be added in the tenth slot [9], values are in divisions. i.e. a value of 10 would mean 10 divisions between the previous state and the new input set, 
//more divisions = slower, time to assume new state in microseconds: t = divisions * setDelay

//Initialisation sets ------------------------------------------------------------------------------
const int16_p PROGMEM startServoState[]   = {0,0,0,0,0,0,0,0,0,1}; // Intant recenter, To neutral position at max speed CURRENTLY UNUSED
const int16_p PROGMEM neutralServoState[] = {0,0,0,0,0,0,0,0,0,10};// Servos centered, with last variable the time variable
const int16_p PROGMEM maxServoState[]     = {100,60,100,100,60,100,0,0,0,10};// Servos at full maximums
const int16_p PROGMEM minServoState[]     = {-100,-100,-50,-100,-100,-50,0,0,0,10};// Servos at full miniimums
//Test Leg state 
const int16_p PROGMEM testServoState[] = {0,0,100,0,0,100,0,0,0,6};// ONLY for testing

//Empty walking sets, defines time variable, other sets time value = 0 ----------------------------------------------------------------------------------
// int emptyMod = 100 This is above in changable states, adjusts height of step
//Only empty sets have a time value to differentiate durations of various stages
//                                          L    L   L   R    R   R   M   M   M   t
const int16_p PROGMEM empty1ServoState[] = {0,   0,  0, -50, -40, 0, -20, 0, 20,  4}; // retracted right leg
const int16_p PROGMEM empty2ServoState[] = {0,   0,  0, -50, -40, 0, -10, 0, 20,  4};
const int16_p PROGMEM empty3ServoState[] = {0,   0,  0,  0,   0,  0,  0,  0,  0,  6};
const int16_p PROGMEM empty4ServoState[] = {0,   0,  0,  0,   0,  0,  0,  0,  0,  6};
const int16_p PROGMEM empty5ServoState[] = {-50,-40, 0,  0,   0,  0, 20,  0, -20, 4}; //retracted left leg
const int16_p PROGMEM empty6ServoState[] = {-50,-40, 0,  0,   0,  0, 10,  0, -20, 4};
const int16_p PROGMEM empty7ServoState[] = {0,   0,  0,  0,   0,  0,  0,  0,  0,  6};
const int16_p PROGMEM empty8ServoState[] = {0,   0,  0,  0,   0,  0,  0,  0,  0,  6};
//Front-back walking sets ------------------------------------------------------------------------------------------------------------------------------
const int GD = gaitAngle/10; //GD = Gait division
const int16_p PROGMEM x1ServoState[] = {0,GD*-1,0,0,GD*5,0,0,0,0,0};
const int16_p PROGMEM x2ServoState[] = {0, GD*1,0,0,GD*-5,0,60,0,0,0};
const int16_p PROGMEM x3ServoState[] = {0, GD*3,0,0,GD*-5,0,30,0,0,0};
const int16_p PROGMEM x4ServoState[] = {0, GD*5,0,0,GD*-3,0,0,0,0,0};
const int16_p PROGMEM x5ServoState[] = {0, GD*5,0,0,GD*-1,0,0,0,0,0};
const int16_p PROGMEM x6ServoState[] = {0,GD*-5,0,0,GD*1,0,-60,0,0,0};
const int16_p PROGMEM x7ServoState[] = {0,GD*-5,0,0,GD*3,0,-30,0,0,0};
const int16_p PROGMEM x8ServoState[] = {0,GD*-3,0,0,GD*5,0,0,0,0,0};
//Left-right walking sets ------------------------------------------------------------------------------------------------------------------------------

const int GSD = gaitSideAngle/10; //GD = Gait side division, simmilar to above
const int16_p PROGMEM y1ServoState[] = {0,0,GSD*-1,0,0,GSD*-5,0,0,0,0};
const int16_p PROGMEM y2ServoState[] = {0,0,GSD*1,0,0,GSD*5,0,0,0,0};
const int16_p PROGMEM y3ServoState[] = {0,0,GSD*3,0,0,GSD*5,0,0,0,0};
const int16_p PROGMEM y4ServoState[] = {0,0,GSD*5,0,0,GSD*3,0,0,0,0};
const int16_p PROGMEM y5ServoState[] = {0,0,GSD*5,0,0,GSD*1,0,0,0,0};
const int16_p PROGMEM y6ServoState[] = {0,0,GSD*-5,0,0,GSD*-1,0,0,0,0};
const int16_p PROGMEM y7ServoState[] = {0,0,GSD*-5,0,0,GSD*-3,0,0,0,0};
const int16_p PROGMEM y8ServoState[] = {0,0,GSD*-3,0,0,GSD*-5,0,0,0,0};
// -----------------------------------------------------------------------------------------------------------------------------------------------------
//1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
// Constant parameters and various other variable definitions 22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222

//Servo Numbers --------------------------------------------------------------------------------------------------------------------------------------------------------------------
const int16_p PROGMEM  LKneeServo = 0;
const int16_p PROGMEM  LHipServo = 1;
const int16_p PROGMEM  LHipSideServo = 2;
const int16_p PROGMEM  RKneeServo = 15;
const int16_p PROGMEM  RHipServo = 14;
const int16_p PROGMEM  RHipSideServo = 13;
const int16_p PROGMEM  lookServo = 4;
const int16_p PROGMEM  pitchServo = 5;
const int16_p PROGMEM  rollServo = 6;
//the above in a set for utilisations in fucntions:
const int servoSet[] = {LKneeServo, LHipServo, LHipSideServo, RKneeServo, RHipServo, RHipSideServo, lookServo, pitchServo, rollServo};

//Calibration for centers of servos, i.e. neutral angular positions,  ----------------------------------------------------------------------------------------------------------------
const int16_p PROGMEM  LKneeCenter = 100; //clockwise looking left
const int16_p PROGMEM  LHipCenter = 88+2;
const int16_p PROGMEM  LHipSideCenter = 90; //lowet, inner leg
const int16_p PROGMEM  RKneeCenter = 165; //counterclockwise looking left
const int16_p PROGMEM  RHipCenter = 90-2;
const int16_p PROGMEM  RHipSideCenter = 85;//Higher, leg goes towards center line
const int16_p PROGMEM  lookCenter = 93;//lower look right
const int16_p PROGMEM  pitchCenter = 88;//lower for higher head
const int16_p PROGMEM  rollCenter = 95;
//the above in a set for utilisations in fucntions:
const int servoCenter[] = { LKneeCenter, LHipCenter, LHipSideCenter, RKneeCenter, RHipCenter, RHipSideCenter, lookCenter, pitchCenter, rollCenter};// Servo centers in a set

//Movement ranges, maximum degrees of movement for each joint ---------------------------------------------------------------------------------------------------------------------
const int16_p PROGMEM  LKneeRange = 50;//range of degrees of Left Knee servo is +-60
const int16_p PROGMEM  LHipRange = 50;
const int16_p PROGMEM  LHipSideRange = 40;
const int16_p PROGMEM  RKneeRange = -50;
const int16_p PROGMEM  RHipRange = -50;
const int16_p PROGMEM  RHipSideRange = -40;
const int16_p PROGMEM  lookRange = 30;
const int16_p PROGMEM  pitchRange = 30;
const int16_p PROGMEM  rollRange = 35;
//the above in a set for utilisations in fucntions:
const int servoRange[] = {LKneeRange, LHipRange, LHipSideRange, RKneeRange, RHipRange, RHipSideRange, lookRange, pitchRange, rollRange};// Servo ranges in a set

//initial command state sent over bluetooth
int state = 0;

//needed for delayed set execution loop below
int timeOfLastLoop = 0;
//needed for delayed IMU measurement loop below
int timeOfLastIMU = 0;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ, yaw, roll, pitch;
  float GyroX;
  float GyroY;
//needed for delayed ultrasonic measurement loop below, in microseconds
int updateDistance = 0; //is set to 1 when distance needs to be updated.
int timeOfLastUltra = 0;
int ultraState = 0; //controls if the ultrasonic sensor is resting or pulsing for 10 microseconds.
int distanceSensed; //global variable storing distance of obastacle sensend by ultrasonic sensor, updates every 20 microseconds.

//temporary memory of its current position, for calculation of eased servo movement to the new state
int oldSet[] = {0,0,0,0,0,0,0,0,0,1}; //old set starts with neutral positions

//the "plate" of sets the code will need to "eat" one by one in the loop every setDelay amount of time.
int pendingSet[maxDivisions+1][9];
  //pendingSet[maxDivisions][9] = 0; // This line is in setup, the maxdivisions value in the set states how many "edible" sets are currently being stored and can be "eaten"

//Set that generate step writes to
int generatedSet[10];

//variable to allow neutral position to be executed only once upon entering neutral position
int enterNeutral = 1;

//Lookup table for fuzzy logic
const int16_p PROGMEM fuzzTable[51][51] = { 
 {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -17, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -34, -33, -32, -31, -30, -28, -27, -26, -24, -23, -21, -20, -18, -17, -16, -16, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -33, -32, -31, -30, -28, -27, -25, -24, -23, -21, -20, -18, -17, -16, -16, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14}, 
  {-31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -31, -30, -29, -28, -26, -25, -23, -22, -21, -19, -18, -17, -16, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13}, 
  {-29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -29, -28, -27, -25, -24, -22, -21, -19, -18, -17, -16, -15, -14, -14, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12}, 
  {-26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -25, -24, -22, -21, -19, -18, -17, -15, -14, -14, -13, -12, -12, -12, -12, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11}, 
  {-23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -23, -22, -21, -19, -18, -16, -15, -14, -13, -12, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10}, 
  {-20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -20, -19, -17, -16, -15, -13, -12, -11, -10, -10, -9, -9, -9, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8}, 
  {-17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -17, -15, -14, -13, -11, -10, -9, -8, -8, -7, -7, -7, -7, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6}, 
  {-14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -13, -13, -12, -10, -9, -8, -7, -6, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4}, 
  {-11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -9, -8, -6, -5, -4, -4, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2}, 
  {-7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -5, -5, -3, -2, -1, -1, -0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
  {-4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -2, -1, -0, 1, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}, 
  {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -0, -0, 0, 1, 1, 2, 3, 5, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7}, 
  {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4, 4, 5, 6, 8, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11}, 
  {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 7, 8, 9, 10, 12, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14}, 
  {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 9, 10, 11, 13, 14, 15, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17}, 
  {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11, 12, 13, 15, 16, 17, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20}, 
  {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 12, 13, 14, 15, 16, 18, 19, 21, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23}, 
  {11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 13, 14, 14, 15, 17, 18, 19, 21, 22, 24, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26}, 
  {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 15, 16, 17, 18, 19, 21, 22, 24, 25, 27, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29}, 
  {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 28, 29, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 16, 16, 17, 18, 20, 21, 23, 24, 25, 27, 28, 30, 31, 32, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 16, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  {14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 16, 17, 17, 18, 20, 21, 23, 24, 26, 27, 28, 30, 31, 32, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34}, 
  }; 

//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
// Predefined Functions and calculations 333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

//Servodriver Pulse calculations -------------------------------------------------------------------------------------------------------------------------------
int anglePulse (int angle){
  int Pulse = map(angle, 0, 180, 110, 400);
  return Pulse;
//, MIN pulse = 110, Max pulse = 610, pulse to 180*= 400
}

//executePosition function, Write position dataset to servos ----------------------------------------------------------------------------------------------------------------------
void executePosition( int pendingSetIndex ){
  //Serial.println(F("executed positions"));
  for (int i=0; i<9; i++){
    int writeDegree = map(pendingSet[pendingSetIndex][i], -100, 100, (-servoRange[i]+servoCenter[i]), (servoRange[i]+servoCenter[i]) ); //maps the -100 to 100 input to the maximum and minimums of each servo
    pwm.setPWM(servoSet[i], 0, anglePulse(writeDegree));
    //Serial.println(pendingSet[pendingSetIndex][i]);
    
  }
}

//Assume position function --------------------------------------------------------------------------------------------------------------------------------------------------------
//generates a new array of arrays into pendingSet that the loop will work through, Nature determines how the movement is performed, either Linear (0) or sine-eased (1)
void assumePosition(int newSet[], int nature){
  //One servo number at a time
//Serial.println("assumePosition() started");

//set of values to be added to pending sets once generated
  int transitionArray[9][maxDivisions] ;
  int divisions = newSet[9]; //the 9th value in a servo set is the time value

  for (int servoNumber = 0; servoNumber<9;servoNumber++){
    
    //First, Multiplier array is made depending on the nature selected
    float multiplierArray[maxDivisions];
    int oldValue = oldSet[servoNumber];
    int newValue = newSet[servoNumber];
    float increment = 0;

    //Generate Multiplier Array ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if (nature == 0){//Linear movement, constant speed
      increment = ( 1.0 / divisions);

      for ( int i = 0; i < divisions ; i++){
        float multiplier = increment * (i+1);
        multiplierArray[i] = multiplier;
      }
    }
    if (nature == 1){//Sin movement, deccelerate
      increment = ( 1.570796 / divisions);

      for ( int i = 0; i < divisions ; i++){
        float multiplier = sin(increment * (i+1) );
        multiplierArray[i] = multiplier;
      }
    }
    if (nature == 2){//Cos movement, accelerate
      increment = ( 1.570796 / divisions);

      for ( int i = 0; i < divisions ; i++){
        float multiplier = (cos((increment * (i+1))+ 3.1415926 )) + 1;
        multiplierArray[i] = multiplier;
      }
    } // Multiplier array now generated +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //make column of values for current servo it is calculating
    for ( int i = 0; i < divisions ; i++){
      
      int value = oldValue + ((newValue-oldValue)*multiplierArray[i]);
      transitionArray[servoNumber][i] = value;  
      }
  }// end of respective servo number loop


//write to pending sets from transition array, transposing matrix
  for ( int i = 0; i < divisions ; i++){
    for ( int s = 0; s < 9 ; s++){
      pendingSet[divisions-i-1][s] = transitionArray[s][i];
      //Serial.println(pendingSet[divisions-i-1][s]);
    }
    //Serial.print("one pending set added at index: ");
    //Serial.println(divisions-i-1);
  }
pendingSet[maxDivisions][8] = divisions;//tell pending set how many things there are to "eat" 

//make oldSet = newSet
  for (int i=0; i<9; i++){
    oldSet[i] = newSet[i];
  }

} // end of assume position function

//Step Direction Combining ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//takes all 3 sets and their modifiers/weightages and updates "generatedSet" to be the new set to be written
void generateSet( int set1[] , int set1Mod , int set2[] , int set2Mod ,int set3[] , int set3Mod ){
  // 10 values to combine and output, modifiers are 0-100, 
  for (int i=0; i<9; i++){
    int newValue = (set1[i]*set1Mod) + (set2[i]*set2Mod) + (set3[i]*set3Mod);
    generatedSet[i] = newValue / 100;
  }
  generatedSet[9] = set1[9];
  
//    for (int i=0; i<10; i++){
//    Serial.print(outputSet[i]);
//    Serial.print(", ");
//  }
//  Serial.println(" ");
  
}


//3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
//44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444



//Setup 
void setup() {
  Serial.println(F("Void Setup Begin."));
  Serial.begin(9600);
  
  pwm.begin();//ServoDriver Required
  pwm.setPWMFreq(60);  // This is the maximum PWM frequency
  yield();

  //Wait for IMU to initialise
  if (!icm.begin_I2C()) {
    Serial.println(F("Failed to locate ICM20948 chip"));
    while (1) {
      delay(10);
    }
  }  
  icm_gyro = icm.getGyroSensor(); //Initialise Gyro
  icm_mag = icm.getMagnetometerSensor(); //Initialise Magnetometer
  icm_accel = icm.getAccelerometerSensor();

  pinMode(10, OUTPUT); //ultrasonic sensor pin initialisation
  pinMode(11, INPUT);


  int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = neutralServoState[i]; } 
   assumePosition(bufferSet, 1);
   cycleStage = mode; // Primes cycle stage to begin, starts with dynamic neutral stage 9
   delay(3000); //3 seconds to place robot on surface before IMU takes calibration data of "flat surface"


} //end of setup 

//444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
//5555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555
// Void Loop 
void loop() {

//Serial.print("cycleStage: "); Serial.print(cycleStage); Serial.print(", xMod:"); Serial.print(xMod); Serial.print(", yMod:"); Serial.println(yMod); 

  //Serial.println();
  //check controller command data  
  //Read Command
 if(Serial.available() > 0){ // Checks whether data is coming from the serial port
    state = Serial.read(); // Reads the data from the serial port
    //Serial.print("Bluetooth: "); Serial.println(state);
 }
 
    int currentTime = millis();
  //int timeOfLastIMU = 0; this line is in section 222222

  //Start of ultrasonic sensor -----------------------------------------------------------------
  if (updateDistance == 1){
  int microsecs = micros(); //micros utilised to avoid using delayMicroseconds()
  int timeSinceLastUltra = microsecs - timeOfLastUltra;
  if (timeSinceLastUltra >= 10){ //this if function runs every 10 microseconds
    if (ultraState == 0){ //turn on pulse
      digitalWrite(10, HIGH);
      ultraState = 1;
    }
    if (ultraState == 1){ //turns off pulse and records duration, returns distance
      digitalWrite(10, LOW );
      int duration = pulseIn(11, HIGH);
      distanceSensed = duration * 0.034 / 2;
      //Serial.println(distanceSensed);
      ultraState = 0;
    }
  }
  updateDistance = 0;
  } //End of ultrasonic sensor -----------------------------------------------------------------
  
  
  //Start of IMU *************************************************************************************************************************************************************************
  int timeSinceLastIMU = currentTime - timeOfLastIMU;
  if (timeSinceLastIMU > IMUDelay - 1 ){ // Start of delayed IMU loop, be=ased on IMU delay
  //Obtain updated IMU readings
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t accel;
  icm_gyro->getEvent(&gyro);
  icm_mag->getEvent(&mag);
  icm_accel->getEvent(&accel);

  float AccX = accel.acceleration.x; 
  float AccY = accel.acceleration.y;
  float AccZ = accel.acceleration.z;
  float MagX = mag.magnetic.x;
  float MagY = mag.magnetic.y;
  float MagZ = mag.magnetic.z;
  GyroX = (gyro.gyro.x *57.3) +1.45;// convert to Deg/s, account for error. 1.45, these 2 are declared above
  GyroY = (gyro.gyro.y *57.3) -0.19;
  float GyroZ = (gyro.gyro.z *57.3) ;
  // float roll, pitch; in variables

  //Calculate position from acceleraometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) +2.4; //error of 2.4
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) -2.4; 

  //Calculate position from Gyro data
  float elapsedTime = (timeSinceLastIMU / 1000.0);
  
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  //Combine data for more accurate positiining w/out drift
  pitch = 0.96 * gyroAngleX + 0.04 * accAngleX;
  roll = 0.96 * gyroAngleY + 0.04 * accAngleY;

  //Serial print X and Y positioning
  //Serial.print(pitch);Serial.print(", ");
//  Serial.print(roll);
//  Serial.println();

    timeOfLastIMU = currentTime;
  }//end of delayed IMU loop ----------------------------------------

//end of IMU *************************************************************************************************************************************************************************

//Produce Direction Information, Xmod and Ymod based on 3 inputs: ultrasonic sensor data, IMU/fuzzy logic, and control info %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//First, map IMU outputs to fuzzy logic Crisp inputs.
//Filter IMU data to avoid spikes/bumpiness from walking
filteredPitch.Filter(pitch); //angle varies +-20
filteredRoll.Filter(roll); //angle varies +-20
filteredAngVelX.Filter(GyroX); // +-100
filteredAngVelY.Filter(GyroY); // +-100

//Map extremes of value into fuzzy logic crisp input table, update depending on size of lookup table
int crispAngX = map(filteredPitch.Current() , -20,20,0,51);
int crispAngY = map(filteredRoll.Current() , -20,20,0,51);
int crispAngVelX = map(filteredAngVelX.Current() , -100,100,0,51);
int crispAngVelY = map(filteredAngVelY.Current() , -100,100,0,51);

//  Serial.print(GyroY);Serial.print(", ");
//  Serial.print(filteredAngVelY.Current());
//  Serial.println();

//extract data from look-up table to get fuzzXmod and fuzzYmod
int fuzzXmod;
int fuzzYmod;
if(crispAngX < 50 && crispAngX > 0 && crispAngY < 50 && crispAngY > 0 && crispAngVelX < 50 && crispAngVelX > 0 && crispAngVelY < 50 && crispAngVelY > 0 ){ //if statment to ensure values are within the bounds of lookuptable and does not extract random data from ram
fuzzXmod = fuzzTable[crispAngX][crispAngVelX];
fuzzYmod = fuzzTable[crispAngY][crispAngVelY];
}

//extract data from ultrasonic sensor
int obstacleMod;
if (distanceSensed < 20){
  obstacleMod = -20 ; //if less than 20cm, nudge backwards
}
if (distanceSensed >= 20){
  obstacleMod = 0 ; 
}

//extract data from remote controller
int controllerModX = 0;
int controllerModY = 0;
int controllerModXA = 0;
int controllerModXB = 0;
int controllerModYA = 0;
int controllerModYB = 0;
if (state >= 10 && state <= 19){ //front
  int frontState = state - 10;
    controllerModXA = frontState*3; //adds a maximum nudge of 30 to mod
 }
if (state >= 50 && state <= 59){ //back
  int backState = state - 50;
    controllerModXB = -backState*3; //adds a maximum nudge of -30 to mod
 }
if (state >= 30 && state <= 39){ //right
  int rightState = state - 30;
    controllerModYA = rightState*3; //adds a maximum nudge of 30 to mod
 }
if (state >= 70 && state <= 79){ //left
  int leftState = state - 70;
    controllerModYB = -leftState*3; //adds a maximum nudge of -30 to mod
 }
 controllerModX = controllerModXA + controllerModXB;
 controllerModY = controllerModYA + controllerModYB;
 


//Serial.print("ControllerModX: ");Serial.print(controllerModX);Serial.print(" ControllerModY: ");Serial.println(controllerModY);

//combine modifiers according to predetermined weights

xMod = (fuzzXmod*fuzzWeight*1 + controllerModX*controllerWeight + obstacleMod*100)/100;
yMod = (fuzzYmod*fuzzWeight*1 + controllerModY*controllerWeight)/100;

//Xmod and Ymod found %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


// Stand at neutral:
  if (cycleStage == 20 && pendingSet[maxDivisions][8] == 0){
    if (enterNeutral == 1){
    //extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = startServoState[i]; }
    assumePosition(bufferSet, 0);
    enterNeutral = 0;
    }
  }
// Test State position:
  if (cycleStage == 30 && pendingSet[maxDivisions][8] == 0){
    
    //extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = testServoState[i]; }
  
    assumePosition(bufferSet, 0);
  }

////first time initialisation loop ----------------------------------------------------------------------
//int cycleStage == 10 this is in changeable variables
  if (cycleStage == 10 && pendingSet[maxDivisions][8] == 0){
    //extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = minServoState[i]; }
    assumePosition(bufferSet, 2);
    cycleStage = 11;
  }
  if (cycleStage == 11 && pendingSet[maxDivisions][8] == 0){
//extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = maxServoState[i]; }  
    assumePosition(bufferSet, 0);
    cycleStage = 12;
  }
  if (cycleStage == 12 && pendingSet[maxDivisions][8] == 0){
//extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = neutralServoState[i]; } 
    assumePosition(bufferSet, 1);
    cycleStage = 1; // Primes cycle stage to begin
  }
////end of first time initialisation --------------------------------------------------------------------


//555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555
//66666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666


//CYCLE STAGES ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Dynamic Neutral, stands still when modifiers are low

if (cycleStage == 9  && (not(xMod < stillThreshold && xMod > -stillThreshold ) || not(yMod < stillThreshold && yMod > -stillThreshold)) && pendingSet[maxDivisions][8] == 0) { //break out of neutral (stage 9)
  cycleStage = 2; //moves rightfoot first
  enterNeutral = 1;
}

if ( (xMod < stillThreshold && xMod > -stillThreshold ) && (yMod < stillThreshold && yMod > -stillThreshold) && pendingSet[maxDivisions][8] == 0){ //forced into neutral
  cycleStage = 9;
  updateDistance = 1; //take distance measurements when stationary

  //only write neutral positions once upon entering neutral
  //int enterNeutral = 0; //this is in pre setup
  if (enterNeutral == 1){
//extract set from PROGMEM into buffer set
    int bufferSet[10];
  for (int i=0; i<10; i++){ bufferSet[i] = neutralServoState[i]; } 
    assumePosition(bufferSet, 1);
    enterNeutral = 0; // prevents constant writing of neutral position during rest
  }
  }


//Normal cycle stages ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  if (cycleStage == 1 && pendingSet[maxDivisions][8] == 0){
    updateDistance = 1; //enables the ultrasonic sensor to obtain a distance reading.
    setDelay = setDelayX; //makes setdelayX the chosen variable above instead of slower initialisation variable.
    state = 0;// reset bluetooth data state
    
 //Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty1ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x1ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y1ServoState[i]; }
    
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 2;
  }
  if (cycleStage == 2 && pendingSet[maxDivisions][8] == 0){
  //Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty2ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x2ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y2ServoState[i]; }
    
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 3;
  }
  if (cycleStage == 3 && pendingSet[maxDivisions][8] == 0){
 //Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty3ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x3ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y3ServoState[i]; }
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 4;
  }
  if (cycleStage == 4 && pendingSet[maxDivisions][8] == 0){
    state = 0;// reset bluetooth data state
//Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty4ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x4ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y4ServoState[i]; }
    
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 5;
  }
  if (cycleStage == 5 && pendingSet[maxDivisions][8] == 0){
//Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty5ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x5ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y5ServoState[i]; }
    
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 6;
  }
if (cycleStage == 6 && pendingSet[maxDivisions][8] == 0){
//Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty6ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x6ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y6ServoState[i]; }
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 7;
  }
  if (cycleStage == 7 && pendingSet[maxDivisions][8] == 0){
//Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty7ServoState[i]; }
  
    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x7ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y7ServoState[i]; }
    
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 8;
  }
  
    if (cycleStage == 8 && pendingSet[maxDivisions][8] == 0){
   //Extract sets from PROGMEM into SRAM buffersets
    int bufferSetA[10];
  for (int i=0; i<10; i++){ bufferSetA[i] = empty8ServoState[i]; }

    int bufferSetB[10];
  for (int i=0; i<10; i++){ bufferSetB[i] = x8ServoState[i]; }

    int bufferSetC[10];
  for (int i=0; i<10; i++){ bufferSetC[i] = y8ServoState[i]; }
    generateSet(bufferSetA, emptyMod, bufferSetB, xMod, bufferSetC, yMod );
    assumePosition(generatedSet, 0);
    cycleStage = 1;
  }
//END OF CYCLE STAGE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//6666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666
//77777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777

//delayed code loop, code below only occurs once every setDelay() milliseconds to execute "edible" sets -----------------------------------------------------------------------------------------------------------------
  //int currentTime = millis(); Already above
  //int timeOfLastLoop = 0; this line is in section 222222
  
  int timeSinceLastLoop = currentTime - timeOfLastLoop;
  
//Code that "eats" and executes from the pending set  
  if (timeSinceLastLoop > setDelay -1){

    int availableSets = pendingSet[maxDivisions][8];// available set determines both the number of sets inpending sets to be executed, and the index of that set to be used (-1 because of zero indexing)
    if (availableSets != 0){
      executePosition(availableSets-1); //takes the most right-side non-zero set and executes it
      //RYAN, you may need to delete the set that has been "eaten" (15/3/22), nope, not needed (1/4/22)
      pendingSet[maxDivisions][8] = (pendingSet[maxDivisions][8]) - 1 ; //number of "edible" sets now decreased by 1 
//      Serial.print("delayed loop ran, and pending sets = ");
//      Serial.println(pendingSet[maxDivisions][8]);
//      Serial.print("executed position in pendingSets indexed at: ");
//      Serial.println((availableSets-1));

    } // end of if statement that eats pending set

    timeOfLastLoop = currentTime;
  }//end of delayed loop ------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Manual Pitch Control------------------------------------------------------------------------

//RYAN add that this can only occur if pavo is in stationary mode and break out of it back to normal balancing if it gets too unbalance while playing with it
//manual pitch control
 if (state >= 110 && state <= 119){
  int pitchState = state - 110;
  int pitchDegree = map(pitchState, 9,0, (-pitchRange+pitchCenter),(pitchRange+pitchCenter));
  pwm.setPWM(pitchServo, 0, anglePulse(pitchDegree));
  
 }
//Manual Roll Control
 if (state >= 120 && state <= 129  && cycleStage == 9){
  int rollState = state - 120;
  int rollDegree = map(rollState, 0,9, (-rollRange+rollCenter),(rollRange+rollCenter));
  pwm.setPWM(rollServo, 0, anglePulse(rollDegree));
 }
 //Manual look Control
 if (state >= 130 && state <= 139  && cycleStage == 9){
  int lookState = state - 130;
  int lookDegree = map(lookState, 0,9, (-lookRange+lookCenter),(lookRange+lookCenter));
  pwm.setPWM(lookServo, 0, anglePulse(lookDegree));
 }
 //------------------------------------------------------------------------------------------



} // End of Void Loop







//                  ___________     .===.
//                                 /-=(o=`.
//                                | ,__.-"``
//        __________             _/ :  /
//                      _..--""`  :   /
//                   .-" : '    :    |
//    __________   ."  :       :     ;
//               ."  :       :       /
//          _,.-" :     .  :        /
//          `---'=-.,_;` '       ,='
//                     `:=-=.=-'====,_
//      _________       //`          `\\
//                     //
//                    //        run Pavo run
//                   `\=   
