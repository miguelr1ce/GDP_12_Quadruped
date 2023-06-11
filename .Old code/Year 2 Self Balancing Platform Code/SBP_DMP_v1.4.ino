
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <Servo.h>
MPU6050 mpu;
#define INTERRUPT_PIN 2 
#define LED_PIN 13 
bool blinkState = false;

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// platform dimensions
float radius;           // radius of platform
float l1;               // perpendicular length from centre to motors B and C
int arm = 2;            // length of motor arm (in cm)

// Servo motor variables
Servo Servo_A;
Servo Servo_B;
Servo Servo_C;
int angle_A;
int angle_B;
int angle_C;
int angle_BC_pitch;
int angle_B_roll;
int angle_C_roll;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock

    // calculate platform dimensions (in cm)
    radius = 6.6/sin(PI/3);
    l1 = 6.6/tan(PI/3);

    // servo input pins
    Servo_A.attach(9);
    Servo_B.attach(10);
    Servo_C.attach(11);
    
    // initialize serial communication
    Serial.begin(115200);
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply gyro offsets, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    // calibration and enable dmp
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialisation failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);

        // Calculate angles for servos

        angle_A = int(round(asin(radius * sin(ypr[1]) / arm) * 180/M_PI));
        angle_BC_pitch = - int(round(asin(l1 * sin(ypr[1]) / arm) * 180/M_PI));
        angle_B_roll = - int(round(asin(6.6 * sin(ypr[2]) / arm) * 180/M_PI));
        
        if (abs((ypr[1])) >= asin(arm/ radius)) {
          angle_A = (ypr[1]/abs(ypr[1])) * 90;
          angle_BC_pitch = - (ypr[1]/abs(ypr[1])) * int(round(asin(l1 / radius) * 180/M_PI));
        }
        if (abs((ypr[2])) >= asin(arm/ radius)) {
          angle_B_roll = - (ypr[2]/abs(ypr[2])) * int(round(asin(6.6 / radius) * 180/M_PI));
        }
        angle_C_roll = - angle_B_roll;
        angle_B = angle_BC_pitch + angle_B_roll;
        angle_C = angle_BC_pitch + angle_C_roll;
        Servo_A.write(90 - angle_A);
        Servo_B.write(90 - angle_B);
        Servo_C.write(90 - angle_C);
        Serial.print("\t");
        Serial.print("A B C\t");
        Serial.print(angle_A);
        Serial.print("\t");
        Serial.print(angle_B);
        Serial.print("\t");
        Serial.println(angle_C);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
