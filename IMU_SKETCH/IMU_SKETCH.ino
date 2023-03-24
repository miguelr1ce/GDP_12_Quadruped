#include <Wire.h>  //used for I2C communication
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>


// creates I2C object for IMU called lsm to call functions from LSM9DS1 library
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
//the following is code from example that i didn't delete [Miguel]:
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


//Initialising gyroscope estimate
  float thetaHat = 0.0f;
  float phiHat = 0.0f;
  int time_delay = 50;

void setupSensor() {
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() {
  Serial.begin(115200);
  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1)
      ;
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, defined above
  setupSensor();
}


void loop() {

  //ask it to read in the data
  lsm.read(); 

  //Get a new sensor event
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  //reading xyz accelerations from accelerometer
  float ax = a.acceleration.x; //defining values for xyz accel
  float ay = a.acceleration.y;
  float az = a.acceleration.z - 3.99867; //offset calibration for z axis

  //reading xyz gyro readings in rad/s
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  //Calculating accelerometer estimate for roll and pitch in radians
  //to calculate in deg, just multiply by * 57296 / 1000
  float thetaa = atan2(ax, az); //x-z tilt, roll
  float phia = atan2(ay, az); //y-z tilt, pitch

  //converting body rates relative to imu pcb to euler rates relative to fixed inertial frame
  //gx gy gz are from accel the hats are from gyro
  float phiDot_rps = gx + gy*tanf(thetaHat)*sinf(phiHat) + gz*tanf(thetaHat)*cosf(phiHat);
  float thetaDot_rps = gy*cosf(thetaHat) - gz*sinf(thetaHat);


  //implementing complimentary filter
  //angle_estimate[rad] = accelerometer_estimate[rad]*alpha + (1-alpha)*(previous_angle_estimate[rad] + sample_time[s]*gyroscope_angle_rate[rad/s])
  //to calculate in deg, just multiply by * 57296 / 1000
  float alpha = 0.20; //alpha is not an angle -> number to decide weightage of accel and gyro 
  float phi = alpha * phia + (1 - alpha) * (phiHat + (time_delay/1000)*phiDot_rps); 
  float theta = alpha * thetaa + (1 - alpha) * (thetaHat + (time_delay/1000)*thetaDot_rps);

  //adjusting offset (final ans) in degrees
  float phi_output_deg = phi* float(57296) / float(1000) + 3.35;
  float theta_output_deg = theta* float(57296) / float(1000) - 0.29;
  float phi_output_rad = phi_output_deg * float(1000) / float(57296);
  float theta_output_rad = theta_output_deg * float(1000) / float(57296);

  //print accelerometer estimate in deg (innacurate)
  //Serial.print(thetaa * float(57296) / float(1000));
  //Serial.print(",");
  //Serial.print(phia* float(57296) / float(1000) + 3.35);
  //Serial.print(",");

  //printing fused sensor estimate in deg (accurate)
  Serial.print(theta_output_deg);
  Serial.print(",");
  //Serial.print(phi_output_deg);
  //Serial.print(",");
  
  //saving fused sensor estimate to previous value
  phiHat = phi; //pitch
  thetaHat = theta; //roll


  //defining length of foot relative to IMU
  float length1 = 100;
  float length2 = 100;

  //calculating offset needed for balance offset
  // a = b sin( angle[rad] )
  float balance_offset1 = length1 * sin(theta_output_rad);
  float balance_offset2 = length2 * sin(phi_output_rad);

  Serial.print(balance_offset1);
  //Serial.print(balance_offset2);


  Serial.println();
  delay(time_delay);
}