#include <Wire.h>  //used for I2C communication
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>
#include <filters.h>


// creates I2C object for IMU called lsm to call functions from LSM9DS1 library
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

//sets LPF parameters
const float cutoff_freq = 15.0; //Cutoff freq in Hz
const float sampling_time = 0.01;  //Sampling time in seconds
IIR::ORDER order = IIR::ORDER::OD3;
Filter f(cutoff_freq, sampling_time, order);


//Initialising gyroscope estimate
  float thetaHat = 0.0f;
  float phiHat = 0.0f;

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

}


void setup() {
  Serial.begin(115200);

  while (!Serial) {
    delay(1);  // will pause Zero, Leonardo, etc until serial monitor opens
  }

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

  lsm.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
    //reading xyz accelerations from accelerometer
    float ax = a.acceleration.x; //defining values for xyz accel
    float ay = a.acceleration.y;
    float az = a.acceleration.z - 3.99867; //offset calibration for z axis

    //Applying low pass filter to the accelerometer raw data    
    float ax_lpf = f.filterIn(ax);
    float ay_lpf = f.filterIn(ay);
    float az_lpf = f.filterIn(az);

    //reading xyz gyro readings
    float gx = g.gyro.x;
    float gy = g.gyro.y;
    float gz = g.gyro.z;
  
     //Applying low pass filter to the gyroscope raw data
    float gx_lpf = f.filterIn(gx);
    float gy_lpf = f.filterIn(gy);
    float gz_lpf = f.filterIn(gz);

  //prints x y z accelerations in serial monitor
  // Serial.print("Accel X: ");
  //Serial.print(ax);
  // Serial.print(" m/s^2");
  //Serial.print(",");
  //Serial.print(ay);
  // Serial.print(" m/s^2 ");
  //Serial.print(",");
  //Serial.print(az);
  // Serial.println(" m/s^2 ");

  //Calculating accelerometer estimate for roll and pitch in radians
  //to calculate in deg, just multiply by * 57296 / 1000
  float thetaa = atan2(ax, az); //x-z tilt, roll
  float phia = atan2(ay, az); //y-z tilt, pitch

  // Serial.print(thetaa);
  // Serial.print(",");
  Serial.print(phia);
  Serial.print(",");

  // Serial.print(",");
  // Serial.print(theta);
  // Serial.print(",");
  // Serial.print(phi);

  // Serial.print(gx_lpf);
  // Serial.print("\t");
  // Serial.print(gx);

  //converting body rates relative to imu pcb to euler rates relative to fixed inertial frame 
  float phiDot_rps = gx + gy*tanf(thetaHat)*sinf(phiHat) + gz*tanf(thetaHat)*cosf(phiHat);
  float thetaDot_rps = gy*cosf(thetaHat) - gz*sinf(thetaHat);

  //implementing complimentary filter
  // angle_estimate = accelerometer_estimate[rad]*alpha + (1-alpha)*(previous_angle_estimate[rad] + sample_time[s]*gyroscope_angle_rate[rad/s])
  float phi = 0.05 * phia + (1 - 0.05) * (phiHat + (70/1000)*phiDot_rps); 
  float theta = 0.05 * thetaa + (1 - 0.05) * (thetaHat + (70/1000)*thetaDot_rps);

  Serial.print(phiHat);
  Serial.print(",");
  // Serial.print(theta);

  phiHat = phi;
  thetaHat = theta;  

  // //prints x y z mag flux density in serial monitor
  // Serial.print("Mag X: ");
  // Serial.print(m.magnetic.x);
  // Serial.print(" uT");
  // Serial.print("\tY: ");
  // Serial.print(m.magnetic.y);
  // Serial.print(" uT");
  // Serial.print("\tZ: ");
  // Serial.print(m.magnetic.z);
  // Serial.println(" uT");

  // //prints x y z angular velocity in serial monitor
  // Serial.print("Gyro X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(" rad/s");
  // Serial.print("\tY: ");
  // Serial.print(g.gyro.y);
  // Serial.print(" rad/s");
  // Serial.print("\tZ: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  Serial.println();

  delay(70);
}