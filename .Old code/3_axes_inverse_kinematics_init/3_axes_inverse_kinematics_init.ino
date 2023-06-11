#include <math.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial& odrive_serial = Serial1;
HardwareSerial& odrive_serial2 = Serial2;

// ODrive object
ODriveArduino odrive1(odrive_serial);
ODriveArduino odrive2(odrive_serial2);

float leglen = 140.0;
float feetlen = 10;
float M0,M1,M2;
int xx;
int yy;
int zz;
double *phi = (double *) malloc(sizeof(double)*3);

//Structure Definition
struct gait_struct{
  double March[8][4][3]={{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}}} ;
  double FB[8][4][3]={{{0, -60, 0}, {0, -20, 0}, {0, 20, 0}, {0, 60, 0}}, {{0, 0, 0}, {0, -40, 0}, {0, 0, 0}, {0, 40, 0}}, {{0, 60, 0}, {0, -60, 0}, {0, -20, 0}, {0, 20, 0}}, {{0, 40, 0}, {0, 0, 0}, {0, -40, 0}, {0, 0, 0}}, {{0, 20, 0}, {0, 60, 0}, {0, -60, 0}, {0, -20, 0}}, {{0, 0, 0}, {0, 40, 0}, {0, 0, 0}, {0, -40, 0}}, {{0, -20, 0}, {0, 20, 0}, {0, 60, 0}, {0, -60, 0}}, {{0, -40, 0}, {0, 0, 0}, {0, 40, 0}, {0, 0, 0}}};
  double LR[8][4][3]={{{-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}, {60, 0, 0}}, {{0, 0, 0}, {-40, 0, 0}, {0, 0, 0}, {40, 0, 0}}, {{60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}}, {{40, 0, 0}, {0, 0, 0}, {-40, 0, 0}, {0, 0, 0}}, {{20, 0, 0}, {60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}}, {{0, 0, 0}, {40, 0, 0}, {0, 0, 0}, {-40, 0, 0}}, {{-20, 0, 0}, {20, 0, 0}, {60, 0, 0}, {-60, 0, 0}}, {{-40, 0, 0}, {0, 0, 0}, {40, 0, 0}, {0, 0, 0}}} ;
};

//Gait Initialisation  -  Copy gait matrix from GUI
gait_struct gait;

//Function Prototypes
void kinematics_3d(double x, double y, double z, double *p);
void drive(int leg,double* p,float a,float b,float c);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  //Arrays: [iterations][legs][x,y,z coordinates]
  //Serial.println(gait.LR[0][1][0]);
  //Serial.println(abs(atan(200/0.00)));
  M0 = odrive1.GetPosition(0);
  M1 = odrive1.GetPosition(1);
  M2 = odrive2.GetPosition(0);
  Serial.print(M0); Serial.print('\t');Serial.print(M1); Serial.print('\t');Serial.print(M2); Serial.print('\n');
  Serial2 << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << odrive2.readFloat() << '\n';
  Serial1 << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << odrive1.readFloat() << '\n';
  odrive1.run_state(0, 8, false);
  odrive1.run_state(1, 8, false);
  odrive2.run_state(0, 8, false);
  //while (!Serial) ; // wait for Arduino Serial Monitor to open
  
  //$$$$$$$$$$$$$$$$$[Leg 1 : FR, Leg 2: BL, Leg 3: BR, Leg 4:FL]$$$$$$$$$$$$$$$$$$$$$$$$$$//
}

void loop() {
  if (Serial.available()) {
    xx = Serial.parseInt();
    yy = Serial.parseInt();
    zz = Serial.parseInt();
    double xf = (double) xx;
    double yf = (double) yy;
    double zf = (double) zz;
    kinematics_3d(xf,yf,zf,phi);
    Serial.println(xf);
    Serial.println(yf);
    Serial.println(zf);
    Serial.println(phi[0]);
    Serial.println(phi[1]);
    Serial.println(phi[2]);
    drive(1,phi,M0,M1,M2);
  }
  delay(5);
}

void kinematics_3d(double x, double y, double z, double *p){

    // negative x to the left, positive to the right
   double c1;
   double c2;
   double theta;
   double beta;
   double alpha;
   double gamma;
   double phi1;
   double phi2;
   double phi3;
   double s;
   double dy;

   z = 200 - z;  //to make the robot stand taller, z should be negative 
   s = sqrt(x*x+z*z);
   if(x>0){
    phi3 = abs(atan(x/z));
   }
   else if(x==0.0){
    phi3 = 0;
   }
   else if(x<0){
    phi3 = -abs(atan(x/z));
   }
   c1 = sqrt(y*y+s*s);
//   Serial.println(c1);
   theta = acos((c1*c1-2*leglen*leglen-2*leglen*feetlen-feetlen*feetlen)/(-2*leglen*leglen-2*leglen*feetlen));
//   Serial.println(theta);
   c2 = sqrt(2*leglen*leglen-2*leglen*leglen*cos(theta));  //theta in rad
//   Serial.println(c2);
   beta = acos(c2/2/leglen);
//   Serial.println(beta);
   alpha = acos((feetlen*feetlen-c1*c1-c2*c2)/(-2*c1*c2));
//   Serial.println(alpha);
//  Serial.print(z);Serial.println(y);
   gamma = abs(atan(z/abs(y)));
//   Serial.println(gamma);
   if(y<=0){
    phi1 = gamma-alpha-beta;
   }
   else if(y>0){
    phi1 = M_PI-gamma-alpha-beta;
   }
   phi2 = M_PI-phi1-2*beta;
   p[0] = 4*(0.25 - constrain(phi1/2/M_PI,-0.25,0.25)-0.134);
   p[1] = 4*(0.25 - constrain(phi2/2/M_PI,-0.25,0.25)-0.1225);
   p[2] = 40*constrain(phi3/2/M_PI,-0.25,0.25);
   // p[0] for left motor (M1) , p[1] for right motor (M0)
   // p[0] and p[1] are zero when legs are vertical, pointing to the ground
   // M(0) for right shoulder, M(1) for left shoulder
   //zero is when the the leg height is 200
}

void drive(int leg,double* p,float a,float b,float c){  
  //a = M0, b = M1, c = M2
  if (leg==1 || leg == 3){ //FR,BR
     odrive1.TrapezoidalMove(0,p[1]+a);
     odrive1.TrapezoidalMove(1,-p[0]+b);
     odrive2.TrapezoidalMove(0,p[2]+c);
  }
  else if (leg==2 || leg == 4){ // FL,BL
     odrive1.TrapezoidalMove(0,p[0]+a);
     odrive1.TrapezoidalMove(1,-p[1]+b);
     odrive2.TrapezoidalMove(1,p[2]+c);
  }
}
//50,50,40
//-50,-50,-40


/* Things To do before implementation:
    - Check direction of shoulder rotation
    - Check direction of M[0] and M[1] - Axis 0 : positive = Anticlockwise
    - Check limit of M[0] and M[1]   - M[0] = Anticlockwise 0.3, Clockwise -1.12
                                     - M[1] = Anticlockwise 1.12, Clockwise  -0.3
                                     - Shoulder Front: positive = outward = 3; negative = inward = -3 
    - Check limit of x,y,z coordinates :  Perspective from FR side view
                                      X: -80 to 100 (on stand) ; 
                                      Y: -130 to 140  (negative to the left, positive to the right)
                                      Z: -80 to 90 (negative stretch down, positive lift up)
                                      (60,60,35) and (60,60,35) is a limit (cannot be more than 35)
 */


// Zero position is when the leg length is 200, standing straight.
/* Current limit from Ken's coordinates 
 *    linear range        angular range
 *                     p[0]           p[1]          p[2]
   Z:     0>50         0>0.15         0>0.13        -
   Y:     -60>60      -0.21>0.16      -0.21>0.16    - 
   X:     60>60          -              -           -1.86>1.86
 */
