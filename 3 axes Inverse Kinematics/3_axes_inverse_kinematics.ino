#include <math.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial2);

float leglen = 140.0;
float feetlen = 10;
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
void drive(int leg,double* p);

void setup() {
  Serial.begin(115200);
  //Arrays: [iterations][legs][x,y,z coordinates]
  Serial.println(gait.LR[0][1][0]);
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
    //drive(1,phi);
  }
  
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
}

void drive(int leg,double* p){
  if (leg==1 || leg == 3){
     odrive1.TrapezoidalMove(0,-p[1]);
     odrive1.TrapezoidalMove(1,p[0]);
     odrive2.TrapezoidalMove(0,p[2]);
  }
  else if (leg==2 || leg == 4){
     odrive1.TrapezoidalMove(0,-p[0]);
     odrive1.TrapezoidalMove(1,p[1]);
     odrive2.TrapezoidalMove(1,p[2]);
  }
}


/* Things To do before implementation:
    - Check direction of shoulder rotation
    - Check direction of M[0] and M[1]
    - Check limit of M[0] and M[1]
    - Check limit of x,y,z coordinates
 */


// Zero position is when the leg length is 200, standing straight.
/* Current limit from Ken's coordinates 
 *    linear range        angular range
 *                     p[0]           p[1]          p[2]
   Z:     0>50         0>0.15         0>0.13        -
   Y:     -60>60      -0.21>0.16      -0.21>0.16    - 
   X:     60>60          -              -           -1.86>1.86
 */
