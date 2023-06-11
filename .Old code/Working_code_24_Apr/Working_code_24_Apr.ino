#include <math.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <Ramp.h>
#include <SerialTransfer.h>
#include <DFRobot_RGBLCD1602.h>
#include "structdef.h"

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial2 = Serial2;
HardwareSerial& odrive_serial3 = Serial3;
HardwareSerial& BT_serial = Serial4;
HardwareSerial& odrive_serial4 = Serial5; 
HardwareSerial& odrive_serial5 = Serial6;
HardwareSerial& odrive_serial6 = Serial7;

//Arrays: [iterations][legs][x,y,z coordinates]
//$$$$$$$$$$$$$$$$$[Leg 1 : FR, Leg 2: BL, Leg 3: BR, Leg 4:FL]$$$$$$$$$$$$$$$$$$$$$$$$$$//

// ODrive object
ODriveArduino odrive1(odrive_serial1);  // Leg 1 FR
ODriveArduino odrive2(odrive_serial2);  // Shoulder front
ODriveArduino odrive3(odrive_serial3);  // Leg 4 FL
ODriveArduino odrive4(odrive_serial4);  // Leg 3 BR
ODriveArduino odrive5(odrive_serial5);  // Shoulder back
ODriveArduino odrive6(odrive_serial6);  // Leg 2 BL

//Bluetooth Serial
//SoftwareSerial BT_serial(10, 11);
SerialTransfer receive_packet;

//LCD
DFRobot_RGBLCD1602 lcd(16,2);

float leglen = 140.0;
float feetlen = 10;
float LRrat;
float FBrat;
float RTrat;
float vellim = 60.0;
float accellim = 10.0;
float decellim = 10.0;
float posg = 80.0;
float velg = 0.15;
float velintg = 0.2;

int xx=100;
int yy=100;
int zz=100;
int stepd;
int numm;
int citr;
int itrflag;
int closedloopflag;
int mode;
int modeflag;
int offsetflag;
int homeflag;

long btmillis;
unsigned long currentmillis;
unsigned long prevmillis;
unsigned long homemillis;
unsigned long crawlmillis;

double *phi = (double *) malloc(sizeof(double)*3);
double interpout [legn][coor];
double inipos[legn][coor];
double intermpos[legn][coor];
double mat[legn][coor];
double invmat[legn][coor];

//Gait Initialisation  -  Copy gait matrix from GUI
gait_struct crawl;

//Function Prototypes
void state(int x);
float thresholdfilter(float inp);
void pos_est(double pos[legn][coor]);
//void drive(int legnum,double* p,float a,float b,float c);
//void kinematics_3d(double x, double y, double z, double *p);
void create_mat(double q[4][3],gait_struct *gait,float a,float b,float c,int iter);
void kinematics_mat(double inp[legn][coor], double out[legn][coor],double pos[legn][coor]);
void actuate(double inp[legn][coor]);
void checkchange();
void modify_lim_gain();

class Interpolation{
  public:
  rampDouble myRamp;
  int interpolationFlag = 0;
  double savedValue;

  double go_dbl(double input, int duration) {

   if (input != savedValue) {   // check for new data
     interpolationFlag = 0;
   }
     savedValue = input;          // bookmark the old value

   if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
     myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
     interpolationFlag = 1;
    }

   double output = myRamp.update();
   return output;
}
};

Interpolation interp[legn][coor];

/*double mapf(long x, long in_min, long in_max, float out_min, float out_max)
{
  return double((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
};*/

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  BT_serial.begin(9600); //baud rate for software serial (bluetooth serial)
  receive_packet.begin(BT_serial); //setting serial transfer object to bluetooth
//
  lcd.init(); //initialising LCD and starting I2C bus
  lcd.setRGB(0, 0, 255); //r g b color brightness we only can use b
  
  int num_itr = sizeof(crawl)/sizeof(double)/legn/coor/3;
  int itrcount=0;

  //Obtain initial pos estimate from encoders
  pos_est(inipos);
  for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
          Serial<<"inipos["<<ii<<"]["<<jj<<"] = "<<inipos[ii][jj]<<'\t';
          if(inipos[ii][jj] == 0){
            lcd.setCursor(1,0);
            lcd.print("Offset error");
            offsetflag = 1;
          }
        }
        Serial.print('\n');
    }
  if(offsetflag!= 1){
    lcd.setCursor(0, 0); //row 1 col 1
    lcd.print("MQRPII is ready");
  }
  modify_lim_gain();
  checkchange();
}

void loop() {
  currentmillis = millis();
  
  if (receive_packet.available()){  //Check BT connection is established)
   receive_packet.rxObj(packet);
   btmillis = currentmillis;
   lcd.setCursor(0, 0);
   lcd.print("BlueT connected ");
   LRrat = thresholdfilter(packet.item1);
   FBrat = thresholdfilter(packet.item2);
   RTrat = thresholdfilter(packet.item3);
//   Serial.println(LRrat);
//   Serial.println(FBrat);
//   Serial.println(RTrat);
//   Serial.println(packet.item5);
//   Serial.println(packet.item6);
//   Serial.println(packet.item7);
//   Serial.println(packet.item8);
//   Serial.println(packet.item9);
//   Serial.println(packet.item10);
  }

  if(currentmillis-btmillis > 2000){
    //state(1); //All odrives enter IDLE state 
//    LRrat = 0;
//    FBrat = 0;
//    RTrat = 0;
    lcd.setCursor(0, 0);
    lcd.print("BlueT disconnected");
// 
  }

  if(closedloopflag==0 && packet.item5 == 2){
    state(8); //All odrives enter closed loop control
    closedloopflag = 1;
    lcd.setCursor(0, 1);
    lcd.print("CLC ");
  }

  if(closedloopflag==1 && packet.item5 == 1){
    state(1); //All odrives enter IDLE state
    closedloopflag = 0;
    lcd.setCursor(0, 1);
    lcd.print("IDLE");
  }

  if(closedloopflag == 1 && packet.item7 == 1){
    mode = 1;
    prevmillis = millis();
    modeflag = 1;
    homeflag = 0;
  }
  if(closedloopflag == 0 && packet.item7 == 1){
    lcd.setCursor(0, 1);
    lcd.print("IDLE");
  }
  if(modeflag == 1 && packet.item6 == 1){
    mode = 0;
    homeflag = 1;
    lcd.setCursor(10, 1);
    lcd.print("     ");
    modeflag = 0;
    citr = 0;
    homemillis = millis();
    }
  if(homeflag == 1 && currentmillis-homemillis<1.5*timer){
    for(int ii=0;ii<legn;ii++){
      for(int jj=0;jj<coor;jj++){
        interpout[ii][jj] = interp[ii][jj].go_dbl(inipos[ii][jj],timer);
          Serial.print(interpout[ii][jj]);Serial.print('\t');
    }
        Serial.print('\n');
    }
    actuate(interpout);
  }
  if(mode == 1 && closedloopflag ==1){
    lcd.setCursor(10, 1);
    lcd.print("CRAWL");
    create_mat(mat,&crawl,0.5,0,0,citr);
//  create_mat(mat,&crawl,0.5,0.5,0,citr);
//  for(int ii=0;ii<legn;ii++){
//      for(int jj=0;jj<coor;jj++){
//        Serial.print(mat[ii][jj]);
//        Serial.print('\t');  
//      }
//      Serial.print('\n');
//  }
    for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
          interpout[ii][jj] = interp[ii][jj].go_dbl(mat[ii][jj],timer);
//          Serial.print(interpout[ii][jj]);Serial.print('\t');
        }
//        Serial.print('\n');
    }
  //Serial.print('\n');
    kinematics_mat(interpout,invmat,inipos);
//    for(int ii=0;ii<legn;ii++){
//        for(int jj=0;jj<coor;jj++){
//          Serial.print(invmat[ii][jj]);
//          Serial.print('\t');  
//        }
//        Serial.print('\n');
//    }
    actuate(invmat);
//    pos_est(intermpos);
//      for(int ii=0;ii<legn;ii++){
//        for(int jj=0;jj<coor;jj++){
//          Serial<<"intermpos["<<ii<<"]["<<jj<<"] = "<<intermpos[ii][jj]<<'\t';
//        }
//        Serial.print('\n');
//    }
    /*for(int ii=0;ii<legn;ii++){
        for(int jj=0;jj<coor;jj++){
          if(intermpos[ii][jj]-invmat[ii][jj] > 0.001 || intermpos[ii][jj]-invmat[ii][jj] < -0.001){
            itrflag = 0;
            break;
          }
          else{
            itrflag = 1; // Move to next iteration
          }
        }
    }*/
    crawlmillis = millis();
     if(crawlmillis - prevmillis > timer){
      prevmillis = crawlmillis;
      itrflag = 1;
    }
    if(itrflag==1 && citr<7){
      citr += 1;
      itrflag = 0;
    }
    else if(itrflag == 1 && citr == 7){
      citr = 0;
      itrflag = 0;
    }

  
  /*if (Serial.available()) {
    xx = Serial.parseInt();
    yy = Serial.parseInt();
    zz = Serial.parseInt();*/
//    double xf = interpx1.go_dbl((double) xx,5000);
//    double yf = interpy1.go_dbl((double) yy,5000);
//    double zf = interpz1.go_dbl((double) zz,5000);
//    kinematics_3d(0,0,0,phi);
//    /*Serial.println(xf);
//    Serial.println(yf);
//    Serial.println(zf);*/
//    Serial.println(phi[0]);
//    Serial.println(phi[1]);
//    Serial.println(phi[2]);

    /*if(stepd == 0 && xf==xx && yf == yy && zf==zz){
      xx = 0;
      yy = 0;
      zz = 0;
      stepd = 1;
    }

    if(stepd == 1 && xf==xx && yf == yy && zf==zz){
      xx = 100;
      yy = 100;
      zz = 100;
      stepd = 0;
    }

    delay(5);
    //drive(1,phi);
//  }
//    int LRratio = Serial.parseInt();
//    int FBratio = Serial.parseInt();
//    numm = Serial.parseInt(); //number of iteration
//    LRrat = 0.1*LRratio;
//    FBrat = 0.1*FBratio;
  */
  Serial<<"citr ="<<citr;
}
}

void state(int x){
  for(int i=0;i<2;i++){
  odrive1.run_state(i,x,false);
  odrive2.run_state(i,x,false);
  odrive3.run_state(i,x,false);
  odrive4.run_state(i,x,false);
  odrive5.run_state(i,x,false);
  odrive6.run_state(i,x,false);
}
}

float thresholdfilter(float inp){
  float out;
  if(inp < 0.1 && inp > 0){
    out = 0.00;
  }
  else if(inp < 0 && inp > -0.1){
    out = 0.00;
  }
  else{
    out = inp;
  }
  return out;
}

void pos_est(double pos[legn][coor]){
  //Needs to be manipulated manually if different number of legs or degree of freedom are used
  pos[0][0] = (double) odrive1.GetPosition(0); //Leg 1
  pos[0][1] = (double) odrive1.GetPosition(1);
  pos[0][2] = (double) odrive2.GetPosition(1);
  pos[1][0] = (double) odrive6.GetPosition(0); //Leg 2
  pos[1][1] = (double) odrive6.GetPosition(1);
  pos[1][2] = (double) odrive5.GetPosition(1);
  pos[2][0] = (double) odrive4.GetPosition(0); //Leg 3
  pos[2][1] = (double) odrive4.GetPosition(1);
  pos[2][2] = (double) odrive5.GetPosition(0);
  pos[3][0] = (double) odrive3.GetPosition(0); //Leg 4
  pos[3][1] = (double) odrive3.GetPosition(1);
  pos[3][2] = (double) odrive2.GetPosition(0);
}

/*void kinematics_3d(double x, double y, double z, double *p){

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
   p[0] = constrain(4*(0.25 - constrain(phi1/2/M_PI,-0.25,0.25)-0.134),-0.3,1);
   p[1] = constrain(4*(0.25 - constrain(phi2/2/M_PI,-0.25,0.25)-0.1225),-1,0.3);
   p[2] = constrain(40*constrain(phi3/2/M_PI,-0.25,0.25),-2.5,2.5);
   // p[0] for left motor (M1) , p[1] for right motor (M0)
   // p[0] and p[1] are zero when legs are vertical, pointing to the ground
   // M(0) for right shoulder, M(1) for left shoulder
}*/

/*void drive(int legnum,double* p,float a,float b,float c){
  if (legnum==1 || legnum == 3){
     odrive1.TrapezoidalMove(0,p[1]+a); // Leg 1
     odrive1.TrapezoidalMove(1,-p[0]+b);
     odrive2.TrapezoidalMove(0,p[2]+c);
  }
  else if (legnum==2 || legnum == 4){
     odrive1.TrapezoidalMove(0,p[0]+a);
     odrive1.TrapezoidalMove(1,-p[1]+b);
     odrive2.TrapezoidalMove(1,p[2]+c);
  }
}*/

void actuate(double inp[legn][coor]){
  
   odrive1.TrapezoidalMove(0,inp[0][1]); // Leg 1
   odrive1.TrapezoidalMove(1,inp[0][0]);
   odrive2.TrapezoidalMove(1,inp[0][2]);
   odrive6.TrapezoidalMove(0,inp[1][1]); // Leg 2
   odrive6.TrapezoidalMove(1,inp[1][0]);
   odrive5.TrapezoidalMove(1,inp[1][2]);
   odrive4.TrapezoidalMove(0,inp[2][1]); // Leg 3
   odrive4.TrapezoidalMove(1,inp[2][0]);
   odrive5.TrapezoidalMove(0,inp[2][2]);
   odrive3.TrapezoidalMove(0,inp[3][1]); // Leg 4
   odrive3.TrapezoidalMove(1,inp[3][0]);
   odrive2.TrapezoidalMove(0,inp[3][2]);
}

void create_mat(double q[legn][coor],gait_struct *gait,float a,float b,float c,int iter){
  //a,b are end effect multiplier from controller, i is the iteration number, p is the new matrix and gait is the stored gait
    a = (double) a;
    b = (double) b;
    c = (double) c;
    for(int ii=0;ii<legn;ii++){
      for(int jj=0;jj<coor;jj++){
        q[ii][jj] = gait->March[iter][ii][jj]+a*gait->FB[iter][ii][jj]+b*gait->LR[iter][ii][jj]+c*gait->RT[iter][ii][jj];
      }
    }
}


void kinematics_mat(double inp[legn][coor], double out[legn][coor],double pos[legn][coor]){

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
   double z;

   // INPUT : x= inp[i][0] ; y=inp[i][1] ; z=inp[i][2]
   // OUTPUT : phi1 = out[i][0]; phi2 = out[i][1] ; phi3 = out[i][2]
  for(int i=0;i<legn;i++){
     z = 200 - inp[i][2];  //to make the robot stand taller, z should be negative 
//     Serial.println(z);
     s = sqrt(inp[i][0]*inp[i][0]+z*z);
//     Serial.println(s);
     if(inp[i][0]>0){
      phi3 = abs(atan(inp[i][0]/z));
     }
     else if(inp[i][0]==0.0){
      phi3 = 0;
     }
     else if(inp[i][0]<0){
      phi3 = -abs(atan(inp[i][0]/z));
     }
     c1 = sqrt(inp[i][1]*inp[i][1]+s*s);
//     Serial.println(c1);
     theta = acos((c1*c1-2*leglen*leglen-2*leglen*feetlen-feetlen*feetlen)/(-2*leglen*leglen-2*leglen*feetlen));
//     Serial.println(theta);
     c2 = sqrt(2*leglen*leglen-2*leglen*leglen*cos(theta));  //theta in rad
//     Serial.println(c2);
     beta = acos(c2/2/leglen);
//     Serial.println(beta);
     alpha = acos((feetlen*feetlen-c1*c1-c2*c2)/(-2*c1*c2));
//     Serial.println(alpha);
     gamma = abs(atan(z/abs(inp[i][1])));
//     Serial.print(inp[i][2]);Serial.print(' ');Serial.print(inp[i][1]);
//     Serial.println(gamma);
   if(inp[i][1]<=0){
    phi1 = gamma-alpha-beta;
   }
   else if(inp[i][1]>0){
    phi1 = M_PI-gamma-alpha-beta;
   }
   phi2 = M_PI-phi1-2*beta;
   if(i == 0 || i == 2){
   out[i][0] = constrain(-4*(0.25 - constrain(phi1/2/M_PI,-0.25,0.25)-0.134),-0.3,1)+pos[i][1];
   out[i][1] = constrain(4*(0.25 - constrain(phi2/2/M_PI,-0.25,0.25)-0.1225),-1,0.3)+pos[i][0];
   out[i][2] = constrain(40*constrain(phi3/2/M_PI,-0.25,0.25),-2.5,2.5)+pos[i][2];
   // p[0] for left motor (M1) , p[1] for right motor (M0)
   // p[0] and p[1] are zero when legs are vertical, pointing to the ground
   // M(0) for right shoulder, M(1) for left shoulder
  }
  else if(i == 1 || i == 3){
   out[i][0] = constrain(-4*(0.25 - constrain(phi2/2/M_PI,-0.25,0.25)-0.1225),-0.3,1)+pos[i][1]; 
   out[i][1] = constrain(4*(0.25 - constrain(phi1/2/M_PI,-0.25,0.25)-0.134),-1,0.3)+pos[i][0]; // do not change 0 and 1
   out[i][2] = constrain(-40*constrain(phi3/2/M_PI,-0.25,0.25),-2.5,2.5)+pos[i][2];
  }
}
}

void modify_lim_gain(){
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_limit"<< '\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "vel_limit"<< '\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_limit"<< '\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_limit"<< '\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "vel_limit"<< '\n';
    Serial7 << "w axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_limit"<< '\n';


    Serial1 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "accel_limit"<< '\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "accel_limit"<< '\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"accel_limit"<< '\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "accel_limit"<< '\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "accel_limit"<< '\n';
    Serial7 << "w axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "accel_limit"<< '\n';

    Serial1 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "decel_limit"<< '\n';
    Serial2 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "decel_limit"<< '\n';
    Serial3 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"decel_limit"<< '\n';
    Serial5 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "decel_limit"<< '\n';
    Serial6 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "decel_limit"<< '\n';
    Serial7 << "w axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "decel_limit"<< '\n';

    Serial1 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "pos_gain"<< '\n';
    Serial3 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"pos_gain"<< '\n';
    Serial5 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "pos_gain"<< '\n';
    Serial7 << "w axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "pos_gain"<< '\n';

    Serial1 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_gain"<< '\n';
    Serial3 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_gain"<< '\n';
    Serial5 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_gain"<< '\n';
    Serial7 << "w axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_gain"<< '\n';

    Serial1 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_integrator_gain"<< '\n';
    Serial3 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_integrator_gain"<< '\n';
    Serial5 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_integrator_gain"<< '\n';
    Serial7 << "w axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_integrator_gain"<< '\n';
  }
}

void checkchange(){
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_limit "<< odrive1.readFloat()<< '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "vel_limit "<< odrive2.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_limit "<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_limit "<< odrive4.readFloat()<< '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "vel_limit "<< odrive5.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".trap_traj.config.vel_limit "<<vellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_limit "<< odrive6.readFloat()<< '\n';


    Serial1 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "accel_limit "<< odrive1.readFloat()<< '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "accel_limit "<< odrive2.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"accel_limit "<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "accel_limit "<< odrive4.readFloat()<< '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "accel_limit "<< odrive5.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".trap_traj.config.accel_limit "<<accellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "accel_limit "<< odrive6.readFloat()<< '\n';

    Serial1 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "decel_limit "<< odrive1.readFloat()<< '\n';
    Serial2 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial2 ="<< "decel_limit "<< odrive2.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"decel_limit "<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "decel_limit "<< odrive4.readFloat()<< '\n';
    Serial6 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial6 ="<< "decel_limit "<< odrive5.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".trap_traj.config.decel_limit "<<decellim<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "decel_limit "<< odrive6.readFloat()<< '\n';

    Serial1 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "pos_gain"<< odrive1.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"pos_gain"<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "pos_gain"<< odrive4.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".controller.config.pos_gain "<<posg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "pos_gain"<< odrive6.readFloat()<< '\n';

    Serial1 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_gain"<< odrive1.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_gain"<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_gain"<< odrive4.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".controller.config.vel_gain "<<velg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_gain"<< odrive6.readFloat()<< '\n';

    Serial1 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial1 ="<< "vel_integrator_gain"<< odrive1.readFloat()<< '\n';
    Serial3 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial3 ="<<"vel_integrator_gain"<< odrive3.readFloat()<< '\n';
    Serial5 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial5 ="<< "vel_integrator_gain"<< odrive4.readFloat()<< '\n';
    Serial7 << "r axis"<<axis<<".controller.config.vel_integrator_gain "<<velintg<<'\n';
    Serial<< "axis"<<axis<< "Serial7 ="<< "vel_integrator_gain"<< odrive6.readFloat()<< '\n';
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

/* Potential Troubleshooting required:
 *  -check how long it takes to read pos estimate from all odrives
 *  -check Bluetooth
 *  -check LCD
 *  -check Serial communication
 *  -check if hardware serial port is needed for bluetooth module
 */