#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <math.h>
#include <Ramp.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

double GetCurrent(int motor_number);
int go_int(int input, int duration);
void kinematics(double x, double y, double *p);
void move_incremental(double x, int motornum);

int intx;
int inty;
double legoffset = 0.1;
int timer = 1000;
int x_cor = 0;
int y_cor = 0;
float cur0;
float cur1;
unsigned long previousmilli;

class Interpolation{
  public:
  rampInt myRamp;
  int interpolationFlag = 0;
  int savedValue;

  int go_int(int input, int duration) {

   if (input != savedValue) {   // check for new data
     interpolationFlag = 0;
   }
     savedValue = input;          // bookmark the old value

   if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
     myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
     interpolationFlag = 1;
    }

   int output = myRamp.update();
   return output;
}
};

Interpolation interpx;
Interpolation interpy;

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);
  
  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  double *phi = (double *) malloc(sizeof(double)*2);
  kinematics(0.0,200.0, phi);
  Serial.println("Odrivearduino is Ready!");
  Serial.println(phi[0]);
  Serial.println(phi[1]);
  free(phi);

  odrive_serial<< "w axis0.trap_traj.config.vel_limit "<< 5.0 << '\n';
  odrive_serial<< "r axis0.trap_traj.config.vel_limit"<< '\n';
  Serial.print(odrive.readFloat());
}

void loop() {
  
  if (Serial.available()) {
    char c = Serial.read();

    if(c=='9'){
      Serial << "Restarting ODrives" << '\n';
      odrive_serial << "sr" << '\n';
    }

    if(c=='s'){
      char state_c = Serial.read();
      int state_i = state_c - '0';
      int motornum;
      Serial << "Setting both motors to mode"<< state_i << '\n';
      for(motornum=0;motornum<2;motornum++){
        odrive.run_state(motornum, state_i, false);
      }
    }

    if(c=='r'){
      static const unsigned long duration = 15000;
      unsigned long start = millis();
      char var = Serial.read();
      int var_i = var - '0';
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          switch(var_i){
            case 1:
              Serial << odrive.GetPosition(motor) << ' ';
              break;
            case 2:
              Serial << GetCurrent(motor) << ' ';
              break;
        }
      }

      delay(500);
        Serial << '\n';
      }
    }

    if(c=='o'){
      Serial << "Apply offsets to both motors"<< '\n';
      odrive.SetPosition(0,legoffset);
      odrive.SetPosition(1,-legoffset);
    }

    if(c=='p'){
      int val = Serial.parseInt();
      float val_f = val*0.001;
      Serial << "Apply offsets to both motors"<< '\n';
      odrive.SetPosition(0,val_f);
      //odrive.SetPosition(1,-val_f);
    }

    if(c=='l'){
      int val = Serial.parseInt();
      float val_f = val*0.001;
      Serial << "Apply offsets to both motors"<< '\n';
      //odrive.SetPosition(0,val_f);
      odrive.SetPosition(1,-val_f);
    }

    if(c=='b'){
      Serial << "Apply offsets to both motors"<< '\n';
      odrive.SetPosition(0,0);
      odrive.SetPosition(1,0);
    }
    if(c=='g'){
      char v = Serial.read();
      int v_i = v - '0';
      double *tphi = (double *) malloc(sizeof(double)*2);
      x_cor = 0;
      y_cor = 0;
      switch(v_i){
        case 1:
        x_cor = 50;
        y_cor = 200;
        break;
        case 2:
        x_cor = 0;
        y_cor = 200;
        break;
        case 3:
        x_cor = 0;
        y_cor = 250;
        break;
        }
      previousmilli = millis();
      while(millis() - previousmilli <=timer+100){
        intx = interpx.go_int(x_cor,timer);
        inty = interpy.go_int(y_cor,timer);
        kinematics(intx,inty,tphi);
        //Serial.print(intx);
        //Serial.println(inty);
        //Serial.print(tphi[0]);
        //Serial.println(tphi[1]);
        //cur0 = odrive.GetPosition(0);
        //cur1 = odrive.GetPosition(1);
        //Serial.println(cur0);
        //Serial.println(cur1);
        //odrive.SetPosition(0,0.25-tphi[0]-legoffset+cur0);
        //odrive.SetPosition(1,-0.25+tphi[1]+legoffset+cur1);
        cur0 = 4*(0.25-tphi[0]-0.125);
        //cur0 = constrain(cur0, -0.25,0.5);
        cur1 = -4*(0.25-tphi[1]-0.125);
        //cur1 = constrain(cur1,-0.5,0.25);
        Serial.print(cur0);
        Serial.println(cur1);
        //odrive.SetPosition(0,cur0);
        //odrive.SetPosition(1,cur1);
        //delay(50);
       //odrive.TrapezoidalMove(0,cur0);
        //odrive.TrapezoidalMove(1,cur1);
    }
    free(tphi);
  }

  if(c=='t'){
    int xarr[] = {0, 0, 100};
    int yarr[] = {200,250,200};
    double *tphi = (double *) malloc(sizeof(double)*2);
    for(int i =0;i<3;i++){
      for(int ii =0;ii<3;ii++){
        previousmilli = millis();
        while(millis() - previousmilli < timer){
         intx = interpx.go_int(xarr[ii],timer);
         inty = interpy.go_int(yarr[ii],timer);
         kinematics(intx,inty,tphi);
         cur0 = 4*(0.25-tphi[0]-0.125);
         cur1 = -4*(0.25-tphi[1]-0.125);
         Serial.print(cur0);
         Serial.println(cur1);
         odrive.TrapezoidalMove(0,cur0);
         odrive.TrapezoidalMove(1,cur1);
        }
      }
    }
    free(tphi);
  }

}
/*x_cor = 5000;
y_cor = 200;
intx = interpx.go_int(x_cor,timer);
inty = interpy.go_int(y_cor,timer);
Serial.print(intx);
Serial.print(" ");
Serial.println(inty); */
}


double GetCurrent(int motor_number) {
  odrive_serial<< "r axis" << motor_number << ".motor.current_control.Iq_measured\n";
  return odrive.readFloat();
}

void move_incremental(double x, int motornum){
  odrive_serial<< "w axis" << motornum << ".controller.move_incremental" << x << '\n';
} //Does not work on Teensy

void kinematics(double x, double y, double *p){
  double leglen = 140.0;
  double phi1;
  double phi2;
  double c;
  double beta;

  c = sqrt(x*x + y*y);
  beta = acos(c/2/leglen)*180/M_PI;
  phi1 = (acos(x/c))*180/M_PI-beta;
  phi2 = 180-phi1-2*beta;
  phi1 = constrain(abs(phi1)/360,0,0.25);
  phi2 = constrain(abs(phi2)/360,0,0.25);

  if(x>0 && y>0){
    p[0] = -phi1;
    p[1] = -phi2;
  }
  else if(x<=0 && y>0){
    p[0] = phi1;
    p[1] = phi2;
  }

}
