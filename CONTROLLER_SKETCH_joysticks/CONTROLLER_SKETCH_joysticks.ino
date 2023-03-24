
#include <SoftwareSerial.h>
SoftwareSerial BT_serial(10, 11); //RX pin , TX pin //library so we can use hardware serial to monitor and debug

#include <SerialTransfer.h> //library for packing and parsing data packet
SerialTransfer transmit_packet;

#include "DFRobot_RGBLCD1602.h" //library for lcd display
DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show

//struct that will be stored into data packet named "packet"
struct STRUCT {
  float item1;
  float item2;
  float item3;
  float item4;
  int item5;
  int item6;
  int item7;
} packet;

//pin names for the L and R joystick and buttons
const int LX_pin = A0;
const int LY_pin = A1;
const int RX_pin = A2;
const int RY_pin = A3;

//signal pin names
const int button_1 = 2;
const int button_2 = 3;
const int button_3 = 4;

//variable to check 0v or 5v (pin pushed or not)
int button1state = 0;
int button2state = 0;
int button3state = 0;

//variable to store 1 if button is pushed
int button1val = 0;
int button2val = 0;
int button3val = 0;

//defining previous values
float old_LX = 0;
float old_LY = 0;
float old_RX = 0;
float old_RY = 0;
int oldbutton1val = 0;
int oldbutton2val = 0;
int oldbutton3val = 0;

//initialising mode;
int mode = 1; 

//initialising zones of varyingspeed for joystick
// int LX_zone;
// int LY_zone;
// int RX_zone;
// int RY_zone;

void setup() {
  Serial.begin(9600); //hardware serial baud rate
  BT_serial.begin(9600); //software serial baud rate
  transmit_packet.begin(BT_serial); //start packet transmission
  lcd.init(); //initialising LCD and starting I2C bus
  lcd.setRGB(0, 0, 1); //r g b color brightness we only can use b

  //print row 1 of LCD only once because it dosent change
  lcd.setCursor(0, 0); //row 1 col 1
  lcd.print("LX "); lcd.print("LY "); lcd.print("RX "); lcd.print("RY "); lcd.print("Mode");
  lcd.setCursor(0, 1); //row 1 col 1
  lcd.print("We");
  }


void loop() {

  //storing joystick values and calibrating individually to produce float from -1 to 1
  int LX_joy = analogRead(LX_pin);
  float LX = 0;
  LX_joy = map(LX_joy, 3, 1024, -100, 100) + 2;
  if (LX_joy < 0){LX = float(LX_joy) / float(98);}
  if (LX_joy > 0){LX = float(LX_joy) / float(101);}

  int LY_joy = analogRead(LY_pin);
  float LY = 0;
  LY_joy = map(LY_joy, 3, 1024, -100, 100) + 3;
  if (LY_joy < 0){LY = float(LY_joy) * -1 / float(97);} //joystick was reversed so up is forward
  if (LY_joy > 0){LY = float(LY_joy) * -1 / float(102);}
  
  int RX_joy = analogRead(RX_pin);
  float RX = 0;
  RX_joy = map(RX_joy, 3, 1024, -100, 100) - 2;
  if (RX_joy < 0){RX = float(RX_joy) / float(102);}
  if (RX_joy > 0){RX = float(RX_joy) / float(97);}

  int RY_joy = analogRead(RY_pin);
  float RY = 0;
  RY_joy = map(RY_joy, 3, 1024, -100, 100) + 2; //joystick was reversed so up is forward
  if (RY_joy < 0){RY = float(RY_joy) * -1 / float(98);}
  if (RY_joy > 0){RY = float(RY_joy) * -1 / float(101);}

  //defining joystick speed zones
  //quite slow, just calcuate from joystick value
  // if ((0.25 > LX) && (LX > 0.02)) {LX_zone = 0;}
  // if ((0.50 > LX) && (LX > 0.25)) {LX_zone = 1;}
  // if ((0.75 > LX) && (LX > 0.50)) {LX_zone = 2;}
  // if ((0.98 > LX) && (LX > 0.75)) {LX_zone = 3;}

  button1state = digitalRead(2);
  button2state = digitalRead(3);
  button3state = digitalRead(4);


  //Reading all button states to see if pressed (buttonstate == 5v), if pressed return 1, else return 0
  if (button1state == HIGH) {button1val = 1;}
  else {button1val = 0;}

  if (button2state == HIGH) {button2val = 1;}
  else {button2val = 0;}

  if (button3state == HIGH) {button3val = 1;}
  else {button3val = 0;}
  

  //button 1 iterating through 0-5 if the button is pressed
  if ((button1val == 1) && (oldbutton1val == 0)){
    mode = mode + 1;
  }
  if(mode == 6){
    mode = 1;
  }

  //printing serial plotter
  Serial.print("Left X: "); Serial.print(LX); Serial.print("\t");
  Serial.print("Left Y: "); Serial.print(LY); Serial.print("\t");
  Serial.print("Right X: "); Serial.print(RX); Serial.print("\t");
  Serial.print("Right Y: "); Serial.print(RY); Serial.print("\t");

  Serial.print("Button 1: "); Serial.print(mode); Serial.print("\t");
  Serial.print("Button 2: "); Serial.print(button2val); Serial.print("\t");
  Serial.print("Button 3: "); Serial.print(button3val); Serial.println("\t");



  //printing on LCD only if controller inputs have changed
  // if ( 
  // (LX != old_LX) ||
  // (LY != old_LY) ||
  // (RX != old_RX) ||
  // (RY != old_RY) ||
  // (button1val != oldbutton1val) ||
  // (button2val != oldbutton2val) ||
  // (button3val != oldbutton3val) ){
  //   lcd.setCursor(0, 1);//row 2
  //   lcd.print(int(LX * 10)); lcd.print(" ");
  //   lcd.print(int(LY * 10)); lcd.print("  ");
  //   lcd.print(int(RX * 10)); lcd.print(" ");
  //   lcd.print(int(RY * 10)); lcd.print(" ");

  //   lcd.setCursor(11,1);
  //   lcd.print("  ");
  //   lcd.setCursor(12,1);     
  //   lcd.print(mode);
  // }

 
  //storing all control inputs in struct
  packet.item1 = LX;
  packet.item2 = LY;
  packet.item3 = RX;
  packet.item4 = RY;
  packet.item5 = mode;
  packet.item6 = button2val;
  packet.item7 = button3val;

  //putting struct in packet and transmitting packet
  transmit_packet.sendDatum(packet);

  //storing old values for comparison - do something if values change
  // old_LX = LX;
  // old_LY = LY;
  // old_RX = RX;
  // old_RY = RY;
  // oldbutton1val = button1val;
  // oldbutton2val = button2val;
  // oldbutton3val = button3val;
  
  //VERY IMPORTANT the sum of delays in void loop (packet sender) needs to equal the master (packet receiver)
  //if not there will be stale packets and eventually it will stop working
  delay(50);
}
