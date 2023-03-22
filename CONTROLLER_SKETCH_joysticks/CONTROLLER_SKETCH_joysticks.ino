
#include <SoftwareSerial.h>
SoftwareSerial BT_serial(10, 11); //RX pin , TX pin //library so we can use hardware serial to monitor and debug

#include <SerialTransfer.h> //library for packing and parsing data packet
SerialTransfer transmit_packet;


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

//struct that will be stored into data packet named "packet"
struct STRUCT {
  int item1;
  int item2;
  int item3;
  int item4;
  int item5;
  int item6;
  int item7;
} packet;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //hardware serial baud rate
  BT_serial.begin(9600); //software serial baud rate
  transmit_packet.begin(BT_serial); //start packet transmission

  }

void loop() {

  //storing joystick values and button states
  int LX_joy = analogRead(LX_pin);
  int LY_joy = analogRead(LY_pin);
  
  int RX_joy = analogRead(RX_pin);
  int RY_joy = analogRead(RY_pin);
  
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
  
  Serial.print("Left X: "); Serial.print(LX_joy); Serial.print("\t");
  Serial.print("Left Y: "); Serial.print(LY_joy); Serial.print("\t");
  Serial.print("Right X: "); Serial.print(RX_joy); Serial.print("\t");
  Serial.print("Right Y: "); Serial.print(RY_joy); Serial.print("\t");

  Serial.print("Button 1: "); Serial.print(button1val); Serial.print("\t");
  Serial.print("Button 2: "); Serial.print(button2val); Serial.print("\t");
  Serial.print("Button 3: "); Serial.print(button3val); Serial.println("\t");

  //storing all control inputs in struct
  packet.item1 = LX_joy;
  packet.item2 = LY_joy;
  packet.item3 = RX_joy;
  packet.item4 = RY_joy;
  packet.item5 = button1val;
  packet.item6 = button2val;
  packet.item7 = button3val;

  //putting struct in packet and transmitting packet
  transmit_packet.sendDatum(packet);
  
  //VERY IMPORTANT the sum of delays in void loop (packet sender) needs to equal the master (packet receiver)
  //if not there will be stale packets and eventually it will stop working
  delay(50);
}
