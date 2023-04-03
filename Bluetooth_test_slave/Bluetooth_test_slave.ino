#include <SoftwareSerial.h>
SoftwareSerial BT_serial(10, 11); //RX pin , TX pin

#include "SerialTransfer.h" //library for packing and parsing data packet
SerialTransfer receive_packet; //defining serial transfer object

int packet;

struct STRUCT { //defining struct called test_packet
  float item1;
  float item2;
  int item3;
  int item4;
} test_packet;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //baud rate for hardware serial (serial monitor)
  BT_serial.begin(9600); //baud rate for software serial (bluetooth serial)
  receive_packet.begin(BT_serial); //setting serial transfer object to bluetooth

}

void loop() {
  // put your main code here, to run repeatedly:
  if (receive_packet.available()){
    //packet = BT_serial.read(); //read from slave bluetooth serial 
    receive_packet.rxObj(test_packet);
    Serial.print("JoyX: "); Serial.print(test_packet.item1); Serial.print("\t");
    Serial.print("JoyY: "); Serial.print(test_packet.item2); Serial.print("\t");
    Serial.print("Zbutton: "); Serial.print(test_packet.item3); Serial.print("\t");
    Serial.print("Cbutton: "); Serial.println(test_packet.item4); Serial.print("\t");
    
    delay(50);
  }
}
