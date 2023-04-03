#include <SoftwareSerial.h>
SoftwareSerial BT_serial(10, 11); //RX pin , TX pin

#include "SerialTransfer.h" //library for packing and parsing data packet
SerialTransfer receive_packet; //defining serial transfer object


struct STRUCT { //defining struct called test_packet
  float item1;
  float item2;
  float item3;
  float item4;
  int item5;
  int item6;
  int item7;
} packet;

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
    receive_packet.rxObj(packet);
    Serial.print("LX: "); Serial.print(packet.item1); Serial.print("\t");
    Serial.print("LY: "); Serial.print(packet.item2); Serial.print("\t");
    Serial.print("RX: "); Serial.print(packet.item3); Serial.print("\t");
    Serial.print("RY: "); Serial.print(packet.item4); Serial.print("\t"); 
    Serial.print("Button 1(mode): "); Serial.print(packet.item5); Serial.print("\t");
    Serial.print("Button 2: "); Serial.print(packet.item6); Serial.print("\t");
    Serial.print("Button 3: "); Serial.print(packet.item7); Serial.println("\t");
    
    //VERY IMPORTANT the sum of delays in void loop (packet receiver) needs to equal the master (packet sender)
    //if not there will be stale packets and eventually it will stop working
    delay(50);
  
  }
}
