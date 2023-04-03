#include <NintendoExtensionCtrl.h> //library for nunchuck
Nunchuk nchuck;

#include <SoftwareSerial.h>
SoftwareSerial BT_serial(10, 11); //RX pin , TX pin //library so we can use hardware serial to monitor and debug

#include <SerialTransfer.h> //library for packing and parsing data packet
SerialTransfer transmit_packet;

int packet;
float X;
float Y;

struct STRUCT {
  float item1;
  float item2;
  int item3;
  int item4;
} test_packet;

//nchuck wire colors
//Red – 3v3
//White – GND
//Green – Clock, A5(uno) or SCL(seeduino)
//Blue – Data, A4(uno) or SDA(seeduino)
//Black – Unsused

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //hardware serial baud rate
  BT_serial.begin(9600); //software serial baud rate
  transmit_packet.begin(BT_serial);
  nchuck.begin();
  

  while (!nchuck.connect()){
    Serial.println("Nunchuck not detected!");
    delay(1000);
    
  }
}

void loop() {
  // storing the values for X, Y, Zbutton, Cbutton from the Nintendo Nunchuck with library
  
  nchuck.update();
  float JoyX = double(nchuck.joyX());
  //mapping raw joystick X readings to between -1 and 1 -> 129 middle, 1 low, 254 high
  if (JoyX == 129){ X = 0;}
  if (JoyX > 129){ X = double(JoyX - 129) / double(125);}
  if (JoyX < 129){ X = double(JoyX - 129) / double(128);}

  //mapping raw joystick Y readings to between -1 and 1 -> 126 middle, 0 low, 255 high
  int JoyY = nchuck.joyY();
  if (JoyY > 126){ Y = double(JoyY - 126) / double(129);}
  if (JoyY < 129){ Y = double(JoyY - 126) / double(126);}

  int Zbutton = nchuck.buttonZ();
  int Cbutton = nchuck.buttonC();

  boolean success = nchuck.update();


  if (success == true){
    Serial.print("X value: ");
    Serial.print(X);
    //append to input array codeblock here
    test_packet.item1 = X;

    Serial.print("  ");

    Serial.print("Y value:");
    Serial.print(Y);
    //append to input array codeblock here
    test_packet.item2 = Y;

    Serial.print("  ");

    Serial.print("Z button: ");
    Serial.print(Zbutton);
    test_packet.item3 = Zbutton;

    Serial.print("  ");

    Serial.print("C button: ");
    Serial.println(Cbutton);
    test_packet.item4 = Cbutton;

    //send packet over bluetooth to HC-06 slave
    //BT_serial.write(Cbutton);
    transmit_packet.sendDatum(test_packet);

  }
  delay(50);
}
