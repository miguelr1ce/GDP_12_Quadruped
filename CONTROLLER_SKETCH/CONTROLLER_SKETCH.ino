#include <NintendoExtensionCtrl.h>
Nunchuk nchuck;

//Red – 3v3
//White – GND
//Green – Clock, A5(uno) or SCL(seeduino)
//Blue – Data, A4(uno) or SDA(seeduino)
//Black – Unsused

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  nchuck.begin();

  while (!nchuck.connect()){
    Serial.println("Nunchuck not detected!");
    delay(1000);
  }
}

void loop() {
  // storing the values for X, Y, Zbutton, Cbutton from the Nintendo Nunchuck with library
  float X;
  float Y;
  nchuck.update();
  float JoyX = double(nchuck.joyX());
  //mapping raw joystick X readings to between -1 and 1 -> 129 middle, 1 low, 254 high
  if (JoyX == 129){ X = 0;}
  if (JoyX > 129){ X = double(JoyX - 129) / double(125);}
  if (JoyX < 129){ X = double(JoyX - 129) / double(128);}

  //mapping raw joystick X readings to between -1 and 1 -> 126 middle, 0 low, 255 high
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

    Serial.print("\t");

    Serial.print("Y value:");
    Serial.println(Y);
    //append to input array codeblock here

    //send to bluetooth codeblock here
  }
  delay(5);
}
