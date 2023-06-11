//Ryan Khoo 2022, Bipedal Robot Controller Code

#include <NintendoExtensionCtrl.h>
Nunchuk nchuk;

int green = 8; //Red LED pin (0 for ON)
int red = 7; //Orange LED pin

void setup() {
  Serial.begin(9600);
  nchuk.begin();
  nchuk.connect();
  
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  digitalWrite(green, 1);//green light starts off
  digitalWrite(red, 1);//red light starts off
}

void loop() {

  nchuk.update();
  //Obtain Current State of remote
  int Cbutton = nchuk.buttonC();
  int Zbutton = nchuk.buttonZ();
  int Xvalue = nchuk.joyX();//0-255, middle at 129
  int Yvalue = nchuk.joyY();//middle at 126
  int Roll = nchuk.rollAngle();
  int Pitch = nchuk.pitchAngle();
  //Serial.print("Yvalue:");Serial.println(Yvalue);



  //Manual Look control red light mode
  if (Cbutton == 1){ 
    digitalWrite(red, 0); //red light on
    //Manual Pitch
    int pitchCommand = map(Pitch,-30,30,112,116);
    if (pitchCommand >=112 && pitchCommand <= 116){
      Serial.write(pitchCommand);//-------------------------------------------Command Send
    }
    //Manual Roll
    int rollCommand = map(Roll,-45,45,129,120);//swapped
    if (rollCommand >=120 && rollCommand <= 129){
      Serial.write(rollCommand);//-------------------------------------------Command Send
    }
    //Manual Look left and right
    int lookCommand = map(Xvalue,0,253,132,137);
    if (lookCommand >=132 && lookCommand <= 137){
      Serial.write(lookCommand);//-------------------------------------------Command Send
    }
    delay(60);
  }

  //normal joy operation

    //green light when joy moved
    if(((Xvalue > 130 or Xvalue < 127)or((Yvalue > 127 or Yvalue < 125))) && Cbutton==0 && Zbutton ==0){
      digitalWrite(green, 0);//green light on

    //Forward
    int forwardCommand = map(Yvalue,129,255,10,19);
    if (forwardCommand >12 && forwardCommand <= 19){
      Serial.write(forwardCommand);//-------------------------------------------Command Send
      Serial.println(forwardCommand);
      
    }
        //Back
    int backCommand = map(Yvalue,0,126,59,50);
    if (backCommand >52 && backCommand <= 59){
      Serial.write(backCommand);//-------------------------------------------Command Send
      Serial.println(backCommand);
      
    }
        //Right
    int rightCommand = map(Xvalue,126,254,30,39);
    if (rightCommand >32 && rightCommand <= 39){
      Serial.write(rightCommand);//-------------------------------------------Command Send
      Serial.println(rightCommand);
      
    }

        //Left
    int leftCommand = map(Xvalue,0,125,79,70);
    if (leftCommand >72 && leftCommand <= 79){
      Serial.write(leftCommand);//-------------------------------------------Command Send
      Serial.println(leftCommand);
      
    }
    delay(100);


      
    }
 
  //temp z button activation
  if (Zbutton == 1){
    digitalWrite(green, 0);//yellow light on
    digitalWrite(red, 0);
  }



  //Yellow for Z button and green for normal moving operation



//Turn off lights if nothing pressed
  if (Yvalue==126 && (Xvalue==129 or Xvalue==128) && Zbutton==0 && Cbutton == 0){
    digitalWrite(green, 1);//green light off
    digitalWrite(red, 1);//red light off
  }
  delay(30); //delay to avoid overloading bluetooth bus, hi future ryan!
}
