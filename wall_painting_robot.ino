// Include the Arduino Stepper Library
#include <Stepper.h>
#include <Ultrasonic.h>
Ultrasonic ultrasonic (4, 3);
// Number of steps per output rotation
const int stepsPerRevolution = 200;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 13, 12, 11, 10);
int currentPos;                                       //iniitial position
int lastPos, distanceGap = 10;                       //the distance to be  traveled by the robot i.e. the width of the nozzle 
int pauseTime = 2000;
int  minThreshold = 10, maxThreshold = 400;         //the area coverage of the robot




// Motor A connections
int in1 = 9;
int in2 = 8;
// Motor B connections
int in3 = 7;
int in4 = 6;

int buttonPin = 2; 
int buttonState = 0;  


void setup() {


  Serial.begin(9600);
  // Set all the DC motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // set the speed of the stepper motor at 60 rpm:
  myStepper.setSpeed(60);
  
  
  updatePos();
}

void loop() {


  updatePos();
Serial.println(currentPos);
Serial.println(lastPos);
delay(200);
if (currentPos >= 0) {
    if (abs(currentPos - lastPos) >= distanceGap && currentPos != 0) {
//     Stop();
     Serial.println("Stop");
      MotorStop();
      stepperGO();   //when 10cm threshold is reached stop the vehicle and make on the servo
      
      lastPos = currentPos; //update the current position
      delay (pauseTime);
    } else if (currentPos < minThreshold) {
      //if we reached on parellel wall with less than 10cm range, stop the vehicle to avoid collision
      Serial.println(" last Stop");
      MotorStop();
     
    } else {
      //////compute current position and if the servo finish its work, forward the vehicle with 10cm
      Serial.println("forward");
      go();
    
    }
  }
  
}

// This function lets you control spinning direction of motors
void go() {

  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
  
  // Now change motor directions
 /* digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  
  digitalWrite(in4, HIGH);
  delay(2000);*/
  
  // Turn off motors
 void MotorStop()
 {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  

 
}

void stepper(){
  
 // step one revolution in one direction:
  myStepper.step(stepsPerRevolution);
  delay(1000);



  // step one revolution in the other direction:
 
  myStepper.step(-stepsPerRevolution);
  delay(1000);
  
  }

    /////////////////new function
    int updatePos () {
  int pos = ultrasonic.read();
  if (pos < maxThreshold)
    currentPos = pos;
 return currentPos;
}

// stop function for the dc motor rotation

//stepper movement function
void stepperGO(){
   myStepper.step(stepsPerRevolution);
    delay(500);
    //Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(500);
    //Serial.println("clockwise");
  }

