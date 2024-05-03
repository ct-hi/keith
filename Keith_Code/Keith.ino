#include <Wire.h>                     // libraries needed for
#include <Adafruit_PWMServoDriver.h>  // robotic arm

/* 
# Message from 2024 CCCC MET & EET senior project class:

EET students Dale, Dawson, and Charles increased the number of motor driver
boards from 2 to 6, allowing for all 6 of Keith's wheels to be controlled
independently. We added a robotic arm and the Arduino shield board that
powers and controls the servos that move the robotic arm.

MET students designed and 3D printed a suspension system for Keith.

# Message from 2023 CCCC EET senior project class:

This program should be on the arduino board inside KEITH already.
The main problem that KEITH has is the inability to move all four directions.

I think the last time that everything was up and "working" was when the robot
moved forwards, backwards and to the right. This is why this program has this name.

If you can debug this code and get the robot to move all 4 directions that would be amazing.

Feel free to add to this program or write an entirely new one. As you can see this one is
pretty barebones/bad.

The other programs in this file are for testing the electronics and not really important
for the robot operation.

Good Luck

-Rob
*/

// declare a variable 'pwm' to use with Adafruit servo library functions
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// servo library provided by Adafruit, info here: 
// https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library

/* OLD
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  4096 // This is the 'maximum' pulse length count (out of 4096)
*/

// variables and constants for robotic arm
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define LOWER_STOP_RANGE_MOVE -20
#define UPPER_STOP_RANGE_MOVE 20
#define LOWER_STOP_RANGE_TURN -20
#define UPPER_STOP_RANGE_TURN 20

#define ELBOW2_SERVO 0
#define CLAW_SERVO 1
#define PRONATION_SERVO 2
#define WRIST_SERVO 3
#define ELBOW_SERVO 4
#define SHOULDER_SERVO 5
#define BASE_SERVO 6

#define X_MAX 2000
#define X_MIN 1000
#define Y_MAX 2000
#define Y_MIN 1000

/* FrSky V8FR-II 2.4GHz Receiver */
// define pins for servo controls (left remote control stick)
const int Channel3 = 9;
const int Channel4 = 10;
const int Channel5 = 11;
const int Channel6 = 12;

// define pins for robot driving controls (right remote control stick)
const int drivingYposition = 7;
const int drivingXposition = 8;

// servo movement variables
int xPos = 0;
int yPos = 0;
int xArmPos = 0;
int yArmPos = 0;
int xBasePos = 0;
int yBasePos = 0;

int minAngle = 0;  // Minimum angle for the servo
int maxAngle = 180;  // Maximum angle for the servo

/*
defining pins for six motor driver boards...

from Songhe BTS7960 43A High Power Motor Driver PDF file,
we have the following info to make use of...

1.Prwm:forward level or pwm signal input,high level valid 
2.Lpwm:reverse level or pwm signal input,high level valid 
3.R_En:forward drive enable input,high level enable,low level off 
4.L_En:reverse driver enable input,high level enable,low level off 
5.R_Is:forward drive side current alarm output
6.L_Is:reverse driver side current alarm output
*/

// left front
const int LF_R_En = 28;
const int LF_L_En = 29;
const int LF_Prwm = 30;
const int LF_Lpwm = 31;

// right front motor driver board
const int RF_R_En = 32;
const int RF_L_En = 33;
const int RF_Prwm = 34;
const int RF_Lpwm = 35;

// left middle motor driver board
const int LM_R_En = 22;
const int LM_L_En = 23;
const int LM_Prwm = 2;
const int LM_Lpwm = 3;

// right middle motor driver board
const int RM_R_En = 26;
const int RM_L_En = 27;
const int RM_Prwm = 4;
const int RM_Lpwm = 5;

// left rear motor driver board
const int LR_R_En = 36;
const int LR_L_En = 37;
const int LR_Prwm = 38;
const int LR_Lpwm = 39;

// right rear motor driver board
const int RR_R_En = 40;
const int RR_L_En = 41;
const int RR_Prwm = 42;
const int RR_Lpwm = 43;

void setup() {
  // this is code for driving the robot
  // initialize the left and right signal pins as inputs
  // (signals for/from remote control receiver)
  pinMode(drivingYposition, INPUT);
  pinMode(drivingXposition, INPUT);

  // this is code for the robotic arm
  pinMode(Channel3, INPUT);
  pinMode(Channel4, INPUT);
  pinMode(Channel5, INPUT);
  pinMode(Channel6, INPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  // initialize the motor driver pins as outputs
  pinMode(LF_R_En, OUTPUT);
  pinMode(LF_L_En, OUTPUT);
  pinMode(LF_Prwm, OUTPUT);
  pinMode(LF_Lpwm, OUTPUT);

  pinMode(RF_R_En, OUTPUT);
  pinMode(RF_L_En, OUTPUT);
  pinMode(RF_Prwm, OUTPUT);
  pinMode(RF_Lpwm, OUTPUT);

  pinMode(LM_R_En, OUTPUT);
  pinMode(LM_L_En, OUTPUT);
  pinMode(LM_Prwm, OUTPUT);
  pinMode(LM_Lpwm, OUTPUT);

  pinMode(RM_R_En, OUTPUT);
  pinMode(RM_L_En, OUTPUT);
  pinMode(RM_Prwm, OUTPUT);
  pinMode(RM_Lpwm, OUTPUT);

  pinMode(LR_R_En, OUTPUT);
  pinMode(LR_L_En, OUTPUT);
  pinMode(LR_Prwm, OUTPUT);
  pinMode(LR_Lpwm, OUTPUT);

  pinMode(RR_R_En, OUTPUT);
  pinMode(RR_L_En, OUTPUT);
  pinMode(RR_Prwm, OUTPUT);
  pinMode(RR_Lpwm, OUTPUT);

  // set up serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  /* DRIVING THE ROBOT STUFF */
  // read the duration of the pulse on the left signal pin
  int stickYposition = pulseIn(drivingYposition, HIGH);

  // read the duration of the pulse on the right signal pin
  int stickXposition = pulseIn(drivingXposition, HIGH);
 
  // print the pulse durations for debugging
  Serial.print("\ny value: ");
  Serial.print(stickYposition);
  Serial.print("\nx value: ");
  Serial.print(stickXposition);
  
  /* ROBOTIC ARM STUFF */
  // read the duration of the pulses that control robotic arm
  int ch3 = pulseIn(Channel3, HIGH); 
  int ch4 = pulseIn(Channel4, HIGH);
  int ch5 = pulseIn(Channel5, HIGH);
  int ch6 = pulseIn(Channel6, HIGH);
  
  yArmPos = map(ch3, 0, 1023, 0, 180); //center over zero
  xArmPos = map(ch4, 0, 1023, 0, 180);
 
 // uncomment these two lines to get robot arm diagnostic messages printed
 // in the Serial Monitor pane at the bottom of the Arduino IDE
 // window...

 // Serial.println("\nxArmPos = " + String(xArmPos) + "\n");
 // Serial.println("yArmPos = " + String(yArmPos) + "\n");

  xPos = map(ch5, 980, 1999, 150, 1999); //center over zero
  xPos = constrain(xPos, 150, 1999);
                                    
  yPos = map(ch5, 980, 1999, 600, 2400);
  yPos = constrain(yPos, 600, 2400);

  // Serial.println("channel 5 = " + String(ch5));
  // Serial.println("channel 6 = " + String(ch6));
  // Serial.println("xPos = " + String(ch3));
  // Serial.println("yPos = " + String(ch4) + "\n");

  xBasePos = map(ch3, 980, 1999, 150, 1999); //center over zero
  xBasePos = constrain(xBasePos, 150, 1999);
                                    
  yBasePos = map(ch4, 980, 1999, 600, 2400);
  yBasePos = constrain(yBasePos, 600, 2400);
  
  int mappedClawAngle = map(ch5, 0, 1023, minAngle, maxAngle);
  int mappedAngle = map(ch6, 0, 1023, minAngle, maxAngle);

  //mappedAngle = constrain(mappedAngle, 0, 1023);
  //Serial.println("baseAngle = " + String(mappedAngle) + "\n");
  // xBasePos = map(ch6, 980, 1999, 0, 180); 
  // xBasePos = constrain(xBasePos, 0, 180); 

  // yBasePos = map(ch6, 980, 1999, 0, 180); 
  // yBasePos = constrain(yBasePos, 0, 180); 
  
  /*
  Serial.print("\nxBasePos: ");
  Serial.print(xBasePos);
  Serial.print("\nyBasePos: ");
  Serial.print(yBasePos);
  */

  moveBase(mappedAngle);
  moveClaw(mappedClawAngle);
  moveArm2(xArmPos, yArmPos);
  moveArm3(xArmPos, yArmPos);

  /* MORE DRIVING THE ROBOT STUFF */
  /*
  The remote control transmitter sends a number between 900 and 2000 for the
  up-and-down position of a stick, and another number between 900 and 200
  for the left-and-right position of a stick.

  We use these number ranges to make decisions on how to control the robotic
  arm and how the remote control should make Keith's wheels spin.
  */
  // determine which direction the robot should move based on the pulse durations
  if (stickYposition < 1510 && stickYposition > 1495) {
    if (stickXposition < 1500 && stickXposition > 1480) {
      // both signals are low, stop the robot
      Serial.println("\nStop");
      int stickYposition = pulseIn(stickYposition, LOW);
      int stickXposition = pulseIn(stickXposition, LOW);
      digitalWrite(LF_R_En, LOW);
      digitalWrite(LF_L_En, LOW);
      digitalWrite(RF_R_En, LOW);
      digitalWrite(RF_L_En, LOW);
      digitalWrite(LM_R_En, LOW);
      digitalWrite(LM_L_En, LOW);
      digitalWrite(RM_R_En, LOW);
      digitalWrite(RM_L_En, LOW);
      digitalWrite(LR_R_En, LOW);
      digitalWrite(LR_L_En, LOW);
      digitalWrite(RR_R_En, LOW);
      digitalWrite(RR_L_En, LOW);

      analogWrite(LF_Prwm, 0);
      analogWrite(LF_Lpwm, 0);
      analogWrite(RF_Prwm, 0);
      analogWrite(RF_Lpwm, 0);
      analogWrite(LM_Prwm, 0);
      analogWrite(LM_Lpwm, 0);
      analogWrite(RM_Prwm, 0);
      analogWrite(RM_Lpwm, 0);
      analogWrite(LR_Prwm, 0);
      analogWrite(LR_Lpwm, 0);
      analogWrite(RR_Prwm, 0);
      analogWrite(RR_Lpwm, 0);
    }
  }

  else if (stickYposition < 1000 && stickYposition > 985) {
    if (stickXposition < 1530 && stickXposition > 1480) {
      Serial.println("\nBackward");
      digitalWrite(LF_R_En, HIGH);
      digitalWrite(LF_L_En, HIGH);
      digitalWrite(RF_R_En, HIGH);
      digitalWrite(RF_L_En, HIGH);
      digitalWrite(LM_R_En, HIGH);
      digitalWrite(LM_L_En, HIGH);
      digitalWrite(RM_R_En, HIGH);
      digitalWrite(RM_L_En, HIGH);
      digitalWrite(LR_R_En, HIGH);
      digitalWrite(LR_L_En, HIGH);
      digitalWrite(RR_R_En, HIGH);
      digitalWrite(RR_L_En, HIGH);

      analogWrite(LF_Prwm, 0);
      analogWrite(LF_Lpwm, 255);
      analogWrite(RF_Prwm, 255);
      analogWrite(RF_Lpwm, 0);
      analogWrite(LM_Prwm, 0);
      analogWrite(LM_Lpwm, 255);
      analogWrite(RM_Prwm, 0);
      analogWrite(RM_Lpwm, 255);
      analogWrite(LR_Prwm, 0);
      analogWrite(LR_Lpwm, 255);
      analogWrite(RR_Prwm, 255);
      analogWrite(RR_Lpwm, 0);
    }
}

else if (stickYposition < 1675 && stickYposition > 1285) {
  if (stickXposition < 1000 && stickXposition > 985) {
      // Spin the robot left
    Serial.println("\nSpin Left");

    digitalWrite(LF_R_En, HIGH);
    digitalWrite(LF_L_En, HIGH);
    digitalWrite(RF_R_En, HIGH);
    digitalWrite(RF_L_En, HIGH);
    digitalWrite(LM_R_En, HIGH);
    digitalWrite(LM_L_En, HIGH);
    digitalWrite(RM_R_En, HIGH);
    digitalWrite(RM_L_En, HIGH);
    digitalWrite(LR_R_En, HIGH);
    digitalWrite(LR_L_En, HIGH);
    digitalWrite(RR_R_En, HIGH);
    digitalWrite(RR_L_En, HIGH);

    analogWrite(LF_Prwm, 0);
    analogWrite(LF_Lpwm, 255);
    analogWrite(RF_Prwm, 0);
    analogWrite(RF_Lpwm, 255);
    analogWrite(LM_Prwm, 0);
    analogWrite(LM_Lpwm, 255);
    analogWrite(RM_Prwm, 255);
    analogWrite(RM_Lpwm, 0);
    analogWrite(LR_Prwm, 0);
    analogWrite(LR_Lpwm, 255);
    analogWrite(RR_Prwm, 0);
    analogWrite(RR_Lpwm, 255);
    }

    if (stickXposition < 2000 && stickXposition > 1975) {
    // turn spin the robot right
    Serial.println("\nSpin Right");

    digitalWrite(LF_R_En, HIGH);
    digitalWrite(LF_L_En, HIGH);
    digitalWrite(RF_R_En, HIGH);
    digitalWrite(RF_L_En, HIGH);
    digitalWrite(LM_R_En, HIGH);
    digitalWrite(LM_L_En, HIGH);
    digitalWrite(RM_R_En, HIGH);
    digitalWrite(RM_L_En, HIGH);
    digitalWrite(LR_R_En, HIGH);
    digitalWrite(LR_L_En, HIGH);
    digitalWrite(RR_R_En, HIGH);
    digitalWrite(RR_L_En, HIGH);

    analogWrite(LF_Prwm, 255);
    analogWrite(LF_Lpwm, 0);
    analogWrite(RF_Prwm, 255);
    analogWrite(RF_Lpwm, 0);
    analogWrite(LM_Prwm, 255);
    analogWrite(LM_Lpwm, 0);
    analogWrite(RM_Prwm, 0);
    analogWrite(RM_Lpwm, 255);
    analogWrite(LR_Prwm, 255);
    analogWrite(LR_Lpwm, 0);
    analogWrite(RR_Prwm, 255);
    analogWrite(RR_Lpwm, 0);
  }
}

else if (stickYposition < 2000 && stickYposition > 1975) {
    if (stickXposition < 1000 && stickXposition > 975) {
    // turn the robot left
    Serial.println("\nLeft\n");

    digitalWrite(LF_R_En, LOW);
    digitalWrite(LF_L_En, LOW);
    digitalWrite(RF_R_En, HIGH);
    digitalWrite(RF_L_En, HIGH);
    digitalWrite(LM_R_En, LOW);
    digitalWrite(LM_L_En, LOW);
    digitalWrite(RM_R_En, HIGH);
    digitalWrite(RM_L_En, HIGH);
    digitalWrite(LR_R_En, LOW);
    digitalWrite(LR_L_En, LOW);
    digitalWrite(RR_R_En, HIGH);
    digitalWrite(RR_L_En, HIGH);

    analogWrite(LF_Prwm, 0);
    analogWrite(LF_Lpwm, 0);
    analogWrite(RF_Prwm, 0);
    analogWrite(RF_Lpwm, 255);
    analogWrite(LM_Prwm, 0);
    analogWrite(LM_Lpwm, 0);
    analogWrite(RM_Prwm, 255);
    analogWrite(RM_Lpwm, 0);
    analogWrite(LR_Prwm, 0);
    analogWrite(LR_Lpwm, 0);
    analogWrite(RR_Prwm, 0);
    analogWrite(RR_Lpwm, 255);
  }
  if (stickXposition < 1630 && stickXposition > 1480) {
      // both signals are high, move the robot forward
      Serial.println("\nForward");

      digitalWrite(LF_R_En, HIGH);
      digitalWrite(LF_L_En, HIGH);
      digitalWrite(RF_R_En, HIGH);
      digitalWrite(RF_L_En, HIGH);
      digitalWrite(LM_R_En, HIGH);
      digitalWrite(LM_L_En, HIGH);
      digitalWrite(RM_R_En, HIGH);
      digitalWrite(RM_L_En, HIGH);
      digitalWrite(LR_R_En, HIGH);
      digitalWrite(LR_L_En, HIGH);
      digitalWrite(RR_R_En, HIGH);
      digitalWrite(RR_L_En, HIGH);

      analogWrite(LF_Prwm, 255);
      analogWrite(LF_Lpwm, 0);
      analogWrite(RF_Prwm, 0);
      analogWrite(RF_Lpwm, 255);
      analogWrite(LM_Prwm, 255);
      analogWrite(LM_Lpwm, 0);
      analogWrite(RM_Prwm, 255);
      analogWrite(RM_Lpwm, 0);
      analogWrite(LR_Prwm, 255);
      analogWrite(LR_Lpwm, 0);
      analogWrite(RR_Prwm, 0);
      analogWrite(RR_Lpwm, 255);
    }
    if (stickXposition < 2000 && stickXposition > 1975) {
      // left signal is low and right signal is high, turn the robot right
      Serial.println("\nRight");
    
      digitalWrite(LF_R_En, HIGH);
      digitalWrite(LF_L_En, HIGH);
      digitalWrite(RF_R_En, LOW);
      digitalWrite(RF_L_En, LOW);
      digitalWrite(LM_R_En, HIGH);
      digitalWrite(LM_L_En, HIGH);
      digitalWrite(RM_R_En, LOW);
      digitalWrite(RM_L_En, LOW);
      digitalWrite(LR_R_En, HIGH);
      digitalWrite(LR_L_En, HIGH);
      digitalWrite(RR_R_En, LOW);
      digitalWrite(RR_L_En, LOW);

      analogWrite(LF_Prwm, 255);
      analogWrite(LF_Lpwm, 0);
      analogWrite(RF_Prwm, 0);
      analogWrite(RF_Lpwm, 0);
      analogWrite(LM_Prwm, 255);
      analogWrite(LM_Lpwm, 0);
      analogWrite(RM_Prwm, 0);
      analogWrite(RM_Lpwm, 0);
      analogWrite(LR_Prwm, 255);
      analogWrite(LR_Lpwm, 0);
      analogWrite(RR_Prwm, 0);
      analogWrite(RR_Lpwm, 0);
    }
  }
}

void moveClaw(int x) {
 pwm.setPWM(CLAW_SERVO, 0, x);
}

void moveBase(int x) {
  pwm.setPWM(PRONATION_SERVO, 0, x);

}

void moveArm(int x, int y) {
  double L1 = 40.0; // Length of the first arm
  double L2 = 30.0; // Length of the second arm

  double angle1_rad = atan2(x, y); // Calculate the angle of the base servo
  double D = sqrt(sq(x) + sq(y));
  double angle2_rad = acos((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2)); // Calculate the angle of the shoulder servo
  double angle3_rad = acos((sq(D) + sq(L1) - sq(L2)) / (2 * D * L1)); // Calculate the angle of the elbow servo

  int baseAngle = map(angle1_rad * 180 / M_PI, 0, 180, SERVOMIN, SERVOMAX); // Convert angles to servo values
  int shoulderAngle = map(angle2_rad * 180 / M_PI, 0, 180, SERVOMIN, SERVOMAX);
  int elbowAngle = map(angle3_rad * 180 / M_PI, 0, 180, SERVOMIN, SERVOMAX);

  //pwm.setPWM(CLAW_SERVO, 0, baseAngle); // Move the servos to the calculated positions
  //pwm.setPWM(WRIST_SERVO, 0, shoulderAngle);
  pwm.setPWM(ELBOW_SERVO, 0, shoulderAngle);
  //pwm.setPWM(PRONATION_SERVO, 0, baseAngle);
  //pwm.setPWM(SHOULDER_SERVO, 0, baseAngle);
  //pwm.setPWM(BASE_SERVO, 0, baseAngle);

  //pwm.setPWM(ELBOW2_SERVO, 0, elbowAngle);

  //Serial.println("baseAngle = " + String(baseAngle) + "\n");
  //Serial.println("shoulderAngle = " + String(shoulderAngle) + "\n");
  //Serial.println("elbowAngle = " + String(elbowAngle) + "\n\n");
}

void moveArm2(int x, int y) {
  int xAngle = map(x, 0, 1023, 0, 180); // Map x value to servo angle

  int yAngle = map(y, 0, 1023, 0, 180); // Map y value to servo angle
  
  pwm.setPWM(ELBOW_SERVO, 0, x);
  //pwm.setPWM(PRONATION_SERVO, 0, baseAngle);
  //pwm.setPWM(SHOULDER_SERVO, 0, baseAngle);
  //pwm.setPWM(BASE_SERVO, 0, baseAngle);

  //pwm.setPWM(ELBOW2_SERVO, 0, elbowAngle);

  //Serial.println("baseAngle = " + String(baseAngle) + "\n");
  //Serial.println("shoulderAngle = " + String(shoulderAngle) + "\n");
  //Serial.println("elbowAngle = " + String(elbowAngle) + "\n\n");
}
void moveArm3(int x, int y) {
  int xAngle = map(x, 0, 1023, 0, 180); // Map x value to servo angle

  int yAngle = map(y, 0, 1023, 0, 180); // Map y value to servo angle
  
  //pwm.setPWM(ELBOW_SERVO, 0, x);
  //pwm.setPWM(PRONATION_SERVO, 0, baseAngle);
  pwm.setPWM(SHOULDER_SERVO, 0, y);
  //pwm.setPWM(BASE_SERVO, 0, baseAngle);

  //pwm.setPWM(ELBOW2_SERVO, 0, elbowAngle);

  //Serial.println("baseAngle = " + String(baseAngle) + "\n");
  //Serial.println("shoulderAngle = " + String(shoulderAngle) + "\n");
  //Serial.println("elbowAngle = " + String(elbowAngle) + "\n\n");
}
