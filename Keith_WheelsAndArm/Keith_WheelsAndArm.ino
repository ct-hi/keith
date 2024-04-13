/*********** SERVO INCLUDES ***********/
#include <Wire.h>                     
#include <Adafruit_PWMServoDriver.h> 
/********* END SERVO INCLUDES *********/

/* Message from 2024 CCCC MET & EET senior project class:

EET students Dale, Dawson, and Charles increased the number of motor driver
boards from 2 to 6, allowing for all 6 of Keith's wheels to be controlled
independently. We added a robotic arm and the Arduino shield board that
powers and controls the servos that move the robotic arm.

*/

/* Message from 2023 CCCC EET senior project class:

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

// defining servo variables and constants
// servo library provided by Adafruit, info here: 
// https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* OLD
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  4096 // This is the 'maximum' pulse length count (out of 4096)
*/
//NEW
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define LOWER_STOP_RANGE_MOVE -20
#define UPPER_STOP_RANGE_MOVE 20
#define LOWER_STOP_RANGE_TURN -20
#define UPPER_STOP_RANGE_TURN 20

// our servo # counter
uint8_t servonum = 0;

/* FrSky V8FR-II 2.4GHz Receiver */
// define pins for servo controls (left remote control stick)
const int Channel3 = 9;
const int Channel4 = 10;

// from servo arm tutorial, currently unused?
boolean stop_state = true;

// define pins for robot driving controls (right remote control stick)
const int leftPin = 7;
const int rightPin = 8;

// variables in which to store the pulse width info coming from the remote
// to be used to control the robotic arm
int ch3; 
int ch4; 

// servo movement variables
int moveValue;
int turnValue;

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

// right front
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

// left rear
const int LR_R_En = 36;
const int LR_L_En = 37;
const int LR_Prwm = 38;
const int LR_Lpwm = 39;

// right rear
const int RR_R_En = 40;
const int RR_L_En = 41;
const int RR_Prwm = 42;
const int RR_Lpwm = 43;

void setup() {
  // this is code for the robotic arm
  pinMode(Channel3, INPUT);
  pinMode(Channel4, INPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //delay(10);

  // this is code for driving the robot
  // initialize the left and right signal pins as inputs
  // (signals for/from remote control receiver)
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  
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
  int leftPulseDuration = pulseIn(leftPin, HIGH);

  // read the duration of the pulse on the right signal pin
  int rightPulseDuration = pulseIn(rightPin, HIGH);

  // print the pulse durations for debugging
  Serial.print("\nLeft: ");
  Serial.print(leftPulseDuration);
  Serial.print("\nRight: ");
  Serial.print(rightPulseDuration);
  /* END DRIVING */

/* ROBOTIC ARM STUFF */
  // read the duration of the pulses that control robotic arm
  ch3 = pulseIn(Channel3, HIGH, 25000); 
  ch4 = pulseIn(Channel4, HIGH, 25000);

  moveValue = map(ch4, 980, 1999, 150, 1999); //center over zero
  moveValue = constrain(moveValue, 150, 1999);
                                   
  turnValue = map(ch4, 980, 1999, 600, 2400);
  turnValue = constrain(turnValue, 600, 2400);
  //Serial.println("ch3 = " + String(ch3));
  //Serial.println("ch4 = " + String(ch4));
  Serial.println("turnValue = " + String(turnValue));
  Serial.println("moveValue = " + String(moveValue));
 
 for (int i = 0; i < 10; i++) {
  // Example of moving the end effector to different positions
    calculateServoAngles(100, 50, 150); // Move to position (100, 50, 150)
    delay(1000);
    calculateServoAngles(50, 100, 150); // Move to position (50, 100, 150)
    delay(1000);
    calculateServoAngles(0, 0, 150); // Move to position (0, 0, 150)
    delay(1000);
 }
/* OLD
  pwm.setPWM(0, 0, turnValue);
  pwm.setPWM(0, 0, moveValue);
  pwm.setPWM(1, 0, turnValue);
  pwm.setPWM(1, 0, moveValue);
  pwm.setPWM(2, 0, turnValue);
  pwm.setPWM(2, 0, moveValue);
  pwm.setPWM(3, 0, turnValue);
  pwm.setPWM(3, 0, moveValue);
  pwm.setPWM(4, 0, turnValue);
  pwm.setPWM(4, 0, moveValue);
  pwm.setPWM(5, 0, turnValue);
  pwm.setPWM(5, 0, moveValue);
  */

  /* END ROBOTIC ARM */
  
  /* MORE DRIVING THE ROBOT STUFF */
  // determine which direction the robot should move based on the pulse durations
  if (leftPulseDuration < 1480 && leftPulseDuration > 1466) {
    if (rightPulseDuration < 1500 && rightPulseDuration > 1496) {
      // both signals are low, stop the robot
      Serial.println("\nStop");
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

  else if (leftPulseDuration < 1485 && leftPulseDuration > 1475) {
    if (rightPulseDuration < 1990 && rightPulseDuration > 1980) {
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
  }

  else if (leftPulseDuration < 1489 && leftPulseDuration > 1480) {
    if (rightPulseDuration < 995 && rightPulseDuration > 990) {
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

else if (leftPulseDuration < 1000 && leftPulseDuration > 975) {
  if (rightPulseDuration < 1500 && rightPulseDuration > 1300) {
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
}

 else if (leftPulseDuration < 1975 && leftPulseDuration > 1965) {
    if (rightPulseDuration < 1505 && rightPulseDuration > 1495) {
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

else if (leftPulseDuration < 990 && leftPulseDuration > 980) {
    if (rightPulseDuration < 1000 && rightPulseDuration > 990) {
    // turn the robot left
    Serial.println("\nLeft");

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
    analogWrite(RF_Prwm, 255);
    analogWrite(RF_Lpwm, 0);
    analogWrite(LM_Prwm, 0);
    analogWrite(LM_Lpwm, 0);
    analogWrite(RM_Prwm, 255);
    analogWrite(RM_Lpwm, 0);
    analogWrite(LR_Prwm, 0);
    analogWrite(LR_Lpwm, 0);
    analogWrite(RR_Prwm, 255);
    analogWrite(RR_Lpwm, 0);
  }
}

  else if (leftPulseDuration < 1975 && leftPulseDuration > 1966) {
    if (rightPulseDuration < 1990 && rightPulseDuration > 1980) {
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
/* END DRIVING */
}

// for the robotic arm
// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setServoAngle(int servoNum, float angle) {
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  uint16_t pwmVal = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pwmVal);
}

void calculateServoAngles(float x, float y, float z) {
  // Inverse kinematics calculation goes here to determine servo angles based on x, y, z coordinates
  // This example assumes simple trigonometry for calculation
  float theta1 = atan2(y, x);
  float D = sqrt(x*x + y*y);
  float theta2 = acos((z*z - D*D - 100*100) / (-2 * D * 100));
  float theta3 = acos((100*100 - D*D - z*z) / (2 * z * D));

  setServoAngle(0, toDegrees(theta1));
  setServoAngle(1, toDegrees(theta2));
  setServoAngle(2, toDegrees(theta3));
  setServoAngle(3, toDegrees(theta1));
  setServoAngle(4, toDegrees(theta2));
  setServoAngle(5, toDegrees(theta3));
}

float toDegrees(float rad) {
  return rad * 180 / PI;
}

