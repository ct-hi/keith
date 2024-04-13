/*This program should be on the arduino board inside KEITH already.
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

// define pins for left and right signals
const int leftPin = 7;
const int rightPin = 8;

// define pins for motor driver
const int R_EN = 22;
const int L_EN = 23;
const int R_EN_1 = 26;
const int L_EN_1 = 27;
const int Rpwm_1 = 2;
const int Lpwm_1 = 3;
const int Rpwm_2 = 4;
const int Lpwm_2 = 5;

void setup() {
  // initialize the left and right signal pins as inputs
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  
  // initialize the motor driver pins as outputs
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN_1, OUTPUT);
  pinMode(L_EN_1, OUTPUT);
  pinMode(Rpwm_1, OUTPUT);
  pinMode(Lpwm_1, OUTPUT);
  pinMode(Rpwm_2, OUTPUT);
  pinMode(Lpwm_2, OUTPUT);
  
  // set up serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // read the duration of the pulse on the left signal pin
  int leftPulseDuration = pulseIn(leftPin, HIGH);

  // read the duration of the pulse on the right signal pin
  int rightPulseDuration = pulseIn(rightPin, HIGH);

  // print the pulse durations for debugging
  Serial.print("Left: ");
  Serial.print(leftPulseDuration);
  Serial.print("  Right: ");
  Serial.print(rightPulseDuration);
  
  // determine which direction the robot should move based on the pulse durations
  if (leftPulseDuration < 1400 && rightPulseDuration < 1400) {
    // both signals are low, stop the robot
    Serial.println("Stop");
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    digitalWrite(R_EN_1, LOW);
    digitalWrite(L_EN_1, LOW);
    analogWrite(Rpwm_1, 0);
    analogWrite(Lpwm_1, 0);
    analogWrite(Rpwm_2, 0);
    analogWrite(Lpwm_2, 0);
  }
  else if (leftPulseDuration > 1700 && rightPulseDuration < 1400) {
    // left signal is high and right signal is low, turn the robot left
    Serial.println("Left");
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN_1, HIGH);
    digitalWrite(L_EN_1, HIGH);
    analogWrite(Rpwm_1, 255);
    analogWrite(Lpwm_1, 0);
    analogWrite(Rpwm_2, 0);
    analogWrite(Lpwm_2, 255);
  }
  else if (leftPulseDuration < 1400 && rightPulseDuration > 1700){
    // left signal is low and right signal is high, turn the robot right
    Serial.println("Right");
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN_1, HIGH);
    digitalWrite(L_EN_1, HIGH);
    analogWrite(Rpwm_1, 0);
    analogWrite(Lpwm_1, 255);
    analogWrite(Rpwm_2, 255);
    analogWrite(Lpwm_2, 0);
    }
  else if (leftPulseDuration > 1700 && rightPulseDuration > 1700) {
    // both signals are high, move the robot forward
    Serial.println("Forward");
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    digitalWrite(R_EN_1, HIGH);
    digitalWrite(L_EN_1, HIGH);
    analogWrite(Rpwm_1, 255);
    analogWrite(Lpwm_1, 0);
    analogWrite(Rpwm_2, 255);
    analogWrite(Lpwm_2, 0);
  }
}