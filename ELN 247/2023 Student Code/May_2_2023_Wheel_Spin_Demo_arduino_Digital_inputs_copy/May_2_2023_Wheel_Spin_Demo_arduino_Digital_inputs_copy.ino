//This Arduino code is developed by I Lab
//Hit the SUBSCRIBE button for following our tutorial on arduino.
//You Tube Channel ID: https://www.youtube.com/c/IngenieroLab?sub_confirmation=1.
//Follow our facebook page: https://www.facebook.com/ingenierorobotico

//This program makes the motors spin forwards and backwards on loop.
//BTS7960 motor driver sketch 
//Right Motor Driver
int R_EN = 22;
int R_PWM = 2;
int L_EN = 23;
int L_PWM = 3;

//Left Motor Driver
int R_EN_1 = 26;
int R_PWM_1 = 4;
int L_EN_1 = 27;
int L_PWM_1 = 5;

void setup() {
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);

 pinMode(R_EN_1, OUTPUT);
 pinMode(R_PWM_1, OUTPUT);
 pinMode(L_EN_1, OUTPUT);
 pinMode(L_PWM_1, OUTPUT);
 digitalWrite(R_EN_1, HIGH);
 digitalWrite(L_EN_1, HIGH);
}

void loop() {
  int i;
  for(i = 0; i <= 255; i= i+10){ //clockwise rotation
   analogWrite(R_PWM, i);
   analogWrite(L_PWM, 0);
   analogWrite(R_PWM_1, i);
   analogWrite(L_PWM_1, 0);
   delay(500);
  }
  delay(500);
  for(i = 0; i <= 255; i= i+10){ //counter clockwise rotation
   analogWrite(R_PWM, 0);
   analogWrite(L_PWM, i);
   analogWrite(R_PWM_1, 0);
   analogWrite(L_PWM_1, i);
   delay(500);
  }
  delay(500);
}
