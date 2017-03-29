#include "QTRSensors.h"
#include <Servo.h>

#define PWM 0
#define INA 1
#define INB 2
#define ISR_PIN 3
#define SIG2 4

#define BUTTONPIN 52
#define SERVOPIN 11

#define STEPPER_DIR_PIN 9
#define STEPPER_STEP_PIN 8

#define CALIBRATE 1
#define MOVE_TO_FIRST_PUTT 2
#define PUTT 3
#define TURN_AROUND 4
#define MOVE_TO_BUTTON 5
#define PUSH_BUTTON 6
#define MOVE_TO_WINDMILL 7 
#define WINDMILL_PUTT 8

#define xInput A0
#define yInput A1
#define zInput A2

const int sensorWidth = 70;
int bmin = 600;
int m1Pins[] ={12,40,41,2,3};
int m2Pins[] ={13,42,43,4,5};

int distance[] = {5, 10, 15, 20, 25, 30, 35, 40};
double voltage[] = {2.39, 1.67, 1.28, 1.07, .90, .80, .71};
int sensorPin = A3;
double sensorVal = 0;

Servo servo;
int angleMin = 15, angleMax = 172;

unsigned char inputPins[] = {22,23,24,25,26,27,28,29};
unsigned int inputVals[8];
QTRSensorsRC qtr(inputPins, 8);

int state = -1;

//encoders
volatile long countM1=0;
volatile long countM2=0;
volatile long dCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(m1Pins[INA], OUTPUT);
  pinMode(m1Pins[INB], OUTPUT);  
  pinMode(m2Pins[INA], OUTPUT);
  pinMode(m2Pins[INB], OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  
  for(int i = 0; i < 8; i++) {
    pinMode(inputPins[i], INPUT);
  }
  pinMode(52, INPUT_PULLUP);
//  Serial.print("Calibrating.\n");
//  while(digitalRead(52)==HIGH) {
//    qtr.calibrate();
//  }
//  Serial.println("Starting following");
  servo.attach(SERVOPIN);
 servo.write(15);
  attachInterrupt(digitalPinToInterrupt(m1Pins[ISR_PIN]),isrM1rising, RISING);
  attachInterrupt(digitalPinToInterrupt(m1Pins[ISR_PIN]),isrM1falling, FALLING);
  attachInterrupt(digitalPinToInterrupt(m2Pins[ISR_PIN]),isrM2rising, RISING);    
  attachInterrupt(digitalPinToInterrupt(m2Pins[ISR_PIN]),isrM2falling, FALLING);
  pinMode(m1Pins[ISR_PIN], INPUT_PULLUP);
  pinMode(m1Pins[SIG2], INPUT_PULLUP);
  pinMode(m2Pins[ISR_PIN], INPUT_PULLUP);
  pinMode(m2Pins[SIG2], INPUT_PULLUP);
  state = CALIBRATE;
}

double readDist() {
  sensorVal = analogRead(sensorPin);
  if((sensorVal/1023)*5 < .8) {
     return 40;
  }
  double Vin = (sensorVal/1023)*5;
  for(int i = 1; i < 6; i++) {
   if(voltage[i]-Vin < 0) {
     return abs(((distance[i]-distance[i-1])/(voltage[i]-voltage[i-1]))*((sensorVal/1023*5)-voltage[i-1])+distance[i-1]);
   }
  }
}

// Read 10 samples and report the average
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  for (int i = 0; i < 10; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading/10;
}

int flatX = 397, flatY = 397, flatZ = 400;
bool isFlat() {
  int x = ReadAxis(xInput);
  int y = ReadAxis(yInput);
  int z = ReadAxis(zInput);
  return abs(x-flatX) <= 30 && abs(y-flatY) <= 30 && abs(z-flatZ) <= 30;
}

bool atStop() {
  int seeBlack = 0;
  qtr.read(inputVals);
  for(int i = 0; i < 8; i++) {
    if(inputVals[i] >=  bmin) {
      seeBlack++;
    }
  }
  
  return seeBlack >= 5;
}

int changeMax = 100;
void followLine() {
  int linepos = qtr.readLine(inputVals)-3500; 
  int pwmLeft = 175-changeMax, pwmRight = 175-changeMax;
  if(linepos>=3250){
    pwmLeft = 200;
    pwmRight= 0;
  }else if (linepos<=-3250){
    pwmLeft = 0;
    pwmRight = 200;
  } else if(linepos < 0) {
    pwmLeft -= map(linepos, -3500, 0, changeMax, 0);
    pwmRight += map(linepos, -3500, 0, changeMax, 0);
  } else if(linepos > 0) {
    pwmLeft += map(linepos, 0, 3500, 0, changeMax);
    pwmRight -= map(linepos, 0, 3500, 0, changeMax);
  }
  
  digitalWrite(m2Pins[INA], LOW);
  digitalWrite(m2Pins[INB], HIGH); 
  analogWrite(m2Pins[PWM], pwmLeft);
  digitalWrite(m1Pins[INA], LOW); 
  digitalWrite(m1Pins[INB], HIGH);
  analogWrite(m1Pins[PWM], pwmRight);
}

void stopMoving() {
  digitalWrite(m2Pins[INA], LOW);
  digitalWrite(m2Pins[INB], LOW); 
  analogWrite(m2Pins[PWM], 255);
  digitalWrite(m1Pins[INA], LOW); 
  digitalWrite(m1Pins[INB], LOW);
  analogWrite(m1Pins[PWM], 255);
}

void forwardpwm(int pwmL, int pwmR){
  digitalWrite(m2Pins[INA], LOW);
  digitalWrite(m2Pins[INB], HIGH); 
  analogWrite(m2Pins[PWM], pwmL);
  digitalWrite(m1Pins[INA], LOW); 
  digitalWrite(m1Pins[INB], HIGH);
  analogWrite(m1Pins[PWM], pwmR);
}

void backwardpwm(int pwmL, int pwmR){
  digitalWrite(m2Pins[INA], HIGH);
  digitalWrite(m2Pins[INB], LOW); 
  analogWrite(m2Pins[PWM], pwmL);
  digitalWrite(m1Pins[INA], HIGH); 
  digitalWrite(m1Pins[INB], LOW);
  analogWrite(m1Pins[PWM], pwmR);
}

void spinpwm1(int pwmL, int pwmR){
  digitalWrite(m2Pins[INA], LOW);
  digitalWrite(m2Pins[INB], HIGH); 
  analogWrite(m2Pins[PWM], pwmL);
  digitalWrite(m1Pins[INA], HIGH); 
  digitalWrite(m1Pins[INB], LOW);
  analogWrite(m1Pins[PWM], pwmR);
}

void spinpwm2(int pwmL, int pwmR){
  digitalWrite(m2Pins[INA], HIGH);
  digitalWrite(m2Pins[INB], LOW); 
  analogWrite(m2Pins[PWM], pwmL);
  digitalWrite(m1Pins[INA], LOW); 
  digitalWrite(m1Pins[INB], HIGH);
  analogWrite(m1Pins[PWM], pwmR);
}

void isrM1rising() {
  if (digitalRead(m1Pins[ISR_PIN])==HIGH)
    countM1++;  //moving forward
  else
    countM1--;  //moving backwards

}

void isrM1falling() {
  if (digitalRead(m1Pins[ISR_PIN])==LOW)
    countM1++;  //moving forward
  else
    countM1--;  //moving backwards

}

void isrM2rising() {
  if (digitalRead(m2Pins[ISR_PIN])==HIGH)
    countM2++;  //moving forward
  else
    countM2--;  //moving backwards

}

void isrM2falling() {
  if (digitalRead(m2Pins[ISR_PIN])==LOW)
    countM2++;  //moving forward
  else
    countM2--;  //moving backwards

}


long starttime = 0;
void calibratelinesensor(){
  long currenttime = millis();
  while( currenttime-starttime<=2500){
    qtr.calibrate(); 
    if(currenttime-starttime<=600){
      spinpwm1(150,150);
    }else if(currenttime-starttime<=1800){
      spinpwm2(150,150);
    } else if(currenttime-starttime<=2500) {
      spinpwm1(150,150);
    }
    currenttime = millis();
  }
  stopMoving();
}

void putt(){
  servo.write(angleMin);
  delay(500);
  servo.write(90);
  delay(500);
  servo.write(angleMin);
}

void pushbutton(){
  digitalWrite(STEPPER_DIR_PIN, HIGH);
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  for(int i = 0; i < 100; i++) {
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delay(1);
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delay(1);  
  }
  digitalWrite(STEPPER_DIR_PIN, LOW);
  digitalWrite(STEPPER_STEP_PIN, LOW  );
  for(int i = 0; i < 100; i++) {
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delay(1);
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delay(1); 
  }
}

void cmForward(int x) {
  dCount = ceil(x*32.0/(6.90*PI));
  countM1 = 0;
  countM2 = 0;
  forwardpwm(170, 170);
  while(abs(countM1) < dCount && abs(countM2) < dCount) {  }
  stopMoving();
}

void calibrateAccellerometer() {
  flatX = ReadAxis(xInput);
  flatY = ReadAxis(yInput);
  flatZ = ReadAxis(zInput);
}

void turnaround(){
   countM1 = 0;
  countM2 = 0;
  int dCount=40; //(19/2.0*PI)*(32/6.9)
  spinpwm2(175,175);
  while(abs(countM1) < dCount && abs(countM2) < dCount) {
    Serial.print(countM1);
    Serial.print("  ");
    Serial.print(countM2);}
}
bool hitSlope = false;
bool good=false;
void loop() {
  Serial.println(state);
  switch(state){
    case CALIBRATE:
      while(digitalRead(52)==HIGH){}
      Serial.println("calibrate");
       starttime = millis();
       calibratelinesensor();      //calibrate line sensor
       calibrateAccellerometer();
       state = MOVE_TO_FIRST_PUTT;
      break;
    case MOVE_TO_FIRST_PUTT:
     if(!good){
          while(digitalRead(52)==HIGH){}
          good=true;
        }
     if(!atStop()) {             //follow line to first hole
        followLine();
     } else {
      stopMoving();
      state=PUTT;
     }                        
    break;
    case PUTT:       //put the ball
      //putt the ball
      putt();
      state=TURN_AROUND;
      break;
    case TURN_AROUND:       //reverse direction
      turnaround(); 
      state=MOVE_TO_BUTTON;
      break;
    case MOVE_TO_BUTTON: //follow line to button
      if(!isFlat()) {
        hitSlope = true;
        Serial.print("\n\n\n !!!AT FLAT!!!\n\n\n");
      }
      if(hitSlope) {
        if(atStop()) {
          cmForward(50); // Move forward enough for stepper to hit button
          state = PUSH_BUTTON;
        } else {
          followLine();
        }
      } else {
        followLine();
      }
      break;
    case PUSH_BUTTON: //push button
      pushbutton();
      state = MOVE_TO_WINDMILL;
      break;
    case MOVE_TO_WINDMILL: //follow line to final hole
      if(!atStop()) {             //follow line to first hole
          followLine();
         } else {
           stopMoving();         
           state=WINDMILL_PUTT;
         }
      break;
    case WINDMILL_PUTT: //putt golf ball
      if(readDist() >= 40) {
        putt();
      }
      // else if(readDist < 39) {
// wait
      //}
      state=9;
      break;
    case 9: //stop once finished
      stopMoving();
      break;
  }
}


