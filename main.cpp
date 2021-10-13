#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include "Timer.h"

BlueMotor blueMotor;
long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;

int motorEffort = -1;
int bringDown = -150;
int bringUp = 250;
float encoderCount = 0.0;
float changed = 0.0;
float PIDEffort = 0.0;
float setDBCalculatedEffort = 0.0; 

float Fourty5DegreeEncoderCount = 3500.0;
float Twenty5DegreeEncoderCount = 7700.0;
float stagingBlock = 0.0;

// PID variables
const float Kp = 0.35; // at 0.4, oscillation occurs
const float Ki = 0.0000005;
const float Kd = 0.05;
bool firstTime;
boolean reached45 = false;

void setup()
{
  Serial.begin(9600);
  blueMotor.setup();
  blueMotor.reset();
  firstTime = true;
  delay(500);
}

/*
used to see encoder count as blueMotor spins with positive effort
*/
void positiveEffort() {

  //Take a screenshot of the effort when the encoder count starts to change.
  for(float motorEffort = 1.0; motorEffort < 255.0; motorEffort++) {
    delay(100);
    blueMotor.setEffort(motorEffort);
    encoderCount = blueMotor.getPosition();
    printf("Encoder Count:%f\t Effort:%f\n", encoderCount, motorEffort);
  }
}

/*
used to see encoder count as blueMotor spins with negative effort
*/
void negativeEffort() {
  //Take a screenshot of the effort when the encoder count starts to change.
  for(float motorEffort = -1.0; motorEffort > -255.0; motorEffort--) {
    delay(100);
    blueMotor.setEffort(motorEffort);
    encoderCount = blueMotor.getPosition();
    printf("Encoder Count:%f\t Effort:%f\n", encoderCount, motorEffort);
  }
}


/*
  function which calls seteffort which scales the given motor effort to adjusted motor effort in positive 
*/
void deadBandPositive() {
  
  //Take a screenshot of the effort when the encoder count starts to change.
  for(float motorEffort = 1.0; motorEffort < 255.0; motorEffort++) {
    now = millis();
    delay(100);
    // gives it to setEffortWithoutDB which calculuates new effort
    changed = blueMotor.setEffortWithoutDB(motorEffort);
    encoderCount = blueMotor.getPosition();
    speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
    oldPosition = blueMotor.getPosition();
    printf("Time:%ld\t Effort:%f\t Adjusted Effort:%f\t Encoder Count:%f\t Speed(RPM):%d\n", now, motorEffort, changed, encoderCount, speedInRPM);
  }

}

/*
  function which calls seteffort which scales the given motor effort to adjusted motor effort in negative
*/
void deadBandNegative() {

  for(float motorEffort = -1.0; motorEffort > -255.0; motorEffort--) {
    now = millis();
    delay(100);
    // gives it to setEffortWithoutDB which calculuates new effort
    changed = blueMotor.setEffortWithoutDB(motorEffort);
    encoderCount = blueMotor.getPosition();
    speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
    oldPosition = blueMotor.getPosition();
    printf("Time:%ld\t Effort:%f\t Adjusted Effort:%f\t Encoder Count:%f\t Speed(RPM):%d\n", now, motorEffort, changed, encoderCount, speedInRPM);
 }

}



float previousCount = 0.0;
float sumOfErrors = 0.0;

// timer which is set to 1/100th of a second or 10 milliseconds
Timer PIDTimer(10);
float output;

// PID Control Program
float evaluate(float encoderCount, float targetCount) {
  if(PIDTimer.isExpired()) {
    float error = targetCount - encoderCount;
    sumOfErrors += error;
    output = Kp * error - Kd * (encoderCount - previousCount) + Ki * sumOfErrors;
    printf("Current Encoder Count:%f\t Set Encoder Count:%f\t Output:%f\t", encoderCount, previousCount, output);
    previousCount = encoderCount;
  }
  return output;
}

// takes in the calculated effort from setEffortWithoutDB and prints argument
void printEverything(float argScaledEffort) {
    long now = millis();
    encoderCount = blueMotor.getPosition();
    speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
    oldPosition = blueMotor.getPosition();
    printf("Time:%ld\t Adjusted Effort:%d\n", now, argScaledEffort);
}

// used to find encoder counts as motor spins a certain way
void findEncoderCount() {
  blueMotor.setEffort(-100);
  long answer = blueMotor.getPosition();
  printf("count:%ld\n", answer);
}

// used to bring motor down to staging block after checking if it reached a certain position
void comeBackDown() {
  if((7500.0 < blueMotor.getPosition()) && (blueMotor.getPosition() < 8000.0)) {
    reached45 = true;
    delay(200);
  }
  if (reached45) {
    PIDEffort = evaluate(blueMotor.getPosition(), stagingBlock);
    setDBCalculatedEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    printEverything(setDBCalculatedEffort);
  }

}

void loop(){
  //blueMotor.setEffort(bringDown);
  // positiveEffort();
  // negativeEffort();
  // deadBandPositive();
  // deadBandNegative();
  // findEncoderCount();

  if(firstTime) {
    PIDEffort = evaluate(blueMotor.getPosition(), Twenty5DegreeEncoderCount);
    setDBCalculatedEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    firstTime = false;
  }
  comeBackDown();
  
  
  // takes in calculated effort and prints everything
  //printEverything(setDBCalculatedEffort);

}