// 3/21/2023
// Gilbert Traczyk & Moosa Azfar
// Lab Assignment #7 – Object Mapping – Version 1
// robot maps a circlular arena then drives between 2 objects to exit the arena

#include <Servo.h>
#include "SimpleRSLK.h"

#define PULSES_CM 16
#define PULSES_DEG 2

#define WHEEL_SPEED_LEFT_BASE 15
#define WHEEL_SPEED_RIGHT_BASE 15 
#define TURN_SPEED 5

#define RIGHT 0                 // Macro to hold value to signal a right turn
#define LEFT 1                  // Macro to hold value to signal a left turn

#define CYLINDER_RADIUS_CM 91.44

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor   

bool buttonPressed = false;

float arenaMap[8];
int temp[] = {4, 3, 2, 1};

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);   //the trigger pin will output pulses of electricity 
  //the echo pin will measure the duration of pulses coming back from the distance sensor
  pinMode(echoPin, INPUT);
  // Setup RSLK 
  setupRSLK();

  // Set left button as wait button
  setupWaitBtn(LP_LEFT_BTN);

  // Setup red LED
  setupLed(RED_LED);
  setupLed(GREEN_LED);
  
  myservo.attach(38);   // attaches the servo on Port 2.4 (P2.4 or pin 38)to the servo object
  myservo.write(0);     // Send it to the default position
  delay(3000);           // 3 second delay to allow servo to return to start position of 0 before sweep begins
}

void loop() {

  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function takes in an input in centimeters (distanceCm) and instructs the robot to drive straight for distanceCm centimeters
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void driveStraightCm(int distanceCm) {
    // Initialize wheel speeds to macros declared above
    int wheelSpeedLeft = WHEEL_SPEED_LEFT_BASE;
    int wheelSpeedRight = WHEEL_SPEED_RIGHT_BASE;

    // Initialize left and right encoder pulse counts to zero
    int leftPulseCount = 0;
    int rightPulseCount = 0;

    // Convert input distance of centimeters to encoder pulses
    int encoderDist = distanceCm * PULSES_CM; 

    // Reset left and right encoders, then delay for 1 second
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
    
    delay(1000);

    // Set both motor directions to forward
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    
    // Enable both motors
    enableMotor(BOTH_MOTORS);
    
    // Set left motor speed to wheelSpeedLeft
    setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft);
    
    // Set right motor speed to wheelSpeedRight
    setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);

    // Drive straight while left and right encoder values are less than total encoder distance
    while((leftPulseCount < encoderDist) && (rightPulseCount < encoderDist)) {
      // Update left encoder count
      leftPulseCount = getEncoderLeftCnt();

      // Update right encoder count
      rightPulseCount = getEncoderRightCnt(); 

      // Increase right wheel speed if it is lagging
      if(leftPulseCount > rightPulseCount + 8) {
        wheelSpeedRight += 1;
        
        setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);  
        setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft); 
      }

      // Increase left wheel speed if it is lagging
      if(rightPulseCount > leftPulseCount + 4) {
        wheelSpeedLeft += 1;
        
        setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft); 
        setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);  
      }

      // Reset both wheel speeds to base if it starts speeding out of control. The speeds are reset if they are 20% greater than the base value.
      if((wheelSpeedLeft > WHEEL_SPEED_LEFT_BASE * 1.2) || (wheelSpeedRight > WHEEL_SPEED_RIGHT_BASE * 1.2)) {
          wheelSpeedLeft = WHEEL_SPEED_LEFT_BASE;
          wheelSpeedRight = WHEEL_SPEED_RIGHT_BASE;

          setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft); 
          setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);  
      }

      // Wait for 0.25 seconds to let the system stablize
      delay(250);

      // Update left and right pulse counts before re-entering the while loop
      leftPulseCount = getEncoderLeftCnt();
      rightPulseCount = getEncoderRightCnt(); 
    }

    // Stop robot after it has traversed the requested distance
    setMotorSpeed(LEFT_MOTOR, 0); 
    setMotorSpeed(RIGHT_MOTOR, 0);  
    //disableMotor(BOTH_MOTORS);
}

float getDistance() {
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistanceInches;         //variable to store the distance calculated from the echo time
  float calculatedDistanceCentimeters;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, LOW); //ensures a clean pulse beforehand
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor in microseconds
  
  //calculate the distance in inches of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceInches = echoTime / 148.0;
  //calculate the distance in centimeters of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceCentimeters = echoTime / 58.0;  

  return calculatedDistanceCentimeters;              //send back the distance that was calculated
}

void turnInPlace(int speed, int direction, int degree) {
  int wheelSpeedLeft = speed;
  int wheelSpeedRight = speed;

  // Initialize left and right encoder pulse counts to zero
  int leftPulseCount = 0;
  int rightPulseCount = 0;

  // Set encoder distance to one full wheel rotation ~ 180 degree
  float encoderDist = PULSES_DEG * degree;
  Serial.print(encoderDist);

  // Reset left and right encoders, then delay for 1 second
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  delay(1000);

  // Set motors in opposite directions based on turn direction
  if (direction == RIGHT)
  {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  }
  else if (direction == LEFT)
  {
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  }

  // Enable both motors
  enableMotor(BOTH_MOTORS);

  // Set left motor speed to wheelSpeedLeft
  setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft);

  // Set right motor speed to wheelSpeedRight
  setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);

  while ((leftPulseCount < encoderDist) && (rightPulseCount < encoderDist))
  {
    // Update left encoder count
    leftPulseCount = getEncoderLeftCnt();

    // Update right encoder count
    rightPulseCount = getEncoderRightCnt();

    // Increase right wheel speed if it is lagging
    if (leftPulseCount > rightPulseCount + 8)
    {
      wheelSpeedRight += 1;

      setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);
      setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft);
    }

    // Increase left wheel speed if it is lagging
    if (rightPulseCount > leftPulseCount + 4)
    {
      wheelSpeedLeft += 1;

      setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft);
      setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);
    }

    // Reset both wheel speeds to base if it starts speeding out of control.
    // The speeds are reset if they are 20% greater than the base value.
    if ((wheelSpeedLeft > WHEEL_SPEED_LEFT_BASE * 1.2) || (wheelSpeedRight > WHEEL_SPEED_RIGHT_BASE * 1.2))
    {
      wheelSpeedLeft = WHEEL_SPEED_LEFT_BASE;
      wheelSpeedRight = WHEEL_SPEED_RIGHT_BASE;

      setMotorSpeed(LEFT_MOTOR, wheelSpeedLeft);
      setMotorSpeed(RIGHT_MOTOR, wheelSpeedRight);
    }

    // Wait for 0.09 seconds to let the system stablize
    delay(90);

    // Update left and right pulse counts before re-entering the while loop
    leftPulseCount = getEncoderLeftCnt();
    rightPulseCount = getEncoderRightCnt();
  }

  // Stop robot after it has traversed the requested distance
  disableMotor(BOTH_MOTORS);
}
