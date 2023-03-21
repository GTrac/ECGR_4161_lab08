// 3/21/2023
// Gilbert Traczyk & Moosa Azfar
// Lab Assignment #7 – Object Mapping – Version 1
// robot maps a circlular arena then drives between 2 objects to exit the arena

#include "SimpleRSLK.h"

#define PULSES_CM 16
#define PULSES_DEG 2

#define WHEEL_SPEED_LEFT_BASE 15
#define WHEEL_SPEED_RIGHT_BASE 15 
#define TURN_SPEED 5

#define RIGHT 0                 // Macro to hold value to signal a right turn
#define LEFT 1                  // Macro to hold value to signal a left turn

#define CYLINDER_RADIUS_CM 91.44
#define FILTER_LENGTH 3

// Map arena by turning 5 degree increments, turning on LED if object is detected
// Determine indicies in arenaMap[] that are close to radius
// Find edges of indicies
// Take average of edges to determine escape angle
// Turn escape angle, drive straight

const float MAP_ANGLE = 18;
const int MAP_SEGMENTS = 360 / MAP_ANGLE;

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor   

bool buttonPressed = false;

float arenaMap[MAP_SEGMENTS];
float filterArray[FILTER_LENGTH];
int objectDetections = 0;

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
}

void loop() {

  escapePlan();
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

/////////////////////////////////////////////////////////////////////////////
// function turns in a full circle and records distance values as it does
// this builds a map of objects within the arena
/////////////////////////////////////////////////////////////////////////////
void mapArena() {
  //for a full rotation of a circle
  for(int i = 0; i < MAP_SEGMENTS; i++) {
    //turn in place 1 fraction of the full circle
    turnInPlace(TURN_SPEED, LEFT, MAP_ANGLE);

    delay(50);
    //record the distance in front of robot
    arenaMap[i] = getDistance(); 
    delay(50);

    //if object is detected turn on blue LED    
    if(arenaMap[i] < CYLINDER_RADIUS_CM) {
      digitalWrite(BLUE_LED, HIGH);
    } else {//else leave led off
      digitalWrite(BLUE_LED, LOW);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////
//Function uses the arenaMap data to calculate the robots escape path
//then it turns to match this path and drives forward 3.5 feet to escape the arena
//////////////////////////////////////////////////////////////////////////////////
void leaveArena() {
  int j = 0;

  //checks if values read from the arena are less than the limit
  //signifying a detected object
  for(int i = 0; i < MAP_SEGMENTS; i++) {
    if((arenaMap[i] <= CYLINDER_RADIUS_CM + 5)) {
       objectDetections++;//total number of detections
    }  
  }

  //records the locations of the objects detected
  int objectLocations[objectDetections];
  for(int i = 0; i < MAP_SEGMENTS; i++) {
    if((arenaMap[i] <= CYLINDER_RADIUS_CM + 5)) {
       objectLocations[j] = i;//where the object is detected
       j++;
    }  
  }

  //uses the location of the objects to calculate the mid point between them
  float median;
  if(objectDetections % 2 == 0) {
    int mid1 = (objectDetections / 2) - 1;
    int mid2 = mid1 + 1;

    median = (objectLocations[mid1] + objectLocations[mid2])/2;
  } else {
    median = objectLocations[objectDetections / 2];
  }

  //calculates how far to turn based on the midpoint location
  float turnAngle = ((median + 1) * MAP_ANGLE)-2;
  //changes turnspeed based on how far it has to turn
  int turnSpeed = WHEEL_SPEED_LEFT_BASE - ((1-((turnAngle+200)/(360+200)))*WHEEL_SPEED_LEFT_BASE);

  delay(500);
  //final turn to calculated escape path
  turnInPlace(turnSpeed, LEFT, (turnAngle)); 
  //final turn is adjusted by a factor of the total turn to account for drift from missed encoder ticks.

  //blinks LED green
  digitalWrite(GREEN_LED, HIGH);
  delay(2000);
  digitalWrite(GREEN_LED, LOW);

  //drives straight to leave the arena
  driveStraightCm(CYLINDER_RADIUS_CM + 15);
}

//////////////////////////////////////////////////////////////////////////////////////
// Function waits for button to be pressed then maps the arena, finds the two objects
// then drives between the objects to exit the arena
////////////////////////////////////////////////////////////////////////////////////////
void escapePlan() {
  if((digitalRead(LP_LEFT_BTN) == 0) && buttonPressed == false) {
    mapArena();//robot turns in circle and measures distances
    delay(4000);//waits 4 seconds
    
    leaveArena();//robot calculates escape path and then leaves
    
    buttonPressed = true;//stops repeat of function
  }
}


