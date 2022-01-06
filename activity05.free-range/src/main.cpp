/*
 * Activity 04 -- Where are we headed?
 *
 * Make a delivery using an internal map of the arena.
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

// Include Servo32u4 library
#include <Servo32u4.h>

// Include rangefinder library
#include <Rangefinder.h>

// Include navigator
#include "delivery.h"

// Create delivery object
Delivery delivery;

uint16_t darkThreshold = 500;
float baseSpeed = 10;

// Declare a chassis object with nominal dimensions
// TODO (optional): Adjust the parameters: wheel diam, encoder counts, wheel track
// to what you found in previous activities
Chassis chassis(7.0, 1440, 14.9);

// Declare a servo object
// Due to library constraints, servo MUST be connected to pin 5
Servo32U4 servo;

#define SERVO_UP 2300
#define SERVO_DOWN 500

// TODO, Section 5.1: Define the addtional servo positions for each of the platforms

// Declare rangefinder object
Rangefinder rangefinder(11, 4);

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// TODO, Section 5.1: Add dropping state
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINE_FOLLOWING, ROBOT_BAGGING, ROBOT_FREE_RANGE};
ROBOT_STATE robotState = ROBOT_IDLE;

//Action handleIntersection(Delivery& del);
void handleIntersection(void);

// A helper function to stop the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  // Stop the motors 
  chassis.idle();

  // Reset the destinations to NONE
  delivery.deliveryDest = NONE;
  delivery.currDest = NONE;

  // Set state to idle
  robotState = ROBOT_IDLE;
}

// A helper command to drive a set distance
void drive(float dist, float speed)
{
  Serial.println("drive()");
  setLED(HIGH);
  chassis.driveFor(dist, speed);
  robotState = ROBOT_DRIVE_FOR;
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  Serial.println("turn()");
  setLED(HIGH);
  chassis.turnFor(ang, speed);
  robotState = ROBOT_DRIVE_FOR;
}

void beginLineFollowing(void)
{
  setLED(HIGH);
  robotState = ROBOT_LINE_FOLLOWING;
}

// Updated handleMotionComplete, which returns to line following after a motion (e.g., turning) is complete
void handleMotionComplete(void)
{
  if(delivery.deliveryDest != NONE) beginLineFollowing();
  else idle();
}

// Function to begin bagging
void beginBagging(void)
{
  robotState = ROBOT_BAGGING;
  baseSpeed = 5;
}

void beginDropping(void)
{
  // TODO, Section 5.1: Edit function to begin dropping sequence
  idle(); // Don't forget to remove idle()
}

// Function to detect if bag is close enough
bool checkBagEvent(uint16_t threshold)
{
  static uint16_t prevDistance = 99;

  bool retVal = false;

  uint16_t currDistance = rangefinder.getDistance();

  if(prevDistance > threshold && currDistance <= threshold) retVal = true;
  prevDistance = currDistance;

  return retVal;
}

// Function to pick up bag
void pickupBag(void)
{
  Serial.print("Bagging...");

  servo.writeMicroseconds(SERVO_DOWN);

  chassis.driveFor(8, 2);
  while(!chassis.checkMotionComplete()) {delay(1);} //blocking
  Serial.println("done!");
  servo.writeMicroseconds(SERVO_UP);

  delivery.currDest = delivery.deliveryDest;

  turn(180, 45); //do a u-turn
}

// TODO, Section 5.1: Complete dropOffBag() function
void dropOffBag(void)
{
    Serial.print("Dropping...");

    // TODO, Section 5.2: reset destination to start
 }

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // ENTER_SAVE idles, regardless of state -- E-stop
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == UP_ARROW) drive(50, 10);
      else if(keyPress == DOWN_ARROW) drive(-50, 10);
      else if(keyPress == LEFT_ARROW) turn(90, 45);
      else if(keyPress == RIGHT_ARROW) turn(-90, 45);
      else if(keyPress == SETUP_BTN) beginLineFollowing();
      else if(keyPress == REWIND) beginBagging(); // Rewind is used to test bagging

      // TODO, Section 4.1: Handle house B
      else if(keyPress == NUM_2) 
      {
        delivery.currDest = PICKUP;
        delivery.deliveryDest = HOUSE_B;
        beginLineFollowing();
      }

      break;
      
    case ROBOT_LINE_FOLLOWING:
      if(keyPress == VOLplus)  //VOL+ increases speed
      {
        baseSpeed += 5;
      }

      if(keyPress == VOLminus)  //VOL- decreases speed
      {
        baseSpeed -= 5;
      }
      break;
 
     default:
      break;
  }
}

void handleLineFollowing(float speed)
{
  const float Kp = 0.1;

  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
  
  int16_t error = leftADC - rightADC;
  float turnEffort = Kp * error;
  
  chassis.setTwist(speed, turnEffort);
}

// Checks for the _event_ of reaching an intersection -- opportunity to introduce boolean logic
bool checkIntersectionEvent(int16_t darkThreshold)
{
  static bool prevIntersection = false;

  bool retVal = false;

  bool leftDetect = analogRead(LEFT_LINE_SENSE) > darkThreshold ? true : false;
  bool rightDetect = analogRead(RIGHT_LINE_SENSE) > darkThreshold ? true : false;

  bool intersection = leftDetect && rightDetect;
  if(intersection && !prevIntersection) retVal = true;
  prevIntersection = intersection;

  return retVal;
}

void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  // TODO (optional): Update PID coefficients to values in previous activity
  chassis.setMotorPIDcoeffs(5, 0.5);

  // Attach the servo
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  servo.writeMicroseconds(SERVO_UP); 

  // Initialize rangefinder
  rangefinder.init();

  // Initialize the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
       if(chassis.checkMotionComplete()) handleMotionComplete(); 
       break;

    case ROBOT_LINE_FOLLOWING:
      handleLineFollowing(baseSpeed); //argument is base speed
      if(checkIntersectionEvent(darkThreshold)) handleIntersection();
      break;

    // Handle bagging state
    case ROBOT_BAGGING:
      handleLineFollowing(baseSpeed); //crawl towards bag
      if(checkBagEvent(8)) {pickupBag();}
      break;

    // TODO, Section 5.1: Manage dropping state


    case ROBOT_FREE_RANGE:
      float distance = rangefinder.getDistance();
      Serial.println(distance);

      if(distance < 40) 
      {
        turn(20, 15);
        while(!chassis.checkMotionComplete()) {}
        drive(distance - 14, 5);
        while(!chassis.checkMotionComplete()) {}
        pickupBag();
      }

      break;

    default:
      break;
  }
}

void beginFreeRanging(void)
{
  Serial.println("Searching for the free range bag!");
  turn(180, 15);
  robotState = ROBOT_FREE_RANGE;
}

/**
 * handleIntersection() is called when the robot reaches an intersection. 
 * We've coded up a map of the arena -- every time we get to an intersection, 
 * we can decide what we want to do next based on our current location and 
 * destination. 
 * */
void handleIntersection(void)
{
    Serial.println("Intersection!");

    // Drive forward by dead reckoning to center the robot
    chassis.driveFor(8, 5);

    // We'll block for this one to reduce the complexity
    while(!chassis.checkMotionComplete()) {}

    Serial.println("Cleared");

    Serial.print("intersection: ");
    Serial.print(delivery.currLocation);
    Serial.print('\t');
    Serial.print(delivery.currDest);
    Serial.print('\t');
 
    switch(delivery.currLocation)
    {
        case ROAD_MAIN:
            if(delivery.currDest == PICKUP)
            {
                delivery.currLocation = ROAD_PICKUP;
                float distance = rangefinder.getDistance();
                Serial.println(distance);
                if(distance < 30) beginBagging();

                else 
                {
                  beginFreeRanging();
                }
            }

            //TODO, Section, 4.1: Handle all other conditions with else

            break;

        // TODO, Section 4.1: Add case to handle ROAD_PICKUP

        // TODO, Section 4.1: Add case for ROAD_ABC

        // TODO, Section 5.2: handle ROAD_B to get back to start

        default: 
          Serial.println("Unhandled case!");
          idle();
    }
}

