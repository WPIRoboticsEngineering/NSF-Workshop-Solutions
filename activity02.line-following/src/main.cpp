/*
 * Activity 02 -- Staying on track
 *
 * Line following with speed control. Pauses at an intersection and waits for a turn command.
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

uint16_t darkThreshold = 500;
float speed = 10;

// Declare a chassis object with nominal dimensions
// TODO: Adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(7.0, 1440, 14.9);

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

// Define the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINE_FOLLOWING};
ROBOT_STATE robotState = ROBOT_IDLE;

// A helper function to stop the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  //stop motors 
  chassis.idle();

  //set state to idle
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
  Serial.println("beginLineFollowing()");
  setLED(HIGH);
  robotState = ROBOT_LINE_FOLLOWING;
}

// Used to check if the motions above are complete
void handleMotionComplete(void)
{
  idle();
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  //ENTER_SAVE idles, regardless of state -- E-stop
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == UP_ARROW) drive(50, 10);
      else if(keyPress == DOWN_ARROW) drive(-50, 10);
      else if(keyPress == LEFT_ARROW) turn(90, 45);
      else if(keyPress == RIGHT_ARROW) turn(-90, 45);
      else if(keyPress == SETUP_BTN) beginLineFollowing();
      break;
      
    case ROBOT_LINE_FOLLOWING:
      if(keyPress == VOLplus)  //VOL+ increases speed
      {
        speed += 5;
      }

      if(keyPress == VOLminus)  //VOL- decreases speed
      {
        speed -= 5;
      }
      break;
 
     default:
      break;
  }
}

// TODO: Add line following handler
void handleLineFollowing(float baseSpeed)
{
  const float Kp = 0.1;

  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
  
  int16_t error = leftADC - rightADC;
  float turnEffort = Kp * error;
  
  chassis.setTwist(baseSpeed, turnEffort);
}

// TODO: Add intersection checker
// Here's a nice opportunity to introduce boolean logic
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

// TODO: handle intersection
void handleIntersection(void)
{
  Serial.println("Intersection!");

  //drive forward by dead reckoning to center the robot
  chassis.driveFor(8, 5);

  robotState = ROBOT_DRIVE_FOR;
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  //these can be undone for the student to adjust
  chassis.setMotorPIDcoeffs(5, 0.5);

  // initialize the IR decoder
  decoder.init();

  // Ensure the line sesnors are inputs
  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);

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
      handleLineFollowing(speed); //argument is base speed
      if(checkIntersectionEvent(darkThreshold)) handleIntersection();
      break;

    default:
      break;
  }
}
