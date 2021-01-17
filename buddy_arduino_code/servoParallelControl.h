#include <Servo.h> 

#include "droidSpeak.h"

// Servos
Servo baseServo;
Servo nodServo;
Servo tiltServo;

// Robot Joint Motion Stuctures
struct headPos {
  int baseServoAngle;
  int nodServoAngle ;
  int tiltServoAngle ;
  int desiredDelay ;
};

struct headPos faceMotion;  // Joint Positions of Head


int servoParallelControl (int thePos, Servo theServo, int theSpeed ) {

  int startPos = theServo.read();        //read the current pos
  int newPos = startPos;
  //int theSpeed = speed;

  //define where the pos is with respect to the command
  // if the current position is less that the actual move up
  if (startPos < (thePos - 5)) {
    newPos = newPos + 1;
    theServo.write(newPos);
    delay(theSpeed);
    return 0;
  }

  else if (newPos > (thePos + 5)) {
    newPos = newPos - 1;
    theServo.write(newPos);
    delay(theSpeed);
    return 0;
  }

  else {
    return 1;
  }

}

void moveTo(struct headPos faceMotion) {

  int status1 = 0;
  int status2 = 0;
  int status3 = 0;
  int done = 0 ;

  while ( done == 0) {
    //move all servos to the desired position
    //this loop will cycle through the servos sending each the desired position.
    //Each call will cause the servo to iterate about 1-5 degrees
    //the rapid cycle of the loop makes the servos appear to move simultaneously
    status1 = servoParallelControl(faceMotion.baseServoAngle, baseServo, faceMotion.desiredDelay);
    status2 = servoParallelControl(faceMotion.nodServoAngle, nodServo, faceMotion.desiredDelay);
    status3 = servoParallelControl(faceMotion.tiltServoAngle, tiltServo, faceMotion.desiredDelay);

    //continue until all have reached the desired position
    if (status1 == 1 & status2 == 1 & status3 == 1 ) {
      done = 1;
    }
  }// end of while
} //function end

void buddyControlCB(const buddy_msg::buddy_control& buddyControlMsg){

  if (buddyControlMsg.wordCount != 0) droidSpeak(buzzerPin, buddyControlMsg.wordCount);

  if (buddyControlMsg.baseServoAngle == 0 && buddyControlMsg.tiltServoAngle==0 && buddyControlMsg.nodServoAngle == 0) return;

  faceMotion.baseServoAngle = constrain(buddyControlMsg.baseServoAngle, 10, 170);
  faceMotion.tiltServoAngle = constrain(buddyControlMsg.tiltServoAngle, 20, 150);
  faceMotion.nodServoAngle = constrain(buddyControlMsg.nodServoAngle, 80, 150);
  
  faceMotion.desiredDelay = buddyControlMsg.desiredDelay;
  moveTo(faceMotion);
}
