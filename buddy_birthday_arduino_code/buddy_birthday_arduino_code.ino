#include <ros.h>
#include <ros/time.h>
#include <buddy_msg/buddy_music_note.h>
#include <buddy_msg/servo_control.h>

// import order here matters, as some of the imports rely on others
ros::NodeHandle  nh;
#include "pinMapping.h"
#include "servoParallelControl.h"
#include "droidSing.h"

// this call back is in servoParallelControl.h file
ros::Subscriber<buddy_msg::servo_control> subServo("servoControl", servoControlCB);

// this call back is in droidSing.h
ros::Subscriber<buddy_msg::buddy_music_note> subMusic("musicNote", singCB); 


void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(subServo);
  nh.subscribe(subMusic);
  
  baseServo.attach(baseServoPin);
  nodServo.attach(nodServoPin);
  tiltServo.attach(tiltServoPin);

  baseServo.write(90);        //intial positions of servos
  nodServo.write(90);
  tiltServo.write(90);
  
  pinMode(buzzerPin, OUTPUT);

  noTone(buzzerPin);
  delay(200);
  faceMotion.desiredDelay = 3;
  faceMotion.baseServoAngle = baseServo.read();
  faceMotion.nodServoAngle = 130;
  faceMotion.tiltServoAngle = tiltServo.read();
  for (int i = 0; i < 4; i++) {

    faceMotion.tiltServoAngle = 90 + 50;
    moveTo(faceMotion);
    faceMotion.tiltServoAngle = 90 - 50;
    moveTo(faceMotion);
  }

  faceMotion.tiltServoAngle = 90;
  moveTo(faceMotion);
}


void loop()
{
  
  nh.spinOnce();
}
