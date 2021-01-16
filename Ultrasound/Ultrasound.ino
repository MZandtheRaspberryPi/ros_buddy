#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <buddy_msg/servo_tone_msg.h>

#include "droidSpeak.h"

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

#define buzzerPin 10        // Pin for the buzzer labeled as S9 on the board
buddy_msg::servo_tone_msg buddy_control_msg;
void buddyControlCB(const buddy_msg::servo_tone_msg& buddy_control_msg){
  droidSpeak(buzzerPin, buddy_control_msg.wordCount);
}

ros::Subscriber<buddy_msg::servo_tone_msg> sub("/buddy_control", buddyControlCB);

char frameid[] = "/ultrasound";


//+++++++++++++++ULTRASONIC VARIABLES+++++++++++++++++++++++++++
#define trigPin A3          // Trigger Pin
#define echoPin A2          // Echo Pin
float readDistance;          // the output distance from the sensor
//This function pings the Ultrasonic Sensor and returns a distance in CM

float checkUltra(int theEchoPin, int theTrigPin) {
  //this fucntion caluclates and returns the distance in cm

  float duration, distance; // Duration used to calculate distance
  /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(theTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(theTrigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(theTrigPin, LOW);
  duration = pulseIn(theEchoPin, HIGH);

  //Calculate the distance (in cm) based on the speed of sound.
  distance = (duration * .0343)/2;
  return distance;    

}



//// Libraries
//#include <Servo.h>  //arduino library
//#include <math.h>   //standard c library
//
//#include "servoParallelControl.h"
//
//// Servos
//Servo baseServo;
//Servo nodServo;
//Servo tiltServo;
//
//// Robot Joint Motion Stuctures
//struct headPos {
//  int baseServoAngle;
//  int nodServoAngle ;
//  int tiltServoAngle ;
//  int desiredDelay ;
//};
//
//struct headPos faceMotion;  // Joint Positions of Head
//void moveTo( struct headPos faceMotion); 



void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub);
  
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 2.0;
  range_msg.max_range = 400.0;
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  noTone(buzzerPin);
}


long range_time;

void loop()
{
  
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = checkUltra(echoPin, trigPin);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  
  nh.spinOnce();
}

//void moveTo( struct headPos faceMotion) {
//
//  int status1 = 0;
//  int status2 = 0;
//  int status3 = 0;
//  int done = 0 ;
//
//  while ( done == 0) {
//    //move all servos to the desired position
//    //this loop will cycle through the servos sending each the desired position.
//    //Each call will cause the servo to iterate about 1-5 degrees
//    //the rapid cycle of the loop makes the servos appear to move simultaneously
//    status1 = servoParallelControl(faceMotion.baseServoAngle, baseServo, faceMotion.desiredDelay);
//    status2 = servoParallelControl(faceMotion.nodServoAngle, nodServo, faceMotion.desiredDelay);
//    status3 = servoParallelControl(faceMotion.tiltServoAngle, tiltServo, faceMotion.desiredDelay);
//
//    //continue until all have reached the desired position
//    if (status1 == 1 & status2 == 1 & status3 == 1 ) {
//      done = 1;
//    }
//  }// end of while
//} //function end
