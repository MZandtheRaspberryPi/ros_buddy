#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

// import order here matters, as some of the imports rely on others
#include "pinMapping.h"
#include "ultrasonicSensor.h"
#include "servoParallelControl.h"

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);
char frameid[] = "/ultrasound";

// variable to keep track of when to fire ultrasound next
long range_time;

struct headPos faceMotion;  // Joint Positions of Head

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 2.0;
  range_msg.max_range = 400.0;

  baseServo.attach(baseServoPin);
  nodServo.attach(nodServoPin);
  tiltServo.attach(tiltServoPin);

  baseServo.write(90);        //intial positions of servos
  nodServo.write(90);
  tiltServo.write(90);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  noTone(buzzerPin);
  delay(200);
  droidSpeak(buzzerPin, 5);
  faceMotion.desiredDelay = 3;
  faceMotion.baseServoAngle = 100;
  faceMotion.nodServoAngle = 140;
  faceMotion.tiltServoAngle = 90;
  for (int i = 0; i < 2; i++) {

    faceMotion.tiltServoAngle = 90 + 50;
    moveTo(faceMotion);
    faceMotion.tiltServoAngle = 90 - 50;
    moveTo(faceMotion);
  }

  faceMotion.tiltServoAngle = 90;
  moveTo(faceMotion);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
}




void loop()
{
  
  //publish the adc value every 100 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = checkUltra(echoPin, trigPin);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 100;
  }

  if (range_msg.range < 30) {
    droidSpeak(buzzerPin, random(2, 10));
    for (int i = 0; i < 2; i++) {

      int priorTiltAngle = faceMotion.tiltServoAngle;
      faceMotion.tiltServoAngle = priorTiltAngle + 50;
      moveTo(faceMotion);
      faceMotion.tiltServoAngle = priorTiltAngle - 50;
      moveTo(faceMotion);
    }
    randomMove(faceMotion);
  }
  
  nh.spinOnce();


}
