#include "ros/ros.h"
#include "buddy_msg/buddy_music_note.h"
#include "buddy_msg/servo_control.h"
#include <unistd.h>

void send_note_msg(uint16_t toneFreq, uint16_t duration, ros::Publisher music_pub, ros::Rate loop_rate_mus) {

    buddy_msg::buddy_music_note musicMsg;
    musicMsg.frequency = toneFreq;
    musicMsg.duration = duration;

    if (ros::ok()) music_pub.publish(musicMsg);
    ros::spinOnce();
    loop_rate_mus.sleep();
  }

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Publisher buddy_music_pub = n.advertise<buddy_msg::buddy_music_note>("musicNote", 1000);
  ros::Publisher buddy_servo_pub = n.advertise<buddy_msg::servo_control>("servoControl", 1000);
  ros::Rate loop_rate(5);
  ros::Rate slow_rate(3.5);
  ros::Rate super_slow_rate(3);
  ros::Rate super_super_slow_rate(1.25);

  while (buddy_music_pub.getNumSubscribers() < 1 && ros::ok()) {

   usleep(1000000);
  }

  const uint16_t c = 261;
  const uint16_t d = 294;
  const uint16_t e = 329;
  const uint16_t f = 349;
  const uint16_t g = 391;
  const uint16_t gS = 415;
  const uint16_t a = 440;
  const uint16_t aS = 455;
  const uint16_t b = 466;
  const uint16_t cH = 523;
  const uint16_t cSH = 554;
  const uint16_t dH = 587;
  const uint16_t dSH = 622;
  const uint16_t eH = 659;
  const uint16_t fH = 698;
  const uint16_t fSH = 740;
  const uint16_t gH = 784;
  const uint16_t gSH = 830;
  const uint16_t aH = 880;


  char notes[] = "ccdcfe ccdcgf ccCHgfed aSaSafgf";

  buddy_msg::servo_control center;
  center.baseServoAngle = 90;
  center.nodServoAngle = 130;
  center.tiltServoAngle = 90;
  center.desiredDelay = 3;
  if (ros::ok()) buddy_servo_pub.publish(center);
  ros::spinOnce();
  usleep(250000);

  send_note_msg(c, 300, buddy_music_pub, loop_rate);
  send_note_msg(c, 150, buddy_music_pub, loop_rate);
  send_note_msg(d, 300, buddy_music_pub, loop_rate);
  send_note_msg(c, 300, buddy_music_pub, loop_rate);
  send_note_msg(f, 300, buddy_music_pub, loop_rate);
  send_note_msg(e, 300, buddy_music_pub, loop_rate); 

  usleep(2000000);
  buddy_msg::servo_control tilt1;
  tilt1.baseServoAngle = 90;
  tilt1.nodServoAngle = 130;
  tilt1.tiltServoAngle = 40;
  tilt1.desiredDelay = 3;

  buddy_msg::servo_control tilt2;
  tilt2.baseServoAngle = 90;
  tilt2.nodServoAngle = 130;
  tilt2.tiltServoAngle = 140;
  tilt2.desiredDelay = 3;


  for (int i =  0; i < 4; i++) {
    ROS_INFO("head tilt cycle %d", i);
    if (ros::ok()) buddy_servo_pub.publish(tilt1);
    ros::spinOnce();
    slow_rate.sleep();
    if (i == 3) {
      ROS_INFO("returning to center after head tilt cycle");
      if (ros::ok()) buddy_servo_pub.publish(center);
    }
    else {
      if (ros::ok()) buddy_servo_pub.publish(tilt2);
    }
    ros::spinOnce();
    slow_rate.sleep();
  }

  usleep(1500000);

  send_note_msg(c, 300, buddy_music_pub, loop_rate);
  send_note_msg(c, 150, buddy_music_pub, loop_rate);
  send_note_msg(d, 300, buddy_music_pub, loop_rate);
  send_note_msg(c, 300, buddy_music_pub, loop_rate);
  send_note_msg(g, 300, buddy_music_pub, loop_rate);
  send_note_msg(f, 300, buddy_music_pub, loop_rate);

  usleep(2000000);

  buddy_msg::servo_control topRightMsg;
  topRightMsg.baseServoAngle = 140;
  topRightMsg.nodServoAngle = 120;
  topRightMsg.tiltServoAngle = 90;
  topRightMsg.desiredDelay = 3;

  buddy_msg::servo_control middle1Msg;
  middle1Msg.baseServoAngle = 90;
  middle1Msg.nodServoAngle = 150;
  middle1Msg.tiltServoAngle = 140;
  middle1Msg.desiredDelay = 3;

  buddy_msg::servo_control topLeftMsg;
  topLeftMsg.baseServoAngle = 40;
  topLeftMsg.nodServoAngle = 120;
  topLeftMsg.tiltServoAngle = 90;
  topLeftMsg.desiredDelay = 3;

  buddy_msg::servo_control middle2Msg;
  middle2Msg.baseServoAngle = 90;
  middle2Msg.nodServoAngle = 150;
  middle2Msg.tiltServoAngle = 40;
  middle2Msg.desiredDelay = 3;


  for (int i = 0; i < 3; i++) {
    ROS_INFO("v dance cycle %d", i);
    if (ros::ok()) buddy_servo_pub.publish(topLeftMsg);
    ros::spinOnce();
    super_slow_rate.sleep();
    if (ros::ok()) buddy_servo_pub.publish(middle1Msg);
    ros::spinOnce();
    super_slow_rate.sleep();
    if (ros::ok()) buddy_servo_pub.publish(topRightMsg);
    ros::spinOnce();
    super_slow_rate.sleep();
    if (i == 2) {
      ROS_INFO("Returning to center after v dance"); 
      if (ros::ok()) buddy_servo_pub.publish(center);
    }
    else {
      if (ros::ok()) buddy_servo_pub.publish(middle2Msg);
    }
    ros::spinOnce();
    super_slow_rate.sleep();

  }
  usleep(1000000);

  send_note_msg(c, 300, buddy_music_pub, slow_rate);
  send_note_msg(c, 150, buddy_music_pub, slow_rate);
  send_note_msg(cH, 300, buddy_music_pub, slow_rate);
  // I think nano may be trying to store messages and running out of memory
  // or, c++ can't publish to arduino fast enough and stuff is getting dropped
  // with sleeps, it works, and slow publish rates
  usleep(250000);
  send_note_msg(a, 300, buddy_music_pub, slow_rate);
  send_note_msg(f, 300, buddy_music_pub, slow_rate);
  send_note_msg(e, 300, buddy_music_pub, slow_rate); 
  usleep(250000);
  send_note_msg(d, 300, buddy_music_pub, slow_rate);
  usleep(1000000);

  buddy_msg::servo_control bottomLeftMsg;
  bottomLeftMsg.baseServoAngle = 40;
  bottomLeftMsg.nodServoAngle = 150;
  bottomLeftMsg.tiltServoAngle = 40;
  bottomLeftMsg.desiredDelay = 4;

  buddy_msg::servo_control bottomRightMsg;
  bottomRightMsg.baseServoAngle = 140;
  bottomRightMsg.nodServoAngle = 150;
  bottomRightMsg.tiltServoAngle = 140;
  bottomRightMsg.desiredDelay = 4;

  for (int i = 0; i < 3; i++) {
    ROS_INFO("side dance cycle %d", i);
    if (ros::ok()) buddy_servo_pub.publish(bottomLeftMsg);
    ros::spinOnce();
    super_super_slow_rate.sleep();
    if (ros::ok()) buddy_servo_pub.publish(bottomRightMsg);
    ros::spinOnce();
    super_super_slow_rate.sleep();
    if (i == 2) {
      ROS_INFO("Returning to center after side dance");
      if (ros::ok()) buddy_servo_pub.publish(center);
      ros::spinOnce();
      super_super_slow_rate.sleep();
    }
  }
  usleep(700000);

  send_note_msg(aS, 150, buddy_music_pub, slow_rate);
  send_note_msg(aS, 150, buddy_music_pub, slow_rate);
  usleep(250000);
  send_note_msg(a, 300, buddy_music_pub, slow_rate);
  usleep(250000);
  send_note_msg(f, 300, buddy_music_pub, slow_rate);
  usleep(250000);
  send_note_msg(g, 300, buddy_music_pub, slow_rate);
  send_note_msg(f, 300, buddy_music_pub, slow_rate); 
  usleep(5000000);

  return 0;
}

