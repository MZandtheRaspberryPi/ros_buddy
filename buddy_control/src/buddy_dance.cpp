#include "ros/ros.h"
#include "buddy_msg/buddy_control.h"
#include <unistd.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher buddy_pub = n.advertise<buddy_msg::buddy_control>("buddyControl", 1000);

  ros::Rate loop_rate(10);

  usleep(1000000);

  // dance message 1 is to go to center, whistle then nod twice

  buddy_msg::buddy_control msg1;
  msg1.wordCount = 0;
  msg1.baseServoAngle = 90;
  msg1.nodServoAngle = 90;
  msg1.tiltServoAngle = 90;
  msg1.desiredDelay = 2;

  buddy_msg::buddy_control msg2;
  msg2.wordCount = 10;
  msg2.baseServoAngle = 0;
  msg2.nodServoAngle = 0;
  msg2.tiltServoAngle = 0;
  msg2.desiredDelay = 2;

  buddy_msg::buddy_control msg3;
  msg3.wordCount = 0;
  msg3.baseServoAngle = 90;
  msg3.nodServoAngle = 130;
  msg3.tiltServoAngle = 90;
  msg3.desiredDelay = 2;

  buddy_msg::buddy_control dance_msgs[] = {msg1, msg2, msg3};
  size_t msg_count = sizeof(dance_msgs)/sizeof(dance_msgs[0]);

  ROS_INFO("%zd Dance Messages to Send", msg_count);

  for (int i = 0; i < msg_count; i++) {
    if (not ros::ok()) break;
    ROS_INFO("Sending dance message %d", i);
    buddy_pub.publish(dance_msgs[i]);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
