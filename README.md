## Buddy Robot Dance with ROS
This was a fun project to get a robot to dance using ROS. I used a robot from Slant Concepts and recoded it to work with ROS, via rosserial and a bluetooth connection.    
![buddy_dance](screenshots/buddy_dance.gif)

rosrun rosserial_python serial_node.py /dev/ttyS0 _baud:=9600

from tutorial, make sure have sourced /devel/setup.bash and that rosmsg show servo_tone_msg is working
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .


Subscriber:
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

http://wiki.ros.org/rosserial_client/Tutorials/Generating%20Message%20Header%20Files custom message gen arduino

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv ros msg creation

mz@mz-VirtualBox:~$ rosmsg show buddy_msg/buddy_control
uint8 baseServoAngle
uint8 nodServoAngle
uint8 tiltServoAngle
uint8 desiredDelay
uint8 wordCount


mz@mz-VirtualBox:~$ rostopic pub -l /buddyControl buddy_msg/buddy_control -- 90 110 110 3 3
publishing and latching message. Press ctrl-C to terminate

