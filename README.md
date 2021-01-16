rosrun rosserial_python serial_node.py /dev/ttyS0 _baud:=9600

from tutorial, make sure have sourced /devel/setup.bash and that rosmsg show servo_tone_msg is working
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
