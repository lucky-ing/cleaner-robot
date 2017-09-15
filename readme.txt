this is a cleaner robot code :v=1.0
using the camera-output 

OneStep:  we should open a terminal to run roscore first

	Command :roscore

(it will remind you if the roscore had start ready)
TwoStep:  we use the serial to connet the cleaner-robot, so we should open a new terminal and open the serial port next:

	Command: sudo chmod 777 /dev/ttyUSB0

then it will ask your password of root, please entry your password
(please remember that the /dev/ttyUSB0 is your serial port that your cleaner-robot had connetted, it is not same always ,but must contain 'ttyUSB'. it can be /dev/ttyUSB1.your use :'ls /dev/' to check your serial port id.)
TreeStep:  please open an other terminal,entry the command to start your rivz to show your map (this step is not necessary if you just to test your running-algorithm without map-show)

	Command: roslaunch turtlebot_rviz_launchers view_navigation.launch

(please be care of this cammand, it need your computer has install turtelbot, otherwise you cannot run the command well)
FourStep:  you should goto your main file of your robot, and open a new terminal, then run your main python file

	Command: python 123.py

after about 1s, your main program will start. you should select your serial port, and click the 'openserial' button to open the serial port. without any error, you can click the 'start' button to start your all program.till 'ok' was showed ,your program can work wll.

