# WRITING_ROBOT
Toshiba robot used for writing or drawing on a piece of paper. The end effector of the robot holds a pen which is used to write or draw any shape that is kept underneath a Kinect2 camera. The image taken by the kinect2 camera is passed on to several nodes for image processing. The coordinate of the pixel is then saved onto a text file which is given into the robotic controller by using the socket communication ros package.

# DEPENDENCIES

• ROS indigo on a computer with Ubuntu 14.04 

http://wiki.ros.org/indigo/Installation/Ubuntu

• Kinect V2 sensor ROS package

https://github.com/code-iai/iai_kinect2


• Opencv 2.4.13 Library

# Procedure to run the package

• First place a A4 sized white paper on the underneath the Kinect V2 camera
• Open a terminal in ubuntu and type "cd catkin ws"
• Source the catkin workspace by typing "source devel/setup.bash"
• After sourcing the workspace the roslaunch command is used to run the launch file. To run the launch file type " roslaunch recognition and delineation of writings recognition and delineation of writings.launch"
• In a new terminal window type "rosservice call/redirect image". This command captures an image from the kinect V2 camera and publishes it into a topic for image processing.
• After the completion of image processing steps, a text file named "robot_commands.txt" is generated which is saved in a folder named "Bilder". This text file is then send to the Toshiba robot controller with the help of another ROS socket communication package. 


