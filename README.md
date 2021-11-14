# ece470project

This repository holds code that simulates an autonomous shopping robot using Webots and ROS.  Since our robot is intended to faciliate fully autonomous shopping in retail stores it will need to simultaneously perform pick and place actions as well as move to locations where the target obstacles are within reach.  To do this, the robot will need some vision capabilities, and knowledge of the Forward and Inverse Kinematics of its actuator.  The code in this repository will accomplish both of these tasks.

Our entire catkin workspace has been uploaded to the repo, however the code we have written is contained in the catkin_ws/src/ directory.  
We decided to create a ros package, similar to those in lab so that we can use a procedure similar to the one in lab to work on our code.  This package is called ece470 and is found in the catkin_ws/src/ece470 directory.  Our simulation worlds can be found in the catkin_ws/src/ece470/worlds/ directory and launch file in ece470.launch file.  

Running our code should be as simple as cloning this repository and creating the workspace but we have noted issues with this process in the days leading up to the final Project Update.  

Finally, much of our code is based on the Webots documentation.  A more thorough citation will be added at a later date.  
