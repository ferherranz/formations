formations
===========

Formations package to ROS, Stage and Turtlebot.

This package provides an implementation of potential fields to manage formations of robots using Stage and the turtlebot platform (see https://github.com/turtlebot). 
It provides a correct configuration of namespaces and frames that allows the robots to perform localization and navigation in real environments.
 
## Usage

Multi-robot formations:

 *  **Bring up simulated robots**


	$ roslaunch formations multi_robot.launch 
 *  **Run formations script**
 
 
 	$ rosrun  formations  formation.py 
