# MTE544 - Autonomous Mobile Robots - Setup

## Install pre-requisites of the course
If you have not installed Ubuntu 22.04 yet, please do that first.
Download the script (or if you installed ```git```, clone this repository with ```git clone https://github.com/aalghooneh/MTE544_student```, then change the branch to ```setup``` with ```git checkout setup```)

Run the script (it needs sudo privilege, so you will need to type in your password when prompted):
```
sh setup_mte544.sh
```
that will take care of installing everything you need for this course, including setting up some environmental variables.

## Check your installation
Once the script has finished the installation, you can quickly check the performance of your system with the Gazebo simulation. Open a terminal and run:
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
``` 
At the bottom of the Gazebo window, you will see the ```Real time factor```, if this number is below 0.5 (assuming you are not running anything else heavy on your VM or on your computer), this emans that your simulation will likely be quite slow, and you may want to consider utilizing one of the alternative systems proposed (you may still use your system for code development and connecting with the real robot, just the testing in simulation might be slow).


## To check the latency of the topics in TurtleBot4s
NOTE: This part may be needed when you will be using the physical robot to check the latency in the communications.

Use the Latency check script like this:

```
./latency_check.py topic msgType
# for example for scan topic
./latency_check.py /scan LaserScan 
```


