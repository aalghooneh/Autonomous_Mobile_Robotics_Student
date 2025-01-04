# LAB 1 - Sensor data processing for mobile robots

## Introduction

Welcome to LAB 1 of the mobile robotics course! In this lab, participants will gain hands-on experience with mobile robots and acquire knowledge about their basic functionalities. By the end of this lab, 
### Participants will be able to:
- Interact with sensors.
- Understand the pipeline of ros2 or any middleware.
- Read and Process robot's data; sensors and actuators.


### Participants will learn:

1. How to connect to your mobile robot and make it move around. 
2. How to properly read and log sensors through ros interfaces and OOP programming. 

### NOTES for pre-lab activities
Given the limited time in the lab, it is highly recommended to go through this manual and start (or complete) your implementation before the lab date, by working on your personal setup (VMWare, remote desktop, lent laptop), and using simulation for testing when needed to verify that your codes are working before coming into the lab. For simulation, refer to `tbt3Simulation.md` in the `main` branch.

During the 3 hours in the lab, you want to utilize this time to test your code, work with the actual robot, get feedback from the TAs, and acquire the in-lab marks (check `rubrics.md` in the same branch).

While in-lab, you are required to use the Desktop PCs and **NOT** your personal setup (VMWare, remote desktop, lent laptop). So, make sure that you have your modified files, either online or on a USB, with you to try it out in-lab.

## Part 1 - Connect to the robot (5 marks)
Open the [connectToUWtb4s.md](https://github.com/aalghooneh/MTE544_student/blob/main/connectToUWtb4s.md) markdown file in the main branch, and follow along. Read and follow each step carefully. Wait for TA approval before going to next step.

## Part 2 - Play with the robot (5 marks)

In this part, you will learn to play with the robot; you will get to undock it from the charger and then move it around by keyboard.  
When you want to dock it again, It should be able to find it only when it is in less than ~0.5 meter around it. Note, that it doesn't
necessarily goes to the dock that it was undocked from, it will just find the next available dock.

The undock command goes through a [action server](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html).

```
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```
You robot should undock.
If not, revisit *Part 1* - Connect to robot via VPN, and make sure the VPN terminal is still running and that you can still see the robot's topics. If you suspect the vpn isn't working, make sure you terminate it, and then run again.

Next run the teleop command to be able to move the robot manually around.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See the prompt for help on the keys. 

To dock the robot, use:
```
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
```

### Lead the robot to your seat and let a TA know to get your checkmark for grading!

### NOTE: when you open a new terminal, you need to source again and set the domain ID, or you will not see the topics:
- Source the .bashrc file: source ~/robohub/turtlebot4/configs/.bashrc
- Declare ros2 domain: export ROS_DOMAIN_ID=X (X being the number of your robot)

## Part 3 - Setting up your code (15 marks)

In this lab, you will complete the provided code ```motions.py``` to move the robot and collect data. 
For robot movement, you will be sending motion commands as velocities (twists). This means you will need to publish velocities over the ```/cmd_vel``` topic. 
For data collection from the IMU, the Lidar (laser scan), and the wheel encoders (odometry), you will be subscribing to ```/imu```, ```/scan```, and ```/odom```, respectively. 

- Find ```utilities.py``` and ```motions.py``` in the current branch (labOne), and download them.
- Open each script, and follow the TODO comments to implement the requirements corresponding to Parts 3 - 5 in this manual. You should replace each ```...``` in the script before you attempt running it.

To setup your code:
- Import the right types of messages needed (see ```motions.py```); to do so, you will need to check for message type and message components given the topic names and using ros2 commands ```ros2 topic info /topic_name``` and ```ros2 interface show message_type```, respectively, as covered in the tutorials. For online documentation of the messages (you need to select the ROS2 distro you are using)
  - https://index.ros.org/p/geometry_msgs/
  - https://index.ros.org/p/sensor_msgs/
  - https://index.ros.org/p/nav_msgs 
- Set up the publisher for the robot's motions;
- Create the QoS profile.
  
**Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4).**

**Use "ros2 topic info /odom --verbose" as explained in Tutorial 3.**

## Part 4 - Implement the motions (15 marks)

In this part, you need to implement 3 different motions for the robot:
- Circle;
- Spiral;
- Straight line.

Follow the comments in ```motions.py``` to implement these motions for the robot, there is one function for each of these motions. 

For real robots, you will need to tune your robot's motion parameters to make sure it fits into the classroom space.

## Part 5 - Implement the data reading and logging (15 marks)
To read from the sensors (IMU, Lidar, wheel encoders), you need to:
- Subscribe to the topics these sensors are publishing data on;
- Create callback functions to log the data.

Follow the comments in the ```motions.py``` and ```utilities.py``` to complete the above functionalities. The loggers that save the data on csv files are already implemented for you.

## Part 6 - Execute on the robot and log the data (10 marks)
If all of the above is completed, you should now be ready to execute the code on the robot. Run each case one by one (circle, spiral, line) to log the data.
Log sufficient data for post processing and analysis. Make sure that the data is actually logged and saved.

First drive the robot to a sufficiently large space, then start your motion sequences. You can use undock and the teleop as you did in *Part 2*. Remember to dock your robot at the end of your tests.

To run your modifed ```motions.py``` script:
- Make sure you can still see your robot's topics before attempting to run your script. The critical topics are ```/scan``` and ```/odom```, make sure they are available by running ```ros2 topic echo /topic_name```  If not, re-visit *Part 1*.
- Open a terminal, go to your modified ```motions.py``` directory and run: ```python3 motion.py --motion line```

Test all motion sequences.

### Show each motion sequence to one of the TAs.

## Part 7 - Process your data and visualize them (10 marks)
By running ```motions.py``` with different motion sequences, ```.csv``` files will be created for each message type subscribed. The files generated by your script can be found in the same directory as ```motions.py```. 

The remaining part of this manual can be done in lab or at home. But it is recommended that you perform some plots to check the quality of the data before leaving the lab. You will not be able to recollect the data at another time if you did not perform this check.

A simple data visualization script is provided ```filePlotter.py```. You can use/adapt/modify this script or create your own to visualize the data.
By running the script, you will be able to create plots for the data collected. 

*Plot the sensor data that you collected for the different movements from Part 6: laser scans, IMU data, and odometry data.*.
These plots should help you with your discussions at the end of this manual.

- Find ```filePlotter.py``` in the current branch (labOne), and download it.
- Navigate to its directory, and run: ```python3 filePlotter.py --files imu_content_spiral.csv```
- Generate plots for the data collected for each motion sequence for IMU and Odom.
- For laserscan, find out how to convert the range matrix into the cartesian, if you have NaN/Inf clean that and then plot only one row of the data.

## Part 8 - Map acquisition (10 marks)

ROS 2 provides some packages that allow to perform mapping of the environment. This utilized SLAM (Simultaneous Localization and Mapping) provided by the Nav2 package. Find out which maze you should map from the ```List of groups``` excel sheet on LEARN.
This will be very useful for the later labs, it will be especially needed for LAB-3 and LAB-4.

In real world with TurtleBot4:

See also this link for more details in [turtlebot4 manual](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html):

- Open a new terminal, make sure your environment is set up and your topics are available.
- Undock your robot if docked.
- Make sure the ```/scan``` and ```/odom``` topics are available by running ```ros2 topic echo /scan``` or ```ros2 topic echo /odom```. Use ```Ctrl+c``` to crash echo process.
- In a new terminal, run the teleop node to drive the robot to the assigned maze entrance. Remember to decrease robot velocity before driving it around to ensure a decent quality of the map acquisition. 
- Once you are at the maze entrance, launch the slam package by ```ros2 launch turtlebot4_navigation slam.launch.py ```. Keep the terminal running. 
- In another terminal run RViz: ```ros2 launch turtlebot4_viz view_robot.launch.py```. This will help you to see the map, robot and the scan. Keep the terminal running.
- Using the teleop node, drive the robot inside the maze until you can sufficiently map it. Bring the RViz window to the front while driving the robot. You should see areas and walls appearing on the map in RViz as the robot gets closer to obstacles/items. Note that the colors of the map reflect the confidence of the robot in thos locations. You should see that as you drive the robot closer to those areas, the confidence increases and the map becomes clearer and the base becomes more opaque.
- In a new terminal, save the map with ```ros2 run nav2_map_server map_saver_cli -f map```. You should see the map saved in the folder where you are currently located. You should have 2 files, one .pgm, and one .yaml.

You can do next part in the lab (time allowing) or at home. If your VM is slow, go ahead and use the lab PC.

In simulation with TurtleBot3:
- Follow the instructions in `tbt3Simulation.md` to run the robot in simulation 
- In second terminal, run the slam: ```ros2 launch slam_toolbox online_sync_launch.py``` this will open RViz and you should see the base of the map.
- In a third terminal, run the teleop node: ```ros2 run turtlebot3_teleop teleop_keyboard```.
- Save the map with ```ros2 run nav2_map_server map_saver_cli -f map```. You should see the map saved in the folder where you are currently located. You should have 2 files, one .pgm, and one .yaml.

You do not have to map the entire room, just a sufficient area to see a portion of the map.

**IMPORTANT!! Before you leave, DELETE all of your codes, map files, etc.**

## Conclusions - Written report (15 marks)
You can do this part in the lab (time allowing) or at home.

Please prepare a written report containing in the front page:
- Names (Family Name, First Name) of all group members;
- Student ID of all group members;
- Station number and robot number.

In a maximum of 2 pages (excluding the front page), report the following:
- The plots you obtained. The plots should have, title, label name for axis, legends, different shapes/colors for each data, and grids.
- A screenshot of your obtained map.
- A brief explanation of the obtained plots (can be in the figure captions), and a brief discussion (interpretation, quality, etc) to show your understanding of the sensor information. *Hint*: you may leverage on the course material and the online documentation of the messages to better interpret your data.

## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder. Only one submission per group is needed:
- **Report**: one single pdf;
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log).


Good luck!
