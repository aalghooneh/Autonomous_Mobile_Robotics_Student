# LAB 3\* - Localization through particle filter

## Introduction

Welcome to LAB 3 of the mobile robotics course! Here, you will make your stack more elaborate through adding a robust localization with particle filter. By the end of this lab, participants will be able to:
- Have a good understanding of the particle filter.
- Integrate the particle filter localization with the point controller from LAB 2.
- Make an interactive interface to work with your navigational stack. 


#### The summary of what you should learn is as following:

1. You will learn how to search effieciently when you have a large space. 
2. You will learn how to do resampling when you have better distributions for your states.
3. You will how to leverage ros visualization tool (Rviz) to make an interactive interface with.


### NOTES for pre-lab activities
Given the limited time in the lab, it is highly recommended to go through this manual and start (or complete) your implementation before the lab date, by working on your personal setup (VMWare, remote desktop, lent laptop), and using simulation for testing when needed to verify that your codes are working before coming into the lab. For simulation, refer to `tbt3Simulation.md` in the `main` branch.

During the 3 hours in the lab, you want to utilize this time to test your code, work with the actual robot, get feedback from the TAs, and acquire the in-lab marks (check `rubrics.md` in the same branch).

While in-lab, you are required to use the Desktop PCs and **NOT** your personal setup (VMWare, remote desktop, lent laptop). So, make sure that you have your modified files, either online or on a USB, with you to try it out in-lab. 

## Part 1 - connecting to the robot (no marks)
Open the [connectToUWtb4s.md](https://github.com/aalghooneh/MTE544_student/blob/main/connectToUWtb4s.md) markdown file in the main branch, and follow along. Read and follow each step carefully.

## Part 2 - Robot teleop (no marks)

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

## NOTE: when you open a new terminal, you need to source again and set the domain ID, or you will not see the topics:

- Source the .bashrc file: source ~/robohub/turtlebot4/configs/.bashrc
- Declare ros2 domain: export ROS_DOMAIN_ID=X (X being the number of your robot)


## Part 3 - Map aquisition (15 marks)

Undock the robot, put the robot in the closest entrance marked for you, and reset the odometry, and then acquire the map as you did in LAB-1 and save it as room for use in the planning.

```
# terminal 1
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose {}
# terminal 2
ros2 launch turtlebot4_navigation slam.launch.py
# terminal 3
ros2 launch turtlebot4_viz view_robot.launch.py
# terminal 4
ros2 run nav2_map_server map_saver_cli -f room
``` 
When the map is acquired, copy the generated files ```room.yaml``` and ```room.pgm``` to the ```your_map``` directory. **In case that you picked the robot or it hit an obstacle**, you should pick your robot back and place it on the entrance, then **reset odometry** again so the odometry matches your map acquisition. **Please make sure you killed the slam and the visualization terminals with Cntrl+C or closing the terminal**.

**Show the map to a TA to score the marks associated to this part.**

## Part 4 - Localize your robot with Particle Filter (25 marks)
Start by first fixing your particle filter, so you can run the standalone ```particleFilter.py``` to localize your robot. 
To fix your particle filter, you will need to:
- Complete the motion model for each particle; follow the comments in ```particle.py```;
- See through impelementing a search algorithm for the map occupant grids and each particle weight calculation; follow the comments in ```particle.py``` and ```mapUtilities.py```;
- Complete the resampling, weighted averaging for the particle filter algorithm; follow the comments in ```particleFilter.py```.
- Once your code is completed test your implementation as following,
  - Make sure that the VPN terminal is still running, i.e., ```Tunnel is ready```, and that you can see the robot's topic list;
  - Remember to source your ```.bashrc``` file and set ```ROS_DOMAIN_ID```, in case you did not set up a permenant environment;
  - Undock your robot if needed, and use the teleop node to drive your robot to a free space, and then,

```
# terminal 1
python3 mapPulisher.py
# terminal 2
python3 particleFilter.py
# terminal 3
rviz2 -d for_pf.rviz
``` 
  - When the Rviz window opened throw a good guess by using ```2D pose estimate``` button in the top toolbar.
  - If the particles converged, try to moving the robot around with the ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```.


**Show your work to TA to score the marks associated to this part.**



## Part 5 - Integration to the stack (15 marks)
Now you need to run the point controller developed in LAB #2 with particle filter in the loop:
- Integrate the particle filter into your localization model, ```localization.py```;
- Design the proper flow; follow the comments in ```decisions.py```;
- Test your navigation stack with particle filter in the loop as following,
  - Make sure that the VPN terminal is still running, i.e., ```Tunnel is ready```, and that you can see the robot's topic list;
  - Remember to source your ```.bashrc``` file and set ```ROS_DOMAIN_ID```, in case you did not set up a permenant environment;
  - Undock your robot if needed, and use the teleop node to drive your robot to a free space, and then,

```
# terminal 1
python3 mapPulisher.py
# terminal 2
python3 particleFilter.py
# terminal 3
rviz2 -d for_pf.rviz
# terminal 4
python3 decisions.py
``` 
  - When the Rviz window opened throw a good guess by using ```2D pose estimate``` button in the top toolbar.
  - When the particles converged, choose your goal using ```2D nav goal``` button in the top toolbar.

**Show the map to a TA to score the marks associated to this part.**
**IMPORTANT!! Before you leave, DELETE all of your codes, files, etc.**

## Conclusions - Written report (25 marks)
You can do this part in the lab (time allowing) or at home. Make sure you have the proper data saved.

Please prepare a written report containing in the front page:
- Names (Family Name, First Name) of all group members;
- Student ID of all group members;
- Station number and robot number.

In a maximum of 3 pages (excluding the front page), report a comparison between the particle filter and the rawSensor method:

* Section 1 - the plot of the logged positions from the particle filter versus the odometry. 
* Section 2 - comparing the particle filter performance by changing the laser scan and the particle generation standard deviation.

## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder. Only one submission per group is needed:
- **Report**: one single pdf;
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log).


Good luck!
