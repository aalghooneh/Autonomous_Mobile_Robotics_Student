# LAB 4 - Path planning and navigation

## Introduction

Welcome to LAB 4 of the mobile robotics course! 
In this lab, participants will gain experience with implementing a path planner and drive the robot using the obtained plan.
By the end of this lab, participants will be able to:
- Use a map to create a graph for graph-based path planner;
- Generate a plan with the path planner and navigate the robot through the path to specified goals.

*Part 1* and *Part 2* are the same as in the previous labs they are here just for your convenience.


#### The summary of what you should learn is as following:
You will learn how to:
- Create a cost map from a pre-aquired map;
- Generate an optimal path using the A* algorithm;
- Execute a path on a mobile robot.

**NOTE** this Lab builds on top of Lab 2 and Lab 3. A complete solution to Lab 2 and Lab 3 is provided within this lab so that even if you did not conclude Lab 2 and Lab 3's implementation, you can still work on Lab 3 without penalties. You are welcome to replace some of the code with your own development from Lab 2 and Lab 3.

Check ```rubrics.md``` for the grading scheme of this lab.

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

## Part 3 - Map aquisition (5 marks)

Undock the robot, put the robot in the entrance marked for you, and reset the odometry, and then acquire the map as you did in LAB-1 and save it as room for use in the planning.

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
When the map is acquired, make sure you **don't pick up the robot** so you wouldn't alter the odometry. If by any change you did, put the robot back on the dock, then undock and reset the odometry. This is to avoid for you to map the enviornment again.

**Show the map to a TA to score the marks associated to this part.**

## Part 4 - Complete the A* algorithm (25 marks)
A mostly-completed A* algorithm is provided in ```a_star.py```.

For this part:
- Follow the comments to complete the code in ```a_star.py``` to plan the path using the A* algorithm. 
- Implement two different heuristics: Manhattan distance and Euclidean distance. A policy for switching between these two heuristics is not implemented, you are free to implement this the way you prefer (hard coded or with switching parameter or any other way).

## Part 5 - Complete the code for testing the path (25 marks + 5 marks bonus)
To utilize the planner, it is necessary to create a cost map and define a goal pose. Differently from previous planners you have used, in this case the searching algorithm will create a list of poses that the robot has to follow (a path). So some adaptations to the code are necessary.

For this part:
- Complete the code in ```planner.py``` to create the cost map using the ```mapManipulator``` from ```mapUtilities.py```;
- Complete the code in ```planner.py``` to create a trajectory that is a list of goal poses returned by the searching algorithm which correspond to the path to follow;
- Complete the code in ```decisions.py``` to adapt the code for the path planner.
- Bonus point if you integrate the code with the Particle Filter you developed in the Lab 3. 


## Part 6 - Test your path planner (20 marks)
To test the path planner:
- Choose at least two different goal poses (can be consecutive goal points during the same execution) on your map that are significantly far from each other and perform path planning and navigation for each ot these two goals.
- Perform the planning for each of these two goal poses with the two heuristics implemented in Part 4 (Manhattan and Euclidean).

**For choosing a goal pose:**
1. Open a terminal and run the mapPublisher.py: ```python3 mapPublisher.py```  
2. In another terminal run the rviz2 with the given configuration: ```rviz2 -d pathPlanner.rviz```
3. In another terminal run the decisions.py: ```python3 decisions.py```
4. On rvzi2 use the 2D goal pose on the toolbar on top to choose the goal pose 
5. Watch the robot go to the specified point 

**Note, given that there is limited time and space in the lab, do the necessary for scoring the in-lab marks (see below), and the rest in simulation.**

**In-lab marks**:
**Check your allocated time slot for testing. Please try to finish within your allocated time.**

- **Show the path execution with at least two goal poses and one heuristic to a TA to score 10 marks. Show the TA where are your goal poses within the map on RViz and what is the used heuristics.**


## Conclusions - Written report (30 marks)
You can do this part in the lab (time allowing) or at home. **Make sure you have the proper data saved**.

Please prepare a written report containing on the front page:
- Names (Family Name, First Name) of all group members;
- Student ID of all group members;
- Station number and robot number.

In a maximum of 3 pages (excluding the front page), report the performance of the path planner. This report should contain the following:

* Describe how you implemented the path planning and navigation for the robot from the planner to the actual motions of the robot, including how the provided code works.
* Figures illustrating the map, with the trajectories generated for the two goal points overlayed on the map. Make sure to clearly mark the starting and ending locations of the robot.
* Compare the two different heuristics (Manhattan and Euclidean distances, they can be on the same plot, but use different colors). Discuss the results. Are they different, if yes, why, if not why. Which one is better, and why?

## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder. Only one submission per group is needed:
- **Report**: one single pdf;
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log and the map files).


Good luck!
