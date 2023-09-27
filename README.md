# Autonomous Mobile Robots


## Introduction

This repo is made to help the students make a complete mobile robot stack from scratch! The course is covered in MTE544 at the University of Waterloo taught by 
[Prof. Yue Hu](https://uwaterloo.ca/mechanical-mechatronics-engineering/profile/y526hu). 

## What do you need to carry through this course?
The course code is developed based on TurtleBot4 (tb4) topics but as long as you have a ROS/ROS2-based mobile robot with lidar, camera, IMU, and encoders you can follow all the labs. 
 
It is worth mentioning that you can still follow the course without a robot and only with simulation. 

## Overview of the course
The course starts with lab1, which covers how to run the tb4 or the simulation, then it covers reading the sensory data and the explanation of the sensory outputs, and how to log them. 

The overall stack for a mobile robot can be summarized as something like this:
![mobile_robotics_plan](https://github.com/aalghooneh/mobile_robotics/assets/51265135/71e22c97-52dc-4d56-a7a0-887827f73a08)

The development of this stack starts from LAB-2 by developing a closed-loop controller. You can see in the diagram the components that you will be developing in the remaining labs and how they are interconnected.

## Expected outcome for the labs

By the end of this course, you should:
- Have learned the basics of ROS2;
- Be able to apply theoretical concepts to practical mobile robotics problems;
- Have a complete software stack that can help you develop further features and functionalities for your robotic applications.

## LAB-1 - Sensor data processing

In this lab, we go over the ROS2 components and libraries in the form of reading and writing the sensory data.

It will be clear how a middleware like ROS2 can be helpful when handling different processes both on the local and other machines. 

You will learn how to log sensors' output when writing to the actuators to make the robot do different trajectories. Logging is one of the most important aspects of robotics. It provides denser information around the goal intended to achieve, so downstream decisions can be made more thoughtfully. 

## LAB-2 - Closed-loop controller

In this lab, you will learn how to lay a thin version of the overall stack. You will start laying the bricks by completing the closed-loop controller. 

The closed loop controller is a concept already covered in your previous control course(s) (ME/MTE360 or a similar course if you are not an MME student). In this lab, you will learn how to implement that without using propriety software like MATLAB. 

One major takeaway from this lab is how to take derivatives and integration from a stream of incoming data. This is extremely useful for real-world applications and is largely utilized in many engineering applications (not only robotics), especially when control is needed.

You will also gain experience implementing Object Oriented Programming (OOP) in robotics using Python. 

## LAB-3 - Localization

In this lab, you will get familiar with different estimation methods for localization. 

The first part would be the implementation of a Kalman Filter (KF). 
You will experience how the motion model of the mobile robot can be used to both have better localization and also filter out the IMU acceleration noise. 

Moreover, you will also record a normal trajectory inside the maze and will be able to localize the robot using a particle filter.

## LAB-4 - Path planning

In this lab, all the stacks will come together. 
All the modules will be activated: first, you will implement two different path-planning methods to design a path inside the maze; then, your controller should follow the waypoints from the path and use your localizer as feedback. As you are moving around, you should log the robot states and the waypoints designed, so afterward you can quantify the performance of your stack. 
