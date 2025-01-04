# Final exam (30% of the final total grade)

## Introduction

In the final exam, you will work on an individual project that is based on the same code structure as the ones used for the labs. **Make sure you familiarize yourself with the entire structure before attempting this exam**.

If you haven't, you can also check the recording from the lecture on Nov. 30th where we explained part of the final exam.

**Rules for the final exam:**
- It must be completed individually, do not work in groups, and do not violate Policy 71 (you can reuse material from your previous labs);
- You may ask questions on Piazza both publicly and privately, when asking privately make sure to include the entire teaching team;
- You may request individual meetings with the teaching team during the exam period (Dec. 8 - Dec. 18), meetings should not exceed 30 minutes;
- In case of emergency, you may request resources (laptops, remote desk computers) by emailing your TA Amr Hamdi (amhamdi@uwaterloo.ca), no resources will be allowed to be requested after 6PM Dec. 18.

Please note that both questions and individual meetings must be given sufficient time to be answered and planned (meetings should be requested with 12 hours notice, preferably). After 6PM Dec. 18, questions are not guaranteed answers, and meetings will not be planned.

**Ask questions on Piazza**

https://piazza.com/uwaterloo.ca/fall2023/mte544

**Individual meetings**

You may contact the following people (please add everyone so the first available person can reply) for an in-person or online meeting (Prof. Hu and Ahmad will be online):
- Ahmad (aralghooneh@uwaterloo.ca)
- Minghao (minghao.ning@uwaterloo.ca)
- Prof. Hu (yue.hu@uwaterloo.ca)

**Emergency resources**
- Amr Hamdi (amhamdi@uwaterloo.ca)

**Environment for the final exam**

For the final exam, no real robot is needed, everything will be conducted in simulation. Refer to `tbt3Simulation.md` in the `main` branch to see how to launch the Gazebo simulation. If you installed your environment using the script provided in `setup`, then it should work without any issues. Likely, you should have checked it's fully functional during the term for your labs.

For the final exam, you will want to run Gazebo in an empty environment without any additional structure (no house, no walls, etc, unless you want to score the bonus).

```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Grading**: refer to ```rubrics.md``` for detailed marking scheme.

## Part 1 - Explanation of the stack (20 marks)
Throughout the labs 2 to 4, you've been working on the same stack that comprises control, state estimation, and path planning.
In this first part, explain how the stack works with reference to the several classes involved:
- ```decisions.py```
- ```planner.py```
- ```localization.py```
- ```controller.py```

In the explanation:
- Refer back to what you've done in the labs and how you progressed each lab towards the complete stack.
- Draw a block diagram displaying how each part are connected to help your discussion. Specify which information/data and how they are exchanged between each block. The diagram can be hand-drawn, or computer drawn, but must be clearly readable.
- Make connections to the theoretical content learned during the lectures as well.
- Maximum 500 words.

## Part 2 - Implement the RRT* algorithm for path planning (25 marks + 5 bonus marks)
In Lab4, you implemented and tested the A* algorithm. For the lab, we provided the almost-complete A* that was already integrated into the stack.

The ```TODO``` items will be mainly in the following files:
- ```rrt_star.py```
- ```planner.py```

For the final exam, we provide you with a skeleton code for the RRT*. For a detailed explanation of this class, refer to the comments on top of the ```rrt_star.py``` file. This class is not integrated into the stack yet. You are requested to perform the following:
- Complete the RRT* algorithm following the comments in the code provided ```rrt_star.py``` (look for the ```TODO``` items), the following functions need to be completed:
  - ```planning```
  - ```choose_parent```
  - ```rewire```
  - ```search_best_goal_node```
- Instead of a map, you will be using virtual obstacles (see Part 3). This code uses some functionalities derived from ```rrt.py``` (see Appendix at the bottom of this page).
- It is recommended that you test your RRT* stand-alone without integrating it into the stack yet to solve any possible bug in the planner.
- Integrate the RRT* planner into the stack such that you can execute the resulting path on the robot in simulation. Please note that this is not map based as you will be using virtual obstacles instead of a map (see Part 3 below). You can also implement it so that a desired planner can be chosen between A* and RRT* (no penalty if this is not done and no bonus if it is).
- Implement trajectory smoothing of your obtained path before execution on the robot (in ```planner.py```).

**Bonus**

Instead of using virtual obstacles, integrate your code so that a map is used. This map should be acquired from the simulation environment (see Part3).

## Part 3 - Test the stack with RRT* in simulation (25 marks + 3 bonus marks)
Test your RRT* with different parameters in simulation.

To test your RRT*:
- Create a virtual map with virtual obstacles (see the RRT* code ```main``` function). This means that these obstacles will not be visualized in Gazebo nor in RViz (you can do extra work to have them visualized in RViz but this is not necessary and there are no bonus points associated).
- Use the Extended Kalman Filter as localizer, use your tuned covariances, or tune them in simulation, we will not deduct marks for poor localization performance. 
- Choose a goal pose: you can use the same RViz interface as the one for Lab4 (see below) to choose a goal pose, or you can hard-code a goal pose.
- Execute the path in Gazebo.
- **Bonus**: Log the poses of the robot and covariances from the EKF for the plots.

**To choose a goal pose in RViz**
(Remember to run the map publisher if you are using the map)
- In a terminal run the rviz2 with the given configuration: ```rviz2 -d pathPlanner.rviz```
- In another terminal run the decisions.py: ```python3 decisions.py```
- On rvzi2 use the 2D goal pose on the toolbar on top to choose the goal pose
- Watch the robot go to the specified point

Show your results with at least **two different sets of virtual obstacles**, and for each set, show at least **two different goal points**. To report your results:
- Tune your RRT* parameters (e.g. circle radius ```connect_circle_dist``` to be considered, expansion distance ```expand_dist```), in the written report you will be asked to discuss this process.
- Plot containing the virtual obstacles, the obtained path (optionally the entire RRT* tree), clearly marking the starting and goal positions. Overlay the generated path with the executed path. For **bonus**, plot the covariances as ellipses (see Appendix). 

To score fully the **bonus** with map: in addition to the above, show the planning using the house or the world of the TurtleBot3 (launch Gazebo with one of these environments and create the map to be used), and execute the motion in this environment.

Record a video of at least one successful trial, the video should contain:
- The choice of the goal point from RViz;
- The path execution in Gazebo.
The path shown in the video must be one of the four plots in the report. Clearly state in the report which one corresponds to the video.

**Video recording guidelines**

Do not use your phone or an external camera pointing at the computer screen for the video, use a screen capture/recorder (e.g. on Windows 11 you can use the snipping tool, in Ubuntu you can use screenshot tool).

## Conclusions - Written report and deliverables (30 marks)
Please prepare a written report containing on the front page:
- Name (Family Name, First Name);
- Student ID.

Page limits (excluding the front page and appendix):
- 4 pages;
- 5 pages if with bonus marks (map integration).

Report the following sections in your report:

* Section 1 - Stack: the description and discussion of the stack (see Part 1), max 500 words.
* Section 2 - RRT* implementation: 
  * Describe how th provided RRT* works and which modifications you implemented.
  * Describe how you integrated the path planning and navigation for the robot from the planner to the actual motions of the robot (you do not need to describe the entire stack again as you should have done this in Part 1), including the implementation of path smoothing.
* Section 3 - Testing: 
  * Report figures as specified in Part 3.
  * Discuss how you tuned your RRT* parameters to reach the final parameters you used for the plots.
  * Discuss the performance of your RRT*.
* Section 4 - Final discussions:
  * Discuss the overall performance of your entire stack, i.e. including your PID controller, EKF, and RRT* planner.


## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder.
- **Report**: one single pdf that can be checked for Turnitin (text must be detectable as such, i.e. not printed pdfs);
- **Code**: make sure to have commented your code! Submit one single zip file with everything (including the csv files obtained from the data log and the map files).
- **Video**: video file or link to the video in the report.


Good luck!

## Appendix

**Functions from rrt.py**

The ```rrt_star.py``` file uses some functions from ```rrt.py```, mainly the following ones, you can find them with comments explaining the functions in ```rrt.py```. It is advised to get familiar with them before proceeding with the completion of the code:
- ```steer```
- ```check_collision```


**(Bonus) Covariances as ellipses**

In this case, we are interested in the covariance in the `x` and `y` directions. The axis of the ellipse are defined by the standard deviations $\sigma_x$ and $\sigma_y$ (note standard deviations and not variance), leading to the following equation of the ellipse:

$$\begin{equation}(\frac{x}{\sigma_x})^2 + (\frac{y}{\sigma_y})^2 = d \end{equation}$$

Note that ```d``` is the scale of the ellipse and can be any number, e.g. 1.
Make sure also to orient the ellipse so that it is tangent to the path (oriented as the heading ```theta``` of the robot).





