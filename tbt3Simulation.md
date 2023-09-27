## Using the Gazebo simulation

The simulation of the TurtleBot4 is extremely slow due to issues with the graphical components, so in simulation we recommend using the TurtleBot3 model. Everything is the same as for the TurtleBot4, except for the model being a bit different. The packages for this simulation were installed with the setup script you ran to install all the packages and dependencies.

To run the TurtleBot3 in simulation, make sure that:

```
echo $TURTLEBOT3_MODEL
```

returns ```burger```.

If not, open your `.bashrc` file in your home folder (`cd ~`), and add the line:

```
export TURTLEBOT3_MODEL=burger
```

Then source your `.bashrc` file (or simply close the temrinal and open a new one for the modification to take effect).

Now open a terminal, and do the following:

```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
``` 

This will open the simulation environment, Gazebo, with the Burger model of the TurtleBot3, and the house environment. You can also use other environments than house, you can find which are possible environments by pressing on the tab key (autocomplete in Ubuntu) after ```ros2 launch turtlebot3_gazebo ``` (existing environments are for example world, dqn_stage1, etc).

Once Gazebo has finished loading all the components, open another terminal. In this terminal, type:

```
ros2 topic list
```

You should see a list of topics, among which `\cmd_vel`, `\odom`, `\joint_states`, etc.
If you do not see these topics, you may want to check if you have sourced your setup.bash file.

### Teleop

Unlike on the real robot, in simulation the robot is not docked, so you do not need to dock and undock it. So now you can test anything you want, for example the teleop. You can use the default teleop to move the robot manually:

```
ros2 run turtlebot3_teleop teleop_keyboard
```
or this is the same command as on the real robot, which works also in simulation:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

See the prompt for help on the keys. 

### Exiting the simulation
To close the simulator, close the Gazebo windows, then CTRL+C in the terminal from which you ran command to launch Gazebo (or directly do CTRL+C).