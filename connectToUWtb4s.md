## Connecting to the robot 

#### During the whole lab make sure only one of the members connected to the robot!

Make sure that you have a working internet connection; for instance go ahead and ping google.ca


```
# make sure you have "ping"

$> which ping

# the output should be something like this
# /usr/bin/ping
# if you don't have ping go ahead and install it
# sudo apt update && sudo apt install iputils

$> sudo apt-get update && sudo apt-get install net-tools iputils-ping
$> ping google.ca
```
The response should be like this,

```
> 64 bytes from yyz10s20-in-f3.1e100.net (142.251.41.67): icmp_seq=5 ttl=117 time=3.97 ms
```

If you have latency more than 30 ms let the TAs know. 

If you do not have the file, clone the robohub/turtlebot4 repository which contains the vpn for connecting to the bots. If you are using one of the desktop computers, please call a TA to input the root password.

Double check if you have the robohub folder in the home directory (/home/$USER), if not go ahead and create it. 

```
$> cd ~
$> mkdir robohub
$> cd robohub 
$> git clone https://git.uwaterloo.ca/robohub/turtlebot4.git
```

either case, go to the turtlebot4 repo location

```
$> cd ~/robohub/turtlebot4
$> git checkout rendezvous
```

check that you have the `.fastdds.xml` file on your computer. you can do so by

```
$> ls /home/$USER -a | grep .fastdds.xml 
```

if not then move the following configs into your home directory

```
 $> cp /home/$USER/robohub/turtlebot4/configs/.fastdds.xml /home/$USER/
```

Based on your excel sheet, find your robot number and connect to it. Make sure the robot is turned on (it is by default on when docked).

```
# suppose that X is your
# robot number
$> cd ~/robohub/turtlebot4/
$> ./rendezvous_vpn/vpn.sh X

```
#### Attention!! This process requires the entire terminal, so open a new tab/new terminal and follow with the rest

Each time you open a new terminal make sure you source the bashrc in the configs.

```
$> source ~/robohub/turtlebot4/configs/.bashrc
```
And change your ros domain!

```
$> export ROS_DOMAIN_ID=X
```

Now see your topics:

```
$> ros2 topic list
```

If you couldn't see them try restarting the ros2 daemon,

```
$> ros2 daemon stop
```

If still you couldn't see the topics, let the TA know. 

#### permanent environment set up file
If you want to avoid having to repeat the above lines each time you open a new terminal, you can also do the following. 

In the new terminal, go to ```home/$USER``` and make the following file by (you can use `vim` or any other text editor, such as `nano`, `gedit`):
```
$> vim .tb4_env
```
and add the following lines:
```
$> source ~/robohub/turtlebot4/configs/.bashrc
$> export ROS_DOMAIN_ID=X
```
Then add the following file to your bashrc,
```
$> source ~/.tb4_env
```
This is a temporary line you add to your bashrc for connection to tb4, when you want to work with your system at home, you should comment it by ``` # ``` characater.
