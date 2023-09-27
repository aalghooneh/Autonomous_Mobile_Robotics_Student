## Connecting to the robot 

##### During the whole lab make sure only one of the members connected to the robot!

For in-lab desktop computers, proceed to the next section.
#### For VMWare users: Internet connection availability

Make sure that you have a working internet connection; for instance go ahead and ping google.ca:
- Open a terminal and run: ```which ping ```
- Output should be something like this: ```/usr/bin/ping```
- If you don't have ping go ahead and install it: ```sudo apt-get update && sudo apt-get install net-tools iputils-ping```
- If you already have it or you installed it go ahead with pinging google so you double check you have internet: ```ping google.ca```
- The response should be like this: ``` 64 bytes from yyz10s20-in-f3.1e100.net (142.251.41.67): icmp_seq=5 ttl=117 time=3.97 ms ```
- Make sure you do not have latency more than 30 ms.

If you do not have the ``` robohub ``` directory in ```~/```, clone the robohub/turtlebot4 repository which contains the vpn for connecting to the bots:
```
cd ~
mkdir robohub
cd robohub 
git clone https://git.uwaterloo.ca/robohub/turtlebot4.git
```

Once you found/created ```robohub``` folder, go to the turtlebot4 repo location, and change to the rendezvous branch.
```
cd ~/robohub/turtlebot4
git checkout rendezvous
```

Check that you have the `.fastdds.xml` file on your computer. you can do so by
```
ls /home/$USER -a | grep .fastdds.xml 
```

if not then move the following configs into your home directory
```
 cp /home/$USER/robohub/turtlebot4/configs/.fastdds.xml /home/$USER/
```
#### For in-lab desktop computers and VMWare users: Connect to robot via VPN

Based on the ```List of groups``` excel sheet on LEARN, find your robot number and connect to it. Make sure the robot is turned on (it is by default on when docked).
- Suppose that X is your robot number, open a terminal and run:
```
cd ~/robohub/turtlebot4/
./rendezvous_vpn/vpn.sh X
```
- Output should contain: ```Remote Reachable```
- Ask one of the TAs to enter the required password.
- Enter your credentials as required and the ouput of the terminal should contain: ```Tunnel ready```
- If output contains: ```Address already in use```. Make sure that you are trying to connect to the turtlebot you are assigned to according to the excel sheet.

##### ATTENTION!! This process requires the entire terminal, so KEEP this VPN terminal open and open a new tab/new terminal and follow with the rest.Each time you open a new terminal remember to source the bashrc in the configs, and declare your ros2 domain.
To do this:
- source the ```.bashrc``` file: ```source ~/robohub/turtlebot4/configs/.bashrc```
- Declare ros2 domain: ```export ROS_DOMAIN_ID=X```
- Wait for a few seconds and try viewing your topics: ```ros2 topic list```
- Output should be a long list of topics, a short list containing the following topics only, means that you still cannot view the robot's topics:
```
/parameter_events
/rosout
```
- If you couldn't see the robot's topics, try viewing the topics again. If not, try restarting the ros2 daemon: ```ros2 daemon stop``` and re-run ```ros2 topic list```.
If still you couldn't see the topics, do not close any terminals and let the TAs know.

#### For VMWare users: Permanent environment set up file
If you want to avoid having to source ```bashrc``` file and declaring ros2 domain each time you open a new terminal, you can also do the following:
- In a new terminal, go to ```home/$USER```
- Create the following file by (you can use `vim` or any other text editor, such as `nano`, `gedit`): ```gedit .tb4_env```
- Add the following lines:
```
source ~/robohub/turtlebot4/configs/.bashrc
export ROS_DOMAIN_ID=X
```
- Then add the following file to your bashrc: ```source ~/.tb4_env```
This is a temporary line you add to your bashrc for connection to tb4, when you want to work with your system at home, you should comment it by ``` # ``` characater.


### Once you are able to see the topics, let the TAs know so you can move to the next step!!
