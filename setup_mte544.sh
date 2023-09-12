
set -e

echo "....installing ros2 humble...."

sleep 2

sudo apt update
sudo apt install -y git wget vim build-essential
sudo apt install -y lsb-core lsb-release
sudo apt-get install -y net-tools iputils-ping

os_codename=$(lsb_release -cs)

if [ $os_codename = "jammy" ]; 
then
	echo "--------------------------------"
	echo "....can install humble on it...."
	echo "--------------------------------"
fi

sleep 1

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale
echo "--------------------------------"
echo "....locales configs successful!...."
echo "--------------------------------"

sudo apt install -y software-properties-common
sudo add-apt-repository --yes universe


sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install -y ros-humble-desktop

echo "--------------------------------"
echo "....ros successfully installed...."
echo "--------------------------------"

sleep 1

sudo apt install -y ros-dev-tools


ros_bashrc_line="source /opt/ros/humble/setup.bash"

if ! grep -qF "$ros_bashrc_line" /home/$USER/.bashrc ; then echo "$ros_bashrc_line" >> /home/$USER/.bashrc ; fi

tb3_bashrc_line="export TURTLEBOT3_MODEL=burger"

if ! grep -qF "$tb3_bashrc_line" /home/$USER/.bashrc ; then echo "$tb3_bashrc_line" >> /home/$USER/.bashrc ; fi

echo "--------------------------------"
echo "....ros env successfully set!...."
echo "--------------------------------"

sleep 1


if [ -d "/home/$USER/robohub" ]
then
    echo "already made the glone"
else

   mkdir /home/$USER/robohub && cd /home/$USER/robohub && git clone https://git.uwaterloo.ca/robohub/turtlebot4.git

fi

sudo apt install -y ros-humble-rmw-fastrtps-cpp

cp /home/$USER/robohub/turtlebot4/configs/.fastdds.xml /home/$USER/

sleep 1

sudo apt install -y ros-humble-turtlebot4-desktop

sleep 1

sudo apt install -y ros-humble-turtlebot3*


echo "--------------------------------"
echo "....turtlebot enviornment is correctly set!...."
echo "--------------------------------"

