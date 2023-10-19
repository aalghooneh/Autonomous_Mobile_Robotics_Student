import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin


rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        odom_qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
            # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        yaw = euler_from_quaternion(pose_msg.pose.pose.orientation)

        self.pose=[pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, yaw, pose_msg.header.stamp]
        #print("Yaw is ", yaw)
        #print("x is ", self.pose[0], " y is ", self.pose[1])
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!

if __name__=="__main__":

    init()

    localization_instance = localization()

    try:
        #Start running th eobject
        spin(localization_instance)
    except KeyboardInterrupt:
        print("Exiting")
    
