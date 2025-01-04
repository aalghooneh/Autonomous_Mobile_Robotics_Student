import sys
import time
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0; kalmanFilter=1

odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    
    def __init__(self, type, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")
        
        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors();
        elif type==kalmanFilter:
            self.initKalmanfilter()
            self.kalmanInitialized = False
        else:
            print("We don't have this type for localization", sys.stderr)
            return            
    
        self.timelast=time.time()
    
    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self):
        
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        
        time_syncher.registerCallback(self.fusion_callback)
        
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        if not self.kalmanInitialized:
            x=np.array([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        euler_from_quaternion(odom_msg.pose.pose.orientation),
                        0,
                        0,
                        0])        

            Q=0.1*np.eye(6)
            R=0.4*np.eye(4)
            P=Q.copy()
            
            self.kf=kalman_filter(P,Q,R, x)
            self.kalmanInitialized = True
        
        dt = time.time() - self.timelast

        self.timelast=time.time()


        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])
        print(dt)
        self.kf.predict(dt)
        self.kf.update(z)
        
        xhat=self.kf.get_states()
        
        self.pose=np.array([xhat[0],
                            xhat[1],
                            normalize_angle(xhat[2]),
                            odom_msg.header.stamp])
        
        
        self.loc_logger.log_values([z[2], z[3], xhat[5], xhat[4]*xhat[3], xhat[4], xhat[3], xhat[0], xhat[1], Time.from_msg(imu_msg.header.stamp).nanoseconds])
        
        print(f"{xhat[0]} and {xhat[1]} vs {odom_msg.pose.pose.position.x} vs {odom_msg.pose.pose.position.y}")
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

        
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    
    spin(LOCALIZER)