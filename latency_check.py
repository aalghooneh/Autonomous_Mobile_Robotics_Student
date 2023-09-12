
from rclpy import init, spin, shutdown, ok, spin_once
from rclpy.node import Node

from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


from rclpy.time import Time

import sys

import random
import string

def generate_random_string(length):
    
    letters = string.ascii_letters
    
    return ''.join(random.choice(letters) for _ in range(length))

class LatencyMeasurementNode(Node):
    def __init__(self, node_name, topic, type_msg, qos_, win_length=10):
        
        super().__init__(node_name)
        
        self.subscription = self.create_subscription(
            type_msg,
            topic,
            self.callback,
            qos_
        )
        
        self.topic=topic

        self.win_length=win_length
        self.collection=[]


        self.create_timer(1.0, self.timer_callback)
        
    def callback(self, msg):
        
        current_time = self.get_clock().now()
        message_time = msg.header.stamp


        latency = current_time.nanoseconds - Time.from_msg(message_time).nanoseconds

        self.collection.append(latency/1e6)


        
        if (len(self.collection) > self.win_length):
            self.collection.pop(0)
        

    
    def timer_callback(self):
        if (len(self.collection) < 3):
            return
        
        _mean_=sum(self.collection)/len(self.collection)
        _std_=(sum( (x-_mean_)**2 for x in self.collection) / (len(self.collection)-1))**0.5
        
        print(f"average latency in ms for {self.topic} is: {_mean_}, with std of {_std_}")

        
def main(args=None):
    
    
    init(args=args)
    
    
    
    node_name=generate_random_string(10) 
    
    print(f"generated random node name: {node_name}")
    
    
    try:
        topic=sys.argv[1]
        msg_type=eval(sys.argv[2])
    
    except IndexError:
        print("Error! provide topic and then msg, like: ./latency_check /scan LaserScan ", file=sys.stdout)
        return
    
    
    
    if topic=="/odom":
        qos_=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    else:
        qos_=10
    
    node_=LatencyMeasurementNode(node_name, topic, msg_type, qos_, win_length=50)
    
    spin(node_)
    
    shutdown()

if __name__ == '__main__':
    main()
