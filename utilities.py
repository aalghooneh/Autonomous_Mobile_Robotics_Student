

import numpy as np
from sensor_msgs.msg import LaserScan
from math import atan2, asin, sqrt
from geometry_msgs.msg import Quaternion

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


M_PI=3.1415926535

def normalize_angle(theta):
    while theta > M_PI:
        theta -= 2 * M_PI


    while theta < -M_PI:
        theta += 2 * M_PI
    return theta


def calculate_displacement(pose1, pose2):
    # Calculate displacement in x and y
    delta_x = pose2.pose.pose.position.x - pose1.pose.pose.position.x
    delta_y = pose2.pose.pose.position.y - pose1.pose.pose.position.y

    # Convert quaternions to Euler angles
    orientation1 = pose1.pose.pose.orientation
    
    orientation2 = pose2.pose.pose.orientation
    
    yaw1 = euler_from_quaternion(orientation1)
    yaw2 = euler_from_quaternion(orientation2)

    # Calculate displacement in theta (yaw angle)
    delta_theta = yaw2 - yaw1

    # Normalize delta_theta to be between -pi and pi
    delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi

    return delta_x, delta_y, delta_theta


def publishTransform(br, x,y,th, stamp):
    
    t=TransformStamped()

    t.header.frame_id = "map"
    t.child_frame_id="base_scan"

    t.header.stamp = stamp

    qt = quaternion_from_euler(th)


    t.transform.rotation =qt
    t.transform.translation.x = x
    t.transform.translation.y = y


    br.sendTransform(t)

def quaternion_from_euler(th):
    qt = Quaternion()
    th = th
    qt.z=np.sin(th/2)
    qt.w=np.cos(th/2)
    return qt
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    # just unpack yaw for tb
    return yaw

def position_2_cell(cartesianPoints: np.array, origin: np.array, res, h):
    return (np.array(np.floor((-origin + cartesianPoints)/res), dtype=np.int32)) - np.array([0, h]) 


def cell_2_position(cells, origin: np.array, res, h):
    return cells * res + origin + np.array([0, -h*res])





def calculate_linear_error(current_pose, goal_pose):
        
    return sqrt( (current_pose[0] - goal_pose[0])**2 +
                (current_pose[1] - goal_pose[1])**2 )

def calculate_angular_error(current_pose, goal_pose):

    error_angular= atan2(goal_pose[1]-current_pose[1],
                        goal_pose[0]-current_pose[0]) - current_pose[2]
    
    if error_angular <= -M_PI:
        error_angular += 2*M_PI
    
    
    elif error_angular >= M_PI:
        error_angular -= 2*M_PI
    
    return error_angular


def convertScanToCartesian(laserScan: LaserScan):

    angle_min = laserScan.angle_min
    angle_increment = laserScan.angle_increment
    range_min = laserScan.range_min
    range_max = laserScan.range_max
    ranges = np.array(laserScan.ranges)

    valid_indices = np.where((ranges != 0) & (ranges <= range_max) & (ranges >= range_min)) 
    valid_ranges = ranges[valid_indices]

    angles = angle_min + valid_indices[0] * angle_increment

    cartesian_points = np.column_stack((valid_ranges * np.cos(angles), valid_ranges * np.sin(angles)))
    cartesian_points_homo = np.column_stack((cartesian_points, np.ones(cartesian_points.shape[0])))

    return cartesian_points, cartesian_points_homo



class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str=""
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line
            

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table