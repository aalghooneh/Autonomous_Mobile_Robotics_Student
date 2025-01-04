import matplotlib.pyplot as plt
import numpy as np 

from sensor_msgs.msg import LaserScan
from math import floor
import math

from math import pi as M_PI
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PointStamped, Quaternion, Point


class map_utilities(Node):


    def __init__(self, laser_sig=0.1, filename_="room.pgm", yaml_filename_="room.yaml"):
        
        
        filename = filename_
        yaml_filename=yaml_filename_
        width, height, max_value, pixels = self.read_pgm(filename, yaml_filename)
      

        self.image_array = np.array(pixels).reshape((height, width))
        self.o_x, self.o_y, self.res, self.thresh = self.read_description(yaml_filename)

        self.laser_sig=laser_sig
        
        self.likelihood_msg=None
        
        
    
    
    def timer_callback(self):
        if self.likelihood_msg is None: 
            return
        self.map_publisher.publish(self.likelihood_msg)
        
    def read_pgm(self, filename, yaml_filename):
        with open(filename, 'rb') as f:
            # Check if it's a PGM file
            header = f.readline().decode().strip()
            if header != 'P5':
                raise ValueError('Invalid PGM file format')

            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()

            # Read width, height, and maximum gray value
            width, height = map(int, line.split())
            max_value = int(f.readline().decode().strip())

            # Read the image data
            image_data = f.read()

        # Convert image data to a list of pixel values
        pixels = [x for x in image_data]

        return width, height, max_value, pixels

    def plot_pgm_image(self, image_array):
        # Convert pixel values to a NumPy array


        # Plot the image
        plt.imshow(image_array, cmap='gray')
        plt.axis('off')
        plt.title('PGM Image')
        plt.show()



    def read_description(self, map_yaml_name):
        import re

        # Open and read the YAML file
        with open(map_yaml_name, 'r') as file:
            yaml_content = file.readlines()

            # Extract the desired fields
            threshold = None
            origin_x = None
            origin_y = None
            resolution = None

            for line in yaml_content:
                if 'occupied_thresh' in line:
                    threshold = float(re.findall(r'\d+\.\d+', line)[0])
                elif 'origin' in line:
                    origin_values = re.findall(r'-?\d+\.\d+', line)
                    origin_x = float(origin_values[0])
                    origin_y = float(origin_values[1])
                elif 'resolution' in line:
                    resolution = float(re.findall(r'\d+\.\d+', line)[0])
        return origin_x, origin_y, resolution, threshold



    def cell_2_position(self, p):
        i=p[0]
        j=p[1]
        return i*self.res + self.o_x, j*self.res + self.o_y
    
    
    def position_2_cell(self, X):
        x=X[0];y=X[1]
        return floor( (x- self.o_x )/self.res), floor( (y-self.o_y)/self.res)


    def make_likelihood_field(self):
        
        image_array=self.image_array

        from sklearn.neighbors import KDTree
        
        
        self.plot_pgm_image(image_array)
        indices = np.where(image_array < 10)
        
        occupied_points = [self.cell_2_position((i, j)) for i, j in zip(indices[0], indices[1])]
        all_positions = [self.cell_2_position((i, j)) for i in range(image_array.shape[0]) for j in range(image_array.shape[1])]

        print(len(occupied_points), "vs", len(all_positions))
        kdt=KDTree(occupied_points)

        dists=kdt.query(all_positions, k=1)[0][:]
        probabilities=np.exp( -(dists**2) / (2*self.laser_sig**2))
        
        likelihood_field=probabilities.reshape(image_array.shape)
        
        likelihood_field_img=np.array(255-255*probabilities.reshape(image_array.shape), dtype=np.int32)
        
        self.likelihood_img=likelihood_field_img
        
        self.occ_points=np.array(occupied_points)
        
                
        self.plot_pgm_image(likelihood_field)
        
        return likelihood_field
                
    
    def _numpy_to_data(self, data):
        """
        Convert the numpy array containing grid data to a python
        list suitable for use as the data field in an OccupancyGrid
        message.
        """
        flat_grid = data.reshape((data.size,)) * 100
        data_ = set(np.array(np.round(flat_grid), dtype='int'))

        return data_
    
    
    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now()
        grid_msg.header.frame_id = "map"

        grid_msg.data = self._numpy_to_data()
        return grid_msg
    
    
    def calculate_score(self,x,y):
        try:
            return self.likelihood_field[self.position_2_cell([x,y])]
        except IndexError:
            return 0
        
        
        
    def map_localization_query(self, laser_msg: LaserScan):
        
        points = laserscan_to_cartesian(laser_msg)

        # Define the range for x, y, and theta
        x_min, x_max = -10, 10
        y_min, y_max = -10, 10
        
        theta_min, theta_max = -M_PI, M_PI

        # Number of particles to generate
        num_particles = 1000

        # Generate random particles within the given range
        particles_x = np.random.uniform(x_min, x_max, num_particles)
        particles_y = np.random.uniform(y_min, y_max, num_particles)
        particles_theta = np.random.uniform(theta_min, theta_max, num_particles)
        
        # Transform points using each particle
        max_score=-1000
        

        for j in range(10):
            

            score_list=[]
            poses_list=[]            
            
            for i in range(num_particles):
                x = particles_x[i]
                y = particles_y[i]
                theta = particles_theta[i]

                transformed_x = points[:,0] * math.cos(theta) - points[:,1] * math.sin(theta) + x
                transformed_y = points[:,0] * math.sin(theta) + points[:,1] * math.cos(theta) + y

                scores = list(map(lambda x, y: self.calculate_score(x, y), transformed_x, transformed_y))
                score=math.prod(scores)
                if  score> 0:
                    score_list.append(score)
                    poses_list.append([x,y,theta])

            sum_weights=sum(score_list)
            if ( sum_weights > 0):
                
                score_list/=sum_weights
                
                x,y,theta=poses_list[np.argmax(np.array(score_list))]
                
                tx=points[:,0] * math.cos(theta) - points[:,1] * math.sin(theta) + x
                ty=points[:,0] * math.sin(theta) + points[:,1] * math.cos(theta) + y
                
                weighted_avg=np.average(np.array(poses_list.copy()), axis=0, weights=score_list)
                weighted_std = np.sqrt(np.average((np.array(poses_list.copy()) - weighted_avg) ** 2, axis=0, weights=score_list))

                print(weighted_std)
                
                particles_x = np.random.normal(weighted_avg[0], 0.2, num_particles)
                particles_y = np.random.uniform(weighted_avg[1], 0.2, num_particles)
                particles_theta = np.random.uniform(weighted_avg[2], 0.2, num_particles)
            else:
                particles_x = np.random.uniform(x_min, x_max, num_particles)
                particles_y = np.random.uniform(y_min, y_max, num_particles)
                particles_theta = np.random.uniform(theta_min, theta_max, num_particles)        

        plt.plot(tx, ty, '*')
        plt.plot(self.occ_points[:,0], self.occ_points[:,1], '.')
        plt.axis('off')
        plt.title('PGM Image')
        plt.show()


def laserscan_to_cartesian(laserscan):
    
    angle_min = laserscan.angle_min
    angle_increment = laserscan.angle_increment
    range_min = laserscan.range_min
    ranges = np.array(laserscan.ranges)

    valid_indices = np.where((ranges != 0) & (ranges <= laserscan.range_max))
    valid_ranges = ranges[valid_indices]

    angles = angle_min + valid_indices[0] * angle_increment

    cartesian_points = np.column_stack((valid_ranges * np.cos(angles), valid_ranges * np.sin(angles)))

    return cartesian_points        



if __name__=="__main__":
    #rclpy.init()

    MAP_UTILITIS=map_utilities()

    #rclpy.spin(MAP_UTILITIS)





