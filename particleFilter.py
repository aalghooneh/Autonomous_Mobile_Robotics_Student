

from particle import particle
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from mapUtilities import mapManipulator
import message_filters
import numpy as np

import time

import random

from utilities import *

from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid




class particleFilter(Node):

    def __init__(self, mapFilename="./your_map/room.yaml", numParticles=500):
        
        super().__init__("particleFiltering")
        
        self.tic = time.time()

        qos_profile_odom=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_profile_laserScanner = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                              durability=DurabilityPolicy.VOLATILE,
                                              depth=10)


        self.particleMarkerArrayPublisher =\
                self.create_publisher(MarkerArray, "/particles/markers", 10)
        

        self.odomSub = message_filters.Subscriber(self, Odometry, "/odom",
                                                   qos_profile=qos_profile_odom)
        self.laserScanSub = message_filters.Subscriber(self, LaserScan, "/scan",
                                                        qos_profile=qos_profile_laserScanner)


        self.initialPoseSubsriber = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initialPose2Dcallback ,10)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.odomSub, self.laserScanSub], queue_size=10, slop=0.1)
        self.timeSynchronizer.registerCallback(self.filterCallback)


        self.pfPosePublisher = self.create_publisher(Odometry, '/pf_pose', 10)

        self.mapUtilities=mapManipulator(mapFilename, laser_sig=0.1)
        self.mapUtilities.make_likelihood_field()


        self.historyOdom = []
        self.particles=[]

        self.numParticles = numParticles
        self.std_particle_x = 0.5
        self.std_particle_y = 0.5

        self.br = TransformBroadcaster(self)

        print("give me your first estimate on rviz, 2D Pose Estimator")
        

        self.dt = 0.2 # initialize with the probable dt value 

        self.initialized = False
        self.championPose=None    
    
    def initializeParticleFilter(self, x, y, th):
        
        numParticles = self.numParticles
        self.particlePoses=np.random.uniform(low=[x-self.std_particle_x, y - self.std_particle_y,  th - 1.0],
                                             high=[x+self.std_particle_x, y + self.std_particle_y, th + 1.0], size=(numParticles, 3))
        
        self.particles = [particle(particle_, 1/numParticles) for particle_ in\
                           self.particlePoses]
        
        self.normalizerFactor = 1/numParticles
        self.weights = [self.normalizerFactor] * numParticles

        self.initialized = True

    def initialPose2Dcallback(self, marker: PoseWithCovarianceStamped):
        
        x, y, th = marker.pose.pose.position.x,\
                   marker.pose.pose.position.y, euler_from_quaternion(marker.pose.pose.orientation)
        

        self.initializeParticleFilter(x,y,th)

    
    def visualizeParticles(self, particles_, stamp):
        
        particles=MarkerArray()
        
        for i, particle_ in enumerate(particles_):
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp

            marker.id = i
            marker.ns = "particles"
            marker.lifetime = Duration(seconds=1.5).to_msg()

            marker.type = marker.ARROW
            marker.action = marker.ADD

            weight = particle_.getWeight()
            if weight <= 0.10:
                weight = 0.1

            marker.scale.x=weight
            marker.scale.y = 0.05; marker.scale.z = 0.05

            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.pose.position.x = particle_.getPose()[0]
            marker.pose.position.y = particle_.getPose()[1]
            marker.pose.orientation.w = np.cos(particle_.getPose()[2]/2)
            marker.pose.orientation.z = np.sin(particle_.getPose()[2]/2)


            particles.markers.append(marker)

        self.particleMarkerArrayPublisher.publish(particles)

    def normalizeWeights(self):
        
        sumWeight = np.sum(self.weights)

        self.normalizerFactor = 1/sumWeight

        self.weights = self.weights * self.normalizerFactor


        for particle in self.particles:
            particle.setWeight( particle.getWeight()*self.normalizerFactor)


    def resample(self, laser_scan, mapUtilInstance):
        
        
        particles_sorted = sorted(self.particles.copy(), key=lambda particle: particle.getWeight(), reverse=True)

        # TODO part 4: choose the top half in the 
        # score as your best particles so you can
        # regenerate particles with them
        best_particles = particles_sorted[:...]

        # TODO part 4: choose an amount of noise for your
        # particles, this amount will try to generate new
        # particle around the best ones you already have.
        std_noise=...
        
        
        
        generated_particles = []
        for bp in best_particles:
            
            x,y,th = bp.getPose()

            new_x = x + random.uniform(-std_noise, std_noise)
            new_y = y + random.uniform(-std_noise, std_noise)
            new_th = th + random.uniform(-std_noise, std_noise)

            # TODO part 4: initilize your generated particle
            new_particle=particle(...)

            # TODO part 4: calculate the generated particle score
            # and normalize the new weight based on your normalizer
            # factor
            new_particle.calculateParticleWeight(...)

            generated_particles.append(new_particle)
            
  

        self.particles = best_particles + generated_particles
        
        return self.particles
    



    def filterCallback(self, odomMsg: Odometry, laserMsg: LaserScan):


        if not self.initialized:
            return


        self.historyOdom.append(odomMsg)

        if len(self.historyOdom) > 2:
            self.historyOdom.pop(0)

        else:
            return

        _, _, dth = calculate_displacement(self.historyOdom[0], self.historyOdom[1])
        w = odomMsg.twist.twist.angular.z
        v = odomMsg.twist.twist.linear.x



        dt = self.dt
        for i, particle_ in enumerate(self.particles):
            
            particle_.motion_model(dth, v, w, dt)
            particle_.calculateParticleWeight(laserMsg, self.mapUtilities)

            self.weights[i] = particle_.getWeight()
            self.particlePoses[i] = particle_.getPose()

        self.normalizeWeights()
        particles_to_viz = self.resample(laserMsg, self.mapUtilities)



        stamp = odomMsg.header.stamp
        self.visualizeParticles(particles_to_viz, stamp)





        # TODO part 4: take an average of your resampled particles which is 
        # a list of your best particles, Just NOTE that 
        # For the rotation (theta), you might use a mean of circular quantities 
        # if the angles are small and don't wrap around.
        # you can go ahead and add your function to the script
        weighted_average_translation, mean_angle = ...

        weighted_average_pose = np.append(weighted_average_translation, mean_angle)

        
        
        # TODO part 4: publish the particle filter idea of the pose
        self.championPose = ...
        self.publishChampionPose()


        self.dt = time.time()-self.tic
        self.tic=time.time()


    def publishChampionPose(self):
        
        msg=Odometry()

        msg.pose.pose.position.x = self.championPose[0]
        msg.pose.pose.position.y = self.championPose[1]
        msg.pose.pose.orientation = quaternion_from_euler(self.championPose[2])

        self.pfPosePublisher.publish(msg)

    def getChampionPose(self):
        return self.championPose

        
        


import rclpy

if __name__=="__main__":
    
    rclpy.init()

    pf = particleFilter()

    rclpy.spin(pf)    
