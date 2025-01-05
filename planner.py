
from math import sin,cos, atan2,atan
import math

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; SPIRAL_4TUNE=2

PARABOLA=0; SIGMOID=1

class planner:

    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()
        

    def point_planner(self, endPose):
        return endPose


    def trajectory_planner(self):
        TRAJECTORY_TYPE=SIGMOID
        degree_rad_conversion=3.14/180.0
        if TRAJECTORY_TYPE == PARABOLA:
            
            path = [[ (x/10.0) ,(x/10.0)**2] for x in range(0,20)]
            # rotate the path by theta degrees
            theta = 60.0 * degree_rad_conversion
            return [[x*cos(theta) - y*sin(theta),
                    x*sin(theta)  + y*cos(theta)] for x,y in path]
        else:
            path = [[ -(x/10.0) , -1/( 1 + math.exp(-(x/10)))] for x in range(0,30)]
            # rotate the path by theta degrees
            theta = 60.0 * degree_rad_conversion
            return [[x*cos(theta) - y*sin(theta),
                    x*sin(theta)  + y*cos(theta)] for x,y in path]