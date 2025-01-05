
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np

class particle:

    def __init__(self, pose, weight):
        self.pose=pose
        self.weight=weight


    # TODO Part 4: Complete the motion model equation 
    # for each particle Using the kinematic bicycle-
    # model (v-omega)
    def motion_model(self,dth, v, w, dt):
        self.pose[0]=...
        self.pose[1]=...
        self.pose[2]=dth + self.pose[2]

    # TODO Part 4: Read the following function and explain to the TA
    # how did we calculate each particle weight, and why we should 
    # use log.
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator):
        
        T = self.__poseToTranslationMatrix()

        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        scanInMap = np.dot(T, scanCartesianHomo.T).T
        
        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        
        origin = mapManipulatorInstance.getOrigin()
        res    = mapManipulatorInstance.getResolution()
        w      = mapManipulatorInstance.width
        h      = mapManipulatorInstance.height
        cellOrigin    = position_2_cell(np.array([0,0]), origin, res, h)
        cellPositions = position_2_cell(scanInMap[:,0:2], origin, res, h)
        cellParticle  = position_2_cell(self.getPose()[0:2], origin, res, h)

        lm_x, lm_y = likelihoodField.shape

        cellPositions = cellPositions[
            np.logical_and(cellPositions[:,0] < lm_y , -cellPositions[:,1] < lm_x), :
        ]

        log_weights = np.log(likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight)
        weight+=1.e-300

        self.setWeight(weight)

    
    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]
    

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        
        translation = np.array([[cos(th), -sin(th),x],
                                [sin(th), cos(th),y],
                                [0,0,1]])

        return translation
