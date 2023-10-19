import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3
MAX_LIN_VEL = 0.31
MAX_ANG_VEL = 1.90

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(PID, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        linear_vel = MAX_LIN_VEL if linear_vel > MAX_LIN_VEL else linear_vel
        linear_vel = -MAX_LIN_VEL if linear_vel < -MAX_LIN_VEL else linear_vel
        angular_vel= MAX_ANG_VEL if angular_vel > MAX_ANG_VEL else angular_vel
        angular_vel= -MAX_ANG_VEL if angular_vel < -MAX_ANG_VEL else angular_vel

        print("linear vel is ", linear_vel)
        print("angular vel is ", linear_vel)

        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=1, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, lookAhead=1.0):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
        self.lookAhead=lookAhead
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        print(goal)

        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal) #avoids slowing down when reaching each way point (right???)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        linear_vel = MAX_LIN_VEL if linear_vel > MAX_LIN_VEL else linear_vel
        linear_vel = -MAX_LIN_VEL if linear_vel < -MAX_LIN_VEL else linear_vel
        angular_vel= MAX_ANG_VEL if angular_vel > MAX_ANG_VEL else angular_vel
        angular_vel= -MAX_ANG_VEL if angular_vel < -MAX_ANG_VEL else angular_vel
        
        #print("linear vel is ", linear_vel)
        #print("angular vel is ", linear_vel)

        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
