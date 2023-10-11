# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(goalPoint)


    def point_planner(self, goalPoint):
        return goalPoint

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, goalPoint):
        pass
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

