import math
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, current_pose, goalPoint=[-1.0, -1.0, 0.0] ):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(current_pose)

    #is goal point relative or absolute??
    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, current_pose):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        mode = 'sigmoid'
        if mode ==  'x^2':
                num_points = 10
                x_inc = 0.1
                trajectory_points = []

                #starting x
                x = 0

                for i in range(num_points):
                    y = x*x
                    trajectory_points.append([x, y])
                    x += x_inc

        if mode == 'sigmoid':
            x_start = -50
            x_end = 1

            increment = 0.1

            trajectory_points = []

            x = x_start
            while (x <= x_end):
                y = 1 / (1 + math.exp(-10 * x))

                trajectory_points.append([x, y])

                x += increment

        print (trajectory_points)

        trajectory_points_transformed = []

        for point in trajectory_points:
            
            x_transformed = (point[0] * math.cos(current_pose[2])) - (point[1] * math.sin(current_pose[2])) + current_pose[0]
            y_transformed = (point[1] * math.cos(current_pose[2])) + (point[0] * math.sin(current_pose[2])) + current_pose[1]
            trajectory_points_transformed.append([x_transformed, y_transformed])

        print(trajectory_points_transformed)

        return trajectory_points_transformed

