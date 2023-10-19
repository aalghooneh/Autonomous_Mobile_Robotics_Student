from rclpy.time import Time
from utilities import Logger

# Controller type
P=0 # poportional
PD=1 # proportional and derivative
PI=2 # proportional and integral
PID=3 # proportional, integral, derivative

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3, filename_="errors.csv"):
        
        # Data for the controller
        self.history_length=history_length
        self.history=[]
        self.type=type_

        # Controller gains
        self.kp=kp    # proportional gain
        self.kv=kv    # derivative gain
        self.ki=ki    # integral gain
        
        self.logger=Logger(filename_)
        # Remeber that you are writing to the file named filename_ or errors.csv the following:
            # error, error_dot, error_int and time stamp

    
    def update(self, stamped_error, status):
        
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

        
    def __update(self, stamped_error):
        
        latest_error=stamped_error[0]
        stamp=stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg=0
        error_dot=0
        
        for i in range(1, len(self.history)):
            
            t0=Time.from_msg(self.history[i-1][1])
            t1=Time.from_msg(self.history[i][1])
            
            dt=(t1.nanoseconds - t0.nanoseconds) / 1e9
            
            dt_avg+=dt

            # use constant dt if the messages arrived inconsistent
            # for example dt=0.1 overwriting the calculation  

            #DOUBLE CHECK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if dt-dt_avg/i > 0.5:
                dt=0.1
            
            # TODO Part 5: calculate the error dot 
            error_dot+=(self.history[i][0] - self.history[i-1][0])/dt
            
        error_dot/=len(self.history)
        dt_avg/=len(self.history)
        
        # Compute the error integral
        sum_=0
        for hist in self.history:
            # TODO Part 5: Gather the integration
            sum_+=hist[0]
        
        error_int=sum_*dt_avg
        
        # TODO Part 4: Log your errors
        self.logger.log_values([latest_error, error_dot, error_int, stamp])
        
        # TODO Part 4: Implement the control law of P-controller
        if self.type == P:
            return self.kp * latest_error
        
        # TODO Part 5: Implement the control law corresponding to each type of controller
        elif self.type == PD:
            return self.kp * latest_error + self.kv * error_dot
        
        elif self.type == PI:
            return self.kp * latest_error + self.ki * error_int
        
        elif self.type == PID:
            return self.kp * latest_error + self.ki * error_int + self.kv * error_dot
