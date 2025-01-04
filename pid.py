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
        self.history_integration = []
        self.history_integration_length = ... # integration needs larger history, however if it is too large, it would create larger actuations, try different numbers here
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
        self.history_integration.append(stamped_error)
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)

        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error


        if (len(self.history_integration) > self.history_integration_length):
            self.history_integration.pop(0)
            
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
            
            # TODO Part 5: calculate the error dot 
            # error_dot+= ... 
            
        error_dot/=len(self.history)
        dt_avg/=len(self.history)
        
        # Compute the error integral
        sum_=0
        for hist in self.history_integration:
            # TODO Part 5: Gather the integration
            # sum_+=...
            pass
        
        error_int=sum_*dt_avg
        
        # TODO Part 4: Log your errors
        self.logger.log_values( ... )
        
        # TODO Part 4: Implement the control law of P-controller
        if self.type == P:
            return ... # complete
        
        # TODO Part 5: Implement the control law corresponding to each type of controller
        elif self.type == PD:
            pass
            # return ... # complete
        
        elif self.type == PI:
            pass
            # return ... # complete
        
        elif self.type == PID:
            pass
            # return ... # complete
