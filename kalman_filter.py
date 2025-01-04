import numpy as np

class kalman_filter:
    
    
    def __init__(self, P,Q,R, x):
        
        self.P=P
        self.Q=Q
        self.R=R
        self.x=x

        
        
    def predict(self, dt):
        
        self.dt = dt

        self.A = self.jacobian_A()
        self.C = self.jacobian_H()
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q
    
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    

    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,
            w,
            vdot,
            v*w
        ])
    
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        

    def get_states(self):
        return self.x
    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0, -v*np.sin(th)*dt, 0, np.cos(th)*dt, 0],
            [0, 1,  v*np.cos(th)*dt, 0, np.sin(th)*dt, 0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0],
            [0,0,0  , 1, 0, 0],
            [0,0,0  , 0, 0, 1],
            [0,0,0  , v, w, 0],
        ])
        
