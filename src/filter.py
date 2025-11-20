#import necessary libraries
import numpy as np
import constants as cte

class Filter:
    def __init__(self, initial_pos):
        self.pos = np.array(initial_pos) # current position estimate
        self.var = np.diag([0.1, 0.1, 0.05]) # variance of the estimation #TODO : à ajuster avec des valeurs à tester
        self.A = np.eye(3)  # state transition matrix
        self.B = None  # control input matrix
        self.C = np.eye(3)  # observation matrix
        self.Q = np.diag([cte.X_UNCERT, cte.Y_UNCERT, cte.THETA_UNCERT])  # displacement noise covariance  #TODO : à ajuster avec des valeurs à tester
        self.R = None  # measurement noise covariance
        self.R_with_camera = np.diag([cte.CAM_X_UNCERT, cte.CAM_Y_UNCERT, cte.CAM_THETA_UNCERT])  # If camera feed is available  #TODO : à ajuster avec des valeurs à tester
        self.R_no_camera = np.diag([np.inf, np.inf, np.inf])  # If no camera feed we set high uncertainty so the measurment is ignored


    def update(self, measured_pos, delta_t, left_speed, right_speed):
        """
        Position Estimation Filter
        
        param thymio_pos: actuator values for forward motion (x,y,theta)
        param measured_pos: measured position along x-axis from camera (x,y,theta)
        param delta_t: time elapsed since last measure in s
        param left_speed: left wheel speed in mm/s
        param right_speed: right wheel speed in mm/s
        
        return estim_pos: new a posteriori state estimation
        """
        # Check_for_camera and change R accordingly
        if measured_pos is None:
            self.R = self.R_no_camera
        else:
            self.R = self.R_with_camera
        
        # Convert function inputs to numpy arrays
        meas_pos = np.array(measured_pos)
        disp = np.array([left_speed * delta_t, right_speed * delta_t]) # displacement from wheel speeds
        self.B = np.array([[np.cos(self.pos[2])/2, np.cos(self.pos[2])/2],
                           [np.sin(self.pos[2])/2, np.sin(self.pos[2])/2],
                           [1/(2*cte.WHEELBASE), - 1/(2*cte.WHEELBASE)]])

        # Compute prediciton
        pos_pred = self.A@self.pos + self.B@disp
        var_pred = self.A@self.var@self.A.T + self.Q

        # Compute innovation
        innov = meas_pos - self.C@pos_pred # innovation 
        S = self.C@var_pred@self.C.T + self.R # covariance of innovation

        # Compute Kalman Gain
        K = var_pred@self.C.T@np.linalg.inv(S)

        # Update estimate with measurement
        pos_estim = pos_pred + K@innov
        var_estim = (np.eye(len(K)) - K@self.C)@var_pred

        self.pos = pos_estim
        self.var = var_estim

        # apply the filter to the data
        return self.pos
    
    #getters and setters can be added as needed



# test code
# you can run python filter.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    filter_instance = Filter()

