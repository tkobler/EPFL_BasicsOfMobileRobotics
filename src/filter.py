#import necessary libraries
import numpy as np
import constants as cte

class Filter:
    def __init__(self, initial_pos):
        self.state = np.array(initial_pos + [0, 0]) # current position estimate (x, y, theta, left_speed, right_speed)
        self.var = np.diag([0.01, 0.01, 0.01, 0.01, 0.01]) # variance of the estimation #? on est très confiant de la première mesure prise par la caméra pour la position initiale du robot
        self.A = None  # state transition matrix
        self.B = None  # control input matrix
        self.C = np.eye(5)  # observation matrix
        #TODO : à ajuster avec des valeurs à tester
        self.Q = np.diag([cte.X_MOVE_UNCERT, cte.Y_MOVE_UNCERT, cte.THETA_MOVE_UNCERT, cte.SPEED_MOVE_UNCERT, cte.SPEED_MOVE_UNCERT])  # displacement noise covariance  
        self.R = None  # measurement noise covariance
        #TODO : à ajuster avec des valeurs à tester
        self.R_with_camera = np.diag([cte.X_MEAS_UNCERT, cte.Y_MEAS_UNCERT, cte.THETA_MEAS_UNCERT, cte.SPEED_MES_UNCERT, cte.SPEED_MES_UNCERT])  # If camera feed is available  
        self.R_no_camera = np.diag([np.inf, np.inf, np.inf, cte.SPEED_MES_UNCERT, cte.SPEED_MES_UNCERT])  # If no camera feed we set high uncertainty so the measurment is ignored


    def update(self, measured_pos, delta_t, left_speed, right_speed, left_command, right_command):
        """
        Position Estimation Filter
        
        param thymio_pos: actuator values for forward motion (x,y,theta)
        param measured_pos: measured position along x-axis from camera (x,y,theta)
        param delta_t: time elapsed since last measure in s
        param left_speed: left wheel speed in mm/s
        param right_speed: right wheel speed in mm/s
        param left_command: last commanded left wheel speed in mm/s
        param right_command: last commanded right wheel speed in mm/s
        
        return estim_pos: new a posteriori state estimation
        """
        # Check_for_camera and change R accordingly
        if measured_pos is None:
            self.R = self.R_no_camera
        else:
            self.R = self.R_with_camera
        
        # Convert function inputs to numpy arrays
        state_meas = np.array(measured_pos + [left_speed, right_speed])  # measurement vector (x, y, theta, left_speed, right_speed)
        command = np.array([left_command-cte.MAIN_MOTOR_SPEED, right_command-cte.MAIN_MOTOR_SPEED]) # displacement from wheel speeds


        #TODO : bien définir le sens de rotation du robot (pour le moment c'est CW positif) -> POUR A ET B
        self.A = np.array([[1, 0, 0, np.cos(self.state[2])/2*delta_t, np.cos(self.state[2])/2*delta_t],
                           [0, 1, 0, np.sin(self.state[2])/2*delta_t, np.sin(self.state[2])/2*delta_t],
                           [0, 0, 1, 1/cte.WHEELBASE*delta_t, - 1/cte.WHEELBASE*delta_t],
                           [0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])
        
        self.B = np.array([[np.cos(self.state[2])/2*delta_t, np.cos(self.state[2])/2*delta_t],
                           [np.sin(self.state[2])/2*delta_t, np.sin(self.state[2])/2*delta_t],
                           [1/cte.WHEELBASE*delta_t, - 1/cte.WHEELBASE*delta_t],
                           [1, 0],
                           [0, 1]])

        # Compute prediciton
        state_pred = self.A@self.state + self.B@command
        var_pred = self.A@self.var@self.A.T + self.Q

        # Compute innovation
        innov = state_meas - self.C@state_pred # innovation 
        S = self.C@var_pred@self.C.T + self.R # covariance of innovation

        # Compute Kalman Gain
        K = var_pred@self.C.T@np.linalg.inv(S)

        # Update estimate with measurement
        state_estim = state_pred + K@innov
        var_estim = (np.eye(len(K)) - K@self.C)@var_pred

        self.state = state_estim
        self.var = var_estim

        # apply the filter to the data
        return self.state[:3].tolist()  # return only the position (x, y, theta) in a normal python list format

    def print_filtering(self):
        return

# test code
# you can run python filter.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    filter_instance = Filter()

