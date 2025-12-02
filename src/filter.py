#import necessary libraries
import numpy as np
import constants as cte

class Filter:
    def __init__(self, initial_pos):
        self.state = np.array(initial_pos + [0, 0]) # current position estimate (x, y, theta, left_speed, right_speed)
        self.var = np.diag([0.01, 0.01, 0.01, 0.01, 0.01]) # variance of the estimation #? on est très confiant de la première mesure prise par la caméra pour la position initiale du robot
        self.A = None  # state transition matrix
        self.J = None  # Jacobian matrix
        self.C = np.eye(5)  # observation matrix
        self.Q = np.diag([cte.X_MOVE_UNCERT, cte.Y_MOVE_UNCERT, cte.THETA_MOVE_UNCERT, cte.SPEED_MOVE_UNCERT_L, cte.SPEED_MOVE_UNCERT_R])  # displacement noise covariance  
        self.R = None  # measurement noise covariance
        self.R_with_camera = np.diag([cte.X_MEAS_UNCERT, cte.Y_MEAS_UNCERT, cte.THETA_MEAS_UNCERT, cte.SPEED_MES_UNCERT_L, cte.SPEED_MES_UNCERT_R])  # If camera feed is available  
        self.R_no_camera = np.diag([np.inf, np.inf, np.inf, cte.SPEED_MES_UNCERT_L, cte.SPEED_MES_UNCERT_R])  # If no camera feed we set high uncertainty so the measurment is ignored

    def update(self, measured_pos, Ts, ls, rs):
        """
        Position Estimation Filter
        
        param thymio_pos: actuator values for forward motion (x,y,theta)
        param measured_pos: measured position along x-axis from camera (x,y,theta)
        param Ts: time elapsed since last measure in s
        param ls: left wheel speed in mm/s
        param rs: right wheel speed in mm/s
        
        return estim_pos: new a posteriori state estimation
        """
        # Check_for_camera and change R accordingly
        if measured_pos == []:
            print("No camera measurement available, relying on odometry only.")
            self.R = self.R_no_camera
            measured_pos = [-1, -1, -1]  # Dummy values since we won't use them
        else:
            self.R = self.R_with_camera
        
        # Convert function inputs to numpy arrays
        state_meas = np.array(measured_pos + [ls, rs])  # measurement vector (x, y, theta, left_speed, right_speed)
        theta = self.state[2]

        # State transition matrix
        # Angle theta is in radians and CCW positive
        self.A = np.array([[1, 0, 0, Ts/2*np.cos(theta), Ts/2*np.cos(theta)],
                           [0, 1, 0, Ts/2*np.sin(theta), Ts/2*np.sin(theta)],
                           [0, 0, 1, -Ts/cte.WHEELBASE, Ts/cte.WHEELBASE],
                           [0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])

        self.J = np.array([[1, 0, -np.sin(theta)*Ts*(ls-rs)/2, Ts/2*np.cos(theta), Ts/2*np.cos(theta)],
                           [0, 1, np.cos(theta)*Ts*(ls-rs)/2, Ts/2*np.sin(theta), Ts/2*np.sin(theta)],
                           [0, 0, 1, -Ts/cte.WHEELBASE, Ts/cte.WHEELBASE],
                           [0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])
    

        # Compute prediciton
        state_pred = self.A@self.state
        var_pred = self.J@self.var@self.J.T + self.Q

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
        return self.state[:3].tolist(), np.diag(self.var)[:3].tolist()  # return only the position (x, y, theta) in a normal python list format

    def print_filtering(self):
        return

# test code
# you can run python filter.py to test a specific function without compiling the entire project
if __name__ == "__main__": 
    filter_instance = Filter()

