# Basic file for defining cte values used in the project


# ======================================
# Motion of the Thymio robot
# ======================================
SPEED_TO_MMS = 0.032769  # current guess: mm/s per Thymio speed unit
WHEELBASE = 122.7  # wheelbase in mm (same as in Navigation.py)

CALIB_LEFT_WHEEL = 1.05
CALIB_RIGHT_WHEEL = 1.0


# ======================================
# Uncertainties for the Kalman Filter
# ======================================
X_UNCERT = 0.1  # uncertainty in x direction (movment noise)
Y_UNCERT = 0.1  # uncertainty in y direction (movment noise)
THETA_UNCERT = 0.05  # uncertainty in theta (movment noise)

CAM_X_UNCERT = 0.5  # uncertainty in x direction from camera (measurement noise)
CAM_Y_UNCERT = 0.5  # uncertainty in y direction from camera (measurement noise)
CAM_THETA_UNCERT = 0.1  # uncertainty in theta from camera (measurement noise)

