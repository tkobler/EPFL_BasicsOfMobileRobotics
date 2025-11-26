# Basic file for defining cte values used in the project
# ======================================
# Motion of the Thymio robot
# ======================================
SPEED_TO_MMS = 0.032769  # current guess: mm/s per Thymio speed unit
WHEELBASE = 122.7  # wheelbase in mm (same as in Navigation.py)
CALIB_LEFT_WHEEL = 1.05
CALIB_RIGHT_WHEEL = 1.0
MAIN_MOTOR_SPEED = 200  # main motor speed command in Thymio units
KIDNAP_THRESHOLD = 100  # threshold in mm for kidnapping detection

# ======================================
# Vision environment config
# ======================================
# Constantes de dimensions (en mm)
ENV_LARGEUR = 1150  # Largeur de l'environnement 
ENV_LONGUEUR = 820  # Longueur de l'environnement 
LARGEUR_MARQUEUR = 67  # Largeur d'un marqueur ArUco

# ======================================
# Uncertainties for the Kalman Filter
# ======================================
# Matrice Q : process noise covariance matrix
X_MOVE_UNCERT = 0.17  #[mm²] uncertainty in x direction (movment noise) #TODO
Y_MOVE_UNCERT = 0.17  #[mm²] uncertainty in y direction (movment noise) #TODO
THETA_MOVE_UNCERT = 0.05  #[rad²] uncertainty in theta (movment noise) #TODO
SPEED_MOVE_UNCERT_L = 0.43  #[ (mm/s)² ] uncertainty in left wheel speed (movment noise) #TODO
SPEED_MOVE_UNCERT_R = 0.45  #[ (mm/s)² ] uncertainty in right wheel speed (movment noise) #TODO
# Matrice R : measurement noise covariance matrix
X_MEAS_UNCERT = 0.25  #[mm²] uncertainty in x direction from camera (measurement noise)
Y_MEAS_UNCERT = 0.25  #[mm²] uncertainty in y direction from camera (measurement noise)
THETA_MEAS_UNCERT = 0.0007  #[rad²] uncertainty in theta from camera (measurement noise)
SPEED_MES_UNCERT_L = 3.46  #[ (mm/s)² ] uncertainty in left wheel speed measurement
SPEED_MES_UNCERT_R = 5.05  #[ (mm/s)² ] uncertainty in right wheel speed measurement

# ======================================
# Global Nav
# ======================================
# Constantes de dimensions (en mm)
INFLATION_R = 140
GOAL_TOL = 20
EPSILON = 1e-9