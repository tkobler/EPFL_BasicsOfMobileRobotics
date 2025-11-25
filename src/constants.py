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
ENV_LARGEUR = 1189  # Largeur de l'environnement 
ENV_LONGUEUR = 841  # Longueur de l'environnement 
LARGEUR_MARQUEUR = 70  # Largeur d'un marqueur ArUco

# ======================================
# Uncertainties for the Kalman Filter
# ======================================
# Matrice Q : process noise covariance matrix
X_MOVE_UNCERT = 0.1  # uncertainty in x direction (movment noise)
Y_MOVE_UNCERT = 0.1  # uncertainty in y direction (movment noise)
THETA_MOVE_UNCERT = 0.05  # uncertainty in theta (movment noise)
SPEED_MOVE_UNCERT = 1.0  # uncertainty in wheel speed (movment noise)

# Matrice R : measurement noise covariance matrix
X_MEAS_UNCERT = 0.1  # uncertainty in x direction from camera (measurement noise)
Y_MEAS_UNCERT = 0.1  # uncertainty in y direction from camera (measurement noise)
THETA_MEAS_UNCERT = 0.9  # uncertainty in theta from camera (measurement noise)
SPEED_MES_UNCERT = 5.0  # uncertainty in wheel speed measurement