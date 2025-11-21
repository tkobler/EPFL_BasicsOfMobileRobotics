# code pour déterminer la matrice de variance de positionnement et de mesure du robot
# Ce code détermine d'abbord les valeurs de Q la variance de mouvenement (x, y, theta, vitesse gauche, vitesse droite)
# puis R la variance de mesure (x, y, theta, vitesse gauche, vitesse droite)
# La détermination de R est faite en deux étapes : une en utilisant le Thymio pour la vitesse et l'autre en utilisant les param de la caméra (laissé vide pour l'instant)

import asyncio
import math
import time
import numpy as np
from tdmclient import ClientAsync


# ======================================
# Dead reckoning state (odometry)
# ======================================
robot_state = {
    "x": 0.0,      # mm
    "y": 0.0,      # mm
    "theta": 0.0,  # radians
}

# ======================================
# Parameters (same as calibration scripts)
# ======================================
SPEED_TO_MMS = 0.032769  # mm/s per Thymio speed unit
L = 122.7                # wheelbase in mm

# Test motion parameters
TEST_SPEED_CMD = 200      # motor speed command
TEST_DURATION = 3.0       # duration of each test run in seconds
NUM_TRIALS = 10           # number of trials to collect data
DT_NOMINAL = 0.05         # nominal sleep between iterations (s)


def update_robot_state(v_l_raw, v_r_raw, dt):
    """
    Update x, y, theta using real units.
    v_l_raw and v_r_raw are Thymio motor speed units.
    dt is the *actual* time step in seconds.
    """
    # Convert raw wheel speeds to mm/s
    v_l = v_l_raw * SPEED_TO_MMS
    v_r = v_r_raw * SPEED_TO_MMS

    # Linear and angular velocity
    v = (v_r + v_l) / 2.0      # mm/s
    omega = (v_r - v_l) / L    # rad/s

    # Integrate position
    robot_state["x"] += v * math.cos(robot_state["theta"]) * dt
    robot_state["y"] += v * math.sin(robot_state["theta"]) * dt
    robot_state["theta"] += omega * dt

    # Normalize angle to [-pi, pi]
    robot_state["theta"] = (robot_state["theta"] + np.pi) % (2 * np.pi) - np.pi


def motors(left, right):
    """Helper to format motor commands."""
    return {
        "motor.left.target": [int(left)],
        "motor.right.target": [int(right)],
    }


async def run_single_trial(node, client, left_cmd, right_cmd, duration):
    """
    Run a single motion trial and return odometry estimate and speed data.
    
    Returns:
        dict: Contains 'x', 'y', 'theta', 'speeds_left', 'speeds_right'
    """
    # Reset odometry
    robot_state["x"] = 0.0
    robot_state["y"] = 0.0
    robot_state["theta"] = 0.0
    
    speeds_left = []
    speeds_right = []
    
    # Start motion
    await node.set_variables(motors(left_cmd, right_cmd))
    
    t_start = time.time()
    last_t = t_start
    
    while True:
        now = time.time()
        dt = now - last_t
        last_t = now
        
        # Read wheel speeds
        v_l = node["motor.left.speed"]
        v_r = node["motor.right.speed"]
        
        # Record speeds
        speeds_left.append(v_l)
        speeds_right.append(v_r)
        
        # Update odometry
        update_robot_state(v_l, v_r, dt)
        
        # Stop condition: duration elapsed
        if now - t_start >= duration:
            break
        
        await client.sleep(DT_NOMINAL)
    
    # Stop robot
    await node.set_variables(motors(0, 0))
    
    return {
        'x': robot_state['x'],
        'y': robot_state['y'],
        'theta': robot_state['theta'],
        'speeds_left': speeds_left,
        'speeds_right': speeds_right
    }


async def determine_Q(node, client):
    """
    Fonction pour déterminer la matrice Q (covariance du bruit de mouvement)
    à l'aide du Thymio et des mesures de l'utilisateur.
    
    Cette fonction effectue plusieurs essais de mouvement et compare les 
    estimations odométriques avec les mesures réelles fournies par l'utilisateur.
    
    Args:
        node: Thymio node object
        client: ClientAsync object
    
    Returns:
        numpy.ndarray: Matrice Q (5x5) de covariance du bruit de mouvement
    """
    # Ensure we see motor speeds
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
    
    print("=" * 60)
    print("DÉTERMINATION DE LA MATRICE Q (VARIANCE DE MOUVEMENT)")
    print("=" * 60)
    print(f"\nCe script effectue {NUM_TRIALS} essais de mouvement en ligne droite.")
    print(f"Chaque essai dure {TEST_DURATION:.1f} secondes.")
    print("Vous devrez mesurer la position finale réelle du robot après chaque essai.")
    print("\nPréparez:")
    print("  - Un mètre ruban ou une règle")
    print("  - Un rapporteur pour mesurer les angles")
    print("  - Une surface plane et dégagée")
    print("\nMarquez la position initiale du robot sur le sol.")
    
    input("\nAppuyez sur ENTRÉE pour commencer les essais...")
    
    # Collect data from multiple trials
    odom_x = []
    odom_y = []
    odom_theta = []
    real_x = []
    real_y = []
    real_theta = []
    all_speeds_left = []
    all_speeds_right = []
    
    for trial in range(NUM_TRIALS):
        print(f"\n{'=' * 60}")
        print(f"ESSAI {trial + 1}/{NUM_TRIALS}")
        print(f"{'=' * 60}")
        
        # Prompt user to reset robot position
        if trial > 0:
            input("Replacez le robot à sa position initiale marquée, puis appuyez sur ENTRÉE...")
        
        # Run trial
        print(f"Démarrage du mouvement (durée: {TEST_DURATION:.1f}s)...")
        result = await run_single_trial(node, client, TEST_SPEED_CMD, TEST_SPEED_CMD, TEST_DURATION)
        
        # Store odometry estimates
        odom_x.append(result['x'])
        odom_y.append(result['y'])
        odom_theta.append(result['theta'])
        all_speeds_left.extend(result['speeds_left'])
        all_speeds_right.extend(result['speeds_right'])
        
        theta_deg = result['theta'] * 180.0 / math.pi
        
        print(f"\nPosition estimée (odométrie):")
        print(f"  x = {result['x']:.1f} mm")
        print(f"  y = {result['y']:.1f} mm")
        print(f"  theta = {theta_deg:.1f}°")
        
        # Get user measurements
        print("\nMesurez maintenant la position RÉELLE du robot:")
        
        while True:
            try:
                x_str = input("  Position x (mm, mesurée depuis la position initiale): ").strip()
                real_x_val = float(x_str)
                break
            except ValueError:
                print("  Erreur: veuillez entrer un nombre valide.")
        
        while True:
            try:
                y_str = input("  Position y (mm, mesurée depuis la position initiale): ").strip()
                real_y_val = float(y_str)
                break
            except ValueError:
                print("  Erreur: veuillez entrer un nombre valide.")
        
        while True:
            try:
                theta_str = input("  Angle theta (degrés, positif = sens antihoraire): ").strip()
                real_theta_deg = float(theta_str)
                real_theta_val = math.radians(real_theta_deg)
                break
            except ValueError:
                print("  Erreur: veuillez entrer un nombre valide.")
        
        real_x.append(real_x_val)
        real_y.append(real_y_val)
        real_theta.append(real_theta_val)
        
        # Show difference
        diff_x = abs(result['x'] - real_x_val)
        diff_y = abs(result['y'] - real_y_val)
        diff_theta = abs(theta_deg - real_theta_deg)
        print(f"\nDifférence (|estimation - mesure|):")
        print(f"  Δx = {diff_x:.1f} mm")
        print(f"  Δy = {diff_y:.1f} mm")
        print(f"  Δtheta = {diff_theta:.1f}°")
    
    # Calculate variances
    print("\n" + "=" * 60)
    print("CALCUL DES VARIANCES")
    print("=" * 60)
    
    # Error arrays
    error_x = np.array(odom_x) - np.array(real_x)
    error_y = np.array(odom_y) - np.array(real_y)
    error_theta = np.array(odom_theta) - np.array(real_theta)
    
    # Variances for position and angle
    var_x = np.var(error_x)
    var_y = np.var(error_y)
    var_theta = np.var(error_theta)
    
    # Variances for wheel speeds
    var_speed_left = np.var(all_speeds_left)
    var_speed_right = np.var(all_speeds_right)
    # Average speed variance
    var_speed = (var_speed_left + var_speed_right) / 2.0
    
    print(f"\nVariances calculées:")
    print(f"  var(x)     = {var_x:.6f} mm²")
    print(f"  var(y)     = {var_y:.6f} mm²")
    print(f"  var(theta) = {var_theta:.6f} rad²")
    print(f"  var(speed_left)  = {var_speed_left:.6f}")
    print(f"  var(speed_right) = {var_speed_right:.6f}")
    print(f"  var(speed_avg)   = {var_speed:.6f}")
    
    # Construct Q matrix (diagonal covariance matrix)
    Q = np.diag([var_x, var_y, var_theta, var_speed, var_speed])
    
    print(f"\nMatrice Q (covariance du bruit de mouvement):")
    print(Q)
    
    print(f"\nValeurs à copier dans constants.py:")
    print(f"X_MOVE_UNCERT = {var_x:.6f}  # uncertainty in x direction (movement noise)")
    print(f"Y_MOVE_UNCERT = {var_y:.6f}  # uncertainty in y direction (movement noise)")
    print(f"THETA_MOVE_UNCERT = {var_theta:.6f}  # uncertainty in theta (movement noise)")
    print(f"SPEED_MOVE_UNCERT = {var_speed:.6f}  # uncertainty in wheel speed (movement noise)")
    
    # Statistical summary
    print(f"\n" + "=" * 60)
    print("RÉSUMÉ STATISTIQUE")
    print("=" * 60)
    print(f"Nombre d'essais: {NUM_TRIALS}")
    print(f"Erreur moyenne x: {np.mean(error_x):.2f} mm (std: {np.std(error_x):.2f} mm)")
    print(f"Erreur moyenne y: {np.mean(error_y):.2f} mm (std: {np.std(error_y):.2f} mm)")
    print(f"Erreur moyenne theta: {np.mean(error_theta)*180/np.pi:.2f}° (std: {np.std(error_theta)*180/np.pi:.2f}°)")
    
    return Q


async def main():
    """Point d'entrée principal du script."""
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()
    
    Q = await determine_Q(node, client)
    
    print("\n" + "=" * 60)
    print("Détermination de Q terminée.")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())

