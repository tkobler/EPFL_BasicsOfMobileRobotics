# code pour déterminer la matrice de variance de positionnement et de mesure du robot
# Ce code détermine d'abord les valeurs de Q la variance de mouvement (x, y, theta, vitesse gauche, vitesse droite)
# puis R la variance de mesure (x, y, theta, vitesse gauche, vitesse droite)
# La détermination de R est faite en deux étapes : une en utilisant le Thymio pour la vitesse et l'autre en utilisant les param de la caméra (laissé vide pour l'instant)

import asyncio
import math
import time
import numpy as np
from tdmclient import ClientAsync
from vision import Map



# ======================================
# Paramètres de calibration
# ======================================
SPEED_TO_MMS = 0.32769  # mm/s par unité de vitesse Thymio
WHEELBASE = 122.7  # empattement en mm
CALIB_LEFT_WHEEL = 1.05
CALIB_RIGHT_WHEEL = 1.0

# ======================================
# État du robot (odométrie)
# ======================================
robot_state = {
    "x": 0.0,      # mm
    "y": 0.0,      # mm
    "theta": 0.0,  # radians
}


def update_robot_state(v_l_raw, v_r_raw, dt):
    """
    Met à jour x, y, theta en utilisant les vraies unités.
    v_l_raw et v_r_raw sont les unités de vitesse moteur Thymio.
    dt est le pas de temps réel en secondes.
    """
    # Convertir les vitesses des roues en mm/s
    v_l = v_l_raw * SPEED_TO_MMS
    v_r = v_r_raw * SPEED_TO_MMS

    # Vitesse linéaire et angulaire
    v = (v_r + v_l) / 2.0      # mm/s
    omega = (v_r - v_l) / WHEELBASE    # rad/s

    # Intégrer la position
    robot_state["x"] += v * math.cos(robot_state["theta"]) * dt
    robot_state["y"] += v * math.sin(robot_state["theta"]) * dt
    robot_state["theta"] += omega * dt

    # Normaliser l'angle à [-pi, pi]
    robot_state["theta"] = (robot_state["theta"] + np.pi) % (2 * np.pi) - np.pi


def motors(left, right):
    return {
        "motor.left.target": [int(left * CALIB_LEFT_WHEEL)],
        "motor.right.target": [int(right * CALIB_RIGHT_WHEEL)],
    }


async def calibrate_Q_matrix(node, client):
    """
    Fonction principale pour déterminer la matrice Q (covariance du bruit de processus).
    
    Cette fonction effectue plusieurs tests de mouvement avec le Thymio et demande à l'utilisateur
    de mesurer les positions réelles. En comparant les estimations odométriques avec les mesures
    réelles, on calcule les variances du bruit de processus.
    
    La matrice Q est diagonale avec 5 éléments:
    - Q[0,0]: variance sur x (mm²)
    - Q[1,1]: variance sur y (mm²)
    - Q[2,2]: variance sur theta (rad²)
    - Q[3,3]: variance sur vitesse roue gauche ((mm/s)²)
    - Q[4,4]: variance sur vitesse roue droite ((mm/s)²)
    
    Returns:
        numpy.ndarray: Matrice Q 5x5 diagonale
    """
    
    print("=" * 60)
    print("CALIBRATION DE LA MATRICE Q (bruit de processus)")
    print("=" * 60)
    print("\nCette procédure va effectuer plusieurs tests pour déterminer")
    print("la variance du modèle de mouvement du robot.")
    print("\nVous aurez besoin de:")
    print("  - Un mètre ruban ou règle")
    print("  - Un rapporteur ou moyen de mesurer des angles")
    print("  - Une surface plane et dégagée")
    print()
    
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
    
    # ======================================
    # TEST 1: Mouvement en ligne droite (X, Y, et vitesses)
    # ======================================
    print("\n" + "=" * 60)
    print("TEST 1: MOUVEMENT EN LIGNE DROITE")
    print("=" * 60)
    print("\nCe test mesure l'incertitude sur X, Y et les vitesses lors d'un mouvement droit.")
    
    num_trials_straight = int(input("\nCombien de répétitions pour le test en ligne droite ? (recommandé: 5-10): ") or "5")
    test_distance_mm = float(input("Distance cible pour chaque essai (mm) ? (recommandé: 500): ") or "500")
    motor_speed = int(input("Vitesse des moteurs (unités Thymio) ? (recommandé: 200): ") or "200")
    
    x_errors = []
    y_errors = []
    speed_errors_left = []
    speed_errors_right = []
    
    for trial in range(num_trials_straight):
        print(f"\n--- Essai {trial + 1}/{num_trials_straight} ---")
        input(f"Placez le robot au point de départ, aligné droit. Appuyez sur ENTRÉE pour démarrer...")
        
        # Réinitialiser l'odométrie
        robot_state["x"] = 0.0
        robot_state["y"] = 0.0
        robot_state["theta"] = 0.0
        
        # Démarrer le mouvement et le chronomètre
        await node.set_variables(motors(motor_speed, motor_speed))
        
        t_start = time.time()
        last_t = t_start
        
        # Collecter les vitesses moyennes pendant le mouvement
        speed_samples_left = []
        speed_samples_right = []
        
        # Mouvement jusqu'à atteindre la distance cible (selon odométrie)
        while True:
            now = time.time()
            dt = now - last_t
            last_t = now
            
            v_l = node["motor.left.speed"]
            v_r = node["motor.right.speed"]
            
            speed_samples_left.append(v_l * SPEED_TO_MMS)
            speed_samples_right.append(v_r * SPEED_TO_MMS)
            
            update_robot_state(v_l, v_r, dt)
            
            dist_odom = math.hypot(robot_state["x"], robot_state["y"])
            
            if dist_odom >= test_distance_mm:
                break
            
            await client.sleep(0.05)
        
        # Arrêter le robot et le chronomètre
        await node.set_variables(motors(0, 0))
        t_end = time.time()
        elapsed_time = t_end - t_start
        
        # Vitesses moyennes prédites par le modèle
        avg_speed_left_model = np.mean(speed_samples_left)
        avg_speed_right_model = np.mean(speed_samples_right)
        
        print(f"\nPosition finale (odométrie):")
        print(f"  x = {robot_state['x']:.1f} mm")
        print(f"  y = {robot_state['y']:.1f} mm")
        print(f"  theta = {robot_state['theta'] * 180 / np.pi:.1f}°")
        print(f"\nTemps écoulé: {elapsed_time:.2f} s")
        print(f"Vitesse moyenne prédite (gauche): {avg_speed_left_model:.2f} mm/s")
        print(f"Vitesse moyenne prédite (droite): {avg_speed_right_model:.2f} mm/s")
        
        # Demander les mesures réelles de position
        real_x = float(input(f"\nMesurez la distance RÉELLE en X parcourue (mm) [devrait être proche de {test_distance_mm:.0f}]: "))
        real_y = float(input("Mesurez la déviation latérale réelle en Y (mm): "))
        
        # Calculer les erreurs de position
        x_errors.append(real_x - robot_state["x"])
        y_errors.append(real_y - robot_state["y"])
        
        # Calculer la vitesse réelle basée sur la mesure de l'utilisateur
        real_speed = real_x / elapsed_time
        
        # Calculer les erreurs de vitesse (modèle vs réalité)
        speed_errors_left.append(real_speed - avg_speed_left_model)
        speed_errors_right.append(real_speed - avg_speed_right_model)
        
        print(f"\nRésultats de l'essai {trial + 1}:")
        print(f"  Erreur X: {x_errors[-1]:.2f} mm")
        print(f"  Erreur Y: {y_errors[-1]:.2f} mm")
        print(f"  Vitesse réelle: {real_speed:.2f} mm/s")
        print(f"  Erreur modèle vitesse gauche: {speed_errors_left[-1]:.2f} mm/s")
        print(f"  Erreur modèle vitesse droite: {speed_errors_right[-1]:.2f} mm/s")
    
    # Calculer les variances pour X, Y et vitesses
    var_x = np.var(x_errors)
    var_y = np.var(y_errors)
    var_speed_left = np.var(speed_errors_left)
    var_speed_right = np.var(speed_errors_right)
    
    print(f"\n=== Résultats TEST 1 ===")
    print(f"Variance X: {var_x:.4f} mm²")
    print(f"Variance Y: {var_y:.4f} mm²")
    print(f"Écart-type X: {np.sqrt(var_x):.2f} mm")
    print(f"Écart-type Y: {np.sqrt(var_y):.2f} mm")
    print(f"\nVariance erreur vitesse gauche: {var_speed_left:.4f} (mm/s)²")
    print(f"Variance erreur vitesse droite: {var_speed_right:.4f} (mm/s)²")
    print(f"Écart-type erreur gauche: {np.sqrt(var_speed_left):.2f} mm/s")
    print(f"Écart-type erreur droite: {np.sqrt(var_speed_right):.2f} mm/s")
    
    # ======================================
    # TEST 2: Rotation sur place
    # ======================================
    print("\n" + "=" * 60)
    print("TEST 2: ROTATION SUR PLACE")
    print("=" * 60)
    print("\nCe test mesure l'incertitude sur l'angle theta.")
    
    num_trials_rotation = int(input("\nCombien de répétitions pour le test de rotation ? (recommandé: 5-10): ") or "5")
    target_angle_deg = float(input("Angle de rotation cible (degrés) ? (recommandé: 90 ou 180): ") or "90")
    target_angle_rad = target_angle_deg * np.pi / 180
    rotation_speed = int(input("Vitesse de rotation (unités Thymio) ? (recommandé: 100): ") or "100")
    
    theta_errors = []
    
    for trial in range(num_trials_rotation):
        print(f"\n--- Essai {trial + 1}/{num_trials_rotation} ---")
        input(f"Placez le robot au centre. Notez sa direction initiale. Appuyez sur ENTRÉE pour démarrer...")
        
        # Réinitialiser l'odométrie
        robot_state["x"] = 0.0
        robot_state["y"] = 0.0
        robot_state["theta"] = 0.0
        
        # Démarrer la rotation (une roue en avant, une en arrière)
        await node.set_variables(motors(rotation_speed, -rotation_speed))
        
        t_start = time.time()
        last_t = t_start
        
        # Rotation jusqu'à atteindre l'angle cible (selon odométrie)
        while True:
            now = time.time()
            dt = now - last_t
            last_t = now
            
            v_l = node["motor.left.speed"]
            v_r = node["motor.right.speed"]
            
            update_robot_state(v_l, v_r, dt)
            
            if abs(robot_state["theta"]) >= abs(target_angle_rad):
                break
            
            await client.sleep(0.05)
        
        # Arrêter le robot
        await node.set_variables(motors(0, 0))
        
        print(f"\nAngle final (odométrie): {robot_state['theta'] * 180 / np.pi:.1f}°")
        
        # Demander la mesure réelle
        real_theta_deg = float(input(f"Mesurez l'angle réel de rotation (degrés) [devrait être proche de {target_angle_deg:.0f}]: "))
        real_theta_rad = real_theta_deg * np.pi / 180
        
        # Calculer l'erreur
        theta_errors.append(real_theta_rad - robot_state["theta"])
        
        print(f"Erreur theta: {theta_errors[-1] * 180 / np.pi:.2f}°")
    
    # Calculer la variance pour theta
    var_theta = np.var(theta_errors)
    
    print(f"\n=== Résultats TEST 2 ===")
    print(f"Variance theta: {var_theta:.6f} rad²")
    print(f"Écart-type theta: {np.sqrt(var_theta) * 180 / np.pi:.2f}°")
    
    # ======================================
    # CONSTRUCTION DE LA MATRICE Q
    # ======================================
    print("\n" + "=" * 60)
    print("RÉSULTATS FINAUX - MATRICE Q")
    print("=" * 60)
    
    # Normaliser les variances par unité de temps ou de distance
    # Pour un mouvement typique, on peut normaliser par le temps de test
    # Ici on utilise les valeurs brutes comme estimation
    
    Q = np.diag([var_x, var_y, var_theta, var_speed_left, var_speed_right])
    
    print("\nMatrice Q (covariance du bruit de processus):")
    print(Q)
    
    print("\n\nValeurs pour constants.py:")
    print(f"X_MOVE_UNCERT = {var_x:.6f}  # mm²")
    print(f"Y_MOVE_UNCERT = {var_y:.6f}  # mm²")
    print(f"THETA_MOVE_UNCERT = {var_theta:.6f}  # rad²")
    print(f"SPEED_MOVE_UNCERT = {(var_speed_left + var_speed_right) / 2:.6f}  # (mm/s)²")
    
    print("\n\nCode Python pour la matrice Q:")
    print(f"Q = np.diag([{var_x:.6f}, {var_y:.6f}, {var_theta:.6f}, "
          f"{var_speed_left:.6f}, {var_speed_right:.6f}])")
    
    return Q


async def calibrate_R_camera():
    """
    Fonction pour déterminer la variance R des mesures de position par caméra.
    
    Cette fonction mesure la variance des CAPTEURS de position (x, y, theta) 
    fournis par la caméra. Le robot reste statique pendant que nous collectons
    de nombreuses mesures pour calculer leur variance (bruit de mesure).
    
    Returns:
        tuple: (var_x_camera, var_y_camera, var_theta_camera) - variances des mesures 
               de position par caméra en (mm², mm², rad²)
    """
    
    print("\n" + "=" * 60)
    print("CALIBRATION DE R - VARIANCE DES MESURES CAMÉRA")
    print("=" * 60)
    print("\nCette procédure mesure la variance des CAPTEURS de position par caméra.")
    print("Le robot restera statique pendant que nous collectons de nombreuses mesures.")
    print("\nVous aurez besoin de:")
    print("  - Caméra correctement positionnée au-dessus de la zone")
    print("  - Les 4 marqueurs ArUco de référence (ID 0-3) visibles")
    print("  - Le Thymio avec son marqueur ArUco (ID 4) sur la carte")
    print("  - Une position stable pour le robot (ne pas le toucher!)")
    print()
    
    # Initialiser la caméra
    print("Initialisation de la caméra...")
    carte = Map()
    carte.init_map(camera_id=0)
    
    print("\nPréparez la carte avec les 4 marqueurs de référence visibles.")
    print("Appuyez sur 's' pour initialiser la map.")
    while True:
        key = carte.afficher_video()
        if key == ord('s'):
            break
    
    print("\nMap initialisée avec succès.")
    print("\nPlacez le Thymio sur la map à une position stable.")
    print("IMPORTANT: Ne touchez plus le robot pendant toute la durée du test!")
    print("Appuyez sur 'c' pour commencer la calibration de la caméra.")
    
    while True:
        key = carte.afficher_video()
        if key == ord('c'):
            break
    
    # Paramètres de la calibration
    num_samples = 100  # nombre d'échantillons à collecter
    delay_ms = 50  # 50ms entre chaque mesure (20Hz)
    
    print(f"\nCollecte de {num_samples} mesures...")
    print("Le robot doit rester TOTALEMENT immobile!")
    print("Appuyez sur CTRL+C pour arrêter prématurément si nécessaire.\n")
    
    # Listes pour stocker les mesures
    x_measurements = []
    y_measurements = []
    theta_measurements = []
    
    # Collecter les mesures
    samples_collected = 0
    failed_detections = 0
    
    import time
    while samples_collected < num_samples:
        try:
            # Afficher la vidéo (non bloquant)
            carte.afficher_video()
            
            # Obtenir la position du Thymio
            pos = carte.get_thymio_position()
            
            if pos is not None and len(pos) == 3:
                x, y, theta = pos
                x_measurements.append(x)
                y_measurements.append(y)
                theta_measurements.append(theta)
                samples_collected += 1
                
                # Afficher la progression tous les 10 échantillons
                if samples_collected % 10 == 0:
                    print(f"Échantillons collectés: {samples_collected}/{num_samples} "
                          f"(détections échouées: {failed_detections})")
            else:
                failed_detections += 1
                if failed_detections > 20:
                    print("\nATTENTION: Trop de détections échouées!")
                    print("Vérifiez que le marqueur du Thymio est bien visible.")
            
            # Attendre avant la prochaine mesure
            time.sleep(delay_ms / 1000.0)
            
        except KeyboardInterrupt:
            print("\n\nArrêt prématuré par l'utilisateur.")
            break
    
    # Arrêter la caméra
    carte.stop()
    
    # Vérifier qu'on a assez de mesures
    if len(x_measurements) < 10:
        print("\n\nERREUR: Pas assez de mesures collectées!")
        print(f"Seulement {len(x_measurements)} mesures valides.")
        return None, None, None
    
    # Convertir en numpy arrays
    x_measurements = np.array(x_measurements)
    y_measurements = np.array(y_measurements)
    theta_measurements = np.array(theta_measurements)
    
    # Calculer les statistiques
    var_x = np.var(x_measurements)
    var_y = np.var(y_measurements)
    var_theta = np.var(theta_measurements)
    
    mean_x = np.mean(x_measurements)
    mean_y = np.mean(y_measurements)
    mean_theta = np.mean(theta_measurements)
    
    std_x = np.sqrt(var_x)
    std_y = np.sqrt(var_y)
    std_theta = np.sqrt(var_theta)
    
    # Afficher les résultats
    print("\n" + "=" * 60)
    print("RÉSULTATS - VARIANCE DES MESURES CAMÉRA")
    print("=" * 60)
    print(f"\nNombre d'échantillons collectés: {len(x_measurements)}")
    print(f"Détections échouées: {failed_detections}")
    
    print("\n--- Position X ---")
    print(f"Moyenne: {mean_x:.2f} mm")
    print(f"Variance: {var_x:.6f} mm²")
    print(f"Écart-type: {std_x:.2f} mm")
    print(f"Min: {np.min(x_measurements):.2f} mm, Max: {np.max(x_measurements):.2f} mm")
    
    print("\n--- Position Y ---")
    print(f"Moyenne: {mean_y:.2f} mm")
    print(f"Variance: {var_y:.6f} mm²")
    print(f"Écart-type: {std_y:.2f} mm")
    print(f"Min: {np.min(y_measurements):.2f} mm, Max: {np.max(y_measurements):.2f} mm")
    
    print("\n--- Angle Theta ---")
    print(f"Moyenne: {mean_theta:.4f} rad ({mean_theta * 180 / np.pi:.2f}°)")
    print(f"Variance: {var_theta:.8f} rad²")
    print(f"Écart-type: {std_theta:.4f} rad ({std_theta * 180 / np.pi:.2f}°)")
    print(f"Min: {np.min(theta_measurements):.4f} rad, Max: {np.max(theta_measurements):.4f} rad")
    
    print("\n" + "=" * 60)
    print("Valeurs pour constants.py (matrice R):")
    print("=" * 60)
    print(f"X_MEAS_UNCERT = {var_x:.6f}  # mm²")
    print(f"Y_MEAS_UNCERT = {var_y:.6f}  # mm²")
    print(f"THETA_MEAS_UNCERT = {var_theta:.8f}  # rad²")
    
    print("\n\nCode Python pour la partie caméra de la matrice R:")
    print(f"R_camera = np.diag([{var_x:.6f}, {var_y:.6f}, {var_theta:.8f}, var_speed_L, var_speed_R])")
    
    return var_x, var_y, var_theta


async def calibrate_R_speeds(node, client):
    """
    Fonction pour déterminer la variance R des mesures de vitesse des roues.
    
    Cette fonction mesure la variance des CAPTEURS de vitesse du robot.
    On fait rouler le robot à vitesse constante et on collecte de nombreuses
    mesures de vitesse pour calculer leur variance (bruit de mesure).
    
    Returns:
        tuple: (var_speed_left, var_speed_right) - variances des mesures de vitesse en (mm/s)²
    """
    
    print("\n" + "=" * 60)
    print("CALIBRATION DE R - VARIANCE DES MESURES DE VITESSE")
    print("=" * 60)
    print("\nCette procédure mesure la variance des CAPTEURS de vitesse.")
    print("Le robot roulera à vitesse constante pendant que nous collectons")
    
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
    
    num_samples = int(input("Combien d'échantillons de vitesse à collecter ? (recommandé: 100-200): ") or "100")
    test_speed = int(input("Vitesse de test (unités Thymio) ? (recommandé: 200): ") or "200")
    
    input("\nPlacez le robot sur le sol. Appuyez sur ENTRÉE pour démarrer...")
    
    # Démarrer le mouvement
    await node.set_variables(motors(test_speed, test_speed))
    print("Attente de stabilisation (1 seconde)...")
    await client.sleep(1.0)  # Attendre la stabilisation
    
    left_speeds = []
    right_speeds = []
    
    print(f"Collection de {num_samples} échantillons de vitesse...")
    for i in range(num_samples):
        v_l = node["motor.left.speed"]
        v_r = node["motor.right.speed"]
        
        left_speeds.append(v_l * SPEED_TO_MMS)
        right_speeds.append(v_r * SPEED_TO_MMS)
        
        if (i + 1) % 20 == 0:
            print(f"  {i + 1}/{num_samples} échantillons collectés...")
        
        await client.sleep(0.05)
    
    # Arrêter le robot
    await node.set_variables(motors(0, 0))
    
    # Calculer les variances (bruit de mesure)
    var_speed_left = np.var(left_speeds)
    var_speed_right = np.var(right_speeds)
    
    print(f"\n=== Résultats - Variance R (mesures de vitesse) ===")
    print(f"Vitesse moyenne mesurée gauche: {np.mean(left_speeds):.2f} mm/s")
    print(f"Vitesse moyenne mesurée droite: {np.mean(right_speeds):.2f} mm/s")
    print(f"Variance mesure vitesse gauche: {var_speed_left:.4f} (mm/s)²")
    print(f"Variance mesure vitesse droite: {var_speed_right:.4f} (mm/s)²")
    print(f"Écart-type mesure gauche: {np.sqrt(var_speed_left):.2f} mm/s")
    print(f"Écart-type mesure droite: {np.sqrt(var_speed_right):.2f} mm/s")
    
    print("\n\nValeurs pour constants.py (partie vitesse de R):")
    print(f"SPEED_MES_UNCERT = {(var_speed_left + var_speed_right) / 2:.6f}  # (mm/s)² - moyenne des deux roues")
    print(f"# Ou séparément:")
    print(f"# SPEED_MES_UNCERT_LEFT = {var_speed_left:.6f}  # (mm/s)²")
    print(f"# SPEED_MES_UNCERT_RIGHT = {var_speed_right:.6f}  # (mm/s)²")
    
    return var_speed_left, var_speed_right


async def main():
    """Point d'entrée principal."""
    client = ClientAsync()
    
    try:
        print("Connexion au Thymio...")
        node = await client.wait_for_node()
        await node.lock()
        print("Thymio connecté et verrouillé.\n")
        
        print("=" * 60)
        print("CALIBRATION DES MATRICES DE COVARIANCE Q et R")
        print("=" * 60)
        print("\nQue souhaitez-vous calibrer ?")
        print("  1. Matrice Q (covariance du bruit de processus)")
        print("  2. Matrice R - partie caméra (covariance du bruit de mesure position)")
        print("  3. Matrice R - partie vitesses (covariance du bruit de mesure vitesse)")
        print("  4. Matrice R complète (caméra + vitesses)")
        print("  5. Tout calibrer (Q puis R complet)")
        
        choice = input("\nVotre choix (1, 2, 3, 4 ou 5) : ").strip()
        
        if choice == "1":
            Q_matrix = await calibrate_Q_matrix(node, client)
            
        elif choice == "2":
            var_x, var_y, var_theta = await calibrate_R_camera()
            
        elif choice == "3":
            var_left, var_right = await calibrate_R_speeds(node, client)
            
        elif choice == "4":
            print("\n### PARTIE 1/2 : Calibration de R (caméra) ###")
            var_x, var_y, var_theta = await calibrate_R_camera()
            
            print("\n\n### PARTIE 2/2 : Calibration de R (vitesses) ###")
            var_left, var_right = await calibrate_R_speeds(node, client)
            
            if var_x is not None and var_left is not None:
                print("\n" + "=" * 60)
                print("MATRICE R COMPLÈTE")
                print("=" * 60)
                print(f"R = np.diag([{var_x:.6f}, {var_y:.6f}, {var_theta:.8f}, "
                      f"{var_left:.6f}, {var_right:.6f}])")
            
        elif choice == "5":
            print("\n### PARTIE 1/3 : Calibration de Q ###")
            Q_matrix = await calibrate_Q_matrix(node, client)
            
            print("\n\n### PARTIE 2/3 : Calibration de R (caméra) ###")
            var_x, var_y, var_theta = await calibrate_R_camera()
            
            print("\n\n### PARTIE 3/3 : Calibration de R (vitesses) ###")
            var_left, var_right = await calibrate_R_speeds(node, client)
            
            if var_x is not None and var_left is not None:
                print("\n" + "=" * 60)
                print("MATRICE R COMPLÈTE")
                print("=" * 60)
                print(f"R = np.diag([{var_x:.6f}, {var_y:.6f}, {var_theta:.8f}, "
                      f"{var_left:.6f}, {var_right:.6f}])")
            
        else:
            print("Choix invalide. Arrêt du programme.")
            return
        
        print("\n" + "=" * 60)
        print("CALIBRATION TERMINÉE")
        print("=" * 60)
        print("\nCopiez les valeurs affichées ci-dessus dans votre fichier constants.py")
    
        
    except KeyboardInterrupt:
        print("\n\nInterruption par l'utilisateur.")
    except Exception as e:
        print(f"\nErreur: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nArrêt du programme.")


if __name__ == "__main__":
    asyncio.run(main())