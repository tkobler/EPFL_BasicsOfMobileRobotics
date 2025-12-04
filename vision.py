import cv2
import numpy as np
import threading
import time
from traitement import analyser_formes_geometriques
import constants as cte


# Dimensions utiles de l'environnement (sans les marqueurs)
DIMENSION_LON = cte.ENV_LONGUEUR - cte.LARGEUR_MARQUEUR
DIMENSION_LARG = cte.ENV_LARGEUR - cte.LARGEUR_MARQUEUR

# Variable globale pour stocker les positions des marqueurs (pointin uniquement)
# Structure: {ID: [x, y]}
positions_aruco = {}

# Variable globale pour stocker la position du Thymio transformée par homographie
# Structure: {"x": x, "y": y, "orientation": radians}
position_thymio_transformee = {}

# Variable globale pour stocker la position de l'objectif transformée par homographie
# Structure: {"x": x, "y": y}
position_objectif_transformee = {}

# Matrice d'homographie globale
matrice_homographie = None

# Variables globales pour stocker les dernières frames
derniere_frame = None
derniere_frame_propre = None

# Variables globales pour le threading et la caméra
cap = None
detector = None
detection_thread = None
running = False


class Map:
    """
    Classe pour accéder aux informations de la carte (positions du Thymio, obstacles et objectif).
    """

    def init_map(self, camera_id=0):
        """
        Initialise la carte en lançant la détection ArUco depuis la webcam dans un thread.

        Args:
            camera_id: ID de la caméra (0 pour webcam par défaut, 1 pour webcam externe)
        """
        global cap, detector, detection_thread, running

        # Initialiser la webcam
        cap = cv2.VideoCapture(camera_id)

        if not cap.isOpened():
            print(f"Erreur: Impossible d'ouvrir la caméra {camera_id}")
            print("Essayez de changer camera_id (0, 1, 2...)")
            return

        # Définir la résolution de la caméra
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Charger le dictionnaire ArUco 4x4
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        print("=== DETECTION ARUCO ACTIVEE ===")

        # Démarrer le thread de détection
        running = True
        detection_thread = threading.Thread(target=boucle_detection, daemon=True)
        detection_thread.start()

    def afficher_video(self):
        """
        Affiche la vidéo avec les marqueurs détectés.
        Doit être appelé depuis le thread principal dans une boucle.

        Returns:
            int: Code de la touche pressée (ou -1 si aucune touche)
        """
        if derniere_frame is not None:
            cv2.imshow('Detection ArUco - Webcam', derniere_frame)
            return cv2.waitKey(1) & 0xFF
        return -1
    
    def dessin_point(self, x, y, angle=None):
        """
        Transforme un point de l'espace transformé vers l'espace caméra (transformation inverse).
        Dessine le point résultant sur la frame avec une couleur vive.
        Si un angle est fourni, dessine également un trait dans la direction de l'angle transformé.

        Args:
            x: Coordonnée x dans l'espace transformé
            y: Coordonnée y dans l'espace transformé
            angle: Angle en radians dans l'espace transformé (optionnel)
        """
        global derniere_frame

        if derniere_frame is None:
            return

        # Utiliser la fonction transformer_vers_camera pour obtenir les coordonnées
        resultat = transformer_vers_camera(x, y, angle)

        if resultat is None:
            return

        if angle is not None:
            # Avec angle : 4 valeurs retournées
            x_camera, y_camera, x2_camera, y2_camera = resultat

            # Dessiner le point sur la frame avec une couleur vive (rouge fluo)
            cv2.circle(derniere_frame, (x_camera, y_camera), 10, (0, 0, 255), -1)

            # Dessiner le trait dans la direction de l'angle transformé
            cv2.line(derniere_frame, (x_camera, y_camera), (x2_camera, y2_camera), (0, 255, 0), 3)
        else:
            # Sans angle : 2 valeurs retournées
            x_camera, y_camera = resultat

            # Dessiner le point sur la frame avec une couleur vive (rouge fluo)
            cv2.circle(derniere_frame, (x_camera, y_camera), 10, (0, 0, 255), -1)

    def dessin_chemin(self, points):
        """
        Dessine un chemin (série de points reliés entre eux) sur la frame.
        Transforme chaque point de l'espace transformé vers l'espace caméra.

        Args:
            points: Liste de points de la forme [[x1, y1], [x2, y2], ...]
        """
        global derniere_frame

        if derniere_frame is None:
            return

        if points is None or len(points) == 0:
            return

        # Liste pour stocker les coordonnées caméra
        points_camera = []

        # Transformer tous les points
        for point in points:
            if len(point) < 2:
                continue

            x, y = point[0], point[1]
            resultat = transformer_vers_camera(x, y)

            if resultat is None:
                continue

            x_camera, y_camera = resultat
            points_camera.append((x_camera, y_camera))

        # Dessiner les points et les lignes
        for i, (x_camera, y_camera) in enumerate(points_camera):
            # Dessiner le point (bleu)
            cv2.circle(derniere_frame, (x_camera, y_camera), 5, (255, 0, 0), -1)

            # Dessiner une ligne vers le point suivant
            if i < len(points_camera) - 1:
                x_next, y_next = points_camera[i + 1]
                cv2.line(derniere_frame, (x_camera, y_camera), (x_next, y_next), (0, 255, 255), 2)

    def dessin_ellipse(self, x, y, orientation, rayon_x, rayon_y, delta_theta):
        """
        Dessine une ellipse avec orientation sur la frame.
        Ajoute un arc de cercle dans l'intervalle [orientation - delta_theta, orientation + delta_theta].

        Args:
            x: Coordonnée x du centre dans l'espace transformé
            y: Coordonnée y du centre dans l'espace transformé
            orientation: Angle d'orientation en radians
            rayon_x: Rayon selon l'axe x de l'ellipse (utilisé aussi pour l'arc)
            rayon_y: Rayon selon l'axe y de l'ellipse
            delta_theta: Demi-ouverture angulaire de l'arc en radians
        """
        global derniere_frame

        if derniere_frame is None:
            return

        if matrice_homographie is None:
            return

        # Générer des points sur l'ellipse (50 points pour un rendu lisse)
        num_points = 50
        angles = np.linspace(0, 2 * np.pi, num_points)

        # Points de l'ellipse dans son repère local (non orienté)
        points_ellipse_local = []
        for angle in angles:
            x_local = rayon_x * np.cos(angle)
            y_local = rayon_y * np.sin(angle)
            points_ellipse_local.append([x_local, y_local])

        # Transformer les points avec rotation et translation vers l'espace transformé
        points_ellipse_monde = []
        for x_local, y_local in points_ellipse_local:
            # Rotation
            x_rot = x_local * np.cos(orientation) - y_local * np.sin(orientation)
            y_rot = x_local * np.sin(orientation) + y_local * np.cos(orientation)
            # Translation
            x_monde = x + x_rot - cte.LARGEUR_MARQUEUR
            y_monde = y + y_rot - cte.LARGEUR_MARQUEUR
            points_ellipse_monde.append([x_monde, y_monde])

        # Transformer les points vers l'espace caméra avec la matrice inverse
        try:
            matrice_inverse = np.linalg.inv(matrice_homographie)
            points_monde_array = np.array(points_ellipse_monde, dtype=np.float32)
            points_monde_array = points_monde_array.reshape(-1, 1, 2)
            points_camera = cv2.perspectiveTransform(points_monde_array, matrice_inverse)

            # Dessiner l'ellipse en reliant les points
            for i in range(len(points_camera)):
                pt1 = (int(points_camera[i][0][0]), int(points_camera[i][0][1]))
                pt2 = (int(points_camera[(i + 1) % len(points_camera)][0][0]),
                       int(points_camera[(i + 1) % len(points_camera)][0][1]))
                cv2.line(derniere_frame, pt1, pt2, (255, 0, 0), 2)

            # Dessiner l'arc de cercle dans l'intervalle angulaire
            num_arc_points = 30
            arc_angles = np.linspace(-delta_theta, delta_theta, num_arc_points)
            points_arc_local = []

            for angle in arc_angles:
                x_local = rayon_x * np.cos(angle)
                y_local = rayon_x * np.sin(angle)  # rayon constant = rayon_x
                points_arc_local.append([x_local, y_local])

            # Transformer les points de l'arc
            points_arc_monde = []
            for x_local, y_local in points_arc_local:
                x_rot = x_local * np.cos(orientation) - y_local * np.sin(orientation)
                y_rot = x_local * np.sin(orientation) + y_local * np.cos(orientation)
                x_monde = x + x_rot - cte.LARGEUR_MARQUEUR
                y_monde = y + y_rot - cte.LARGEUR_MARQUEUR
                points_arc_monde.append([x_monde, y_monde])

            points_arc_array = np.array(points_arc_monde, dtype=np.float32).reshape(-1, 1, 2)
            points_arc_camera = cv2.perspectiveTransform(points_arc_array, matrice_inverse)

            # Dessiner l'arc avec une couleur différente (vert)
            for i in range(len(points_arc_camera) - 1):
                pt1 = (int(points_arc_camera[i][0][0]), int(points_arc_camera[i][0][1]))
                pt2 = (int(points_arc_camera[i + 1][0][0]), int(points_arc_camera[i + 1][0][1]))
                cv2.line(derniere_frame, pt1, pt2, (0, 255, 0), 3)

            # Dessiner le trait d'orientation
            resultat_centre = transformer_vers_camera(x, y, orientation)
            if resultat_centre is not None:
                x_camera, y_camera, x2_camera, y2_camera = resultat_centre
                cv2.line(derniere_frame, (x_camera, y_camera), (x2_camera, y2_camera), (255, 0, 0), 2)

        except Exception:
            return

    def stop(self):
        """
        Arrête la détection et libère les ressources.
        """
        global running

        running = False

        if detection_thread is not None:
            detection_thread.join(timeout=1.0)

        if cap is not None:
            cap.release()

        cv2.destroyAllWindows()
        print("\n=== DETECTION ARUCO ARRETEE ===")

    def get_thymio_position(self):
        """
        Retourne la position transformée du Thymio avec ajustement LARGEUR_MARQUEUR.

        Returns:
            list: [x, y, orientation] ou [] si non détecté
        """
        if position_thymio_transformee and len(position_thymio_transformee) > 0:
            return [
                position_thymio_transformee["x"] + cte.LARGEUR_MARQUEUR,
                position_thymio_transformee["y"] + cte.LARGEUR_MARQUEUR,
                position_thymio_transformee["orientation"]
            ]
        return []

    def get_goal_position(self):
        """
        Retourne la position de l'objectif (goal) avec ajustement LARGEUR_MARQUEUR.

        Returns:
            list: [x, y] ou [] si non détecté
        """
        if position_objectif_transformee and len(position_objectif_transformee) > 0:
            return [
                position_objectif_transformee["x"] + cte.LARGEUR_MARQUEUR,
                position_objectif_transformee["y"] + cte.LARGEUR_MARQUEUR
            ]
        return []

    def get_obstacles_positions(self, afficher=True):
        """
        Retourne les positions des obstacles détectés.
        Capture automatiquement l'image actuelle, applique l'homographie et analyse les formes.

        Args:
            afficher: Si True, affiche les images de détection (défaut: True)

        Returns:
            list: Liste des obstacles, chaque obstacle est une liste de sommets [[x, y], [x, y], ...]
        """
        try:
            # Attendre que la matrice homographique soit calculée
            print("Attente de la matrice homographique...")
            while matrice_homographie is None:
                self.afficher_video()
                if matrice_homographie is not None:
                    break

            # Vérifier que nous avons une frame
            if derniere_frame_propre is None:
                print("Erreur: Pas de frame disponible")
                return []

            print("Matrice homographique détectée!")
            # Capture automatique
            print("\n=== CAPTURE POUR DETECTION D'OBSTACLES ===")

            # Sauvegarder la frame actuelle
            filename = "aruco_capture.jpg"
            cv2.imwrite(filename, derniere_frame)
            print(f"Capture sauvegardée: {filename}")

            # Appliquer l'homographie à l'image propre (sans dessins)
            image_transformee = cv2.warpPerspective(derniere_frame_propre, matrice_homographie, (DIMENSION_LARG, DIMENSION_LON))
            filename_homo = "aruco_capture_homographie.jpg"
            cv2.imwrite(filename_homo, image_transformee)
            print(f"Capture homographie sauvegardée: {filename_homo}")

            # Analyser l'image homographie pour détecter les obstacles
            tableau = analyser_formes_geometriques(filename_homo, precision=0.02, afficher=afficher)

            # Ajuster les coordonnées en ajoutant LARGEUR_MARQUEUR à chaque sommet
            obstacles_ajustes = []
            for obstacle in tableau:
                sommets_ajustes = []
                for sommet in obstacle:
                    x_ajuste = sommet[0] + cte.LARGEUR_MARQUEUR
                    y_ajuste = sommet[1] + cte.LARGEUR_MARQUEUR
                    sommets_ajustes.append([x_ajuste, y_ajuste])
                obstacles_ajustes.append(sommets_ajustes)

            return obstacles_ajustes
        except Exception as e:
            print(f"Erreur lors de la capture/analyse: {e}")
            return []


def calculer_centroide(coin_haut_gauche, coin_haut_droit, coin_bas_droit, coin_bas_gauche):
    """
    Calcule le centroïde (centre) à partir de 4 coins.

    Args:
        coin_haut_gauche: Coin haut-gauche [x, y]
        coin_haut_droit: Coin haut-droit [x, y]
        coin_bas_droit: Coin bas-droit [x, y]
        coin_bas_gauche: Coin bas-gauche [x, y]

    Returns:
        tuple: (centre_x, centre_y)
    """
    centre_x = int((coin_haut_gauche[0] + coin_haut_droit[0] + coin_bas_gauche[0] + coin_bas_droit[0]) / 4)
    centre_y = int((coin_haut_gauche[1] + coin_haut_droit[1] + coin_bas_gauche[1] + coin_bas_droit[1]) / 4)
    return centre_x, centre_y


def calculer_orientation(coin_haut_gauche, coin_haut_droit):
    """
    Calcule l'orientation à partir de deux coins (haut-gauche et haut-droit).

    Args:
        coin_haut_gauche: Coin haut-gauche [x, y]
        coin_haut_droit: Coin haut-droit [x, y]

    Returns:
        float: Orientation en radians
    """
    dx = coin_haut_droit[0] - coin_haut_gauche[0]
    dy = coin_haut_droit[1] - coin_haut_gauche[1]
    return np.arctan2(dy, dx)


def transformer_position_marqueur(corner, avec_orientation=False):
    """
    Extrait les coins d'un marqueur, calcule le centroïde et transforme la position avec l'homographie.

    Args:
        corner: Coins du marqueur
        avec_orientation: Si True, calcule et retourne aussi l'orientation transformée

    Returns:
        tuple: (centre_x, centre_y, centre_transforme_x, centre_transforme_y, orientation_originale, orientation_transformee)
               ou (centre_x, centre_y, centre_transforme_x, centre_transforme_y, None, None) si avec_orientation=False
               ou (centre_x, centre_y, None, None, None, None) si pas d'homographie disponible
    """
    # Extraire les 4 coins
    coin_haut_gauche = corner[0][0].astype(int)
    coin_haut_droit = corner[0][1].astype(int)
    coin_bas_droit = corner[0][2].astype(int)
    coin_bas_gauche = corner[0][3].astype(int)

    # Calculer le centre
    centre_x, centre_y = calculer_centroide(coin_haut_gauche, coin_haut_droit, coin_bas_droit, coin_bas_gauche)

    # Si pas d'homographie, retourner seulement le centre
    if matrice_homographie is None:
        return centre_x, centre_y, None, None, None, None

    # Calculer l'orientation si demandé
    orientation = None
    orientation_transformee = None
    if avec_orientation:
        orientation = calculer_orientation(coin_haut_gauche, coin_haut_droit)

        # Calculer un deuxième point dans la direction de l'orientation
        longueur = 50
        point_direction_x = centre_x + longueur * np.cos(orientation)
        point_direction_y = centre_y + longueur * np.sin(orientation)

        # Points à transformer: centre et point de direction
        points = np.array([
            [centre_x, centre_y],
            [point_direction_x, point_direction_y]
        ], dtype=np.float32)
        points = points.reshape(-1, 1, 2)

        # Transformer les deux points avec l'homographie
        points_transformes = cv2.perspectiveTransform(points, matrice_homographie)

        # Extraire les points transformés
        centre_transforme = points_transformes[0][0]
        direction_transforme = points_transformes[1][0]

        # Calculer l'orientation transformée
        orientation_transformee = calculer_orientation(centre_transforme, direction_transforme)

        return centre_x, centre_y, int(centre_transforme[0]), int(centre_transforme[1]), float(orientation), float(orientation_transformee)
    else:
        # Point à transformer: centre uniquement
        point = np.array([[centre_x, centre_y]], dtype=np.float32)
        point = point.reshape(-1, 1, 2)

        # Transformer le point avec l'homographie
        point_transforme = cv2.perspectiveTransform(point, matrice_homographie)

        # Extraire le point transformé
        centre_transforme = point_transforme[0][0]

        return centre_x, centre_y, int(centre_transforme[0]), int(centre_transforme[1]), None, None


def transformer_vers_camera(x, y, angle=None):
    """
    Transforme un point de l'espace transformé vers l'espace caméra (transformation inverse).

    Args:
        x: Coordonnée x dans l'espace transformé
        y: Coordonnée y dans l'espace transformé
        angle: Angle en radians dans l'espace transformé (optionnel)

    Returns:
        tuple: (x_camera, y_camera) si angle est None
               (x_camera, y_camera, x2_camera, y2_camera) si angle est fourni
               None si pas d'homographie disponible
    """
    if matrice_homographie is None:
        return None

    try:
        # Calculer la matrice d'homographie inverse
        matrice_inverse = np.linalg.inv(matrice_homographie)

        # Ajuster les coordonnées
        x_ajuste = x - cte.LARGEUR_MARQUEUR
        y_ajuste = y - cte.LARGEUR_MARQUEUR

        if angle is not None:
            # Transformer l'angle en créant un deuxième point dans la direction de l'angle
            longueur = 50  # Longueur du vecteur de direction
            x2 = x_ajuste + longueur * np.cos(angle)
            y2 = y_ajuste + longueur * np.sin(angle)

            # Créer les deux points à transformer
            points = np.array([[x_ajuste, y_ajuste], [x2, y2]], dtype=np.float32)
            points = points.reshape(-1, 1, 2)

            # Appliquer la transformation inverse
            points_transformes = cv2.perspectiveTransform(points, matrice_inverse)

            # Extraire les coordonnées des points transformés
            x_camera = int(points_transformes[0][0][0])
            y_camera = int(points_transformes[0][0][1])
            x2_camera = int(points_transformes[1][0][0])
            y2_camera = int(points_transformes[1][0][1])

            return x_camera, y_camera, x2_camera, y2_camera
        else:
            # Créer le point à transformer
            point = np.array([[x_ajuste, y_ajuste]], dtype=np.float32)
            point = point.reshape(-1, 1, 2)

            # Appliquer la transformation inverse
            point_transforme = cv2.perspectiveTransform(point, matrice_inverse)

            # Extraire les coordonnées
            x_camera = int(point_transforme[0][0][0])
            y_camera = int(point_transforme[0][0][1])

            return x_camera, y_camera

    except Exception:
        return None


def calculer_matrice_homographie():
    """
    Calcule et stocke la matrice d'homographie basée sur les 4 marqueurs de référence (ID 0-3).
    Met à jour la variable globale matrice_homographie.
    """
    #met la matric homographie a none
    global matrice_homographie
    matrice_homographie = None
    # Vérifier que les 4 marqueurs de référence sont détectés
    if not all(mid in positions_aruco for mid in [0, 1, 2, 3]):
        return

    # Points sources (positions des marqueurs 0, 1, 2, 3)
    points_sources = np.float32([
        positions_aruco[0],
        positions_aruco[1],
        positions_aruco[2],
        positions_aruco[3]
    ])

    # Points de destination (dimensions sans marqueurs)
    points_dest = np.float32([
        [0, DIMENSION_LON],             # ID 0 -> coin haut-gauche
        [DIMENSION_LARG, DIMENSION_LON], # ID 1 -> coin haut-droit
        [0, 0],                         # ID 2 -> coin bas-gauche
        [DIMENSION_LARG, 0]             # ID 3 -> coin bas-droit
    ])

    # Calculer et stocker la matrice d'homographie
    matrice_homographie = cv2.getPerspectiveTransform(points_sources, points_dest)


def detecter_marqueurs_reference(corners, ids, frame):
    """
    Détecte et stocke les positions des marqueurs de référence (ID 0-3).
    Stocke uniquement le point intérieur (pointin).

    Args:
        corners: Coins des marqueurs détectés
        ids: IDs des marqueurs détectés
        frame: Image pour dessiner les marqueurs
    """
    global positions_aruco

    # Réinitialiser les positions ArUco (sera rempli seulement si les IDs 0-3 sont détectés)
    positions_aruco = {}

    for corner, marker_id in zip(corners, ids):
        mid = marker_id[0]

        # Traiter uniquement les IDs 0, 1, 2, 3
        if mid not in [0, 1, 2, 3]:
            continue

        # Extraire les 4 coins
        coin_haut_gauche = corner[0][0].astype(int)
        coin_haut_droit = corner[0][1].astype(int)
        coin_bas_droit = corner[0][2].astype(int)
        coin_bas_gauche = corner[0][3].astype(int)

        # Sélectionner le point intérieur selon l'ID
        if mid == 0:
            # ID 0: bas-droit
            pointin = coin_bas_droit
        elif mid == 1:
            # ID 1: bas-gauche
            pointin = coin_bas_gauche
        elif mid == 2:
            # ID 2: haut-droit
            pointin = coin_haut_droit
        elif mid == 3:
            # ID 3: haut-gauche
            pointin = coin_haut_gauche

        # Stocker uniquement le point intérieur
        positions_aruco[mid] = [int(pointin[0]), int(pointin[1])]

        # Dessiner le point intérieur
        cv2.circle(frame, tuple(pointin), 8, (255, 0, 0), -1)

        # Afficher l'ID
        cv2.putText(frame, f"ID: {mid}",
                   (int(pointin[0]) - 30, int(pointin[1]) - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def detecter_thymio(corners, ids, frame):
    """
    Détecte le Thymio (ID 4), dessine sa position sur la frame et stocke sa position transformée.
    Utilise la matrice d'homographie pour calculer directement la position transformée.

    Args:
        corners: Coins des marqueurs détectés
        ids: IDs des marqueurs détectés
        frame: Image pour dessiner le marqueur
    """
    global position_thymio_transformee

    # Réinitialiser la position du Thymio (sera rempli seulement si ID 4 est détecté)
    position_thymio_transformee = {}

    for corner, marker_id in zip(corners, ids):
        mid = marker_id[0]

        # Traiter uniquement l'ID 4
        if mid != 4:
            continue

        # Extraire position et orientation (avec transformation)
        centre_x, centre_y, centre_t_x, centre_t_y, orientation, orientation_t = transformer_position_marqueur(corner, avec_orientation=True)

        # Dessiner le centre
        cv2.circle(frame, (centre_x, centre_y), 8, (255, 0, 255), -1)

        # Dessiner une ligne pour montrer l'orientation (utilise l'orientation déjà calculée)
        if orientation is not None:
            longueur_ligne = 50
            fin_x = int(centre_x + longueur_ligne * np.cos(orientation))
            fin_y = int(centre_y + longueur_ligne * np.sin(orientation))
            cv2.line(frame, (centre_x, centre_y), (fin_x, fin_y), (255, 0, 255), 3)

        # Afficher l'ID
        cv2.putText(frame, f"ID: {mid}",
                   (centre_x - 30, centre_y - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        # Afficher la position transformée et l'orientation transformée (avec ajustement LARGEUR_MARQUEUR)
        if centre_t_x is not None and orientation_t is not None:
            x_ajuste = centre_t_x + cte.LARGEUR_MARQUEUR
            y_ajuste = centre_t_y + cte.LARGEUR_MARQUEUR
            texte_transforme = f"({x_ajuste}, {y_ajuste}, ori={orientation_t:.2f}rad)"
            cv2.putText(frame, texte_transforme,
                       (centre_x - 30, centre_y + 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # Stocker la position transformée si disponible
        if centre_t_x is not None:
            position_thymio_transformee["x"] = centre_t_x
            position_thymio_transformee["y"] = centre_t_y
            position_thymio_transformee["orientation"] = orientation_t




def detecter_objectif(corners, ids, frame):
    """
    Détecte l'objectif (ID 5), dessine sa position sur la frame et stocke sa position transformée.
    Utilise la matrice d'homographie pour calculer directement la position transformée.

    Args:
        corners: Coins des marqueurs détectés
        ids: IDs des marqueurs détectés
        frame: Image pour dessiner le marqueur
    """
    global position_objectif_transformee

    for corner, marker_id in zip(corners, ids):
        mid = marker_id[0]

        # Traiter uniquement l'ID 5
        if mid != 5:
            continue

        # Extraire position (avec transformation, sans orientation)
        centre_x, centre_y, centre_t_x, centre_t_y, _, _ = transformer_position_marqueur(corner, avec_orientation=False)

        # Dessiner le centre
        cv2.circle(frame, (centre_x, centre_y), 8, (0, 255, 255), -1)

        # Afficher l'ID
        cv2.putText(frame, f"ID: {mid} (Goal)",
                   (centre_x - 30, centre_y - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Afficher la position transformée (avec ajustement LARGEUR_MARQUEUR)
        if centre_t_x is not None:
            x_ajuste = centre_t_x + cte.LARGEUR_MARQUEUR
            y_ajuste = centre_t_y + cte.LARGEUR_MARQUEUR
            texte_transforme = f"({x_ajuste}, {y_ajuste})"
            cv2.putText(frame, texte_transforme,
                       (centre_x - 30, centre_y + 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Stocker la position transformée si disponible
        if centre_t_x is not None:
            position_objectif_transformee["x"] = centre_t_x
            position_objectif_transformee["y"] = centre_t_y


def boucle_detection():
    """
    Boucle de détection qui tourne à 20Hz dans un thread.
    Détecte les marqueurs ArUco et met à jour les positions globales.
    N'affiche pas la vidéo (doit être fait depuis le thread principal).
    """
    global derniere_frame, derniere_frame_propre

    # 20Hz = 50ms par frame
    target_delay = 1.0 / 20.0

    while running:
        start_time = time.time()

        # Lire une frame de la webcam
        ret, frame = cap.read()

        if not ret:
            print("Erreur: Impossible de lire la frame")
            break


        # Créer une copie pour l'affichage (avec dessins)
        frame_affichage = frame.copy()

        # Mettre à jour les frames globales
        derniere_frame_propre = frame.copy()

        # Convertir en niveaux de gris pour la détection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Détecter les marqueurs ArUco
        corners, ids, _ = detector.detectMarkers(gray)

        # Si des marqueurs sont détectés
        if ids is not None and len(ids) > 0:
            # Dessiner les marqueurs détectés
            cv2.aruco.drawDetectedMarkers(frame_affichage, corners, ids)

            # Détecter les marqueurs de référence (ID 0-3)
            detecter_marqueurs_reference(corners, ids, frame_affichage)

            # Calculer la matrice d'homographie
            calculer_matrice_homographie()

            # Détecter le Thymio (ID 4) et calculer sa position transformée
            detecter_thymio(corners, ids, frame_affichage)

            # Détecter l'objectif (ID 5) et calculer sa position transformée
            detecter_objectif(corners, ids, frame_affichage)

        # Afficher le nombre de marqueurs en haut de l'image
        cv2.putText(frame_affichage, f"Marqueurs: {len(ids) if ids is not None else 0}",
                   (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Stocker la frame pour affichage depuis le thread principal
        derniere_frame = frame_affichage

        # Attendre pour maintenir 20Hz
        elapsed = time.time() - start_time
        if elapsed < target_delay:
            time.sleep(target_delay - elapsed)




# Exemple d'utilisation
if __name__ == "__main__":
    print("=== DETECTION DE MARQUEURS ARUCO 4x4 ===")
    print("Utilisation de la caméra 0")
    print()

    try:
        # Créer une instance de Map et démarrer la détection
        map_instance = Map()
        map_instance.init_map(camera_id=0)

        # Boucle principale pour afficher la vidéo
        print("Appuyez sur Ctrl+C pour arrêter...")
        while True:
            map_instance.afficher_video()
            #
            time.sleep(0.01)  # Petite pause pour ne pas surcharger le CPU

    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur...")
        if 'map_instance' in locals():
            map_instance.stop()

    except Exception as e:
        print(f"\nErreur: {e}")