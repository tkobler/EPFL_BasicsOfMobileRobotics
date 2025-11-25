import cv2
import numpy as np

def analyser_formes_geometriques(chemin_image, precision=0.02, afficher=True):
    """
    Analyse une image pour d�tecter toutes les formes g�om�triques et leurs sommets.

    Args:
        chemin_image: Chemin vers l'image � analyser
        precision: Pr�cision pour l'approximation polygonale (plus petit = plus pr�cis)
        afficher: Si True, affiche l'image avec les contours et sommets d�tect�s

    Returns:
        tableau: Structure [figure][sommet][x=0, y=1] contenant les positions des sommets
    """

    # Charger l'image
    image = cv2.imread(chemin_image)
    if image is None:
        raise ValueError(f"Impossible de charger l'image: {chemin_image}")

    # Cr�er une copie pour l'affichage
    image_affichage = image.copy()

    # Convertir en niveaux de gris
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Appliquer un flou pour r�duire le bruit
    gris = cv2.GaussianBlur(gris, (5, 5), 0)

    # D�tection des contours avec seuillage adaptatif
    #invers la couleur de seuil
    _, seuil = cv2.threshold(gris, 100, 255, cv2.THRESH_BINARY_INV)

    # Trouver tous les contours
    contours, _ = cv2.findContours(seuil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
 
    # Tableau pour stocker les r�sultats
    tableau = []

    # Analyser chaque contour
    for i, contour in enumerate(contours):
        # Ignorer les tr�s petits contours (bruit)
        aire = cv2.contourArea(contour)
        if aire < 100:  # Seuil minimum d'aire
            continue


        # Calculer le p�rim�tre
        perimetre = cv2.arcLength(contour, True)

        # Approximation polygonale pour trouver les sommets
        epsilon = precision * perimetre
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Cr�er le tableau des sommets pour cette figure
        sommets = []
        for point in approx:
            # point[0] contient [x, y]
            x = int(point[0][0])
            y = int(point[0][1])
            sommets.append([x, y])  # [x, y] o� x est � l'index 0 et y � l'index 1

        # Ajouter cette figure au tableau principal
        tableau.append(sommets)

        # Affichage visuel si demand�
        if afficher:
            # Dessiner le contour
            couleur = (160, 50, 0)  # Bleu foncé en BGR
            cv2.drawContours(image_affichage, [approx], -1, couleur, 5)

            # Dessiner les sommets
            for sommet in sommets:
                cv2.circle(image_affichage, tuple(sommet), 10, (0, 0, 255), -1)

            # Ajouter le num�ro de la figure (correspond � l'index dans le tableau)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Utiliser len(tableau)-1 pour avoir le bon index
                cv2.putText(image_affichage, f"obstacle {len(tableau)}", (cx-100, cy),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

    # Afficher toutes les �tapes du traitement
    if afficher:
        cv2.imshow('1 - Image originale', image)
        cv2.imshow('2 - Image en niveaux de gris', gris)
        cv2.imshow('3 - Image apres seuillage', seuil)
        cv2.imshow('4 - Formes detectees', image_affichage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return tableau


def afficher_resultats(tableau):
    """
    Affiche les r�sultats de mani�re lisible.

    Args:
        tableau: Structure [figure][sommet][x=0, y=1]
    """
    print(f"\n=== RESULTATS DE L'ANALYSE ===")
    print(f"Nombre de obstacles détectées: {len(tableau)}\n")

    for i, figure in enumerate(tableau):
        nb_sommets = len(figure)

        print(f"obstacle {i+1} -  ({nb_sommets} sommets):")
        for j, sommet in enumerate(figure):
            x = sommet[0]  # x est � l'index 0
            y = sommet[1]  # y est � l'index 1
            print(f"  Sommet {j}: x={x}, y={y}")
        print()


# Exemple d'utilisation
if __name__ == "__main__":
    # Remplacer par le chemin de votre image
    chemin_image = "aruco_capture_homographie.jpg"

    try:
        # Analyser l'image
        # precision: plus petit = plus pr�cis (0.01 � 0.05 recommand�)
        tableau = analyser_formes_geometriques(chemin_image, precision=0.02, afficher=True)

        # Afficher les r�sultats
        afficher_resultats(tableau)

    except Exception as e:
        print(f"Erreur: {e}")
