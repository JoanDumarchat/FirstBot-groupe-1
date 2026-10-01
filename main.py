"""
carte.py - Carte vue du ciel de la piste (challenge 4).

À chaque image du suivi de ligne :
  1. detect_line (linefollowing_with_cam.py) dit si la ligne est vue et donne son décalage ;
  2. on retrouve le centre (cx, cy) de la ligne dans l'image ;
  3. robot.pixel_to_world(cx, cy) place ce point dans la salle (cm),
     grâce à l'homographie et à la position du robot (odométrie).
On garde aussi la position du robot (son trajet) à chaque pas.
"""
import json

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")          # pas d'écran sur le Raspberry Pi : on dessine dans un fichier
import matplotlib.pyplot as plt

from linefollowing_with_cam import detect_line, get_color_mask

COULEURS_TRACE = {"JAUNE": "gold", "BLEU": "tab:blue", "ROUGE": "tab:red", "VERT": "tab:green"}


def centre_ligne(hsv, couleur):
    """Centre (cx, cy) en pixels de la ligne détectée par detect_line, ou None.
    cx vient directement de detect_line (offset + w/2) ; cy est calculé sur le même contour."""
    h, w = hsv.shape[:2]
    detected, consigne, angle, offset = detect_line(hsv, couleur)
    if not detected:
        return None
    cx = offset + w / 2.0

    # même masque et même plus grand contour que dans detect_line, pour avoir cy
    mask = get_color_mask(hsv, couleur)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    cy = M["m01"] / M["m00"]
    return cx, cy


def angle_robot(robot):
    """Angle du robot (theta ou teta selon la version de robot.py)."""
    return getattr(robot, "theta", getattr(robot, "teta", 0.0))


class Carte:
    def __init__(self, robot):
        self.robot = robot
        self.points_ligne = []   # [couleur, x, y] : points de la ligne vus par la caméra (cm)
        self.trajet = []         # [couleur, x, y] : position du robot à chaque pas (cm)

    def enregistrer(self, frame, couleur):
        """À appeler à chaque image, avec l'image donnée à process_frame et la couleur suivie."""
        self.trajet.append([couleur, float(self.robot.x), float(self.robot.y)])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        pixel = centre_ligne(hsv, couleur)
        if pixel is not None:
            x, y = self.robot.pixel_to_world(*pixel)
            self.points_ligne.append([couleur, float(x), float(y)])

    def sauvegarder(self, fichier="parcours.json"):
        with open(fichier, "w") as f:
            json.dump({"points_ligne": self.points_ligne, "trajet": self.trajet}, f)
        print(f"\nParcours enregistré dans {fichier} ({len(self.points_ligne)} points de ligne)")

    def dessiner(self, fichier="carte.png"):
        plt.figure(figsize=(7, 7))
        for nom, trace in COULEURS_TRACE.items():
            xs, ys = [], []
            precedent = None
            for c, x, y in self.points_ligne:
                if c != nom:
                    continue
                # si le point est loin du précédent (> 10 cm), on coupe la ligne (nan = trou)
                if precedent and ((x - precedent[0]) ** 2 + (y - precedent[1]) ** 2) ** 0.5 > 10:
                    xs.append(float("nan"))
                    ys.append(float("nan"))
                xs.append(x)
                ys.append(y)
                precedent = (x, y)
            if xs:
                plt.plot(xs, ys, "-", color=trace, linewidth=3, label=nom)
        if self.trajet:
            xs, ys = zip(*[(x, y) for c, x, y in self.trajet])
            plt.plot(xs, ys, "-", color="grey", linewidth=0.8, label="trajet du robot")
        plt.plot(0, 0, "k^", markersize=10, label="départ")
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("x (cm)")
        plt.ylabel("y (cm)")
        plt.legend()
        plt.title("Carte de la piste (vue du ciel)")
        plt.savefig(fichier, dpi=150)
        plt.close()
        print(f"Carte dessinée dans {fichier}")


def dessiner_depuis_fichier(fichier="parcours.json", sortie="carte.png"):
    """Redessine la carte à partir du fichier, sans robot : python3 carte.py"""
    carte = Carte(robot=None)
    with open(fichier) as f:
        data = json.load(f)
    carte.points_ligne = data["points_ligne"]
    carte.trajet = data["trajet"]
    carte.dessiner(sortie)


if __name__ == "__main__":
    dessiner_depuis_fichier()