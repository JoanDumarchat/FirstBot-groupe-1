"""
carte.py - Carte vue du ciel de la piste (challenge 4).

À chaque image du suivi de ligne :
  1. detect_line (ci-dessous) trouve le centre (cx, cy) de la ligne dans l'image, si elle est vue
     (avec le masque de couleur get_color_mask de linefollowing_with_cam.py) ;
  2. robot.pixel_to_world(cx, cy) place ce point dans la salle (cm),
     grâce à l'homographie et à la position du robot (odométrie).
On garde aussi la position du robot (son trajet) à chaque pas.
"""
import json

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")          # pas d'écran sur le Raspberry Pi : on dessine dans un fichier
import matplotlib.pyplot as plt

from linefollowing_with_cam import get_color_mask

COULEURS_TRACE = {"JAUNE": "gold", "BLEU": "tab:blue", "ROUGE": "tab:red", "VERT": "tab:green"}


def detect_line(frame, couleur):
    """Centre (cx, cy) en pixels du plus grand morceau de ligne de cette couleur, ou None.
    frame = image BGR (get_color_mask fait elle-même la conversion HSV)."""
    mask = get_color_mask(frame, couleur)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= 120]
    if not valid:
        return None

    M = cv2.moments(max(valid, key=cv2.contourArea))
    if M["m00"] < 1.0:
        return None
    return M["m10"] / M["m00"], M["m01"] / M["m00"]


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

        if couleur not in COULEURS_TRACE:      # "DEPART" ou "FIN" : pas de ligne à suivre
            return
        pixel = detect_line(frame, couleur)
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
                if precedent and ((x - precedent[0]) ** 2 + (y - precedent[1]) ** 2) ** 0.5 < 2:
                    continue            # trop près du point précédent (< 2 cm) : ignoré → moins de zigzag
                # si le point est loin du précédent (> 10 cm), on coupe la ligne (nan = trou)
                if precedent and ((x - precedent[0]) ** 2 + (y - precedent[1]) ** 2) ** 0.5 > 10:
                    xs.append(float("nan"))
                    ys.append(float("nan"))
                xs.append(x)
                ys.append(y)
                precedent = (x, y)
            if xs:
                plt.plot(ys, xs, "-", color=trace, linewidth=3, label=nom)
        if self.trajet:
            xs, ys = zip(*[(x, y) for c, x, y in self.trajet])
            plt.plot(ys, xs, "-", color="grey", linewidth=0.8, label="trajet du robot")
        plt.plot(0, 0, "k^", markersize=10, label="départ")
        plt.axis("equal")
        plt.gca().invert_xaxis()        # y positif (gauche du robot) affiché à gauche : vraie vue du ciel
        plt.grid(True)
        plt.xlabel("y (cm)  ← gauche | droite →")
        plt.ylabel("x (cm)  (avant ↑)")
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
