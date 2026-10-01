"""
simulateur.py - Teste le suivi de ligne + la carte SANS robot (sur le Mac).

- Une piste virtuelle (vue du ciel, 1 pixel = 1 mm) : jaune -> bleu -> rouge, avec un carré vert
  au départ et à chaque changement de couleur.
- Une fausse caméra : pour chaque pixel de l'image, l'homographie de robot.py (pixel_to_robot)
  donne le point du sol vu ; on lit sa couleur sur la piste.
- De faux moteurs : les vitesses envoyées par move_s font bouger le "vrai" robot
  (roue gauche = moteur WHEEL_LEFT_ID, roue droite = moteur WHEEL_RIGHT_ID).
- On lance la VRAIE boucle challenge_line_following de main.py, avec une fausse horloge.

Usage : python3 simulateur.py [nb_images]
Résultats : sim_piste.png (vrai trajet sur la piste), carte.png et parcours.json (la carte du robot).
"""
import math
import sys
import time

import cv2
import numpy as np

import main
from robot import Robot

BGR = {"JAUNE": (20, 210, 230), "BLEU": (200, 90, 20), "ROUGE": (40, 40, 200), "VERT": (60, 170, 40)}
SOL = (125, 125, 125)
LARGEUR_RUBAN = 20      # mm
DT = 1 / 20.0           # 20 images par seconde
MM = 10.0               # 1 cm = 10 pixels de piste


def creer_piste():
    """Boucle en rectangle arrondi, départ en (0, 0) vers +x. Renvoie l'image, l'origine (pixels) et le chemin."""
    img = np.full((1600, 2200, 3), SOL, np.uint8)
    ox, oy = 450, 1450                          # pixel de la piste où est le robot au départ
    # chemin en cm, dans le repère du robot au départ (x devant, y à gauche)
    pts, R = [], 30.0
    def ligne(a, b, n=60):
        for t in np.linspace(0, 1, n, endpoint=False):
            pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    def arc(cx, cy, a0, a1, n=40):
        for a in np.linspace(a0, a1, n, endpoint=False):
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a)))
    L, H = 120.0, 70.0
    ligne((-10, 0), (L, 0)); arc(L, R, -math.pi / 2, 0)
    ligne((L + R, R), (L + R, R + H)); arc(L, R + H, 0, math.pi / 2)
    ligne((L, 2 * R + H), (0, 2 * R + H)); arc(0, R + H, math.pi / 2, math.pi)
    ligne((-R, R + H), (-R, R)); arc(0, R, math.pi, 1.5 * math.pi)
    chemin = np.array(pts)

    def px(p):   # cm (repère départ) -> pixel de la piste (y vers le haut)
        return int(ox + p[0] * MM), int(oy - p[1] * MM)

    n = len(chemin)
    D = 10   # le jaune commence au point 10 (~12 cm devant le robot) : le vert du départ y est, sous la caméra
    couleurs = ["JAUNE"] * (n // 3) + ["BLEU"] * (n // 3) + ["ROUGE"] * (n - 2 * (n // 3))
    couleurs = couleurs[-D:] + couleurs[:-D]
    for i in range(n - 1):
        cv2.line(img, px(chemin[i]), px(chemin[i + 1]), BGR[couleurs[i]], LARGEUR_RUBAN)
    cv2.line(img, px(chemin[-1]), px(chemin[0]), BGR[couleurs[-1]], LARGEUR_RUBAN)

    # carrés verts (4 cm) collés à droite du ruban, à chaque changement de couleur (rouge->jaune = départ)
    for i in [D, D + n // 3, D + 2 * (n // 3)]:
        p, q = chemin[i], chemin[i + 1]
        d = np.array(q) - np.array(p); d /= np.linalg.norm(d)
        droite = np.array([d[1], -d[0]])
        c = np.array(p) + droite * 3.5
        coins = [c + d * s1 * 2 + droite * s2 * 2 for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
        cv2.fillConvexPoly(img, np.array([px(k) for k in coins], np.int32), BGR["VERT"])
    return img, (ox, oy), chemin


class Monde:
    """Le "vrai" robot : sa position réelle et ce que voit sa caméra."""

    def __init__(self, robot, gauche_droite_inverses=False):
        self.piste, (self.ox, self.oy), self.chemin = creer_piste()
        self.robot = robot
        self.X, self.Y, self.TH = 0.0, 0.0, 0.0       # vraie position (cm, rad)
        self.vitesses = {}
        self.trace = []
        g, d = Robot.WHEEL_LEFT_ID, Robot.WHEEL_RIGHT_ID
        self.gauche, self.droite = (d, g) if gauche_droite_inverses else (g, d)
        # pour chaque pixel caméra : point du sol vu, en cm dans le repère du robot (calculé une fois)
        u, v = np.meshgrid(np.arange(320), np.arange(240))
        p = np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
        q = Robot.H @ p
        self.cam_x = ((q[0] / q[2]) + Robot.TX) * 100
        self.cam_y = ((q[1] / q[2]) + Robot.TY) * 100

    def avancer(self, dt):
        r, L = Robot.WHEEL_RADIUS, Robot.WHEEL_SPACING
        vg = math.radians(self.vitesses.get(self.gauche, 0) * Robot.WHEEL_MODIFIER[self.gauche])
        vd = math.radians(self.vitesses.get(self.droite, 0) * Robot.WHEEL_MODIFIER[self.droite])
        v, w = r / 2 * (vg + vd), r / L * (vd - vg)
        self.X += v * math.cos(self.TH + w * dt / 2) * dt
        self.Y += v * math.sin(self.TH + w * dt / 2) * dt
        self.TH += w * dt
        self.trace.append((self.X, self.Y))

    def image_camera(self):
        c, s = math.cos(self.TH), math.sin(self.TH)
        wx = self.X + self.cam_x * c - self.cam_y * s
        wy = self.Y + self.cam_x * s + self.cam_y * c
        px = np.clip((self.ox + wx * MM).astype(int), 0, self.piste.shape[1] - 1)
        py = np.clip((self.oy - wy * MM).astype(int), 0, self.piste.shape[0] - 1)
        img = self.piste[py, px].reshape(240, 320, 3).astype(np.int16)
        img += np.random.randint(-8, 9, img.shape, dtype=np.int16)      # bruit caméra
        return np.clip(img, 0, 255).astype(np.uint8)

    def distance_au_ruban(self):
        return float(np.min(np.hypot(self.chemin[:, 0] - self.X, self.chemin[:, 1] - self.Y)))


def simuler(nb_images=2400, gauche_droite_inverses=False, sortie="sim_piste.png"):
    horloge = [1000.0]
    time.time = time.perf_counter = lambda: horloge[0]
    time.sleep = lambda s: None

    class FauxMoteurs:
        def set_moving_speed(self, cmd):
            monde.vitesses.update(cmd)
        def get_present_speed(self, ids):
            return [monde.vitesses.get(i, 0) for i in ids]

    robot = Robot(FauxMoteurs(), [1, 2])          # comme main.py : found_ids[:2] = [1, 2]
    monde = Monde(robot, gauche_droite_inverses)
    stats = {"images": 0, "ecart_max": 0.0}

    class FausseCamera:
        def __init__(self, *a): pass
        def set(self, *a): return True
        def isOpened(self): return True
        def release(self): pass
        def read(self):
            if stats["images"] >= nb_images:
                raise KeyboardInterrupt
            if stats["images"]:
                monde.avancer(DT)
                horloge[0] += DT
            stats["images"] += 1
            stats["ecart_max"] = max(stats["ecart_max"], monde.distance_au_ruban()) if stats["images"] > 40 else 0
            return True, monde.image_camera()

    main.cv2.VideoCapture = FausseCamera
    main.challenge_line_following(robot)

    # dessin du vrai trajet sur la piste
    img = monde.piste.copy()
    pts = np.array([(monde.ox + x * MM, monde.oy - y * MM) for x, y in monde.trace], np.int32)
    if len(pts) > 1:
        cv2.polylines(img, [pts], False, (0, 0, 0), 4)
    cv2.circle(img, (monde.ox, monde.oy), 15, (0, 0, 0), -1)
    cv2.imwrite(sortie, img)

    tour = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(monde.trace, monde.trace[1:]))
    print(f"\n[SIM] {stats['images']} images ({stats['images'] * DT:.0f} s) | distance parcourue {tour:.0f} cm"
          f" | écart max au ruban {stats['ecart_max']:.1f} cm")
    print(f"[SIM] vraie position finale x={monde.X:.1f} y={monde.Y:.1f} | odométrie x={robot.x:.1f} y={robot.y:.1f}")
    print(f"[SIM] vrai trajet dessiné dans {sortie}")
    return stats, monde, robot


if __name__ == "__main__":
    simuler(int(sys.argv[1]) if len(sys.argv) > 1 else 2400)
