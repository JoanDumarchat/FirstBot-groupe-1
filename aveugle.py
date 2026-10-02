"""
aveugle.py - Tour à l'aveugle (challenge 5) à partir de parcours.json.

On relit les points de la ligne enregistrés par la carte (pixel_to_world),
on garde un point tous les PAS cm, puis le robot va de point en point avec
go_to_xya, sans caméra (seulement l'odométrie).
Le robot doit être posé au même départ que pendant l'enregistrement (x=0, y=0, theta=0).
"""
import json
import math

PAS = 10.0         # cm entre deux points visés
MAX_VIRAGE = 60.0  # degrés : un point qui demande de tourner plus que ça est ignoré


def charger_points(fichier="parcours.json", couleur=None):
    """Points (x, y) de la ligne, dans l'ordre où ils ont été vus, un tous les PAS cm."""
    with open(fichier) as f:
        data = json.load(f)

    points = [(0.0, 0.0)]
    direction = 0.0   # direction du robot au départ (degrés)
    for c, x, y in data["points_ligne"]:
        if couleur is not None and c != couleur:
            continue
        # 1. trop près du dernier point gardé : on l'ignore
        if math.dist(points[-1], (x, y)) < PAS:
            continue
        # 2. il faudrait faire demi-tour pour y aller (point vu pendant une recherche) : on l'ignore
        nouvelle = math.degrees(math.atan2(y - points[-1][1], x - points[-1][0]))
        virage = (nouvelle - direction + 180) % 360 - 180
        if abs(virage) > MAX_VIRAGE:
            continue
        points.append((x, y))
        direction = nouvelle
    return points[1:]


def tour_aveugle(robot, couleur=None, fichier="parcours.json"):
    points = charger_points(fichier, couleur)
    print(f"Tour à l'aveugle {couleur or '(toutes les couleurs)'} : {len(points)} points")

    robot.x, robot.y, robot.theta = 0., 0., 0.
    for i, (x, y) in enumerate(points):
        # on arrive sur le point en regardant vers lui (angle en degrés)
        angle = math.degrees(math.atan2(y - robot.y, x - robot.x))
        robot.go_to_xya(x, y, angle)
        print(f"point {i + 1}/{len(points)} : x={robot.x:.1f} y={robot.y:.1f} theta={robot.theta:.1f}")

    robot.stop()
    print("Tour à l'aveugle terminé")