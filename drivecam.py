"""
drivecam.py - Module de pilotage moteur asservi par la caméra.

Reçoit l'angle et la consigne calculés par la caméra et commande le Robot (robot.py) :
- En autonome : utilise move_s(linear_speed, angular_speed) pour un suivi fluide sans à-coups.
- En manuel : avance, recule ou pivote continuellement tant que la touche/bouton est maintenu.
- En pause : stoppe immédiatement les moteurs.
"""

import math
import numpy as np


class DriveCam:
    """Pilote le robot à partir des informations de vision de la caméra."""

    def __init__(self, robot, step_distance=4.0, **kwargs):
        """
        :param robot:         Instance de Robot (robot.py)
        :param step_distance: Distance d'avance indicative
        """
        self.robot = robot
        self.step_distance = step_distance

    def drive_autonome(self, consigne, is_active, is_searching=False, transition_boost=0, v_effective=7.0):
        """
        Asservissement continu par move_s(linear_speed, angular_speed) :
        - consigne : angle en degrés (>0 droite, <0 gauche)
        - is_active : True si la ligne est détectée
        - v_effective : vitesse linéaire en cm/s
        """
        if not is_active:
            self.robot.stop()
            return

        # Gain d'asservissement angulaire inversé pour correspondre au sens physique du robot :
        # consigne > 0 (cible à droite) -> angular_speed > 0 (tourne à droite)
        # consigne < 0 (cible à gauche) -> angular_speed < 0 (tourne à gauche)
        consigne_rad = math.radians(float(consigne))
        angular_speed = consigne_rad * 1.6

        # Si le robot cherche la ligne (perdue temporairement), il pivote sur place
        if is_searching:
            linear_speed = 0.0
            angular_speed = float(np.sign(angular_speed) * 1.8) if angular_speed != 0 else -1.8
        else:
            linear_speed = float(v_effective)

        if hasattr(self.robot, 'move_s'):
            self.robot.move_s(linear_speed, angular_speed)
        elif hasattr(self.robot, 'drive_consigne'):
            self.robot.drive_consigne(-consigne, distance=self.step_distance)
        else:
            turn_angle = float(consigne)
            if abs(turn_angle) > 2.0:
                self.robot.rotate_center_d(turn_angle)
            if self.step_distance > 0:
                self.robot.move_forward_d(self.step_distance)

    def drive_pause(self, manual_dir=None):
        """Met le robot en arrêt lors de la pause."""
        self.robot.stop()
        return "ROBOT EN PAUSE"

    def drive_manuel(self, manual_dir, v_effective=7.0):
        """
        Pilotage manuel continu tant que la touche/bouton est maintenu :
        utilise move_s pour un déplacement fluide sans arrêt après 1 pas.
        """
        spd = float(v_effective) if v_effective else 7.0

        if not manual_dir:
            self.robot.stop()
            return "MANUEL ARRET"

        if hasattr(self.robot, 'move_s'):
            if manual_dir == 'up':
                self.robot.move_s(spd, 0.0)
                return "MANUEL AVANCER"
            elif manual_dir == 'down':
                self.robot.move_s(-spd, 0.0)
                return "MANUEL RECULER"
            elif manual_dir == 'left':
                self.robot.move_s(0.0, -1.8)
                return "MANUEL GAUCHE"
            elif manual_dir == 'right':
                self.robot.move_s(0.0, 1.8)
                return "MANUEL DROITE"
        else:
            if manual_dir == 'left':
                self.robot.rotate_center_d(15.0)
                return "MANUEL GAUCHE"
            elif manual_dir == 'right':
                self.robot.rotate_center_d(-15.0)
                return "MANUEL DROITE"
            elif manual_dir == 'up':
                self.robot.move_forward_d(self.step_distance)
                return "MANUEL AVANCER"
            elif manual_dir == 'down':
                self.robot.move_forward_d(-self.step_distance)
                return "MANUEL RECULER"

        self.robot.stop()
        return "MANUEL ARRET"

    def stop(self):
        """Arrêt immédiat des moteurs."""
        self.robot.stop()
