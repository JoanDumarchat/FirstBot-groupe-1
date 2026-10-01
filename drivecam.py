"""
drivecam.py - Module de pilotage moteur asservi par la caméra.

Reçoit l'angle calculé par la caméra et commande le Robot (robot.py) :
tourne vers l'angle via le système de rotation centre fourni puis avance tout droit.
"""

class DriveCam:
    """Pilote le robot à partir des informations de vision de la caméra."""

    def __init__(self, robot, step_distance=4.0, **kwargs):
        """
        :param robot:         Instance de Robot (robot.py)
        :param step_distance: Distance d'avance en cm après chaque réalignement
        """
        self.robot = robot
        self.step_distance = step_distance

    def drive_autonome(self, consigne, is_active, *args, **kwargs):
        """
        Asservissement direct par angle :
        - Tourne vers l'angle reçu avec rotate_center_d
        - Avance tout droit (pur avancer) avec move_forward_d
        """
        if not is_active:
            self.robot.stop()
            return

        self.robot.drive_consigne(consigne, distance=self.step_distance)

    def stop(self):
        """Arrêt immédiat des moteurs."""
        self.robot.stop()
