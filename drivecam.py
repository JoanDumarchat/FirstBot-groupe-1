import math
import numpy as np


class DriveCam:
    def __init__(self, robot, step_distance=4.0, **kwargs):
        """
        :param robot:       
        :param step_distance: 
        """
        self.robot = robot
        self.step_distance = step_distance
        self.gain = 1.0   # force des corrections (avant 1.6) ; réglable au clavier avec g / h

    def drive_autonome(self, consigne, is_active, is_searching=False, v_effective=7.0):
        if not is_active:
            self.robot.stop()
            return

        angular_speed = math.radians(float(consigne)) * self.gain

        if is_searching:
            linear_speed = 0.3 * v_effective
            angular_speed = math.copysign(1.8, angular_speed) if angular_speed else -1.8
        else:
            turn_ratio = min(abs(consigne) / 40.0, 1.0)
            linear_speed = v_effective * (1.0 - 0.6 * turn_ratio)

        self.robot.move_s(linear_speed, angular_speed)

    def drive_pause(self, manual_dir=None):
        self.robot.stop()
        return "ROBOT EN PAUSE"

    def drive_manuel(self, manual_dir, v_effective=7.0):

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
        self.robot.stop()
