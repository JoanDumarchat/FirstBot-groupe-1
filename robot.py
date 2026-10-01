import time
import math
import cv2
import numpy as np

### Evite les grands tours inutiles ###

def normalize_angle(a):
    return (a + 180) % 360 - 180

class Robot:

    ## Roues ##

    WHEEL_SPACING = 13.0 # cm (entraxe réel calibré sur le robot physique)
    WHEEL_DIAMETER = 5.16
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
    WHEEL_MODIFIER = {1 : -1, 2 : 1}
    WHEEL_SPEED = 360
    WHEEL_LEFT_ID = 2
    WHEEL_RIGHT_ID = 1
    LOCAL = [(0.0, 0.0), (0.0, -0.04), (0.0295, -0.0055), (0.0295, -0.0305)]
    PIXELS = [(34, 179), (308, 176), (74, 72), (206, 65)]

    ROBOT_ROTATE_CIRC = 2 * math.pi * (WHEEL_SPACING / 2)
  
    ## Tolérance ##
    POSITION_TOL = 0.75 #cm
    ANGLE_TOL = 1.25
    
    def __init__(self, wheels_io, wheel_ids):
        self.wheels_io = wheels_io
        self.wheel_ids = wheel_ids
        self.x = 0.
        self.y = 0.
        self.theta = 0.

    ########## GETTERS ##########
    ### Kinematics ### 
    """Vitesses des roues (rad/s) -> (v en cm/s, omega en rad/s).""" 
    def direct_kinematics(self, v_left, v_right): 
        r = self.WHEEL_RADIUS 
        L = self.WHEEL_SPACING 
        v = (r / 2) * (v_right + v_left) 
        omega = (r / L) * (v_right - v_left) 
        return v, omega
    """(v en cm/s, omega en rad/s) -> vitesses des roues (gauche, droite) en rad/s.""" 
    def inverse_kinematics(self, v, omega): 
        r = self.WHEEL_RADIUS 
        L = self.WHEEL_SPACING 
        v_right = (2 * v + L * omega) / (2 * r) 
        v_left = (2 * v - L * omega) / (2 * r) 
        return v_left, v_right

    ### ODOMETRY ###

    def odom(self, x_dot, theta_dot, dt):
        d_theta = theta_dot * dt
        if abs(theta_dot) < 1e-6:
            return x_dot * dt, 0.0, d_theta
        r = x_dot / theta_dot
        dx = r * np.sin(d_theta)
        dy = r * (1 - np.cos(d_theta))
        return dx, dy, d_theta

    def tick_odom(self, x ,y ,theta , x_dot, theta_dot, dt):
        dx, dy, d_theta = self.odom(x_dot,theta_dot,dt)
        xn = x + dx * np.cos(theta) - dy * np.sin(theta)
        yn = y + dx * np.sin(theta) + dy * np.cos(theta)
        theta_n = d_theta + theta

        return xn ,yn , theta_n

    def update_pose(self, v, omega, dt):
        x, y, theta = self.tick_odom(self.x, self.y, math.radians(self.theta), v, math.radians(omega), dt)
        self.x, self.y = x, y
        self.theta = normalize_angle(math.degrees(theta))

    def update_pose_from_wheels(self, dt):
        """Met à jour x, y, theta à partir de la vitesse réelle des roues (pour move_s)."""
        speeds = self.wheels_io.get_present_speed(self.wheel_ids)
        v_left = math.radians(speeds[0] * self.WHEEL_MODIFIER[self.wheel_ids[0]])
        v_right = math.radians(speeds[1] * self.WHEEL_MODIFIER[self.wheel_ids[1]])
        v, omega = self.direct_kinematics(v_left, v_right)
        self.update_pose(v, math.degrees(omega), dt)

    ##########  MOVE ROBOT ##########

    def set_wheel_speeds(self, cmd):
        self.wheels_io.set_moving_speed(cmd)
        left, right = self.wheel_ids
        v_left = math.radians(cmd[left] * self.WHEEL_MODIFIER[left])
        v_right = math.radians(cmd[right] * self.WHEEL_MODIFIER[right])
        v, omega = self.direct_kinematics(v_left, v_right)

        return v, math.degrees(omega)

    def drive(self, distance=4.0, angle=0.0):
        """
        avvance de base et si reçoit un angle tourne de cette angle"""
        if abs(angle) > self.ANGLE_TOL:
            self.rotate_center_d(angle)

        if distance > 0:
            self.move_forward_d(distance)

    def drive_consigne(self, *args, distance=4.0):
        """
        reçoit l'angle de la cam
        """
        if len(args) == 1:
            consigne_deg = args[0]
        elif len(args) >= 2:
            consigne_deg = args[1]
        else:
            consigne_deg = 0.0

        # Consigne caméra : >0 pour droite, <0 pour gauche.
        # rotate_center_d : négatif pour tourner à droite, positif pour gauche.
        turn_angle = - float(consigne_deg)
        self.drive(distance=distance, angle=turn_angle)

    def move_d (self, distance, angle):
        if angle != 0:
            sign = 1 if angle >= 0 else -1
            cmd = {mid: sign * self.WHEEL_SPEED for mid in self.wheel_ids}
            v, omega = self.set_wheel_speeds(cmd)
            dt = (self.ROBOT_ROTATE_CIRC * ( sign * angle ) / 360) / self.WHEEL_CIRC
            time.sleep(dt)
            self.update_pose(v, omega, dt)

        if distance != 0:
            sign = 1 if distance >= 0 else -1
            cmd = {mid: sign * self.WHEEL_SPEED * self.WHEEL_MODIFIER[mid] for mid in self.wheel_ids}
            v, omega = self.set_wheel_speeds(cmd)
            dist = sign * distance / self.WHEEL_CIRC
            time.sleep(dist)
            self.update_pose(v, omega, dist)

    def move_s (self, linear_speed, angular_speed):
        omega_rad = angular_speed
        if abs(angular_speed) > 3.15:
            omega_rad = math.radians(angular_speed)

        ang_limit = min(abs(omega_rad), math.pi / 2.0)
        corrected_linear = linear_speed * math.cos(ang_limit)

        v_left_rad, v_right_rad = self.inverse_kinematics(corrected_linear, omega_rad)

        left_deg = math.degrees(v_left_rad) * self.WHEEL_MODIFIER[self.wheel_ids[0]]
        right_deg = math.degrees(v_right_rad) * self.WHEEL_MODIFIER[self.wheel_ids[1]]

        max_spd = getattr(self, 'WHEEL_SPEED', 360)
        left_deg = float(np.clip(left_deg, -max_spd, max_spd))
        right_deg = float(np.clip(right_deg, -max_spd, max_spd))

        self.wheels_io.set_moving_speed({
            self.wheel_ids[0]: left_deg,
            self.wheel_ids[1]: right_deg
        })

    def stop (self):
        self.wheels_io.set_moving_speed({mid: 0 for mid in self.wheel_ids})
        return
    
    ### GO TO ###

    def go_to_xya(self, x, y, theta):
        dx = x - self.x
        dy = y - self.y
        distance = np.hypot(dx, dy)

        if distance > self.POSITION_TOL:
            heading = np.degrees(np.arctan2(dy, dx))
 
            turn = normalize_angle(heading - self.theta)
            if abs(turn) <= self.ANGLE_TOL:
                turn = 0.0
 
            self.move_d(distance, turn)     # tourne vers la cible, puis avance
 
        turn = normalize_angle(theta - self.theta)
        if abs(turn) > self.ANGLE_TOL:
            self.move_d(0.0, turn)

    ## Pixel to robot / to world

    TX = 0.104  # milieu des roues -> croix proche gauche, vers l'avant
    TY = 0.01   # milieu des roues -> croix proche gauche, sur le côté

    H, _ = cv2.findHomography(np.float32(PIXELS), np.float32(LOCAL))

    def pixel_to_robot(self, x, y):
        p = self.H @ [x, y, 1]
        return p[0] / p[2] + self.TX, p[1] / p[2] + self.TY

    def pixel_to_world(self, x, y):
        xr, yr = self.pixel_to_robot(x, y)
        xr, yr = xr * 100, yr * 100
        t = math.radians(self.theta)
        return (self.x + xr * math.cos(t) - yr * math.sin(t),
                self.y + xr * math.sin(t) + yr * math.cos(t))