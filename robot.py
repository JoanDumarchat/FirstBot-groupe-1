import time
import math
import cv2
import numpy as np

### Evite les grands tours inutiles ###

def normalize_angle(a):
    return (a + 180) % 360 - 180

class Robot:

    ## Roues ##

    WHEEL_SPACING = 13.0 # cm
    WHEEL_DIAMETER = 5.16
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
    WHEEL_MODIFIER = {1 : -1, 2 : 1}
    WHEEL_SPEED = 360
    LOCAL = [(0.0, 0.0), (0.0, -0.04), (0.0295, -0.0055), (0.0295, -0.0305)]
    PIXELS = [(34, 179), (308, 176), (74, 72), (206, 65)]

    ROBOT_ROTATE_CIRC = 2 * math.pi * (WHEEL_SPACING / 2)
  
    ## Tolérance ##
    POSITION_TOL = 0.5 #cm
    ANGLE_TOL = 1
    
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

    def update_pose(self, v, omega, dist):
        self.x, self.y, self.theta = self.tick_odom(self.x, self.y, self.theta, v, omega, dist)
        self.theta = normalize_angle(self.theta)

    ##########  MOVE ROBOT ##########

    def set_wheel_speeds(self, cmd):
        self.wheels_io.set_moving_speed(cmd)
        left, right = self.wheel_ids
        v_left = math.radians(cmd[left] * self.WHEEL_MODIFIER[left])
        v_right = math.radians(cmd[right] * self.WHEEL_MODIFIER[right])
        v, omega = self.direct_kinematics(v_left, v_right)

        return v, math.degrees(omega)

    def move_forward_d (self, distance):
        sign = 1 if distance >= 0 else -1
        cmd = {mid: sign * self.WHEEL_SPEED * self.WHEEL_MODIFIER[mid] for mid in self.wheel_ids}
        v, omega = self.set_wheel_speeds(cmd)
        dist = sign * distance / self.WHEEL_CIRC
        time.sleep(dist)
        self.stop()
        self.update_pose(v, omega, dist)

    def move_forward_s (self, speed):
        self.wheels_io.set_moving_speed({mid: speed*self.WHEEL_MODIFIER[mid] for mid in self.wheel_ids})

    def rotate_center_d (self, angle):
        sign = 1 if angle >= 0 else -1
        cmd = {mid: sign * self.WHEEL_SPEED for mid in self.wheel_ids}
        v, omega = self.set_wheel_speeds(cmd)
        dt = (self.ROBOT_ROTATE_CIRC * ( sign * angle ) / 360) / self.WHEEL_CIRC
        time.sleep(dt)
        self.stop()
        self.update_pose(v, omega, dt)

    def rotate_center_s (self, speed):
        self.wheels_io.set_moving_speed({mid: speed for mid in self.wheel_ids})
        return

    def stop (self):
        self.wheels_io.set_moving_speed({mid: 0 for mid in self.wheel_ids})
        return
    
    def pixel_to_robot(x, y):
        return x 

    ### GO TO ###

    def go_to_xya(self, x, y, theta):
        dx = x - self.x
        dy = y - self.y
        distance = np.hypot(dx, dy)

        if distance > self.POS_TOL:
            heading = np.degrees(np.arctan2(dy, dx))

            turn = normalize_angle(heading - self.theta)
            if abs(turn) > self.ANGLE_TOL:
                self.rotate_center_d(turn)

            self.move_forward_d(distance)

        turn = normalize_angle(theta - self.theta)
        if abs(turn) > self.ANGLE_TOL:
            self.rotate_center_d(turn)

    ## Pixel to robot / to world

    TX = 0.104  # milieu des roues -> croix proche gauche, vers l'avant
    TY = 0.01   # milieu des roues -> croix proche gauche, sur le côté

    H, _ = cv2.findHomography(np.float32(PIXELS), np.float32(LOCAL))

    def pixel_to_robot(self, x, y):
        p = self.H @ [x, y, 1]
        return p[0] / p[2] + self.TX, p[1] / p[2] + self.TY

    def pixel_to_world(self, x, y):
        xr, yr = self.pixel_to_robot(x, y)
        return (self.x + xr * math.cos(self.theta) - yr * math.sin(self.theta),
                self.y + xr * math.sin(self.theta) + yr * math.cos(self.theta))
