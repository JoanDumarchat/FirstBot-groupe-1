import time
import math

import numpy as np


class Robot:

    WHEEL_SPACING = 13.0 # cm
    WHEEL_DIAMETER = 5.16
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
    WHEEL_MODIFIER = {1 : -1, 2 : 1}

    ROBOT_ROTATE_CIRC = 2 * math.pi * (WHEEL_SPACING / 2)

    def __init__(self, wheels_io, wheel_ids):
        self.wheels_io = wheels_io
        self.wheel_ids = wheel_ids
        self.x = 0.
        self.y = 0.
        self.teta = 0.

    ########## GETTERS ##########
       
    def get_turn_radius(self, wheel_speed) -> float :
        if (wheel_speed[1] - wheel_speed[0] == 0):
            return 0
        return (wheel_speed[0] * (self.WHEEL_SPACING/2)) / (wheel_speed[1] - wheel_speed[0])

    def get_turn_speed (self, wheel_speed) -> float :
        return (wheel_speed[0] - wheel_speed[1]) / self.WHEEL_SPACING

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
        dx = x_dot * dt
        dy = 0.0
        d_theta = theta_dot * dt

        return dx, dy, d_theta

    def tick_odom(self, x ,y ,theta , x_dot, theta_dot, dt):
        dx, dy, d_theta = self.odom(x_dot,theta_dot,dt)
        xn= dx * np.cos(theta) +x
        yn= dx * np.sin(theta) +y
        theta_n =d_theta*dt +theta

        return xn ,yn , theta_n

    ##########  MOVE ROBOT ##########

    def move_forward_d (self, distance):
        self.wheels_io.set_moving_speed({mid: 360*self.WHEEL_MODIFIER[mid] for mid in self.wheel_ids})
        time.sleep(distance / self.WHEEL_CIRC)
        return

    def move_forward_s (self, speed):
        self.wheels_io.set_moving_speed({mid: speed*self.WHEEL_MODIFIER[mid] for mid in self.wheel_ids})

    def rotate_center_d (self, angle):
        self.wheels_io.set_moving_speed({mid: 360 for mid in self.wheel_ids})
        time.sleep((self.ROBOT_ROTATE_CIRC / (angle / 180)) / self.WHEEL_CIRC)
        return

    def rotate_center_s (self, speed):
        self.wheels_io.set_moving_speed({mid: speed for mid in self.wheel_ids})
        return

    def stop (self):
        self.wheels_io.set_moving_speed({mid: 0 for mid in self.wheel_ids})
        return