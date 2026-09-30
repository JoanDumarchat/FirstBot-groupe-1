import time 
import numpy as np
from robot import Robot

def record_movements (robot : Robot):
    loop_freq = 50.0
    sleep_time = 1.0 / 50.0

    last_time = time.perf_counter()
    try:
        while (True):
            current_time = time.perf_counter()
            dt = current_time - last_time
            last_time = current_time

            wheels_speed = robot.wheels_io.get_present_speed(robot.wheel_ids)
            v_left = np.radians(wheels_speed[0])
            v_right = np.radians(wheels_speed[1])

            linear_speed, angular_speed = robot.direct_kinematics(v_left, v_right)

            x, y, teta = robot.tick_odom(robot.x, robot.y, robot.teta, linear_speed, angular_speed, dt)

            robot.x = x
            robot.y = y
            robot.teta = teta

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped recording.")
        return robot.x, robot.y, robot.teta
