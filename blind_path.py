import time 
from robot import Robot

def follow_blind_path(path, robot: Robot):
    driver = DriveCam(robot)
    idx = 0
    period = 1 / 50
    prev_time = time.perf_counter()
    min_dist = 3

    try:
        while True:
            now = time.perf_counter()
            robot.update_pose_from_wheels(now - prev_time)
            prev_time = now
            x, y = robot.x, robot.y

            gx, gy = path[idx]
            while idx < len(path) - 1 and math.hypot(gx - x, gy - y) < min_dist:
                idx += 1
                gx, gy = path[idx]

            if idx == len(path) - 1 and math.hypot(gx - x, gy - y) < 3.0:
                break

            heading = math.degrees(math.atan2(gy - y, gx - x))
            err = normalize_angle(heading - robot.theta)
            driver.drive_autonome(err, True, is_searching=abs(err) > 90, v_effective=7.0)

            time.sleep(max(0.0, period - (time.perf_counter() - now)))
    finally:
        driver.stop()

        