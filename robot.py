import pypot.dynamixel
import time
import math


class Robot:

    WHEEL_SPACING = 15.0 # cm
    WHEEL_DIAMETER = 5.16
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
    WHEEL_MODIFIER = [1, -1]

    ROBOT_ROTATE_CIRC = 2 * math.pi * (WHEEL_SPACING / 2)

    def __init__(self, wheels_io, wheel_ids):
        self.wheels_io = wheels_io
        self.wheel_ids = wheel_ids
        self.x = 0.
        self.y = 0.
        self.teta = 0.
       
       
    def get_turn_radius(self, wheel_speed) -> float :
        if (wheel_speed[1] - wheel_speed[0] == 0):
            return 0
        return (wheel_speed[0] * (self.WHEEL_SPACING/2)) / (wheel_speed[1] - wheel_speed[0])

    def get_turn_speed (self, wheel_speed) -> float :
        return (wheel_speed[0] - wheel_speed[1]) / self.WHEEL_SPACING

    def move_forward (self, distance):
        self.wheels_io.set_moving_speed({mid: 360*self.WHEEL_MODIFIER[mid-1] for mid in self.wheel_ids})
        time.sleep(distance / self.WHEEL_CIRC)
        return


    def rotate_center (self, angle):
        self.wheels_io.set_moving_speed({mid: 360 for mid in self.wheel_ids})
        time.sleep((self.ROBOT_ROTATE_CIRC / (angle / 180)) / self.WHEEL_CIRC)
        return


def main():
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit('No port')
        
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    found_ids = dxl_io.scan(range(10))
    print(f"Found motors with IDs: {found_ids}")
    
    if not found_ids:
        print("Port opened, but no motors responded. Check batteries")

    else :
        robot = Robot(dxl_io, found_ids)
        robot.wheels_io.set_wheel_mode(found_ids)
       
        robot.rotate_center (360)
        robot.wheels_io.disable_torque(found_ids)


if __name__ == '__main__':
    main() 