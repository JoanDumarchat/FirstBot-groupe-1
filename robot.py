import pypot.dynamixel
import time
import math



WHEEL_MODIFIER = [1, -1]

class Robot:

    WHEEL_SPACING = 15.0 # cm
    WHEEL_DIAMETER = 5.16
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
    ROBOT_ROTATE_CIRC = 2 * math.pi * (WHEEL_DIAMETER / 2)
    
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.teta = 0.
        

    def get_turn_radius(wheel_speed) -> float :
        if (wheel_speed[1] - wheel_speed[0] == 0):
            return 0
        return (wheel_speed[0] * (WHEEL_SPACING/2)) / (wheel_speed[1] - wheel_speed[0])

    def get_turn_speed (wheel_speed) -> float :
        return (wheel_speed[0] - wheel_speed[1]) / WHEEL_SPACING

    def move_forward (wheels_io, wheel_ids, distance):
        wheels_io.set_moving_speed({mid: 360*WHEEL_MODIFIER[mid-1] for mid in wheel_ids})
        time.sleep(distance / WHEEL_CIRC)
        return

    def rotate_center (wheels_io, wheel_ids, angle):
        wheels_io.set_moving_speed({mid: 360 for mid in wheel_ids})
        time.sleep(ROBOT_ROTATE_CIRC / (angle / 360) / WHEEL_CIRC)
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
        dxl_io.set_wheel_mode(found_ids)
        
        # rotate_center (dxl_io, found_ids, 360)

        

        dxl_io.set_moving_speed({1 : -45})
        dxl_io.set_moving_speed({2:90})
        time.sleep(3)
        dxl_io.set_moving_speed({mid: 0 for mid in found_ids})
        dxl_io.disable_torque(found_ids)


if __name__ == '__main__':
    main()