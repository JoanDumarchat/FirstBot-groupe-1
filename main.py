"""import pypot.dynamixel
import keyboard
from robot import Robot

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
       
        input()
        
        robot.stop()


        robot.wheels_io.disable_torque(found_ids)


if __name__ == '__main__':
    main() 
"""

import pypot.dynamixel

from robot import Robot


def main():
    ports = pypot.dynamixel.get_available_ports()

    if not ports:
        exit("No port")

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    found_ids = dxl_io.scan(range(10))

    print(f"Found motors with IDs: {found_ids}")

    if len(found_ids) < 2:
        exit("Il faut 2 moteurs.")

    robot = Robot(dxl_io, found_ids[:2])

    robot.wheels_io.set_wheel_mode(robot.wheel_ids)

    try:
        print("Position initiale :")
        print(f"x={robot.x}, y={robot.y}, theta={robot.teta}")

        # Aller à x=30 cm, y=0 cm, theta=90°
        robot.go_to_xya(30, 0, 90)

        print("Position finale :")
        print(f"x={robot.x:.2f}, y={robot.y:.2f}, theta={robot.teta:.2f}")

    finally:
        robot.stop()
        robot.wheels_io.disable_torque(robot.wheel_ids)


if __name__ == "__main__":
    main()