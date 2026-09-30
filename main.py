import pypot.dynamixel
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
       
        robot.move_forward_s (360)

        keyboard.wait('esc')

        robot.stop
        robot.wheels_io.disable_torque(found_ids)


if __name__ == '__main__':
    main() 