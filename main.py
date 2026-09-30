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
       

        while (True):
            if keyboard.is_pressed('z'):
                robot.move_forward_s (360)
            elif keyboard.is_pressed('s'):
                robot.move_forward_s (-360)
            elif keyboard.is_pressed('q'):
                robot.rotate_center_s(360)
            elif keyboard.is_pressed('d'):
                robot.rotate_center_s(-360)
            else:
                robot.stop()

            if keyboard.is_pressed('esc'):
                break

        robot.wheels_io.disable_torque(found_ids)


if __name__ == '__main__':
    main() 