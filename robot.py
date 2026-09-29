import pypot.dynamixel
import time

WHEEL_SPACING = 15.0 # cm

def get_turn_radius(wheel_speed) :
    if (wheel_speed[1] - wheel_speed[0] == 0):
        return 0
    return (wheel_speed[0] * (WHEEL_SPACING/2)) / (wheel_speed[1] - wheel_speed[0])  

def get_turn_speed (wheel_speed):
    return (wheel_speed[0] - wheel_speed[1]) / WHEEL_SPACING

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
        while (1):
            print(get_turn_radius(dxl_io.get_present_speed(found_ids)), get_turn_speed(dxl_io.get_present_speed(found_ids)))
        print(dxl_io.get_control_table(found_ids))

        dxl_io.set_moving_speed({mid: 360 for mid in found_ids})
        time.sleep(1)

        dxl_io.set_moving_speed({mid: 0 for mid in found_ids})
        dxl_io.disable_torque(found_ids)


if __name__ == '__main__':
    main()