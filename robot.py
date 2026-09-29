import pypot.dynamixel
import time

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])

found_ids = dxl_io.scan(range(10)) 
print(f"Found motors with IDs: {found_ids}")

if not found_ids:
    print("Port opened, but no motors responded. Check 12V power supply!")
else :
    dxl_io.set_wheel_mode(found_ids)

    dxl_io.set_moving_speed({mid: 200 for mid in found_ids})
    time.sleep(1)

    dxl_io.set_moving_speed({mid: 0 for mid in found_ids})
    dxl_io.disable_torque(found_ids)