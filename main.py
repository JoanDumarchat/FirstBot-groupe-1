import sys
import pypot.dynamixel
from odometry import record_movements
from robot import Robot


def challenge_goto(robot):
    print("Position initiale :")
    print(f"x={robot.x}, y={robot.y}, theta={robot.teta}")

    # Aller à x=30 cm, y=0 cm, theta=90°
    robot.go_to_xya(30, 0, 90)

    print("Position finale :")
    print(f"x={robot.x:.2f}, y={robot.y:.2f}, theta={robot.teta:.2f}")


def challenge_odom(robot):
    robot.wheels_io.disable_torque(robot.wheel_ids)
    robot.x, robot.y, robot.teta = 0., 0., 0.
    print("Roues libres : pousse le robot, puis Ctrl-C")
    print(record_movements(robot))


def main():
    ports = pypot.dynamixel.get_available_ports()

    if not ports:
        exit("No port")

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    found_ids = dxl_io.scan(range(10))

    print(f"Found motors with IDs: {found_ids}")

    if len(found_ids) < 2:
        exit("Port opened, but motors did not respond. Check batteries")

    robot = Robot(dxl_io, found_ids[:2])
    robot.wheels_io.set_wheel_mode(robot.wheel_ids)

    choix = sys.argv[1] if len(sys.argv) > 1 else input("goto ou odom ? ")

    try:
        if choix == "goto":
            challenge_goto(robot)
        elif choix == "odom":
            challenge_odom(robot)
        else:
            print("Choix inconnu : tape goto ou odom")

    finally:
        robot.stop()
        robot.wheels_io.disable_torque(robot.wheel_ids)


if __name__ == "__main__":
    main()