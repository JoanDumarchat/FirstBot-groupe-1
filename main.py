import sys
import pypot.dynamixel
import time
import cv2
from odometry import record_movements
from robot import Robot
from drivecam import DriveCam
from linefollowing_with_cam import LineFollower


def challenge_goto(robot):
    print("Position initiale :")
    print(f"x={robot.x}, y={robot.y}, theta={robot.theta}")

    # Aller à x=30 cm, y=0 cm, theta=90°
    robot.go_to_xya(100, 50, 90)
    print (robot.x,robot.y)
    print (robot.theta)

    robot.go_to_xya(0, 0, 0)

    print("Position finale :")
    print(f"x={robot.x:.2f}, y={robot.y:.2f}, theta={robot.theta:.2f}")


def challenge_odom(robot):
    robot.wheels_io.disable_torque(robot.wheel_ids)
    robot.x, robot.y, robot.theta = 0., 0., 0.
    print("Roues libres : pousse le robot, puis Ctrl-C")
    print(record_movements(robot))


def challenge_line_following(robot):
    """Suivi de ligne caméra autonome (version épurée sans interface web)."""
    print("\n--- SUIVI DE LIGNE CAMERA ---")
    print("Initialisation de la caméra...")
    backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
    cap = cv2.VideoCapture(0, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    time.sleep(0.5)

    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra.")
        return

    follower = LineFollower(initial_target="JAUNE")
    driver = DriveCam(robot, step_distance=4.0)

    print("Suivi autonome en cours (Ctrl+C pour arrêter)...")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Remise à l'endroit si caméra montée tête en bas sur le châssis
            if sys.platform.startswith("linux"):
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            consigne, is_active, status = follower.process_frame(frame)
            driver.drive_autonome(consigne, is_active)

            sys.stdout.write(f"\r{status}   ")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt du suivi de ligne demandé.")
    finally:
        cap.release()
        driver.stop()


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

    choix = sys.argv[1] if len(sys.argv) > 1 else input("1: Base (goto/odom) ou 2: Suivi de ligne ? ")

    try:
        if choix in ("1", "base"):
            sub_choix = input("goto ou odom ? ")
            if sub_choix == "goto":
                challenge_goto(robot)
            elif sub_choix == "odom":
                challenge_odom(robot)
            else:
                print("Choix inconnu : tape goto ou odom")
        elif choix == "goto":
            challenge_goto(robot)
        elif choix == "odom":
            challenge_odom(robot)
        elif choix in ("2", "line", "cam"):
            challenge_line_following(robot)
        else:
            print("Choix inconnu : tape 1 (base) ou 2 (suivi de ligne)")

    finally:
        robot.stop()
        robot.wheels_io.disable_torque(robot.wheel_ids)


if __name__ == "__main__":
    main()
