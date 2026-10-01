import sys
import time

import cv2
from odometry import record_movements
from robot import Robot
from drivecam import DriveCam
from linefollowing_with_cam import LineFollower
from carte import Carte, dessiner_depuis_fichier


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


def check_terminal_keys():
    """pour recup les inputs claviers"""
    if sys.platform.startswith("linux"):
        try:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                chars = ""
                while select.select([sys.stdin], [], [], 0.003)[0]:
                    c = sys.stdin.read(1)
                    if not c:
                        break
                    chars += c
                return chars
        except Exception:
            pass
    return ""


def challenge_line_following(robot):
    """Suivi de ligne caméra autonome (version épurée sans interface web) + carte."""
    print("\n" + "="*65)
    print(" SUIVI DE LIGNE CAMERA AUTONOME")
    print(" - Touche ESPACE     : STOPPER / REPRENDRE")
    print(" - Touche 'd'        : CHANGER COULEUR FOCUS (Vert->Jaune->Bleu->Rouge)")
    print(" - Touches 'i' / 'k' : Vitesse fine (+0.2 / -0.2)")
    print(" - Touches 'u' / 'j' : Vitesse rapide (+1.0 / -1.0)")
    print(" - Arrêt d'urgence   : Pressez Ctrl+C")
    print("="*65 + "\n")

    print("Initialisation de la caméra...")
    backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
    cap = cv2.VideoCapture(0, backend)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    # Désactivation de l'AWB et fixation d'une température neutre (4200K) pour la cam
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4200)
    if sys.platform.startswith("linux"):
        try:
            import subprocess
            subprocess.run([
                "v4l2-ctl", "-d", "/dev/video0",
                "-c", "white_balance_automatic=0",
                "-c", "white_balance_temperature=4200"
            ], capture_output=True, check=False)
            print("[*] config cam on")
        except Exception:
            pass

    time.sleep(0.5)

    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra.")
        return

    follower = LineFollower(initial_target="VERT")
    driver = DriveCam(robot, step_distance=4.0)

    # --- CARTE --- le départ (marqueur vert) est l'origine
    robot.x, robot.y, robot.theta = 0., 0., 0.
    carte = Carte(robot)   # points enregistrés à chaque image, carte dessinée seulement à la fin

    v_base = 7.0
    speed_factor = 1.0
    is_paused = False

    old_termios = None
    if sys.platform.startswith("linux"):
        try:
            import termios
            import tty
            old_termios = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass

    print("Suivi autonome en cours (Ctrl+C pour arrêter)...")
    last_time = time.perf_counter()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # --- ODOMÉTRIE --- move_s ne met pas à jour la position : on le fait ici
            now = time.perf_counter()
            robot.update_pose_from_wheels(now - last_time)
            last_time = now

            term_chars = check_terminal_keys()
            if term_chars:
                if ' ' in term_chars:
                    is_paused = not is_paused
                if 'd' in term_chars.lower():
                    follower.cycle_target()
                if 'i' in term_chars.lower():
                    speed_factor = round(min(speed_factor + 0.2, 15.0), 2)
                if 'k' in term_chars.lower():
                    speed_factor = round(max(speed_factor - 0.2, 0.2), 2)
                if 'u' in term_chars.lower():
                    speed_factor = round(min(speed_factor + 1.0, 15.0), 2)
                if 'j' in term_chars.lower():
                    speed_factor = round(max(speed_factor - 1.0, 0.2), 2)

            v_effective = v_base * speed_factor

            if is_paused:
                driver.stop()
                status_str = f"[PAUSE] Appuyez sur ESPACE pour reprendre | Vit:{v_effective:.1f}cm/s"
            else:
                consigne, is_active, status_str = follower.process_frame(frame)
                carte.enregistrer(frame, follower.current_target)   # --- CARTE --- detect_line + pixel_to_world
                driver.drive_autonome(consigne, is_active, is_searching=follower.is_searching, v_effective=v_effective)
                status_str += f" | Vit:{v_effective:.1f}cm/s (x{speed_factor:.1f})"

            sys.stdout.write(f"\r{status_str}   ")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt du suivi de ligne demandé.")
    finally:
        if old_termios and sys.platform.startswith("linux"):
            try:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_termios)
            except Exception:
                pass
        cap.release()
        driver.stop()
        # --- CARTE --- enregistrée même après Ctrl-C
        carte.sauvegarder("parcours.json")
        carte.dessiner("carte.png")


def main():
    # Redessiner la carte sans robot : python3 main.py carte
    if len(sys.argv) > 1 and sys.argv[1] == "carte":
        dessiner_depuis_fichier("parcours.json", "carte.png")
        return

    import pypot.dynamixel   # seulement sur le robot
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
            print("Choix inconnu : tape 1 (base), 2 (suivi de ligne) ou carte")

    finally:
        robot.stop()
        robot.wheels_io.disable_torque(robot.wheel_ids)


if __name__ == "__main__":
    main()