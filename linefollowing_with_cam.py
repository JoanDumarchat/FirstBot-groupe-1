
import cv2
import numpy as np
import time

HSV = {

    "JAUNE": [(np.array([16, 22, 50]), np.array([35, 255, 255]))],

    "VERT":  [(np.array([36, 65, 30]), np.array([103, 255, 255]))],

    "BLEU":  [(np.array([104, 60, 40]), np.array([135, 255, 255]))],
    "ROUGE": [
        (np.array([0, 70, 50]), np.array([10, 255, 255])),
        (np.array([155, 70, 50]), np.array([180, 255, 255]))
    ]
}

CYCLE_ORDER = ["VERT", "JAUNE", "BLEU", "ROUGE"]


def get_color_mask(hsv, color):
    """genere le mask bin de la couleur demandé"""
    if color not in HSV:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    mask = None
    for low, high in HSV[color]:
        m = cv2.inRange(hsv, low, high)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def check_green_present(hsv):
    """
    check green sur la cam
    """
    h, w = hsv.shape[:2]
    green_mask = get_color_mask(hsv, "VERT")
    kernel = np.ones((3, 3), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    # zone devant les roue
    y_start = int(h * 0.20)
    green_active = green_mask[y_start:, :]

    contours, _ = cv2.findContours(green_active, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    min_area = int(120 * (w / 320.0) * (h / 240.0))
    return any(cv2.contourArea(c) >= min_area for c in contours)


def detect_line(hsv, target_color):
    """
    detecte la ligne en fonction de la target
    """
    h, w = hsv.shape[:2]
    mask = get_color_mask(hsv, target_color)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0, 0.0, 0.0

    c = max(contours, key=cv2.contourArea)
    min_area = int(25 * (w / 320.0) * (h / 240.0))
    if cv2.contourArea(c) < min_area:
        return False, 0.0, 0.0, 0.0

    M = cv2.moments(c)
    if M['m00'] < 10:
        return False, 0.0, 0.0, 0.0

    # milieu de la cam 
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']

    # decakage par rapport au centre de la cam en pixel
    offset = float(cx - (w / 2.0))

    # Angle de visée vers le centre de la cam depuis la base avant du robot
    dx = cx - (w / 2.0)
    dy = max(h - cy, 10.0)
    angle = float(np.degrees(np.arctan2(dx, dy)))

    # decalage a gauche et a droite 1 -1
    norm_offset = offset / (w / 2.0)
    consigne = float(np.clip(norm_offset * 32.0 + 0.6 * angle, -40.0, 40.0))

    return True, consigne, angle, offset


class LineFollower:
    """Gestionnaire de suivi de ligne épuré."""

    GREEN_COOLDOWN_SECONDS = 7.0  # 7 secondes complètes avant de pouvoir re-changer de couleur

    def __init__(self, initial_target="JAUNE"):
        self.target = initial_target if initial_target in CYCLE_ORDER else "JAUNE"
        self.green_cooldown_until = 0.0
        self.last_consigne = 0.0
        self.last_turn_dir = 0.0      # Côté où la ligne a été vue : +1.0 (droite), -1.0 (gauche)
        self.search_frames = 0
        self.max_search_frames = 90   # 3 sec de recherche
        self.is_searching = False

    @property
    def current_target(self):
        return self.target

    def cycle_target(self):
        """passe manuellement à la couleur suivante (touche 'd') avec cooldown de 7s."""
        curr = self.current_target
        if curr in CYCLE_ORDER:
            idx = CYCLE_ORDER.index(curr)
            next_target = CYCLE_ORDER[(idx + 1) % len(CYCLE_ORDER)]
        else:
            next_target = "JAUNE"
        self.set_target(next_target)
        print(f"\n[FOCUS MANUEL 'd'] {curr} -> {self.current_target} (Verrouille 7s)")
        return self.current_target

    def set_target(self, color):
        """Définit la couleur cible et réenclenche le cooldown de 7 secondes."""
        self.target = color
        self.green_cooldown_until = time.time() + self.GREEN_COOLDOWN_SECONDS
        self.last_consigne = 0.0
        self.last_turn_dir = 0.0
        self.search_frames = 0
        self.is_searching = False

    def process_frame(self, frame):
        """
        Traite une image caméra :
        Retourne : (consigne, is_active, status_str)
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        target = self.current_target
        now = time.time()

        # 1. Détection simple du vert : change de couleur si le cooldown de 7s est passé
        if now >= self.green_cooldown_until:
            if check_green_present(hsv):
                prev = target
                idx = CYCLE_ORDER.index(prev) if prev in CYCLE_ORDER else 0
                next_target = CYCLE_ORDER[(idx + 1) % len(CYCLE_ORDER)]
                self.set_target(next_target)
                print(f"\n[VERT DÉTECTÉ] {prev} -> {self.current_target} (Prochaine transition dans 7s)")

        target = self.current_target

        # 2. Suivi pur de la ligne courante
        detected, consigne_brute, angle, offset = detect_line(hsv, target)

        if detected:
            self.search_frames = 0
            self.is_searching = False

            # Mémorise le côté où se trouve la ligne (pour tourner vers elle si perdue)
            if offset > 6.0:
                self.last_turn_dir = 1.0   # Ligne à droite -> tournera à droite si perdue
            elif offset < -6.0:
                self.last_turn_dir = -1.0  # Ligne à gauche -> tournera à gauche si perdue

            # Lissage léger de la consigne
            self.last_consigne = 0.70 * consigne_brute + 0.30 * self.last_consigne
            is_active = True
            status_str = f"[{target}] Ang:{angle:+5.1f}° | Off:{offset:+5.1f}px | Cmd:{self.last_consigne:+5.1f}°"

        elif self.last_turn_dir != 0.0 and self.search_frames < self.max_search_frames:
            # 3. LIGNE PERDUE : tourne pour la retrouver vers le côté où elle était !
            self.search_frames += 1
            self.is_searching = True
            is_active = True
            self.last_consigne = self.last_turn_dir * 30.0
            direction_str = "DROITE" if self.last_turn_dir > 0 else "GAUCHE"
            status_str = f"[{target}] RECHERCHE {direction_str} ({self.search_frames}/{self.max_search_frames}) | Cmd:{self.last_consigne:+5.1f}°"

        else:
            # Timeout de recherche dépassé : arrêt du robot
            self.is_searching = False
            self.last_consigne = 0.0
            self.last_turn_dir = 0.0
            is_active = False
            status_str = f"[{target} PERDUE] Arrêt robot"

        return self.last_consigne, is_active, status_str
