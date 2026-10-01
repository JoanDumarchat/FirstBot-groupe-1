"""
linefollowing_with_cam.py - Module de suivi de ligne vision autonome épuré.

- Masques de couleur BGR/HSV calibrés avec désactivation AWB (4200K).
- Détection de la balise verte avec temporisation de 7s entre chaque transition.
- Asservissement par axe central de ligne et lissage de consigne.
"""

import cv2
import numpy as np
import time

CYCLE_ORDER = ["VERT", "JAUNE", "BLEU", "ROUGE"]


def get_color_mask(bgr, color):
    """Génère le masque binaire précis pour chaque couleur du circuit."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    b, g, r = cv2.split(bgr)

    # Conversion int16 pour éviter les débordements uint8 lors des soustractions
    r_i = r.astype(np.int16)
    g_i = g.astype(np.int16)
    b_i = b.astype(np.int16)

    if color == "JAUNE":
        # Bande jaune centrale : R et G élevés, B très bas, S modérée à forte
        mask = (
            (h >= 18) & (h <= 38) &
            (s >= 75) & (v >= 60) &
            (r_i > b_i + 35) & (g_i > b_i + 25)
        )
        return mask.astype(np.uint8) * 255

    elif color == "VERT":
        # Vert haut droite (aspect sarcelle/teal sur la cam) : H monte jusqu'à 98
        mask = (
            (h >= 60) & (h <= 98) &
            (s >= 60) & (v >= 40) &
            (g_i > r_i + 20) & (b_i < 190)
        )
        return mask.astype(np.uint8) * 255

    elif color == "BLEU":
        # Bande bleue centrale : B très dominant, S >= 110 pour rejeter le sol gris bleuté
        mask = (
            (h >= 99) & (h <= 135) &
            (s >= 110) & (v >= 50) &
            (b_i > r_i + 40) & (b_i > g_i + 15)
        )
        return mask.astype(np.uint8) * 255

    elif color == "ROUGE":
        # Rouge haut gauche : moins saturé que le bleu, split 0-12 et 155-180
        mask = (
            ((h <= 12) | (h >= 155)) &
            (s >= 50) & (v >= 40) &
            (r_i > g_i + 25) & (r_i > b_i + 25)
        )
        return mask.astype(np.uint8) * 255

    return np.zeros(bgr.shape[:2], dtype=np.uint8)


def check_green_present(bgr):
    """Détecte la présence d'une balise verte devant le robot."""
    h, w = bgr.shape[:2]
    roi = bgr[int(h * 0.30):, :]
    green_mask = get_color_mask(roi, "VERT")
    kernel = np.ones((3, 3), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    min_area = int(120 * (w / 320.0) * (h / 240.0))
    return any(cv2.contourArea(c) >= min_area for c in contours)


def detect_line(bgr, target_color):
    """Détecte le centre du ruban de la couleur cible et calcule la consigne angulaire."""
    h, w = bgr.shape[:2]
    mask = get_color_mask(bgr, target_color)

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

    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']

    offset = float(cx - (w / 2.0))
    dx = cx - (w / 2.0)
    dy = max(h - cy, 10.0)
    angle = float(np.degrees(np.arctan2(dx, dy)))

    norm_offset = offset / (w / 2.0)
    consigne = float(np.clip(norm_offset * 32.0 + 0.6 * angle, -40.0, 40.0))

    return True, consigne, angle, offset


class LineFollower:
    """Gestionnaire de suivi de ligne épuré."""

    GREEN_COOLDOWN_SECONDS = 7.0

    def __init__(self, initial_target="JAUNE"):
        self.target = initial_target if initial_target in CYCLE_ORDER else "JAUNE"
        self.green_cooldown_until = 0.0
        self.last_consigne = 0.0
        self.last_turn_dir = 0.0
        self.search_frames = 0
        self.max_search_frames = 90
        self.is_searching = False

    @property
    def current_target(self):
        return self.target

    def cycle_target(self):
        """Passe manuellement à la couleur suivante (touche 'd') avec cooldown de 7s."""
        curr = self.current_target
        if curr in CYCLE_ORDER:
            idx = CYCLE_ORDER.index(curr)
            next_target = CYCLE_ORDER[(idx + 1) % len(CYCLE_ORDER)]
        else:
            next_target = "JAUNE"
        self.set_target(next_target)
        print(f"\n[FOCUS MANUEL 'd'] {curr} -> {self.current_target} (Verrouillé 7s)")
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
        target = self.current_target
        now = time.time()

        # 1. Détection du vert : transition si le cooldown de 7s est passé
        if now >= self.green_cooldown_until:
            if check_green_present(frame):
                prev = target
                idx = CYCLE_ORDER.index(prev) if prev in CYCLE_ORDER else 0
                next_target = CYCLE_ORDER[(idx + 1) % len(CYCLE_ORDER)]
                self.set_target(next_target)
                print(f"\n[VERT DÉTECTÉ] {prev} -> {self.current_target} (Prochaine transition dans 7s)")

        target = self.current_target

        # 2. Suivi de la ligne courante
        detected, consigne_brute, angle, offset = detect_line(frame, target)

        if detected:
            self.search_frames = 0
            self.is_searching = False

            if offset > 6.0:
                self.last_turn_dir = 1.0
            elif offset < -6.0:
                self.last_turn_dir = -1.0

            self.last_consigne = 0.70 * consigne_brute + 0.30 * self.last_consigne
            is_active = True
            status_str = f"[{target}] Ang:{angle:+5.1f}° | Off:{offset:+5.1f}px | Cmd:{self.last_consigne:+5.1f}°"

        elif self.last_turn_dir != 0.0 and self.search_frames < self.max_search_frames:
            self.search_frames += 1
            self.is_searching = True
            is_active = True
            self.last_consigne = self.last_turn_dir * 30.0
            direction_str = "DROITE" if self.last_turn_dir > 0 else "GAUCHE"
            status_str = f"[{target}] RECHERCHE {direction_str} ({self.search_frames}/{self.max_search_frames}) | Cmd:{self.last_consigne:+5.1f}°"

        else:
            self.is_searching = False
            self.last_consigne = 0.0
            self.last_turn_dir = 0.0
            is_active = False
            status_str = f"[{target} PERDUE] Arrêt robot"

        return self.last_consigne, is_active, status_str
