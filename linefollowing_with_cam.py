"""
linefollowing_with_cam.py - Suivi de ligne avec la caméra (challenge 1).

- get_color_mask      : trouve une couleur dans l'image (masque noir/blanc)
- check_green_present : le marqueur vert est-il devant le robot ?
- LineFollower        : décide de combien tourner (consigne en degrés) à chaque image,
                        et change de couleur à chaque marqueur vert : DEPART → JAUNE → BLEU → ROUGE → JAUNE …
"""
import time

import cv2
import numpy as np

PATH_ORDER = ["DEPART", "JAUNE", "BLEU", "ROUGE", "FIN"]


# ------------------------------------------------------------------ couleurs
def get_color_mask(bgr, color):
    """Masque noir/blanc (255 = la couleur est là) à partir de l'image BGR.
    h = teinte (0..179), s = saturation (couleur vive ou grise), v = luminosité.
    v minimum : dans les zones sombres la teinte n'a plus de sens."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    if color == "JAUNE":
        mask = (h >= 15) & (h <= 40) & (s >= 50) & (v >= 60)
    elif color == "VERT":
        # strict : le mélange jaune/bleu aux croisements et les ombres donnaient du « faux vert »
        mask = (h >= 60) & (h <= 90) & (s >= 90) & (v >= 50)
    elif color == "BLEU":
        mask = (h >= 100) & (h <= 135) & (s >= 80) & (v >= 40)
    elif color == "ROUGE":
        # le rouge est aux deux bouts de la roue des teintes
        mask = ((h <= 10) | (h >= 155)) & (s >= 60) & (v >= 50)
    else:
        return np.zeros(bgr.shape[:2], dtype=np.uint8)

    return mask.astype(np.uint8) * 255


def check_green_present(bgr):
    """True si une grosse tache verte est devant le robot ET qu'une ligne de couleur est visible."""
    h, w = bgr.shape[:2]
    roi = bgr[int(h * 0.30):, :]                      # les 70 % du bas de l'image

    green = get_color_mask(roi, "VERT")
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # enlève le bruit
    contours, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # le vrai marqueur est une grosse tache : on ignore les petites (bords de lignes, reflets)
    min_area = int(400 * (w / 320.0) * (h / 240.0))
    if not any(cv2.contourArea(c) >= min_area for c in contours):
        return False

    # le marqueur est sur la piste : une ligne de couleur doit être visible à côté
    return any(np.count_nonzero(get_color_mask(roi, c)) >= 30 for c in ("ROUGE", "BLEU", "JAUNE"))


# ------------------------------------------------------------------ suivi de ligne
class LineFollower:
    GREEN_COOLDOWN_SECONDS = 7.0   # après un changement, on ignore le vert 7 s (le robot est encore dessus)
    BOOST_SECONDS = 1.8            # après le vert, avancer tout droit pour trouver la nouvelle ligne
    REQUIRED_GREEN_FRAMES = 5      # images de vert d'affilée pour valider le marqueur
    CORRIDOR_HALF_WIDTH = 45       # couloir de ±45 px autour de la ligne suivie (ignore les croisements)
    DEAD_ZONE = 3.0                # consigne < 3° : on va tout droit (évite le zigzag)

    def __init__(self, initial_target="VERT"):
        # "VERT" (ou autre) → DEPART : on attend le vert avant de partir sur JAUNE
        self.seq_idx = {"JAUNE": 1, "BLEU": 2, "ROUGE": 3}.get(initial_target, 0)
        self.green_cooldown_until_sec = 0.0
        self.transition_boost_until_sec = 0.0
        self.green_consecutive_frames = 0
        self.last_consigne = 0.0
        self.current_angle = 0.0
        self.tracked_line_x = None     # position x (px) de la ligne suivie
        self.last_turn_dir = 1.0       # côté où chercher la ligne si on la perd (+1 droite, −1 gauche)
        self.search_frames = 0
        self.max_search_frames = 90
        self.is_searching = False

    @property
    def current_target(self):
        return PATH_ORDER[self.seq_idx] if self.seq_idx < len(PATH_ORDER) else "FIN"

    @property
    def is_finished(self):
        return False

    def advance_to_next_target(self, current_time_sec=None):
        """Couleur suivante : DEPART → JAUNE → BLEU → ROUGE → JAUNE …"""
        if current_time_sec is None:
            current_time_sec = time.time()
        prev = self.current_target
        self.seq_idx = self.seq_idx + 1 if self.seq_idx < 3 else 1

        self.green_cooldown_until_sec = current_time_sec + self.GREEN_COOLDOWN_SECONDS
        self.transition_boost_until_sec = current_time_sec + self.BOOST_SECONDS
        self.green_consecutive_frames = 0
        self.last_consigne = 0.0
        self.current_angle = 0.0
        self.tracked_line_x = None
        self.search_frames = 0
        self.is_searching = False
        # last_turn_dir est gardé : si la nouvelle ligne n'est pas vue, on la cherche au lieu de s'arrêter
        print(f"\n[TRANSITION BOUCLE] {prev} -> {self.current_target} (Poussée avance {self.BOOST_SECONDS}s)")

    def cycle_target(self):
        """Touche 'd' : couleur suivante à la main."""
        self.advance_to_next_target(time.time())
        return self.current_target

    @staticmethod
    def _consigne(cx, cy, w, h):
        """Consigne (degrés, > 0 = tourner à droite) pour aller vers le point (cx, cy) de l'image."""
        offset = float(cx - w / 2.0)                        # décalage horizontal, > 0 : ligne à droite
        dy = max(h - cy, 10.0)                              # évite un angle de ±90° tout en bas de l'image
        angle = float(np.degrees(np.arctan2(offset, dy)))   # direction de la ligne vue depuis le robot
        consigne = float(np.clip(offset / (w / 2.0) * 30.0 + 0.6 * angle, -40.0, 40.0))
        return offset, angle, consigne

    def process_frame(self, frame, current_time_sec=None):
        """Renvoie (consigne en degrés, is_active, texte d'état)."""
        if current_time_sec is None:
            current_time_sec = time.time()
        h, w = frame.shape[:2]

        # 1. compter les images de vert d'affilée
        if check_green_present(frame):
            self.green_consecutive_frames += 1
        else:
            self.green_consecutive_frames = 0
        vert_confirme = self.green_consecutive_frames >= self.REQUIRED_GREEN_FRAMES

        # 2. départ : on attend le vert sans bouger
        if self.current_target == "DEPART":
            if vert_confirme:
                self.advance_to_next_target(current_time_sec)
            else:
                return 0.0, False, (f"[DEPART] En attente du vert "
                                    f"({self.green_consecutive_frames}/{self.REQUIRED_GREEN_FRAMES})...")

        # 3. marqueur vert (après le cooldown) → couleur suivante
        elif current_time_sec >= self.green_cooldown_until_sec and vert_confirme:
            self.advance_to_next_target(current_time_sec)

        target = self.current_target
        if target == "FIN":
            return 0.0, False, "[FIN] Parcours terminé"

        # 4. masque de la couleur suivie
        mask = get_color_mask(frame, target)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # 5. première détection : le morceau de ligne le plus proche du milieu de l'image
        if self.tracked_line_x is None:
            roi = mask[int(0.20 * h):int(0.95 * h), :]
            conts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in conts if cv2.contourArea(c) >= 120]
            if valid:
                x, _, bw, _ = cv2.boundingRect(min(valid, key=lambda c: abs(
                    cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] // 2 - w // 2)))
                self.tracked_line_x = x + bw // 2

        chosen = None
        mode = ""
        if self.tracked_line_x is not None:
            x1 = max(0, self.tracked_line_x - self.CORRIDOR_HALF_WIDTH)
            x2 = min(w, self.tracked_line_x + self.CORRIDOR_HALF_WIDTH)

            # 6a. LIGNE DROITE : la ligne continue dans le couloir → on la suit (ignore les croisements)
            ahead_pixels = np.count_nonzero(mask[int(0.15 * h):int(0.90 * h), x1:x2])
            y_look = int(0.50 * h)
            pts = np.argwhere(mask[max(0, y_look - 20):min(h, y_look + 20), x1:x2] > 0)
            if ahead_pixels >= 150 and len(pts) > 0:
                local_cx = int(np.mean(pts[:, 1])) + x1
                self.tracked_line_x = int(0.75 * local_cx + 0.25 * self.tracked_line_x)   # lissage
                chosen = self._consigne(self.tracked_line_x, y_look, w, h)
                mode = "LIGNE DROITE"

            # 6b. VIRAGE : on va vers le centre du plus grand morceau de ligne
            else:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid = [c for c in contours if cv2.contourArea(c) >= 200]
                if valid:
                    M = cv2.moments(max(valid, key=cv2.contourArea))
                    if M["m00"] >= 10:
                        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        self.tracked_line_x = cx
                        chosen = self._consigne(cx, cy, w, h)
                        mode = "SUIVI"

        # 7. ligne trouvée : on retient son côté, on lisse la consigne, zone morte
        if chosen is not None:
            offset, angle, consigne = chosen
            self.search_frames = 0
            self.is_searching = False
            if offset > 6.0:
                self.last_turn_dir = 1.0
            elif offset < -6.0:
                self.last_turn_dir = -1.0
            self.current_angle = 0.75 * angle + 0.25 * self.current_angle
            self.last_consigne = 0.70 * consigne + 0.30 * self.last_consigne
            commande = 0.0 if abs(self.last_consigne) < self.DEAD_ZONE else self.last_consigne
            return (commande, True,
                    f"[{target}:{mode}] Ang:{angle:+5.1f}° | Off:{offset:+5.1f}px | Cmd:{commande:+5.1f}°")

        # 8. ligne pas trouvée juste après le vert : tout droit
        self.last_consigne = 0.0
        if current_time_sec < self.transition_boost_until_sec:
            self.is_searching = False
            reste = self.transition_boost_until_sec - current_time_sec
            return 0.0, True, f"[{target}:AVANCE] Recherche ligne ({reste:.1f}s)..."

        # 9. ligne perdue : on tourne du côté où elle était (90 images max)
        if self.search_frames < self.max_search_frames:
            self.search_frames += 1
            self.is_searching = True
            self.last_consigne = self.last_turn_dir * 30.0
            cote = "DROITE" if self.last_turn_dir > 0 else "GAUCHE"
            return (self.last_consigne, True,
                    f"[{target}] RECHERCHE {cote} ({self.search_frames}/{self.max_search_frames})")

        # 10. toujours rien : arrêt
        self.is_searching = False
        return 0.0, False, f"[{target} PERDUE] Arrêt"
