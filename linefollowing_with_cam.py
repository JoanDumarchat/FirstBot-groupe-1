import cv2
import numpy as np
import time

PATH_ORDER = ["DEPART", "JAUNE", "BLEU", "ROUGE", "FIN"]
CYCLE_ORDER = ["VERT", "JAUNE", "BLEU", "ROUGE"]


def get_color_mask(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    b, g, r = cv2.split(bgr)

    # evite les debordmene uint8
    r_i = r.astype(np.int16)
    g_i = g.astype(np.int16)
    b_i = b.astype(np.int16)

    if color == "JAUNE":
        mask = (
            (h >= 12) & (h <= 45) &
            (s >= 35) & (v >= 50) &
            (r_i > b_i + 15) & (g_i > b_i + 10)
        )
        return mask.astype(np.uint8) * 255

    elif color == "VERT":
        mask = (
            (h >= 60) & (h <= 98) &
            (s >= 60) & (v >= 40) &
            (g_i > r_i + 20) & (b_i < 190)
        )
        return mask.astype(np.uint8) * 255

    elif color == "BLEU":
        mask = (
            (h >= 99) & (h <= 135) &
            (s >= 110) & (v >= 50) &
            (b_i > r_i + 40) & (b_i > g_i + 15)
        )
        return mask.astype(np.uint8) * 255

    elif color == "ROUGE":
        mask = (
            ((h <= 12) | (h >= 155)) &
            (s >= 50) & (v >= 40) &
            (r_i > g_i + 25) & (r_i > b_i + 25)
        )
        return mask.astype(np.uint8) * 255

    return np.zeros(bgr.shape[:2], dtype=np.uint8)


def check_green_present(bgr):
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


class LineFollower:
    GREEN_COOLDOWN_SECONDS = 7.0
    CORRIDOR_HALF_WIDTH = 45  # Demi-largeur du couloir de suivi axiale (px)
    REQUIRED_GREEN_FRAMES = 3  # Exige 15 images consécutives avec du vert

    def __init__(self, initial_target="VERT"):
        self.seq_idx = 0  # 0: DEPART, 1: JAUNE, 2: BLEU, 3: ROUGE, 4: FIN
        if initial_target == "JAUNE":
            self.seq_idx = 1
        elif initial_target == "BLEU":
            self.seq_idx = 2
        elif initial_target == "ROUGE":
            self.seq_idx = 3

        self.green_cooldown_until_sec = 0.0
        self.transition_boost_until_sec = 0.0
        self.green_consecutive_frames = 0
        self.last_consigne = 0.0
        self.current_angle = 0.0
        self.tracked_line_x = None
        self.last_turn_dir = 0.0
        self.search_frames = 0
        self.max_search_frames = 90
        self.is_searching = False

    @property
    def current_target(self):
        if self.seq_idx < len(PATH_ORDER):
            return PATH_ORDER[self.seq_idx]
        return "FIN"

    @property
    def is_finished(self):
        return False

    def advance_to_next_target(self, current_time_sec=None):
        if current_time_sec is None:
            current_time_sec = time.time()
        prev = self.current_target
        if self.seq_idx == 0:     # DEPART -> JAUNE
            self.seq_idx = 1
        elif self.seq_idx == 1:   # JAUNE -> BLEU
            self.seq_idx = 2
        elif self.seq_idx == 2:   # BLEU -> ROUGE
            self.seq_idx = 3
        else:                     # ROUGE -> Boucle sur JAUNE
            self.seq_idx = 1

        self.green_cooldown_until_sec = current_time_sec + self.GREEN_COOLDOWN_SECONDS
        self.transition_boost_until_sec = current_time_sec + 1.8  # Avance 1-2 tours de roue (1.8s)
        self.green_consecutive_frames = 0
        self.last_consigne = 0.0
        self.current_angle = 0.0
        self.tracked_line_x = None
        self.last_turn_dir = 0.0
        self.search_frames = 0
        self.is_searching = False
        print(f"\n[TRANSITION BOUCLE] {prev} -> {self.current_target} (Poussée avance 1.8s)")

    def cycle_target(self):
        self.advance_to_next_target(time.time())
        return self.current_target

    def reset_steering(self):
        self.last_consigne = 0.0
        self.current_angle = 0.0
        self.tracked_line_x = None

    def process_frame(self, frame, current_time_sec=None):
        if current_time_sec is None:
            current_time_sec = time.time()

        h, w = frame.shape[:2]
        target = self.current_target
        base_x = int(w / 2.0)
        base_y = int(h)

        # Détection du vert avec compteur de confirmation sur N images consécutives
        is_green_this_frame = check_green_present(frame)
        if is_green_this_frame:
            self.green_consecutive_frames += 1
        else:
            self.green_consecutive_frames = 0

        # 0. ÉTAT DE DÉPART : Attend le vert initial confirmé sur 3 images pour partir sur JAUNE
        if target == "DEPART":
            if self.green_consecutive_frames >= self.REQUIRED_GREEN_FRAMES:
                self.advance_to_next_target(current_time_sec)

            target = self.current_target
            if target == "DEPART":
                return 0.0, False, f"[DEPART] En attente du vert ({self.green_consecutive_frames}/{self.REQUIRED_GREEN_FRAMES})..."

        # 1. TRANSITION VERTE EN COURS (après cooldown 7s, confirmé sur 3 images)
        if current_time_sec >= self.green_cooldown_until_sec:
            if self.green_consecutive_frames >= self.REQUIRED_GREEN_FRAMES:
                self.advance_to_next_target(current_time_sec)

        target = self.current_target
        if target == "FIN":
            return 0.0, False, "[FIN] Parcours terminé"

        # masque binaire de la couleur active
        mask = get_color_mask(frame, target)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # detection de la ligne
        if self.tracked_line_x is None:
            init_roi = mask[int(0.20 * h):int(0.95 * h), :]
            conts, _ = cv2.findContours(init_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in conts if cv2.contourArea(c) >= 120]
            if valid:
                valid.sort(key=lambda c: abs((cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] // 2) - base_x))
                x, y_box, bw, bh = cv2.boundingRect(valid[0])
                self.tracked_line_x = x + bw // 2

        chosen = None
        is_straight_continuation = False

        if self.tracked_line_x is not None:
            #test pour si la ligne boucle sur elle meme
            corr_x1 = max(0, self.tracked_line_x - self.CORRIDOR_HALF_WIDTH)
            corr_x2 = min(w, self.tracked_line_x + self.CORRIDOR_HALF_WIDTH)
            corridor_ahead = mask[int(0.15 * h):int(0.90 * h), corr_x1:corr_x2]
            ahead_pixel_count = np.count_nonzero(corridor_ahead)

            if ahead_pixel_count >= 150:
                is_straight_continuation = True
                y_look = int(0.50 * h)
                look_strip = mask[max(0, y_look - 20):min(h, y_look + 20), corr_x1:corr_x2]
                pts = np.argwhere(look_strip > 0)
                if len(pts):
                    local_cx = int(np.mean(pts[:, 1])) + corr_x1
                    self.tracked_line_x = int(0.75 * local_cx + 0.25 * self.tracked_line_x)

                target_cx = self.tracked_line_x
                target_cy = y_look

                offset = float(target_cx - base_x)
                dx = target_cx - base_x
                dy = max(base_y - target_cy, 10.0)
                angle = float(np.degrees(np.arctan2(dx, dy)))

                norm_offset = offset / (w / 2.0)
                consigne = float(np.clip(norm_offset * 30.0 + 0.6 * angle, -40.0, 40.0))

                chosen = {
                    'cx': target_cx,
                    'cy': target_cy,
                    'offset': offset,
                    'angle': angle,
                    'consigne': consigne
                }

            else:
                #check virage ou fin de la ligne droite
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid = [c for c in contours if cv2.contourArea(c) >= 200]
                if valid:
                    candidates = []
                    for c in valid:
                        M = cv2.moments(c)
                        if M['m00'] < 10:
                            continue
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        dx = cx - base_x
                        dy = max(base_y - cy, 10.0)
                        angle = float(np.degrees(np.arctan2(dx, dy)))
                        score = abs(angle - self.current_angle)
                        candidates.append((score, cx, cy, angle))

                    if candidates:
                        candidates.sort(key=lambda item: item[0])
                        _, win_cx, win_cy, win_ang = candidates[0]
                        self.tracked_line_x = win_cx
                        offset = float(win_cx - base_x)
                        norm_offset = offset / (w / 2.0)
                        consigne = float(np.clip(norm_offset * 30.0 + 0.6 * win_ang, -40.0, 40.0))
                        chosen = {
                            'cx': win_cx,
                            'cy': win_cy,
                            'offset': offset,
                            'angle': win_ang,
                            'consigne': consigne
                        }

        if chosen is not None:
            self.search_frames = 0
            self.is_searching = False
            if chosen['offset'] > 6.0:
                self.last_turn_dir = 1.0
            elif chosen['offset'] < -6.0:
                self.last_turn_dir = -1.0

            self.current_angle = 0.75 * chosen['angle'] + 0.25 * self.current_angle
            self.last_consigne = 0.70 * chosen['consigne'] + 0.30 * self.last_consigne
            is_active = True
            mode_str = "LIGNE DROITE" if is_straight_continuation else "SUIVI"
            status_str = f"[{target}:{mode_str}] Ang:{chosen['angle']:+5.1f}° | Off:{chosen['offset']:+5.1f}px | Cmd:{self.last_consigne:+5.1f}°"

        elif current_time_sec < self.transition_boost_until_sec:
            # Avance tout droit (1-2 tours de roue) après le vert pour franchir la zone et trouver la ligne
            self.is_searching = False
            is_active = True
            self.last_consigne = 0.0
            rem = self.transition_boost_until_sec - current_time_sec
            status_str = f"[{target}:AVANCE 1-2 TOURS] Recherche ligne ({rem:.1f}s)..."

        elif self.last_turn_dir != 0.0 and self.search_frames < self.max_search_frames:
            self.search_frames += 1
            self.is_searching = True
            is_active = True
            self.last_consigne = self.last_turn_dir * 30.0
            dir_name = "DROITE" if self.last_turn_dir > 0 else "GAUCHE"
            status_str = f"[{target}] RECHERCHE {dir_name} ({self.search_frames}/{self.max_search_frames})"

        else:
            self.is_searching = False
            self.last_consigne = 0.0
            is_active = False
            status_str = f"[{target} PERDUE] Arrêt"

        return self.last_consigne, is_active, status_str


def detect_line(bgr, color):
    h, w = bgr.shape[:2]
    base_x = int(w / 2.0)
    base_y = int(h)

    mask = get_color_mask(bgr, color)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= 120]
    if not valid:
        return False, 0.0, 0.0, 0.0

    valid.sort(key=lambda c: cv2.contourArea(c), reverse=True)
    M = cv2.moments(valid[0])
    if M['m00'] < 1.0:
        return False, 0.0, 0.0, 0.0

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    dx = cx - base_x
    dy = max(base_y - cy, 10.0)
    angle = float(np.degrees(np.arctan2(dx, dy)))
    offset = float(cx - base_x)
    norm_offset = offset / (w / 2.0)
    consigne = float(np.clip(norm_offset * 30.0 + 0.6 * angle, -40.0, 40.0))

    return True, consigne, angle, offset
