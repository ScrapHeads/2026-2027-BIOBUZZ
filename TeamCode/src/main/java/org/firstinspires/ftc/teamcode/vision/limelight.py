"""Bio-Buzz Limelight SnapScript: 30 field-relative tracked elements (120 doubles).

Input llrobot, supplied by updatePythonInputs(double[]):
 [robot_field_x_m, robot_field_y_m, robot_heading_rad,
  camera_forward_m, camera_left_m, camera_height_m,
  camera_pitch_deg, camera_yaw_deg]

Output: 30 slots of [encoded_id, field_x_m, field_y_m, confidence].
Unused slots: [-1, 0, 0, 0]. IDs encode element type as type*1000 + track_id.
"""
import cv2
import numpy as np
import math
import time

# Camera settings: defaults only used for missing mounting inputs.
CAM_HEIGHT_M = 0.040
CAM_PITCH_DEG = 60.0
CAM_FWD_M = 0.0
CAM_LEFT_M = 0.0
CAM_YAW_DEG = 0.0
HFOV_DEG = 54.5
VFOV_DEG = 42.0

PROC_WIDTH = 640
BLUR_KSIZE = 2
MAX_RANGE_M = 3.00
MIN_AREA_PCT = 0.03
MAX_AREA_PCT = 1.9
MIN_FULLNESS = 0.15
MAX_FULLNESS = 0.9
MIN_ASPECT = 0.5
MAX_ASPECT = 1.90
EROSION_STEPS = 1
DILATION_STEPS = 1
SPECKLE_KEEP_RATIO = 0.03

OUTPUT_SIZE = 120
VALUES_PER_ELEMENT = 4
MAX_REPORTED = OUTPUT_SIZE // VALUES_PER_ELEMENT
EMPTY_ID = -1.0
TYPE_ID_MULTIPLIER = 1000
ROBOT_POSE_VALUES = 3
CAMERA_HEIGHT_INDEX = 5
MIN_CAMERA_HEIGHT_M = 0.0001
TRACK_TIMEOUT_SEC = 2.0
MATCH_GATE_M = 0.25
MIN_CONFIDENCE = 0.0
CONFIDENCE_DISTANCE_SCALE_M = 3
# Use measured ball area at known range to calibrate size consistency later.
# For now, area is used as a relative visibility term, not calibrated probability.
AREA_CONFIDENCE_SCALE_PCT = 0.20

EXPECTED_FULLNESS = 0.785

SIZE_WEIGHT = 0.30
FULLNESS_WEIGHT = 0.30
DISTANCE_WEIGHT = 0.40


DEBUG = True
DEBUG_EVERY_N_FRAMES = 15
DEBUG_MAX_ELEMENTS = 30  # Maximum individual detections to print per debug cycle.

# Original HSV values and drawing colors, unchanged.
CLASS_DEFS = [
    ("POLLEN_Y", 1, (16, 220, 40), (27, 255, 255), None, None, (0, 255, 255)),
    ("NECTAR_B", 2, (110, 220, 1), (121, 255, 255), None, None, (255, 140, 0)),
    ("NECTAR_R", 3, (0, 228, 13), (1, 255, 122), None, None, (0, 0, 255)),
]
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
FONT = cv2.FONT_HERSHEY_SIMPLEX
frame_counter = 0
tracks = {}  # encoded_id -> {x, y, last_seen, last_sent, ...}
next_track_id = {1: 0, 2: 0, 3: 0}


def empty_output():
    return [EMPTY_ID, 0.0, 0.0, 0.0] * MAX_REPORTED


def read_inputs(llrobot):
    """Keep calibrated camera defaults until valid mounting inputs arrive.

    A zero-valued camera X/Y or yaw is legitimate. An absent/zero camera
    height is not; treat it as a signal that calibration has not been sent.
    """
    values = [0.0, 0.0, 0.785398, CAM_FWD_M, CAM_LEFT_M,
              CAM_HEIGHT_M, CAM_PITCH_DEG, CAM_YAW_DEG]
    if llrobot is None:
        return values

    count = min(len(llrobot), len(values))
    for i in range(min(count, ROBOT_POSE_VALUES)):
        values[i] = float(llrobot[i])

    # Only trust camera configuration when a valid, positive height is sent.
    # This prevents a zero-filled llrobot from erasing camera calibration.
    if count > CAMERA_HEIGHT_INDEX:
        height = float(llrobot[CAMERA_HEIGHT_INDEX])
        if math.isfinite(height) and height > MIN_CAMERA_HEIGHT_M:
            for i in range(ROBOT_POSE_VALUES, count):
                candidate = float(llrobot[i])
                if math.isfinite(candidate):
                    values[i] = candidate
    return values


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return x*c - y*s, x*s + y*c


def ground_project(px, py, cx, cy, fx, fy, camera):
    """Project bottom center of contour onto floor in robot coordinates."""
    cam_forward, cam_left, height, pitch_deg, yaw_deg = camera
    if height <= 0:
        return None
    ny = (py - cy) / fy
    nx = (px - cx) / fx
    pitch = math.radians(pitch_deg)
    forward = math.cos(pitch) - ny*math.sin(pitch)
    left = -nx
    up = -math.sin(pitch) - ny*math.cos(pitch)
    if up >= -0.02:
        return None
    t = -height / up
    if t <= 0:
        return None
    x, y = rotate(t*forward, t*left, math.radians(yaw_deg))
    return x + cam_forward, y + cam_left


def estimate_confidence(area_pct, fullness, distance_m):
    """Heuristic positional reliability, NOT a calibrated probability."""
    size_score = area_pct / (area_pct + AREA_CONFIDENCE_SCALE_PCT)
    fullness_score = max(
    0.0,
    1.0 - abs(fullness - EXPECTED_FULLNESS) / EXPECTED_FULLNESS
    )
    range_score = 1.0 / (1.0 + (distance_m / CONFIDENCE_DISTANCE_SCALE_M)**2)
    confidence = (
        SIZE_WEIGHT * size_score +
        FULLNESS_WEIGHT * fullness_score +
        DISTANCE_WEIGHT * range_score
    )
    return max(MIN_CONFIDENCE, confidence)


def update_tracks(detections, now):
    """Greedy same-color, nearest-field-position data association."""
    global tracks
    matched = set()
    detections.sort(key=lambda d: d["confidence"], reverse=True)
    for det in detections:
        best_id, best_dist = None, MATCH_GATE_M
        for track_id, track in tracks.items():
            if track_id in matched or track["type"] != det["type"]:
                continue
            dist = math.hypot(det["field_x"]-track["field_x"],
                              det["field_y"]-track["field_y"])
            if dist < best_dist:
                best_id, best_dist = track_id, dist
        if best_id is None:
            element_type = det["type"]
            # Reuse an available local ID only after its previous track expires.
            used_ids = set(tracks.keys())
            for _ in range(TYPE_ID_MULTIPLIER):
                local_id = next_track_id[element_type]
                next_track_id[element_type] = (local_id + 1) % TYPE_ID_MULTIPLIER
                candidate = element_type * TYPE_ID_MULTIPLIER + local_id
                if candidate not in used_ids:
                    best_id = candidate
                    break
        if best_id is None:
            continue
        previous_sent = tracks[best_id]["last_sent"] if best_id in tracks else -1e9
        tracks[best_id] = dict(det, last_seen=now, last_sent=previous_sent,
                               visible=True)
        matched.add(best_id)
    for track_id in list(tracks):
        if track_id not in matched:
            tracks[track_id]["visible"] = False
        if now - tracks[track_id]["last_seen"] > TRACK_TIMEOUT_SEC:
            del tracks[track_id]


def choose_tracks(now):
    """Prioritize visibility/confidence, then spread across four image regions."""
    regions = [[] for _ in range(4)]
    for track_id, track in tracks.items():
        if track["visible"]:
            regions[track["region"]].append((track_id, track))

    def priority(entry):
        t = entry[1]
        return min(2.0, now - t["last_sent"]) + t["confidence"]

    for region in regions:
        region.sort(key=priority, reverse=True)
    selected = []
    # Round robin provides coverage without hard-capping busy regions.
    while len(selected) < MAX_REPORTED and any(regions):
        for region in regions:
            if region and len(selected) < MAX_REPORTED:
                selected.append(region.pop(0))
    return selected


def runPipeline(image, llrobot):
    global frame_counter
    frame_counter += 1
    now = time.monotonic()
    llpython = empty_output()
    best_contour = np.array([[]], dtype=np.int32)
    if image is None or image.size == 0:
        if DEBUG and frame_counter % DEBUG_EVERY_N_FRAMES == 0:
            print("[BIOBUZZ] ERROR: No camera image; returning {} empty values".format(
                len(llpython)))
        return best_contour, image, llpython

    robot_x, robot_y, robot_heading, cam_fwd, cam_left, cam_h, cam_pitch, cam_yaw = read_inputs(llrobot)
    camera = (cam_fwd, cam_left, cam_h, cam_pitch, cam_yaw)
    full_h, full_w = image.shape[:2]
    scale = min(1.0, float(PROC_WIDTH) / full_w)
    sw, sh = max(1, int(round(full_w*scale))), max(1, int(round(full_h*scale)))
    small = cv2.resize(image, (sw, sh), interpolation=cv2.INTER_AREA) if scale < 1 else image.copy()
    hsv = cv2.cvtColor(cv2.blur(small, (BLUR_KSIZE, BLUR_KSIZE)), cv2.COLOR_BGR2HSV)
    fx = (sw/2.0) / math.tan(math.radians(HFOV_DEG/2.0))
    fy = (sh/2.0) / math.tan(math.radians(VFOV_DEG/2.0))
    cx, cy, frame_area = sw/2.0, sh/2.0, float(sw*sh)
    detections = []
    raw_counts = {}
    passed_contours = 0
    rejected_projection = 0
    rejected_range = 0
    rejected_speckle = 0

    for name, class_id, lo1, hi1, lo2, hi2, draw_color in CLASS_DEFS:
        mask = cv2.inRange(hsv, np.array(lo1, np.uint8), np.array(hi1, np.uint8))
        if lo2 is not None:
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv, np.array(lo2, np.uint8), np.array(hi2, np.uint8)))
        mask = cv2.erode(mask, MORPH_K, iterations=EROSION_STEPS)
        mask = cv2.dilate(mask, MORPH_K, iterations=DILATION_STEPS)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        raw_counts[class_id] = len(contours)
        kept = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area <= 0:
                continue
            area_pct = 100.0 * area / frame_area
            if not (MIN_AREA_PCT <= area_pct <= MAX_AREA_PCT):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 2 or h < 2 or not (MIN_ASPECT <= float(w)/h <= MAX_ASPECT):
                continue
            fullness = area / float(w*h)
            if not (MIN_FULLNESS <= fullness <= MAX_FULLNESS):
                continue
            kept.append((area, contour, x, y, w, h, area_pct, fullness))
        if not kept:
            continue
        kept.sort(key=lambda item: item[0], reverse=True)
        cutoff = kept[0][0] * SPECKLE_KEEP_RATIO
        for area, contour, x, y, w, h, area_pct, fullness in kept:
            if area < cutoff:
                rejected_speckle += 1
                continue
            passed_contours += 1
            robot_point = ground_project(x + w*0.5, y + h, cx, cy, fx, fy, camera)
            if robot_point is None:
                rejected_projection += 1
                continue
            relative_x, relative_y = robot_point
            distance_m = math.hypot(relative_x, relative_y)
            if distance_m > MAX_RANGE_M:
                rejected_range += 1
                continue
            dx, dy = rotate(relative_x, relative_y, robot_heading)
            center_x, center_y = x + w*0.5, y + h*0.5
            region = min(1, int(center_x*2/sw)) + 2*min(1, int(center_y*2/sh))
            detections.append({
                "type": class_id, "name": name, "color": draw_color,
                "contour": contour, "bbox": (x, y, w, h), "area": area,
                "field_x": robot_x + dx, "field_y": robot_y + dy,
                "confidence": estimate_confidence(area_pct, fullness, distance_m),
                "region": region,
            })

    # Preserve largest-contour Limelight target behavior.
    detections.sort(key=lambda d: d["area"], reverse=True)
    if detections:
        best_contour = (detections[0]["contour"].astype(np.float32) / scale).astype(np.int32)

    update_tracks(detections, now)
    selected = choose_tracks(now)
    for index, (track_id, track) in enumerate(selected):
        base = index * VALUES_PER_ELEMENT
        llpython[base:base + VALUES_PER_ELEMENT] = [
            float(track_id), track["field_x"], track["field_y"], track["confidence"]
        ]
        track["last_sent"] = now

    output = image.copy()
    for det in detections:
        x, y, w, h = det["bbox"]
        x, y, w, h = [int(round(value / scale)) for value in (x, y, w, h)]
        cv2.rectangle(output, (x, y), (x+w, y+h), det["color"], 2)
        label = "{} ({:.0f},{:.0f}) C:{:.2f}".format(
            det["name"], det["field_x"], det["field_y"], det["confidence"])
        cv2.putText(output, label, (x, max(15, y-5)), FONT, 0.4,
                    det["color"], 1, cv2.LINE_AA)
    if DEBUG and frame_counter % DEBUG_EVERY_N_FRAMES == 0:
        print("[BIOBUZZ] Frame:{} | Raw Y:{} B:{} R:{} | Valid:{} | "
              "Active tracks:{} | Output:{} | Length:{}".format(
            frame_counter, raw_counts.get(1, 0), raw_counts.get(2, 0),
            raw_counts.get(3, 0), len(detections), len(tracks),
            len(selected), len(llpython)))
        print("  Robot field pose: X={:.1f}m Y={:.1f}m Heading={:.2f}rad".format(
            robot_x, robot_y, robot_heading))
        print("  Camera: Fwd={:.1f}m Left={:.1f}m Height={:.1f}m "
              "Pitch={:.1f}deg Yaw={:.1f}deg".format(
            cam_fwd, cam_left, cam_h, cam_pitch, cam_yaw))
        print("  Filter stages: contour passed={} | speckle rejected={} | "
              "projection rejected={} | distance rejected={}".format(
            passed_contours, rejected_speckle,
            rejected_projection, rejected_range))
        if not detections:
            print("  No objects survived filtering; see counts above.")
        else:
            for index, (track_id, track) in enumerate(selected[:DEBUG_MAX_ELEMENTS]):
                print("  {:02d}: ID={} {} X={:.1f}m Y={:.1f}m "
                      "Confidence={:.3f} Region={}".format(
                    index, track_id, track["name"], track["field_x"],
                    track["field_y"], track["confidence"], track["region"]))
            if len(selected) > DEBUG_MAX_ELEMENTS:
                print("  ... {} additional output elements not printed".format(
                    len(selected) - DEBUG_MAX_ELEMENTS))
        print("  Output test: {} doubles, {} populated, {} empty slots".format(
            len(llpython), len(selected), MAX_REPORTED - len(selected)))
    return best_contour, output, llpython