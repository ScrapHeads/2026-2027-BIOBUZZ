import cv2
import numpy as np
import math


# =====================================================================
# CAMERA GEOMETRY
# Measure these on the actual robot.
# Units: centimeters
# =====================================================================

CAM_HEIGHT_CM = 25.0
CAM_PITCH_DEG = 20.0

# Camera position relative to robot center
CAM_FWD_CM = 15.0
CAM_LEFT_CM = 0.0

# Limelight 3A FOV
HFOV_DEG = 54.5
VFOV_DEG = 42.0


# =====================================================================
# PROCESSING
# =====================================================================

PROC_WIDTH = 320
BLUR_KSIZE = 2

MAX_RANGE_CM = 300.0


# =====================================================================
# LIMELIGHT CONTOUR SETTINGS
#
# Based on the settings you provided:
#
# Sort: Largest
# Area: 0.0041% - 7.4818%
# Fullness: 64.1% - 100%
# W/H Ratio: 0 - 20
# Direction Filter: None
# Smart Speckle Rejection: 90
#
# =====================================================================

MIN_AREA_PCT = 0.0041
MAX_AREA_PCT = 7.4818

MIN_FULLNESS = 0.35
MAX_FULLNESS = 1.00

MIN_ASPECT = 0.0
MAX_ASPECT = 20.0

EROSION_STEPS = 1
DILATION_STEPS = 1


# Approximation of Limelight's smart speckle behavior.
#
# Keep contours at least 10% the size of the largest contour
# of the same color.
#
# We can adjust this after testing.
SPECKLE_KEEP_RATIO = 0.20


# =====================================================================
# OUTPUT
#
# FTC getPythonOutput() gives us 32 values.
#
# [0] = number of objects returned
#
# Each object gets:
#
# type
# X forward cm
# Y left/right cm
#
# 10 objects = 30 values
#
# Index 31 = total detections before output limit
# =====================================================================

MAX_REPORTED = 10
OUTPUT_SIZE = 32


# =====================================================================
# OBJECT IDS
#
# 1 = Yellow pollen
# 2 = Blue nectar
# 3 = Red nectar
# =====================================================================


# =====================================================================
# HSV SETTINGS
#
# These are your current starting values.
# They WILL need tuning using actual BIOBUZZ elements and field lighting.
#
# OpenCV Hue range is 0-179.
# CLASSES:  name, id, hsvLo1, hsvHi1, hsvLo2, hsvHi2, drawColor(BGR)
# =====================================================================

CLASS_DEFS = [

    (
        "POLLEN_Y",
        1,

        (23, 192, 33),
        (37, 255, 255),

        None,
        None,

        (0, 255, 255)
    ),

    (
        "NECTAR_B",
        2,

        (95, 110, 70),
        (125, 255, 255),

        None,
        None,

        (255, 140, 0)
    ),

    (
        "NECTAR_R",
        3,

        (0, 120, 10),
        (3, 255, 60),

        (179, 150, 20),
        (180, 255, 50),

        (0, 0, 255)
    )
]


# =====================================================================
# MORPHOLOGY
# =====================================================================

MORPH_K = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE,
    (3, 3)
)

FONT = cv2.FONT_HERSHEY_SIMPLEX


# =====================================================================
# DEBUGGING
#
# Prints approximately every 15 frames instead of flooding the terminal.
# Set DEBUG = False once everything works.
# =====================================================================

DEBUG = True
DEBUG_EVERY_N_FRAMES = 15

frame_counter = 0


# =====================================================================
# GROUND PROJECTION
#
# Converts image pixel position to a robot-relative location.
#
# Robot coordinate system:
#
# +X = forward
# +Y = left
#
# =====================================================================

def groundProject(
        px,
        py,
        cx,
        cy,
        fx,
        fy,
        cam_h,
        sinp,
        cosp):

    nx = (px - cx) / fx
    ny = (py - cy) / fy

    # Ray direction in robot coordinates
    forward = cosp - ny * sinp
    left = -nx
    up = -sinp - ny * cosp

    # Pixel ray does not point toward floor
    if up > -0.02:
        return False, 0.0, 0.0

    t = -cam_h / up

    if t <= 0.0:
        return False, 0.0, 0.0

    X = t * forward + CAM_FWD_CM
    Y = t * left + CAM_LEFT_CM

    return True, X, Y


# =====================================================================
# SCALE CONTOUR BACK TO ORIGINAL IMAGE SIZE
# =====================================================================

def scaleContour(contour, scale_factor):

    if scale_factor == 1.0:
        return contour

    scaled = contour.astype(np.float32)

    scaled[:, :, 0] *= scale_factor
    scaled[:, :, 1] *= scale_factor

    return scaled.astype(np.int32)


# =====================================================================
# MAIN LIMELIGHT PIPELINE
# =====================================================================

def runPipeline(image, llrobot):

    global frame_counter

    frame_counter += 1


    # -----------------------------------------------------------------
    # ALWAYS construct valid output first.
    # -----------------------------------------------------------------

    llpython = [0.0] * OUTPUT_SIZE

    best_contour = np.array([[]])


    if image is None or image.size == 0:

        if DEBUG:
            print("[VISION] ERROR: No camera image")

        return best_contour, image, llpython


    full_h, full_w = image.shape[:2]


    # -----------------------------------------------------------------
    # OPTIONAL LIVE CAMERA VALUES FROM CONTROL HUB
    #
    # llrobot[0] = camera pitch
    # llrobot[1] = camera height
    #
    # If zero/not supplied, constants above are used.
    # -----------------------------------------------------------------

    cam_pitch = CAM_PITCH_DEG
    cam_height = CAM_HEIGHT_CM

    try:

        if llrobot is not None:

            if len(llrobot) >= 1:
                if abs(llrobot[0]) > 0.001:
                    cam_pitch = float(llrobot[0])

            if len(llrobot) >= 2:
                if llrobot[1] > 0.001:
                    cam_height = float(llrobot[1])

    except Exception:
        pass


    # -----------------------------------------------------------------
    # RESIZE IMAGE
    # -----------------------------------------------------------------

    if full_w > PROC_WIDTH:

        scale = float(PROC_WIDTH) / float(full_w)

        sw = PROC_WIDTH
        sh = max(
            1,
            int(round(full_h * scale))
        )

        small = cv2.resize(
            image,
            (sw, sh),
            interpolation=cv2.INTER_AREA
        )

    else:

        scale = 1.0

        sw = full_w
        sh = full_h

        small = image.copy()


    inverse_scale = 1.0 / scale


    # -----------------------------------------------------------------
    # BLUR + HSV
    # -----------------------------------------------------------------

    small = cv2.blur(
        small,
        (BLUR_KSIZE, BLUR_KSIZE)
    )

    hsv = cv2.cvtColor(
        small,
        cv2.COLOR_BGR2HSV
    )


    # -----------------------------------------------------------------
    # CAMERA MODEL
    # -----------------------------------------------------------------

    fx = (sw * 0.5) / math.tan(
        math.radians(HFOV_DEG * 0.5)
    )

    fy = (sh * 0.5) / math.tan(
        math.radians(VFOV_DEG * 0.5)
    )

    cx = sw * 0.5
    cy = sh * 0.5

    frame_area = float(sw * sh)


    pitch_rad = math.radians(cam_pitch)

    sinp = math.sin(pitch_rad)
    cosp = math.cos(pitch_rad)


    # -----------------------------------------------------------------
    # ALL DETECTED ELEMENTS
    # -----------------------------------------------------------------

    detections = []

    raw_counts = {
        1: 0,
        2: 0,
        3: 0
    }


    # =================================================================
    # PROCESS EACH COLOR
    # =================================================================

    for (
        name,
        cid,
        lo1,
        hi1,
        lo2,
        hi2,
        draw_color
    ) in CLASS_DEFS:


        # -------------------------------------------------------------
        # HSV MASK
        # -------------------------------------------------------------

        mask = cv2.inRange(
            hsv,
            np.array(lo1, dtype=np.uint8),
            np.array(hi1, dtype=np.uint8)
        )


        # Red requires two hue ranges
        if lo2 is not None:

            second_mask = cv2.inRange(
                hsv,
                np.array(lo2, dtype=np.uint8),
                np.array(hi2, dtype=np.uint8)
            )

            mask = cv2.bitwise_or(
                mask,
                second_mask
            )


        # -------------------------------------------------------------
        # REMOVE SMALL COLOR NOISE
        # -------------------------------------------------------------

        mask = cv2.erode(mask, MORPH_K, iterations=EROSION_STEPS)
        mask = cv2.dilate(mask, MORPH_K, iterations=DILATION_STEPS)


        # -------------------------------------------------------------
        # FIND CONTOURS
        # -------------------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        raw_counts[cid] = len(contours)


        if len(contours) == 0:
            continue


        # -------------------------------------------------------------
        # FIRST FILTER
        #
        # Area
        # Aspect Ratio
        # Fullness
        # -------------------------------------------------------------

        kept = []


        for contour in contours:

            area = cv2.contourArea(contour)

            if area <= 0.0:
                continue


            # -------------------------
            # AREA %
            # -------------------------

            area_pct = (
                100.0 *
                area /
                frame_area
            )


            if (
                area_pct < MIN_AREA_PCT or
                area_pct > MAX_AREA_PCT
            ):
                continue


            # -------------------------
            # BOUNDING RECTANGLE
            # -------------------------

            x, y, w, h = cv2.boundingRect(
                contour
            )


            if w < 2 or h < 2:
                continue


            # -------------------------
            # WIDTH / HEIGHT
            # -------------------------

            aspect = float(w) / float(h)


            if (
                aspect < MIN_ASPECT or
                aspect > MAX_ASPECT
            ):
                continue


            # -------------------------
            # FULLNESS
            #
            # How much of the bounding
            # rectangle contains the
            # actual contour.
            # -------------------------

            rectangle_area = float(w * h)

            if rectangle_area <= 0:
                continue


            fullness = area / rectangle_area


            if (
                fullness < MIN_FULLNESS or
                fullness > MAX_FULLNESS
            ):
                continue


            kept.append({
                "contour": contour,
                "area": area,
                "area_pct": area_pct,

                "x": x,
                "y": y,
                "w": w,
                "h": h,

                "fullness": fullness
            })


        if len(kept) == 0:
            continue


        # -------------------------------------------------------------
        # SORT LARGEST FIRST
        # -------------------------------------------------------------

        kept.sort(
            key=lambda d: d["area"],
            reverse=True
        )


        # -------------------------------------------------------------
        # SMART SPECKLE REJECTION
        #
        # Relative to largest object OF THIS COLOR.
        # -------------------------------------------------------------

        largest_area = kept[0]["area"]

        speckle_cutoff = (
            largest_area *
            SPECKLE_KEEP_RATIO
        )


        # -------------------------------------------------------------
        # PROJECT EACH VALID ELEMENT ONTO FLOOR
        # -------------------------------------------------------------

        for item in kept:

            if item["area"] < speckle_cutoff:
                continue


            x = item["x"]
            y = item["y"]

            w = item["w"]
            h = item["h"]


            # Horizontal center
            px = x + (w * 0.5)

            # Bottom of detected ball.
            # Better representation of ground contact point.
            py = float(y + h)


            ok, X, Y = groundProject(
                px,
                py,

                cx,
                cy,

                fx,
                fy,

                cam_height,

                sinp,
                cosp
            )


            if not ok:
                continue


            distance = math.hypot(
                X,
                Y
            )


            if distance > MAX_RANGE_CM:
                continue


            detections.append({

                "name": name,

                "cid": cid,

                "color": draw_color,

                "contour": item["contour"],

                "area": item["area"],

                "area_pct": item["area_pct"],

                "fullness": item["fullness"],

                "x": x,
                "y": y,
                "w": w,
                "h": h,

                "X": X,
                "Y": Y,

                "distance": distance
            })


    # =================================================================
    # SORT ALL TARGETS
    #
    # Matches your Limelight "Largest" sorting setting.
    # =================================================================

    detections.sort(
        key=lambda d: d["area"],
        reverse=True
    )


    # =================================================================
    # BEST CONTOUR
    #
    # This gives Limelight its normal "best target".
    # =================================================================

    if len(detections) > 0:

        best_contour = scaleContour(
            detections[0]["contour"],
            inverse_scale
        )


    # =================================================================
    # BUILD FTC OUTPUT
    #
    # Index 0:
    # number actually reported
    #
    # Then:
    #
    # type, X, Y
    # type, X, Y
    # ...
    #
    # =================================================================

    reported_count = min(
        len(detections),
        MAX_REPORTED
    )


    llpython[0] = float(
        reported_count
    )


    for i in range(reported_count):

        detection = detections[i]

        base = 1 + (i * 3)

        llpython[base] = float(
            detection["cid"]
        )

        llpython[base + 1] = float(
            detection["X"]
        )

        llpython[base + 2] = float(
            detection["Y"]
        )


    # Last value tells Control Hub how many objects
    # existed before the 10-object reporting limit.

    llpython[31] = float(
        len(detections)
    )


    # =================================================================
    # DRAW DETECTIONS
    # =================================================================

    output = image.copy()


    for index, detection in enumerate(detections):

        x = int(
            detection["x"] *
            inverse_scale
        )

        y = int(
            detection["y"] *
            inverse_scale
        )

        w = int(
            detection["w"] *
            inverse_scale
        )

        h = int(
            detection["h"] *
            inverse_scale
        )


        color = detection["color"]


        cv2.rectangle(
            output,

            (x, y),

            (x + w, y + h),

            color,

            2
        )


        label = "{} X:{:.0f} Y:{:.0f}".format(

            detection["name"],

            detection["X"],

            detection["Y"]
        )


        cv2.putText(
            output,

            label,

            (
                x,
                max(15, y - 5)
            ),

            FONT,

            0.45,

            color,

            1,

            cv2.LINE_AA
        )


    # =================================================================
    # DEBUG TERMINAL OUTPUT
    #
    # This is the part your previous script was missing.
    # Returning llpython DOES NOT automatically print it.
    # =================================================================

    if (
        DEBUG and
        frame_counter % DEBUG_EVERY_N_FRAMES == 0
    ):

        print(
            "[VISION] Raw contours "
            "Y:{} B:{} R:{} | Valid:{} | Returned:{}".format(

                raw_counts[1],
                raw_counts[2],
                raw_counts[3],

                len(detections),

                reported_count
            )
        )


        if reported_count > 0:

            for i in range(reported_count):

                d = detections[i]

                print(
                    "  {}: {}  X={:.1f}cm Y={:.1f}cm Dist={:.1f}cm Area={:.4f}%".format(

                        i,

                        d["name"],

                        d["X"],

                        d["Y"],

                        d["distance"],

                        d["area_pct"]
                    )
                )


        else:

            print(
                "  No objects survived filtering."
            )


    # =================================================================
    # RETURN TO LIMELIGHT
    # =================================================================

    return best_contour, output, llpython