# Limelight SnapScript: AprilTag @ 320x240 90fps (grayscale) → inches + console + llpython
import cv2
import numpy as np
import math

# =========================
# CONFIG — EDIT THESE
# =========================
# 1) Tag size in INCHES (outer black square side)
TAG_SIZE_IN = 6.5
INCH_TO_M   = 0.0254
M_TO_INCH   = 39.3701
TAG_SIZE_M  = TAG_SIZE_IN * INCH_TO_M

# 2) Paste your calibration from Limelight's calibration page (AT THE RES YOU CALIBRATED)
# If you calibrated at (CALIB_W, CALIB_H) different from 320x240, we'll rescale K automatically.
CALIB_W, CALIB_H = 1280, 960  # <--- change to whatever res you used to calibrate
camera_matrix_calib = np.array([[950.0,   0.0, 640.0],
                                [  0.0, 950.0, 480.0],
                                [  0.0,   0.0,   1.0]], dtype=np.float32)
dist_coeffs = np.array([0.01, -0.02, 0.0, 0.0, 0.0], dtype=np.float32)

# Optional: restrict to certain tag IDs
ACCEPT_IDS = None  # e.g., {1,2,3}

# Console print rate (frames)
PRINT_EVERY = 5

# =========================
# Detector setup (tuned for 320x240)
# =========================
DICT = cv2.aruco.DICT_APRILTAG_36h11
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
params = cv2.aruco.DetectorParameters()

# Low-res friendly tweaks
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.adaptiveThreshWinSizeStep = 10
params.minMarkerPerimeterRate = 0.02  # accept smaller tags in low-res
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# =========================
# Helpers
# =========================
_frame = 0
_cached_shape = None
_camera_matrix_runtime = None

def scale_intrinsics(K, from_w, from_h, to_w, to_h):
    sx = to_w / float(from_w)
    sy = to_h / float(from_h)
    K2 = K.copy()
    K2[0,0] *= sx   # fx
    K2[1,1] *= sy   # fy
    K2[0,2] *= sx   # cx
    K2[1,2] *= sy   # cy
    return K2

def rvec_to_euler_xyz(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy >= 1e-6:
        roll  = math.degrees(math.atan2(R[2,1], R[2,2]))   # X
        pitch = math.degrees(math.atan2(-R[2,0], sy))      # Y
        yaw   = math.degrees(math.atan2(R[1,0], R[0,0]))   # Z
    else:
        roll  = math.degrees(math.atan2(-R[1,2], R[1,1]))
        pitch = math.degrees(math.atan2(-R[2,0], sy))
        yaw   = 0.0
    return roll, pitch, yaw

# =========================
# SnapScript entry
# =========================
def runPipeline(image, llrobot):
    global _frame, _cached_shape, _camera_matrix_runtime
    _frame += 1

    h, w = image.shape[:2]
    # Lazily scale intrinsics to the actual runtime res (expect 320x240)
    if _cached_shape != (w, h):
        _cached_shape = (w, h)
        _camera_matrix_runtime = scale_intrinsics(
            camera_matrix_calib, CALIB_W, CALIB_H, w, h
        )
        print(f"[Init] Runtime res={w}x{h}, intrinsics scaled from {CALIB_W}x{CALIB_H}")

    K = _camera_matrix_runtime
    D = dist_coeffs

    # Grayscale for detection (fast & robust)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Choose preview: color (original) or grayscale-BGR
    annotated = image
    # annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)  # <- uncomment for gray preview

    largestContour = np.array([[]])
    llpython = [0, -1, 0, 0, 0, 0, 0, 0]  # [valid, id, x", y", z", roll, pitch, yaw]

    # Detect tags on gray
    corners, ids, _rej = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        if _frame % PRINT_EVERY == 0:
            print("[AprilTag] No tag detected")
        cv2.putText(annotated, "No tag", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)
        cv2.putText(annotated, "No tag", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        return largestContour, annotated, llpython

    ids_flat = ids.flatten().tolist()
    keep = list(range(len(ids_flat))) if ACCEPT_IDS is None else \
           [i for i,t in enumerate(ids_flat) if t in ACCEPT_IDS]
    if not keep:
        if _frame % PRINT_EVERY == 0:
            print(f"[AprilTag] IDs {ids_flat} detected but filtered out")
        cv2.putText(annotated, "Tag filtered", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)
        cv2.putText(annotated, "Tag filtered", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        return largestContour, annotated, llpython

    kept_corners = [corners[i] for i in keep]
    kept_ids     = [ids_flat[i] for i in keep]

    # Pose (tag in camera frame, meters)
    rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
        kept_corners, TAG_SIZE_M, K, D
    )

    # Pick closest (smallest z)
    best = 0
    best_z = float('inf')
    for i in range(len(kept_ids)):
        z = float(tvecs[i][0][2])
        if z < best_z:
            best, best_z = i, z

    tag_id = int(kept_ids[best])
    rvec = rvecs[best].reshape(3,1)
    tvec = tvecs[best].reshape(3,1)

    # Draw overlays (keep light for 90 fps)
    cv2.aruco.drawDetectedMarkers(annotated, [kept_corners[best]], np.array([[tag_id]]))
    cv2.drawFrameAxes(annotated, K, D, rvec, tvec, TAG_SIZE_M * 0.5)

    # Convert to inches & Euler angles (deg)
    # --- angles from OpenCV (about camera X, Y, Z) ---
    roll_x, pitch_y, yaw_z = rvec_to_euler_xyz(rvec)

    # --- remap to your expected labels ---
    # Your report: roll was actually yaw, pitch was actually roll, yaw was actually pitch.
    roll_out  = yaw_z    # roll label should show what used to be yaw (Z)
    pitch_out = roll_x   # pitch label should show what used to be roll (X)
    yaw_out   = pitch_y  # yaw label should show what used to be pitch (Y)

    # If you need sign flips to taste, do them here, e.g.:
    # yaw_out *= -1
    # pitch_out *= -1
    # roll_out *= -1

    x_m, y_m, z_m = [float(v) for v in tvec.flatten()]
    x_in, y_in, z_in = x_m * M_TO_INCH, y_m * M_TO_INCH, z_m * M_TO_INCH

    # On-image text (compact for 320x240)
    line1 = f"ID {tag_id} x={x_in:.1f}\" y={y_in:.1f}\" z={z_in:.1f}\""
    line2 = f"roll={roll_out:.1f} pitch={pitch_out:.1f} yaw={yaw_out:.1f} deg"
    cv2.putText(annotated, line1, (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2, cv2.LINE_AA)
    cv2.putText(annotated, line1, (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(annotated, line2, (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2, cv2.LINE_AA)
    cv2.putText(annotated, line2, (8, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

    if _frame % PRINT_EVERY == 0:
        print(f"[AprilTag] id={tag_id}  x={x_in:.2f}\" y={y_in:.2f}\" z={z_in:.2f}\"  "
            f"roll={roll_out:.1f} pitch={pitch_out:.1f} yaw={yaw_out:.1f}")

    llpython = [1, tag_id, x_in, y_in, z_in, roll_out, pitch_out, yaw_out]

    # Return the chosen tag contour so LL crosshair can lock to it
    largestContour = kept_corners[best].astype(np.int32)
    return largestContour, annotated, llpython
