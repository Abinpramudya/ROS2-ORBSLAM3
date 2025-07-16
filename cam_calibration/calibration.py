import cv2
import numpy as np
import yaml

# === Settings ===
chessboard_size = (9, 7)  # Inner corners
square_size = 0.025       # Size of a square (e.g., 2.5 cm)
video_source = 0          # Change if not webcam, e.g., 'underwater_video.mp4'

# === Prepare object points (0,0,0), (1,0,0) ...
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                      0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

cap = cv2.VideoCapture(video_source)

print("Press SPACE to capture a frame, ESC to finish")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    display = frame.copy()
    if found:
        cv2.drawChessboardCorners(display, chessboard_size, corners, found)

    cv2.imshow("Calibration", display)
    key = cv2.waitKey(1)

    if key == 27:  # ESC
        break
    elif key == 32 and found:  # SPACE
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        print(f"[INFO] Captured frame {len(objpoints)}")

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 5:
    print("Not enough valid frames for calibration.")
    exit()

# === Camera Calibration ===
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

fx, fy = mtx[0, 0], mtx[1, 1]
cx, cy = mtx[0, 2], mtx[1, 2]
k1, k2, p1, p2 = dist[0][0:4]

# === YAML File Creation ===
yaml_dict = {
    "File.version": "1.0",
    "Camera.type": "PinHole",
    "Camera1.fx": float(fx),
    "Camera1.fy": float(fy),
    "Camera1.cx": float(cx),
    "Camera1.cy": float(cy),
    "Camera1.k1": float(k1),
    "Camera1.k2": float(k2),
    "Camera1.p1": float(p1),
    "Camera1.p2": float(p2),
    "Camera.width": int(gray.shape[1]),
    "Camera.height": int(gray.shape[0]),
    "Camera.newWidth": int(gray.shape[1] * 0.8),
    "Camera.newHeight": int(gray.shape[0] * 0.8),
    "Camera.fps": 30,
    "Camera.RGB": 1,
    "ORBextractor.nFeatures": 1000,
    "ORBextractor.scaleFactor": 1.2,
    "ORBextractor.nLevels": 8,
    "ORBextractor.iniThFAST": 20,
    "ORBextractor.minThFAST": 7,
    "Viewer.KeyFrameSize": 0.05,
    "Viewer.KeyFrameLineWidth": 1.0,
    "Viewer.GraphLineWidth": 0.9,
    "Viewer.PointSize": 2.0,
    "Viewer.CameraSize": 0.08,
    "Viewer.CameraLineWidth": 3.0,
    "Viewer.ViewpointX": 0.0,
    "Viewer.ViewpointY": -0.7,
    "Viewer.ViewpointZ": -1.8,
    "Viewer.ViewpointF": 500.0
}

with open("camera_calibration.yaml", "w") as f:
    f.write("%YAML:1.0\n\n")
    yaml.dump(yaml_dict, f, default_flow_style=False)

print("\n✅ Calibration complete and saved to camera_calibration.yaml")
