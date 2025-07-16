import cv2

for i in range(10):

    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"✅ /dev/video{i}: {frame.shape}")
        else:
            print(f"❌ /dev/video{i}: empty frame")
        cap.release()
