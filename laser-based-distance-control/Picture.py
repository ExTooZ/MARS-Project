import cv2

cap = cv2.VideoCapture(0)

# Capture a single frame
ret, frame = cap.read()

if ret:
    cv2.imwrite('first.jpg', frame)
else:
    print("Error: Could not capture image")

# Release the camera
cap.release()