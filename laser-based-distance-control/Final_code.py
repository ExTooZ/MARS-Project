from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2
import RPi.GPIO as GPIO
from time import sleep
import time
import math

# This program is used as the final code for the distance control with line laser
# The first image as the ground reference will be taken manually
# The second image will be cycled through as the robot is moved

MM_PER_SEC = 4.3 # mm/s, might need to be calibrated every now and then
PWM_DUTY_CYCLE = 100  

in1 = 24
in2 = 23
en = 25

# Setting up the GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)


def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
def get_pixel_per_cm(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged = cv2.Canny(gray, 50, 100)  					# Detect edges (Canny edge detector). Lowering canny threshold can detect more edges (or noise)
    edged = cv2.dilate(edged, None, iterations=1)  		# Dilate (thicken edges) to help bridge some edges due to canny edge detector
    edged = cv2.erode(edged, None, iterations=1)  		# Erode (shrink edges)

    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # sort the contours from left-to-right and initialize the
    # 'pixels per metric' calibration variable
    (cnts, _) = contours.sort_contours(cnts)
    pixelsPerMetric = None

    # loop over the contours individually
    for c in cnts:
        if cv2.contourArea(c) < 4000:                   # if the contour is not sufficiently large, ignore it
            continue

        # compute the rotated bounding box of the contour
        orig = image.copy()  
        box = cv2.minAreaRect(c)                        # Get rotated rectangle
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")                # Convert to integer coordinates

        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        box = perspective.order_points(box)             # Order: TL, TR, BR, BL
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)  # Draw box

        # unpack the ordered bounding box, then compute the midpoint
        # between the top-left and top-right coordinates, followed by
        # the midpoint between bottom-left and bottom-right coordinates
        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)

        # compute the midpoint between the top-left and top-right points,
        # followed by the midpoint between the top-right and bottom-right
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)

        # compute the Euclidean distance between the midpoints
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))		# Get the distance from	left to right THIS WILL BE USED LATER
        
        # compute it as the ratio of pixels to supplied metric
        # (in this case, cm)
        if pixelsPerMetric is None:
            pixelsPerMetric = dB / 4							# Gets the pixel/cm
        return pixelsPerMetric

# Get the distance of between the lasers
# Negative means laser is going to the left
# Positive means laser is going to the right
def get_laser_distance(img1,img2):
    distanceX = 0
    pixel_per_cm= get_pixel_per_cm(img2)
    print(pixel_per_cm," pixels per cm")
    hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([160, 58, 245])
    upper_red1 = np.array([180, 255, 255]) # USED FOR THE RED LINE
    
    mask1 = cv2.inRange(hsv1, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv2, lower_red1, upper_red1)
    kernel = np.ones((7,7), np.uint8)
    new_mask1 = cv2.erode(mask1, kernel, iterations=1)
    new_mask1 = cv2.dilate(mask1, kernel, iterations=1) 
    new_mask2 = cv2.erode(mask2, kernel, iterations=1 )
    new_mask2 = cv2.dilate(mask2, kernel, iterations=1)

    # Find contours in the red mask
    contours1, _ = cv2.findContours(new_mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours2, _ = cv2.findContours(new_mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours1 and contours2:
        # Get the largest contour (assuming it's the lines)
        cnt1 = max(contours1, key=cv2.contourArea)
        cnt2 = max(contours2, key=cv2.contourArea)
        
        # Compute centroids
        M1 = cv2.moments(cnt1)                                  # Compute moments (statistical properties of the contour). Returns all the moments in dictionary form ["m00", "m01", "m10", etc]
        M2 = cv2.moments(cnt2)                
        
        if M1["m00"] != 0 and M2["m00"] != 0: # If the image has white pixels in the mask
            cX1 = int(M1["m10"] / M1["m00"])  # Centroid X in image1
            cX2 = int(M2["m10"] / M2["m00"])  # Centroid X in image2
            print(cX1, "centroid 1")
            print(cX2, "centroid 2")
            distanceX = cX2 - cX1
    print(distanceX, " pixels distance")
    distance = distanceX / pixel_per_cm
    print(distance, " cm laser distance")
    return distance                                             # in cm

# Get the height base on the formula
def get_height(between_lasers):
    height_change = between_lasers * math.tan(math.radians(61.39))
    return height_change
# Adjust the height of the linear actuator
def height_control(distance_mm):
    if distance_mm == 0:
        return

    duration = abs(distance_mm) / MM_PER_SEC

    if distance_mm > 0:
        #downwards motion
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:
        #upwards motion
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)

    p.ChangeDutyCycle(PWM_DUTY_CYCLE)
    sleep(duration)

    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
def main():
    global p
    p = GPIO.PWM(en, 1000)
    p.start(0)
    try:
        # Load reference image
        image1 = cv2.imread("first.jpg")
        if image1 is None:
            print(f"Error: Could not load reference image {image1}")
            return

        # Initialize camera with V4L2 backend
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("Error: Could not open camera")
            return

        # Adjusting camera settings
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not capture frame")
            return
        
        # Process images
        between_lasers = get_laser_distance(image1, frame)
        height = get_height(between_lasers) * 10  # cm to mm
        print(height, "cm")
        # Control height
        height_control(height)
    finally:
        # Cleanup in correct order
        if 'cap' in locals() and cap.isOpened():
            cap.release()
        try:
            if 'p' in globals():
                p.ChangeDutyCycle(0)  # Stop PWM signal first
                p.stop()              # Then stop PWM
                sleep(0.1)            # Short delay
        except:
            pass
        
        GPIO.cleanup()
        cv2.destroyAllWindows()

main()                                # Run program
