import cv2
import numpy as np  

image = cv2.imread("D:\\Dustin\\Python\\Sample_images\\second2(2).png") # Fill the proper file directory of the saved image

# Convert to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# blurred = cv2.GaussianBlur(hsv, (3, 3), 0)

lower_red1 = np.array([140, 58, 200])       # to be adjusted
upper_red1 = np.array([180, 255, 255]) 
# lower_red1 = np.array([150, 110, 220])
# upper_red1 = np.array([180, 255, 255])
    
# Upper range (170-180)
# lower_red2 = np.array([170, 40, 200])
# upper_red2 = np.array([180, 255, 255])
    
# Create masks
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
kernel = np.ones((3,3), np.uint8)
mask = cv2.erode(mask1, kernel, iterations=1)
mask = cv2.dilate(mask1, kernel, iterations=1)          # For better masking coverage (middle portion of the laser is not detected)

# mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
# Combine masks
#red_mask = cv2.bitwise_or(mask1, mask2)

cv2.imshow("original", image)
cv2.imshow("red", mask)

# cv2.imshow("inverted", inverted)
cv2.waitKey(0)