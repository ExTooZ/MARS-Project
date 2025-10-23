# MARS-Project
The goal of the project "Camera and Laser Based Distance Control" is to maintain a certain constant distance between the ground and camera to ensure high quality images taken by the camera.

## Laser Based Distance Control Manual
Before starting the final code, some steps are needed to ensure that the final code runs perfectly

1. The initial image must first be taken by running the program Picture.py. It is a simple program to capture an image using a camera.
2. Using the red_detection.py program, the HSV values can be adjusted accordingly to get the best outcome with minimal noise. Environment factors such as lighting will also play a huge roll in detecting the line laser.
3. Another picture can be taken. This time the second position of the robot to make sure that the robot detects 
   the reference object using the object_size.py 

After everything had been adjusted accordingly, the final code can then be executed
