# Centerstage-Samples
FIRST Tech Challenge sample programs for the CENTERSTAGE season.

These programs were created for a start of season event hosted by FIRST Robotics BC at UVIC on September 23 2023. Additional programs were later added.

- RobotAutoDriveToAprilTagOmni.java - copy of FIRST provided sample program for our demobot. Uses April Tag 584.
- AutoPixel1.java - Autonmous program that uses Tensorflow to detect the spike make with the pixel. It then drops off the purple pixel and drives to the backdrop using April Tag navigation where it attempts to place the yellow pixel before parking backstage. The program drives by simple time based motor control. See YouTube video here: https://www.youtube.com/watch?v=ZWS4QLBFS_0
- AutoPixelFront.java - Like AutoPixel1, but from the front starting position. It also uses the IMU for some turns and for guidance driving backstage. See YouTube video here: https://www.youtube.com/watch?v=_avcy1a8_TU
- Demobot1.java - Teleop driving program.
- ConceptScanServo.java - Initially used to verify servo operation. Later modified to test the movement of the arm against the backdrop to determine the correct servo position for placing the pixel.
- TFOD_BlueFrontAprilTag.java - an example CENTERSTAGE autonomous program that incorporates TensorFlow Team Prop detection, IMU controlled driving, use of the front wall April Tag to drive to position, IMU driving to the backstage area, April Tag driving to line up on the correct backdrop area, and finally a touch sensor to stop when moving up to the backdrop prior to placing the yellow pixel. YouTube video here: https://youtu.be/xFjiMfd2YHA

The programs make use of a demonstration robot uses mecanum wheels and four motors. 
![Model](https://raw.githubusercontent.com/acharraggi/Centerstage-Samples/main/images/robot-side-image.jpg)


- It has two webcams, one points down for Pixel detection, one points forward for April Tag detection.
- It has a plow-like device under the robot that can deliver the purple pixel to the spike marks. It's able to collect pixels from the wings, but not very well.
- It has a small arm that can attempt to place the yellow pixel on the backdrop. It's not able to pick up pixels.
- It has a Rev touch sensor on the front.
- The control hub has an internal Inertial Measurement Unit (IMU), that one of the autonomous programs uses to assist in controlled turns.

![Front View](https://raw.githubusercontent.com/acharraggi/Centerstage-Samples/main/images/robot-front-image.jpg)
