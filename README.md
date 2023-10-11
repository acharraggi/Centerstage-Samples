# Centerstage-Samples
FIRST Tech Challenge sample programs for the CENTERSTAGE season.
- RobotAutoDriveToAprilTagOmni.java - copy of FIRST provided sample program for our demobot. Uses April Tag 584.
- AutoPixel1.java - Autonmous program that uses Tensorflow to detect the spike make with the pixel. It then drops off the purple pixel and drives to the backdrop using April Tag navigation where it attempts to place the yellow pixel before parking backstage. The program drives by simple time based motor control. See YouTube video here: https://www.youtube.com/watch?v=ZWS4QLBFS_0
- AutoPixelFront.java - Like AutoPixel1, but from the front starting position. It also uses the IMU for some turns and for guidance driving backstage. See YouTube video here: https://www.youtube.com/watch?v=_avcy1a8_TU
- Demobot1.java - Teleop driving program.
- ConceptScanServo.java - Initially used to verify servo operation. Later modified to test the movement of the arm against the backdrop to determine the correct servo position for placing the pixel.