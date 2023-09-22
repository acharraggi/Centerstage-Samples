# Centerstage-Samples
FIRST Tech Challenge sample programs for the CENTERSTAGE season.
- RobotAutoDriveToAprilTagOmni.java - copy of FIRST provided sample program for our demobot. Uses April Tag 584.
- AutoPixel1.java - Autonmous program that uses Tensorflow to detect the spike make with the pixel. It then drops off the purple pixel and drives to the backdrop where it attempts to place the yellow pixel before parking backstage. The program drives by simple time based motor control.
- AutoPixelFront.java - Like AutoPixel1, but from the front starting position. It also uses the IMU for some turns and for guidance driving backstage.
- Demobot1.java - Teleop driving program.
- ConceptScanServo.java - Initially used to verify servo operation. Later modified to test the movement of the arm against the backdrop to determine the correct servo position for placing the pixel.