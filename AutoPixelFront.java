/* AutoPixelFront - example autonomous mode program for Centerstage

    The program attempts to recognize the pixel on a spike more, place purple pixel on that mark 
        and yellow pixel in the corresponding area on the backdrop.
        
    It makes use of TensorFlow to detect a Pixel and April Tags to detect the backdrop
    and also uses the IMU for some turns and driving towards backstage.
        
    Starting position - red alliance, audience side of field, no Team Art
    
    Program flow:
    - move forward to that Tensorflow will be able to recognize a pixel
    - examine each spike more in turn
    - if not found on the left side or centre, assume pixel on right side
    - drive toward indicated spike mark, 
    - then spin to enable backing up towards middle of field
    - back away from spike mark, leaving behind purple pixel on the mark
    - use the IMU to turn to line up to go backstage
    - use the IMU to maintain direction while moving backstage
    - use the IMU to turn and face the backdrop
    - use code from RobotAutoDriveToAprilTagOmni to drive and line up on area in backdrop
    - deploy yellow pixel using arm
    - park to the left side of backstage to allow alliance partner robot access to the backdrop.
    
    Note: not all options/steps programmed, this only works if it finds the pixel on the left spike mark.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;
import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The April Tag drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a the motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 */
@Autonomous(name="AutoPixelFront", group = "Demo")

public class AutoPixelFront extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)
    
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = 0;     // 584 Choose the tag you want to approach or set to -1 for ANY tag.
    private int BACK_TAG_ID = 0;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int     myExposure  ;
    private int     myGain      ;
    
    private WebcamName webcam1, webcam2;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    
    IMU imu;
    
    TouchSensor touchSensor;  // Touch sensor Object
    
    // servo variables
    static final double INCREMENT   = 0.08;     // .01 amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.25;     // 0.0 Minimum rotational position

    // Define class members
    Servo   servo;
    double position = MAX_POS;   // start with arm down
    
    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        boolean pixelFound      = false;    // set to trye when pixel found on spike mark
        String pixelLocation   = "";        // set to left, centre, or right
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        int     currentStep            = 1;
        ElapsedTime runtime = new ElapsedTime();
        
        // Initialize the Apriltag Detection process
        //initAprilTag();
        
        initVisionPortal();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        
        // init IMU
        imu = hardwareMap.get(IMU.class, "imu");
        double targetYaw = 0;
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        visionPortal.setActiveCamera(webcam1);
         
        // get a reference to our touchSensor object.
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        
        // Connect to servo "arm"
        servo = hardwareMap.get(Servo.class, "arm");
        servo.setPosition(MAX_POS);
        
        waitForStart();

        runtime.reset();  // start timer for step 1
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            // STEP 1 move forward
            if (currentStep==1) {
                if (runtime.milliseconds() < 400) {
                    moveRobot(1, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 2;
                    runtime.reset();  // start timer for step 2
                }
            }
            
            // STEP 2 slight rotation anti-clockwise - look at left spike mark
            if (currentStep==2) {
                if (runtime.milliseconds() < 100) {
                    moveRobot(0, 0, 1);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 3;
                    runtime.reset();  // start timer for step 3
                }
            }
            
            // STEP 3 use Tensorflow to check for pixel on left spike mark, allow 5 seconds to elapse
            if (currentStep==3) {
                if (runtime.milliseconds() < 30000) { 
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    // Step through the list of recognitions and look for pixel
                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == "Pixel") {
                            currentStep = 4;
                            pixelLocation = "left";
                            DESIRED_TAG_ID = 4;
                            pixelFound = true;
                            runtime.reset();  // start timer for step 4
                        }
                        else {
                            sleep(50);
                        }
                    }   // end for() loop
                }
                else {
                    // pixel not found, try centre spike mark
                    currentStep = 6;
                    runtime.reset();  // start timer for step 6
                }
            }
            
            // STEP 4 move forward towards left spike mark
            if (currentStep==4) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(1, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;
                    runtime.reset();  // start timer for step 5
                }
            }
            
            // STEP 5 rotate counter clockwise around left spike mark
            if (currentStep==5) {
                if (runtime.milliseconds() < 900) {
                    moveRobot(0, 0, 1.0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 6;  // backoff
                    runtime.reset();  // start timer for step 6
                }
            }
            
            // STEP 6 back off, leaving purple pixel on mark
            // will end up facing the red April Tags on the front wall
            if (currentStep==6) {
                if (runtime.milliseconds() < 650) {
                    moveRobot(-1, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 22; // prepare to go backstage
                    targetYaw = 90;
                    runtime.reset();  // start timer for step 22
                }
            }
            
            // Note: only programmed to handle the left spike mark position
            
            // STEP 7 [not used] bigger rotation clockwise from right mark to backdrop
            // STEP 10 - [not used] use Tensorflow to check for pixel on centre spike mark, allow 5 seconds to elapse
            // STEP 11 - [not used] move forward towards centre spike mark
            // STEP 12 [not used] backoff from centre spike mark, dropping off purple pixel
            // STEP 13 [not used] bigger rotation clockwise from center mark to backdrop
            // STEP 15 - [not used] turn towards right spike mark
            // STEP 16 [not used] move forward towards right spike mark
            // STEP 17  [not used] move backward from right spike mark
            // STEP 21 - [not used] get April Tag values for tag 7 (big red tag)

            // STEP 22 - do IMU turn to line up for run to backstage
            if (currentStep==22) {
                if ((orientation.getYaw(AngleUnit.DEGREES) > targetYaw) 
                    && (runtime.milliseconds() < 1000)) {
                    moveRobot(0, 0, -0.5); // clockwise
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 23;  // go to backdrop
                    visionPortal.setActiveCamera(webcam2);
                    runtime.reset();  // start timer
                }
            }
            
            // STEP 23 - drive by IMU Yaw heading to go backstage
            if (currentStep==23) {
                if (runtime.milliseconds() < 2000) {
                    double  headingError    = targetYaw - orientation.getYaw(AngleUnit.DEGREES);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    moveRobot(-1, 0, turn); // go backwards, adjusting for yaw
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 24;  // rotate to face backdrop
                    targetYaw = -100;
                    runtime.reset();  // start timer
                }
            }
            
            // STEP 24 - IMU turn to face backdrop
            if (currentStep==24) {
                if ((orientation.getYaw(AngleUnit.DEGREES) > targetYaw) 
                    && (runtime.milliseconds() < 3500)) {
                    moveRobot(0, 0, 0.9); // counterclockwise
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 30;  // drive to backdrop
                    //runtime.reset();  // start timer
                }
            }
            
            // STEP 30 - use backgrop April Tag values to move to backdrop
            if (currentStep==30) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)){
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
                
                if ( targetFound) {
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;
                                  
                    if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
                        drive = 0;
                        turn = 0;
                        strafe = 0;
                        currentStep = 31;  // drive to backdrop
                    } 
                    else {
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    }
                    
                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);
                }
                else {
                    moveRobot(0, 0, 0);
                    sleep(10);
                }
            }
            
            // STEP 31 touch backdrop, drive until touch sensor
            if (currentStep==31) {
                if (touchSensor.isPressed()) { 
                    //stop when sensor touched
                    moveRobot(0, 0, 0);
                    currentStep=32;  // deploy arm
                }
                else {
                    moveRobot(0.2, 0, 0);  // move forward slowly
                }
            }
            
            // STEP 32 deploy arm with yellow pixel to backdrop
            if (currentStep==32) {
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    currentStep = 33;  // raise arm
                    sleep(250);        // let pixel fall
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }
            
            // STEP 33 retract arm
            if (currentStep==33) {
                position += 0.02;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    currentStep = 34;  // go park
                    runtime.reset();  // start timer, allow for final sleep cycle
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }
            
            // STEP 34 park, want to back away a little bit and strafe left -X, +Y
            if (currentStep==34) {
                if (runtime.milliseconds() < 900) {
                   moveRobot(-0.1, 0.8, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 100;  // we are done
                }
            }
            
            telemetry.addData("current step", currentStep);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("pixel found", pixelFound);
            telemetry.addData("pixel location", "%s", pixelLocation);
            telemetry.addData("tag target", DESIRED_TAG_ID);
            telemetry.addData("tag found", targetFound);
            if (targetFound)  {
                telemetry.addData("tag bearning", desiredTag.ftcPose.bearing);
            } 
            telemetry.addData("target yaw", targetYaw);
            telemetry.update();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
    

    private void initVisionPortal() {
        
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
            .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        
                // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()   
            // Use setModelAssetName() if the TF Model is built in as an asset.
            // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)
            //.setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)
            .build();
        tfod.setZoom(1.5);
        
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //.setLensIntrinsics(3164.64, 3164.64, 1935.8, 1092.57) // hires webcam
        .setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157) // logitech 920
        .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .setCameraResolution(new Size(1920, 1080))
                    .addProcessor(aprilTag)
                    .addProcessor(tfod)
                    .build();
    }
    
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // Use setModelAssetName() if the TF Model is built in as an asset.
            // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)

            //.setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)
            
            .build();
        tfod.setZoom(1.5);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        /* positive values of x move forward
           positive values of y move sideways to the right 
           positive values of yaw rotate clockwise
        */
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //.setLensIntrinsics(3164.64, 3164.64, 1935.8, 1092.57) // hires webcam
        .setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157) // logitech 920
        .build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .setCameraResolution(new Size(1920, 1080))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
