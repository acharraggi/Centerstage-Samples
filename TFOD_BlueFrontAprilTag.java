/* TFOD_BlueFrontAprilTag - example autonomous mode program for Centerstage

    The program attempts to recognize a Team Prop on a spike more, place the purple pixel on that mark 
        and yellow pixel in the corresponding area on the backdrop.
        
    It makes use of TensorFlow to detect the Team Prop and April Tags to detect the backdrop
    and also uses the IMU for turns and driving backstage.
        
    Starting position - blue alliance, audience side of field, Duplo Team Props
    
    Program flow:
    - Use TensorFlow to detect which spike mark has the Team Prop
    - drive toward indicated spike mark, 
    - backup to deploy the purple pixel on the mark
    - turn to face the front wall April Tag
    - April Tag drive towards the blue field wall
    - use the IMU to turn and face backstage
    - drive towards the backstage area
    - Turn to face the backdrop
    - use code from RobotAutoDriveToAprilTagOmni to drive and line up on area in backdrop
    - deploy yellow pixel using arm
    - park to the right side of backstage to allow alliance partner robot access to the backdrop.
    
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
@Autonomous(name="TFOD_BlueFrontAprilTag", group = "CENTERSTAGE")

public class TFOD_BlueFrontAprilTag extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)
    
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = 10;     // 584 Choose the tag you want to approach or set to -1 for ANY tag.
    private int backdropTagID = 0;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int     myExposure  ;
    private int     myGain      ;
    private boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    
    private WebcamName webcam1 /*, webcam2*/;
    
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    final String TFOD_MODEL_FILE = "redBlueDuploFar.tflite";
    public static final String[] LABELS = {"blueDuplo", "redDuplo"};
    
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

        boolean propFound      = false;    // set to true when Team Prop found on spike mark
        String propLocation   = "";        // set to left, centre, or right
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        int     currentStep            = 1;
        
        // Initialize TensorFlow, April Tag, and Vision Portal
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
        
        //visionPortal.setActiveCamera(webcam1);
         
        // get a reference to our touchSensor object.
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        
        // Connect to servo "arm"
        servo = hardwareMap.get(Servo.class, "arm");
        servo.setPosition(MAX_POS);
        
        waitForStart();

        visionPortal.stopLiveView();  // turn off liveView while robot moving
        runtime.reset();  // start timer for step 1
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            
            // STEP 1 use Tensorflow to check for Team Prop, timeout after 3 seconds
            if (currentStep==1) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    if (currentRecognitions.size() == 0){
                        sleep(50);  // give TensorFlow time to find the prop before trying again
                    }
                    else {
                        // Step through the list of recognitions and look for pixel
                        for (Recognition recognition : currentRecognitions) {
                            if (recognition.getLeft() < 50) {
                                currentStep = 2;
                                propLocation = "left";
                                backdropTagID = 1;
                                propFound = true;
                                runtime.reset();  // start timer for step 2
                                break;
                            }
                            else if (recognition.getLeft() < 350) {
                                currentStep = 3;
                                propLocation = "centre";
                                backdropTagID = 2;
                                propFound = true;
                                runtime.reset();  // start timer for step 3
                                break;
                            }
                            else {
                                currentStep = 4;
                                propLocation = "right";
                                backdropTagID = 3;
                                propFound = true;
                                runtime.reset();  // start timer for step 4
                                break;
                            }
                        }   // end for() loop
                    }
                }
                else {  // Prop not found after timeout, assume Left mark as outside marks harder to detect
                    currentStep = 2;
                    propLocation = "left";
                    backdropTagID = 1;
                    runtime.reset();  // start timer for step 2
                }
            }
            
            // STEP 2 drive forward to left spike mark
            if (currentStep==2) {
                if (runtime.milliseconds() < 500) {
                    imuMove(MAX_AUTO_SPEED, 0.0);  // 
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;    // backup step
                    runtime.reset();  // start timer for step 3
                }
            }
            
            // STEP 3 drive forward to center spike mark
            if (currentStep==3) {
                if (runtime.milliseconds() < 1500) {
                    imuMove(MAX_AUTO_SPEED, 0.0);  // drive forward 
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;    // backup 
                    runtime.reset();  // start timer for step 5
                }
            }
            
            // STEP 4 move forward towards right spike mark
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
            
            // STEP 5 back off, leaving purple pixel on mark
            if (currentStep==5) {
                if (runtime.milliseconds() < 700) {
                    moveRobot(-0.5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    visionPortal.setProcessorEnabled(tfod,false);  // turn off TensorFlow
                    currentStep = 6;  // turn to face front wall
                    runtime.reset();  // start timer for step 6
                }
            }
            
            // STEP 6 turn to heading 90 to face front wall
            // will end up facing the blue April Tags on the front wall
            if (currentStep==6) {
                if (runtime.milliseconds() < 2500) {
                    imuTurn(-90);
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - -90) < 7)){
                        moveRobot(0, 0, 0); // stop any moves
                        currentStep = 9; // go backstage
                        runtime.reset();  // start timer for step 9 
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 7; // prepare to go backstage
                    runtime.reset();  // start timer for step 7
                }
            }
            
            // STEP 7 drive to April Tag target position
            //  - should end up in tile A2
            if (currentStep==7) {
                if (runtime.milliseconds() < 3000) {
                    aprilTagDrive(DESIRED_TAG_ID,23.0,0.0,-38.0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 8; // prepare to go backstage
                    runtime.reset();  // start timer for step 8
                }
            }
            
            // STEP 8 turn to face backstage
            if (currentStep==8) {
                if (runtime.milliseconds() < 4000) {
                    imuTurn(90);
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - 90) < 7)){
                        moveRobot(0, 0, 0); // stop any moves
                        currentStep = 9; // go backstage
                        runtime.reset();  // start timer for step 9 
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 9; // go backstage
                    runtime.reset();  // start timer for step 9
                }
            }
     
            // STEP 9 drive backstage
            if (currentStep==9) {
                if (runtime.milliseconds() < 3500) {
                    imuMove(MAX_AUTO_SPEED, 90);  
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 10;    // turn to face backdrop
                    runtime.reset();     // start timer for step 10
                }
            }
            
            // STEP 10 turn to face backdrop
            if (currentStep==10) {
                if (runtime.milliseconds() < 1250) {
                    imuTurn(45);
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - 45) < 7)){
                        moveRobot(0, 0, 0); // stop any moves
                        currentStep = 11; // drive to backdrop
                        DESIRED_TAG_ID = backdropTagID;
                        setManualExposure(6, 250);
                        runtime.reset();  // start timer for step 11
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 11; // drive to backdrop
                    DESIRED_TAG_ID = backdropTagID;
                    setManualExposure(6, 250);
                    runtime.reset();  // start timer for step 11
                }
            }
            
            // STEP 11 drive to backdrop April Tag
            if (currentStep==11) {
                if (runtime.milliseconds() < 3200) {
                    aprilTagDrive(DESIRED_TAG_ID, 12, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 12; // move to backdrop
                    runtime.reset();  // start timer for step 12
                }
            }
            
            // STEP 12 shift right to position arm for backdrop area
            if (currentStep==12) {
                if (runtime.milliseconds() < 600) {
                   moveRobot(0, -0.3, 0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 31; // move to backdrop
                    //runtime.reset();  // start timer for step 13
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
                if (runtime.milliseconds() < 1000) {
                   moveRobot(-0.1, 0.8, 0);     // park in corner
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 100;  // we are done
                }
            }
            
            telemetry.addData("current step", currentStep);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("prop found", propFound);
            telemetry.addData("prop location", "%s", propLocation);
            telemetry.addData("tag target", DESIRED_TAG_ID);
            telemetry.addData("tag found", targetFound);
            if (targetFound)  {
                telemetry.addData("tag bearning", desiredTag.ftcPose.bearing);
                telemetry.addData("target yaw", desiredTag.ftcPose.yaw);
            } 
            
            telemetry.update();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }
    

    private void initVisionPortal() {
        
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        //webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        //CameraName switchableCamera = ClassFactory.getInstance()
        //    .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
        
                // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()   
            // Use setModelAssetName() if the TF Model is built in as an asset.
            // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)
            .build();
        //tfod.setZoom(1.5);
        
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //.setLensIntrinsics(3164.64, 3164.64, 1935.8, 1092.57) // hires webcam
        //.setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157) // logitech 920
        .build();
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                    //.setCamera(switchableCamera)
                    .setCamera(webcam1)
                    //.setCameraResolution(new Size(1920, 1080))
                    .addProcessor(aprilTag)
                    .addProcessor(tfod)
                    .build();
    }
    
    /**
     * Move robot to a designated position from an April Tag
     * targetTag is the tag to look for
     * targetRange is the desired target range in inches
     * targetBearing is the desired target bearing in degrees
     *  - set zero so camera is looking at the tag
     * targetYaw is the desired target yaw in degrees
     *  - set zero so robot lines up in front of the tag
     * 
     * Note: you must call runtime.reset() just prior to the step that calls aprilTagMove
     */
    public void aprilTagDrive(int targetTag, double targetRange, double targetBearing, double targetYaw) {
        double drive, strafe, turn;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        targetFound = false;
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == targetTag)){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }
        
        if ( targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - targetRange);
            double  headingError    = desiredTag.ftcPose.bearing - targetBearing;
            double  yawError        = desiredTag.ftcPose.yaw - targetYaw;
                          
        //    if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
        //        drive = 0;
        //        turn = 0;
        //        strafe = 0;
        //        currentStep = 31;  // drive to backdrop
        //    } 
        //    else {
        // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            drive=0;  strafe=0; turn=0; 
        }
        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
    }
    
    /**
     * Move robot using a power level, and heading
     * Positive powerLevel is forward
     * duration assume to be positive and in milliseconds
     * heading is direction in IMU coordinate system
     * 
     * Note: you must call runtime.reset() just prior to the step that calls imuMove
     */
    public void imuMove(double powerLevel, /*double duration,*/ double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        if (powerLevel < 0) {
            turn = turn * -1;  // reverse turn if going backwards
        }
        moveRobot(powerLevel, 0, turn);
    }

    /**
     * Turn robot using a power level, and heading
     * Positive powerLevel is forward
     * duration assume to be positive and in milliseconds
     * heading is direction in IMU coordinate system
     * 
     * Note: you must call runtime.reset() just prior to the step that calls imuMove
     */
    public void imuTurn(/*double powerLevel, double duration,*/ double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        moveRobot(0, 0, turn);
    }

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
        sleep(10);
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
