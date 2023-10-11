/* AutoPixel1.java - sample autonomous for Centerstage.

    The program attempts to recognize the pixel on a spike more, place purple pixel on that mark 
        and then a yellow pixel in the corresponding area on the backdrop.
        
    Starting position - red alliance, backstage, no Team Prop
    
    Program flow:
    - move forward to that Tensorflow will be able to recognize a pixel
    - examine each spike more in turn
    - if not found on the left side or centre, assume pixel on right side
    - drive toward indicated spike mark, then reverse, leaving behind purple pixel on the mark
    - turn and face the backdrop
    - change from TensorFlow to April Tag vision processing 
        Note: this should be replaced with a switchable camera logic and dual processors, see AutoPixelFront
    - use code from RobotAutoDriveToAprilTagOmni to drive and line up on area in backdrop
    - deploy yellow pixel using arm
    - park to the right side of backstage to allow alliance partner robot access to the backdrop.
    
    Limitations: 
    - not best use of Vision Portal functions
    - Uses timed motor commands for movement, works ok for this bot as long as battery
      fully charged. Will not work well otherwise. Should use IMU and other methods for positioning
      Maybe using the April Tags for X,Y position checking, not just direction
    - This programs uses a Linear OpMode, but with all it's Step/State logic should probably
      be just a regular OpMode using state machine logic.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import android.util.Size;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

/**  Comments from the RobotAutoDriveToAprilTagOmni program
 * 
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
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
@Autonomous(name="AutoPixel1", group = "Demo")

public class AutoPixel1 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 10; //  this is how close the camera should get to the target (inches)
    
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.03  ;   // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   // Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = 0;     //  Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int     myExposure  ;
    private int     myGain      ;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    
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
        
        initTfod();

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

        // get a reference to our touchSensor object.
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        
        // Connect to servo "arm"
        servo = hardwareMap.get(Servo.class, "arm");
        servo.setPosition(MAX_POS);
    
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        runtime.reset();  // start timer for step 1
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            
            // STEP 1 move forward
            if (currentStep==1) {
                if (runtime.milliseconds() < 450) {
                    moveRobot(10, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 2;
                    runtime.reset();  // start timer for step 2
                }
            }
            
            // STEP 2 slight rotation anti-clockwise - look at left spike mark
            if (currentStep==2) {
                if (runtime.milliseconds() < 175) {
                    moveRobot(0, 0, 5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 3;
                    runtime.reset();  // start timer for step 3
                }
            }
            
            // STEP 3 use Tensorflow to check for pixel on left spike mark, allow 5 seconds to elapse
            if (currentStep==3) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    // Step through the list of recognitions and look for pixel
                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == "Pixel") {
                            currentStep = 4;
                            pixelLocation = "left";
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
                if (runtime.milliseconds() < 460) {
                    moveRobot(5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;
                    runtime.reset();  // start timer for step 5
                }
            }
            
            // STEP 5 backoff from left spike mark, dropping off purple pixel
            if (currentStep==5) {
                if (runtime.milliseconds() < 460) {
                    moveRobot(-5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 7;  // point at backdrop
                    runtime.reset();  // start timer for step 7
                }
            }
            
            // STEP 6 slight rotation clockwise to check centre spike mark
            if (currentStep==6) {
                if (runtime.milliseconds() < 125) {
                    moveRobot(0, 0, -5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 10;
                    runtime.reset();  // start timer for step 10
                }
            }
            
            // STEP 7 bigger rotation clockwise from right mark to backdrop
            if (currentStep==7) {
                if (runtime.milliseconds() < 750) {
                    moveRobot(0, 0, -5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;    // head to backdrop
                    //runtime.reset();  // start timer for step 10
                }
            }
            
            // STEP 10 - use Tensorflow to check for pixel on centre spike mark, allow 5 seconds to elapse
            if (currentStep==10) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    // Step through the list of recognitions and look for pixel
                    for (Recognition recognition : currentRecognitions) {
                        if (recognition.getLabel() == "Pixel") {
                            currentStep = 11; // drop off at centre
                            pixelLocation = "centre";
                            pixelFound = true;
                            runtime.reset();  // start timer for step 11
                        }
                        else {
                            sleep(50);
                        }
                    }   // end for() loop
                }
                else {
                    // pixel not found, assume right spike mark
                    pixelLocation = "right";
                    currentStep = 15; // drop off on right mark
                    runtime.reset();  // start timer
                }
            }
            
            // STEP 11 - move forward towards centre spike mark
            if (currentStep==11) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 12;
                    runtime.reset();  // start timer for step 12
                }
            }
            
            // STEP 12 backoff from centre spike mark, dropping off purple pixel
            if (currentStep==12) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(-5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 13;    // turn toward backdrop
                    runtime.reset();  // start timer for step 13
                }
            }
            
            // STEP 13 bigger rotation clockwise from center mark to backdrop
            if (currentStep==13) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, 0, -5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;    // head to backdrop
                }
            }
            
            // STEP 15 - turn towards right spike mark
            if (currentStep==15) {
                if (runtime.milliseconds() < 200) {
                    moveRobot(0, 0, -5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 16;  // move towards mark
                    runtime.reset();  // start timer
                }
            }
            
            // STEP 16  move forward towards right spike mark
            if (currentStep==16) {
                if (runtime.milliseconds() < 480) {
                    moveRobot(5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 17;
                    runtime.reset();  // start timer for step 17
                }
            }
            
            // STEP 17  move backward from right spike mark
            if (currentStep==17) {
                if (runtime.milliseconds() < 480) {
                    moveRobot(-5, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 18; 
                    runtime.reset();  // start timer for step 18
                }
            }
            
            // STEP 18 - point webcam 2 at background
            if (currentStep==18) {
                if (runtime.milliseconds() < 250) {
                    moveRobot(0, 0, -5);
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 20;  // go to backdrop
                    //runtime.reset();  // start timer
                }
            }
            // STEP 20 - use April Tags to move toward backdrop
            //  start by closing TFOD and starting April Tag
            if (currentStep==20) {
                visionPortal.close();
                sleep(50);
                // Initialize the Apriltag Detection process
                initAprilTag();
                sleep(50);
                setManualExposure(20, 250);  // Use low exposure time high gain to reduce motion blur

                if (pixelLocation == "left")
                    DESIRED_TAG_ID = 4;
                else if (pixelLocation == "centre")
                    DESIRED_TAG_ID = 5;
                else DESIRED_TAG_ID = 6;
                
                currentStep = 22; 
                //runtime.reset();  // timer for step 21
            }
            
            // STEP 21 - move to backdrop - not really needed
            
            // STEP 22 - move to backdrop using April Tag
            if (currentStep==22) {
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
                    if ((rangeError < 7) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)) {
                        // if we're close enough, stop using April Tag logic
                        drive = 0;
                        turn = 0;
                        strafe = 0;
                        currentStep = 23;  // drive to backdrop
                    }  
                    else {
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    }
    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                    sleep(10);
                }
                else {
                    moveRobot(0, 0, 0);
                    sleep(10);
                }
            }
            
            // STEP 23 touch backdrop, drive until touch sensor
            if (currentStep==23) {
                if (touchSensor.isPressed()) { 
                    //stop when sensor touched
                    moveRobot(0, 0, 0);
                    currentStep=24;  // deploy arm
                }
                else {
                    moveRobot(0.2, 0, 0);  // move forward slowly
                }
            }
            
            // STEP 24 deploy arm with yellow pixel to backdrop
            if (currentStep==24) {
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    currentStep = 25;  // raise arm
                    sleep(250);        // let pixel fall
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }
            
            // STEP 25 retract arm
            if (currentStep==25) {
                position += 0.02;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    currentStep = 30;  // go park
                    runtime.reset();  // start timer, allow for final sleep cycle
                }
                servo.setPosition(position);
                sleep(CYCLE_MS);  // let servo have time to move
            }
            // STEP 30 - move to the right and park 
            if (currentStep==30) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, -5, 0);
                    sleep(10);
                }
                else { // end of autonomous, robot will do nothing else
                    moveRobot(0, 0, 0);
                    sleep(10);  
                }
            }
                
            telemetry.addData("current step", currentStep);
            telemetry.addData("pixel found", pixelFound);
            telemetry.addData("pixel location", "%s", pixelLocation);
            telemetry.addData("tag target", DESIRED_TAG_ID);
            telemetry.addData("tag found", targetFound);
            telemetry.update();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
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
