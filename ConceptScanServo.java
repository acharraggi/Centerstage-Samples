/* This started at the ConceptScanServo sample program.
    Then modified to control the arm and set the arm to position 100 (down).
    With the robot placed at the backdrop I was able to use this program to determine 
    the correct position needed to place the pixel on the backdrop.
    
    I learned that "throwing" the pixel didn't work.
    This happened naturally because this arm only has a pixel holder, it doesn't grip the
    pixel so it just falls out as the arm goes past vertical.
    The arm needed to actually place the pixel on the backdrop and then release it.
    
    By moving the arm a bit faster the pixel stays in the holder and is pinned to the backdrop
    by the holder. Then the arm retracts to release the pixel.
    
    Also learned that a pixel can bounce off the bottom if released too high on the backdrop,
    so the arm was shortened.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Concept: Scan Servo", group = "Concept")

public class ConceptScanServo extends LinearOpMode {

    static final double INCREMENT   = 0.08;     // .01 amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.25;     // 0.0 Minimum rotational position

    // Define class members
    Servo   servo;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position = MAX_POS;
    boolean rampUp = false; //true;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "arm");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        
        servo.setPosition(position);
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
            //    position += INCREMENT ;
            //    if (position >= MAX_POS ) {
            //        position = MAX_POS;
            //        rampUp = !rampUp;   // Switch ramp direction
            //    }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
