/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Gold Align Depot", group="DogeCV")

public class GoldAlignMineDepot extends LinearOpMode {
    // Detector object
    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();

    // Drive
    private DcMotor lf = null;
    private DcMotor lr = null;
    private DcMotor rf = null;
    private DcMotor rr = null;

    // Gamepad 2
    private DcMotor Lift = null;
    private DcMotor Slide = null;
    private DcMotor Flip = null;
    private DcMotor inTake = null;

    // Servo
    private Servo AngleR = null;
    private Servo AngleL = null;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5; // Speed originally 0.9
    static final double TURN_SPEED = 0.3;
    static final double LIFT_SPEED = 1.0;
    static final double  EIGHT_SPEED = 0.8;


    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Flip = hardwareMap.get(DcMotor.class, "Flip");
        inTake = hardwareMap.get(DcMotor.class, "inTake");

        AngleR = hardwareMap.get(Servo.class, "AngleR");
        AngleL = hardwareMap.get(Servo.class, "AngleL");

        inTake.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Slide.setDirection(DcMotor.Direction.REVERSE);

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path0", "Starting at %7d :%7d",
//                lf.getCurrentPosition(),
//                lr.getCurrentPosition(),
//                rf.getCurrentPosition(),
//                rr.getCurrentPosition());
//
//        telemetry.update();

        telemetry.addData("Status", "DogeCV 2019.1 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

//        if (opModeIsActive()) {
        /* Activate DogeCV Detection. */
        if (detector != null) {
            detector.enable();
        }

//        detector.enable(); // Start the detector!
        waitForStart();
        runtime.reset();

//        encoderLift(LIFT_SPEED, 23, 5.0);

        // Up
        AngleL.setPosition(1.0);
        AngleR.setPosition(0.15);
        sleep(1000);

        // Strafe right
//        encoderDrive(TURN_SPEED, 3, -3, -3, 3, 2.0);
//        sleep(10);

        // telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
        // If its aligned == gold position MIDDLE
        // If its NOT aligned == gold position RIGHT
        // If its not found == gold position LEFT
        while (opModeIsActive() && runtime.seconds() < 2.0) {
            if (detector.isFound()) {
                // if else, else statements
                if (detector.getAligned()) { // gold MID
                    telemetry.addData("Aligned?", "YES");
                    telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
                    // Forward
                    encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 3.0);
                    sleep(10);

                    // Strafe left SLOW
                    encoderDrive(DRIVE_SPEED, -4, 4, 4, -4, 2.0);
                    sleep(10);

                    // Forward
                    encoderDrive(DRIVE_SPEED, 13, 13, 13, 13, 3.0);
                    sleep(10);

                    // Half way
                    AngleL.setPosition(0.5);
                    AngleR.setPosition(0.5);
                    sleep(2000);

                    // inTake Out
                    inTake.setPower(1);
                    sleep(1000);

                    inTake.setPower(0);
                    sleep(10);

                    // Up
                    AngleL.setPosition(1.0);
                    AngleR.setPosition(0.15);
                    sleep(1000);


                    // Backward
                    encoderDrive(DRIVE_SPEED, -17, -17, -17, -17, 3.0);
                    sleep(10);

                    // Turn right
                    encoderDrive(DRIVE_SPEED, -13, -13, 13, 13, 3.0);
                    sleep(10);

                    // Forward
                    encoderDrive(DRIVE_SPEED, 23, 23, 23, 23, 5.0);
                    sleep(10);

                    // Turn right
                    encoderDrive(DRIVE_SPEED, -7, -7, 7, 7, 3.0);
                    sleep(10);

                    // Strafe right SLOW
                    encoderDrive(TURN_SPEED, 12, -12, -12, 12, 3.0);
                    sleep(10);

                    // Strafe right
                    encoderDrive(DRIVE_SPEED, 4, -4, -4, 4, 3.0);
                    sleep(10);

                    // Strafe left
                    encoderDrive(TURN_SPEED, -2, 2, 2, -2, 1.0);
                    sleep(10);

                    // Forward
                    encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 3.0);
                    sleep(10);

                    // Down
                    AngleL.setPosition(0.24);
                    AngleR.setPosition(0.86);
                    sleep(1000);
                    sleep(10000);

                }

                if (!detector.getAligned()) { // gold RIGHT
                    telemetry.addData("Aligned?", "NO");
                    telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.

                    //Forward
                    encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 3.0);
                    sleep(10);

                    // Strafe right
                    encoderDrive(DRIVE_SPEED, 8, -8, -8, 8, 3.0);
                    sleep(10);

                    // Forward
                    encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 3.0);
                    sleep(10);

                    // Backward
                    encoderDrive(DRIVE_SPEED, -6.5, -6.5, -6.5, -6.5, 2.0);
                    sleep(10);

                    // Strafe left
                    encoderDrive(DRIVE_SPEED, -37, 37, 37, -37, 5.0);
                    sleep(10);

                    // Turn left
                    encoderDrive(DRIVE_SPEED, 6, 6, -6, -6, 3.0);
                    sleep(10);

                    // Strafe left SLOW
                    encoderDrive(TURN_SPEED, -14, 14, 14, -14, 4.0);
                    sleep(10);

                    // Strafe left
                    encoderDrive(DRIVE_SPEED, -3, 3, 3, -3, 2.0);
                    sleep(10);

                    // Strafe right SLOW
                    encoderDrive(TURN_SPEED, 2, -2, -2, 2, 2.0);
                    sleep(10);

                    // Forward
                    encoderDrive(DRIVE_SPEED, 28, 28, 28, 28, 5.0);
                    sleep(10);


                    // Half way
                    AngleL.setPosition(0.5);
                    AngleR.setPosition(0.5);
                    sleep(2000);

                    // inTake Out
                    inTake.setPower(1);
                    sleep(1000);

                    inTake.setPower(0);
                    sleep(10);

                    // Up
                    AngleL.setPosition(1.0);
                    AngleR.setPosition(0.15);
                    sleep(1000);




                    // Backward
                    encoderDrive(DRIVE_SPEED, -20, -20, -20, -20, 4.0);
                    sleep(10);

                    // Turn right
                    encoderDrive(DRIVE_SPEED, -26, -26, 26, 26, 5.0);
                    sleep(10);

                    // Strafe right SLOW
                    encoderDrive(TURN_SPEED, 6, -6, -6, 6, 2.0);
                    sleep(10);

                    // Strafe left
                    encoderDrive(TURN_SPEED, -3, 3, 3, -3, 2.0);
                    sleep(10);

                    // Forward
                    encoderDrive(EIGHT_SPEED, 14, 14, 14, 14, 3.0);
                    sleep(10);



                    // Down
                    AngleL.setPosition(0.24);
                    AngleR.setPosition(0.86);
                    sleep(1500);
                    sleep(10000);

                }
            }

            else { // gold LEFT
                telemetry.addData("Aligned?", "NO");
                telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.

                // Forward
                encoderDrive(DRIVE_SPEED, 7, 7, 7, 7, 3.0);
                sleep(10);

                // Strafe left
                encoderDrive(DRIVE_SPEED, -13, 13, 13, -13, 4.0);
                sleep(10);

                // Forward
                encoderDrive(DRIVE_SPEED, 20, 20, 20, 20, 3.0);
                sleep(10);

                // Turn left
                encoderDrive(DRIVE_SPEED, 6, 6, -6, -6, 2.0);

                // Half way
                AngleL.setPosition(0.5);
                AngleR.setPosition(0.5);
                sleep(2000);

                // inTake Out
                inTake.setPower(1);
                sleep(1000);

                inTake.setPower(0);
                sleep(10);

                // Up
                AngleL.setPosition(1.0);
                AngleR.setPosition(0.15);
                sleep(1000);


                // Backward
                encoderDrive(DRIVE_SPEED, -17, -17, -17, -17, 3.0);
                sleep(10);

                // Strafe left SLOW
                encoderDrive(TURN_SPEED, -10, 10, 10, -10, 3.0);
                sleep(10);

                // Strafe left
                encoderDrive(DRIVE_SPEED, -3, 3, 3, -3, 1.0);
                sleep(10);

                // Strafe right SLOW
                encoderDrive(TURN_SPEED, 4, -4, -4, 4, 2.0);
                sleep(10);

                // Backward
                encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 4.0);
                sleep(10);

                // Turn right
                encoderDrive(DRIVE_SPEED, -26, -26, 26, 26, 5.0);
                sleep(10);

                // Strafe right SLOW
                encoderDrive(TURN_SPEED, 6, -6, -6, 6, 2.0);
                sleep(10);

                // Strafe left
                encoderDrive(TURN_SPEED, -2, 2, 2, -2, 2.0);
                sleep(10);

                // Forward
                encoderDrive(TURN_SPEED, 12, 12, 12, 12, 3.0);
                sleep(10);

                // Down
                AngleL.setPosition(0.24);
                AngleR.setPosition(0.86);
                sleep(1500);
                sleep(10000);

            }
        }
        if (detector != null) {
            detector.disable(); //Make sure to run this on stop!
        }
    }

    public void encoderDrive(double speed, double leftFInches, double leftRInches, double rightFInches, double rightRInches, double timeoutS) {
        int newLeftFTarget;
        int newLeftRTarget;
        int newRightFTarget;
        int newRightRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFTarget = lf.getCurrentPosition() + (int)(leftFInches * COUNTS_PER_INCH);
            newLeftRTarget = lr.getCurrentPosition() + (int)(leftRInches * COUNTS_PER_INCH);
            newRightFTarget = rf.getCurrentPosition() + (int)(rightFInches * COUNTS_PER_INCH);
            newRightRTarget = rr.getCurrentPosition() + (int)(rightRInches * COUNTS_PER_INCH);

            lf.setTargetPosition(newLeftFTarget);
            lr.setTargetPosition(newLeftRTarget);
            rf.setTargetPosition(newRightFTarget);
            rr.setTargetPosition(newRightRTarget);

            // Turn On RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lf.setPower(Math.abs(speed));
            lr.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            rr.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFTarget, newLeftRTarget, newRightFTarget, newRightRTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        lf.getCurrentPosition(),
//                        lr.getCurrentPosition(),
//                        rf.getCurrentPosition(),
//                        rr.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);

            // Turn off RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderLift(double speed, double liftInch, double timeoutSS) {
        int newLiftInch;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftInch = Lift.getCurrentPosition() + (int)(liftInch * COUNTS_PER_INCH);

            Lift.setTargetPosition(newLiftInch);

            // Turn On RUN_TO_POSITION
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Lift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSS) &&
                    (Lift.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLiftInch);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        Lift.getCurrentPosition(),
//                        telemetry.update());
                telemetry.addData("Hello", "Lift is running");
            }

            // Stop all motion;
            Lift.setPower(0);

            // Turn off RUN_TO_POSITION
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move

        }
    }
}