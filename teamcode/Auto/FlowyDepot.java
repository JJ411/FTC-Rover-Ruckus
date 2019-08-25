/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
 
/*  This is program for DEPOT SIDE
 *  No TEAM MARKER
 *  LAND, SAMPLE, PARK
 */

@Autonomous(name = "FlowyDepot", group = "Concept")

public class FlowyDepot extends LinearOpMode {
  
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
    
    // Servo
    private Servo inTake = null;
    private Servo AngleR = null;
    private Servo AngleL = null;
    
    // DigitalChannel digitalTouch;
    
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURN_SPEED              = 0.2;
    
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Ad9g5RD/////AAAAGda4OsPbykPeuG0EhWVFG9oU3hgL1Ix8WQbwpCo1KoiYlt3IgiwonNpbu6NKgDVSu4zuqOcOEPAqcT+GI+O3N4O9Nd+XtdiBI6s98/1sRq8Ocz3lfmRqo9vGJPFkoxro0s/JpQt4lg7c2GBMVZX/Ojwy9Nw09Zy7h5pVce2WQTZ8UqEOfPNjRdNFx9d2Pv9mouraYTLQtXvY/72cQok0k7RKoMI/BquzM5RkEXL/FJs3ZXDwsDruE6Th5+U3UIAP1rrvHq11zT7ApdR2RD/uzjIsd/gAYUdGCaJOMX3NpLz0Dr2FFezPy/pUDY88YBMCqWQH4JNikBQVDmANQ+NxH57ZNCISYwVoPOTupyAO8P+k";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Flip = hardwareMap.get(DcMotor.class, "Flip");
        
        inTake = hardwareMap.get(Servo.class, "inTake");
        AngleR = hardwareMap.get(Servo.class, "AngleR");
        AngleL = hardwareMap.get(Servo.class, "AngleL");
        
        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          lf.getCurrentPosition(),
                          lr.getCurrentPosition(),
                          rf.getCurrentPosition(),
                          rr.getCurrentPosition());
                          
        telemetry.update();


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            
          // Having this in the front delays the TF
          Lift.setPower(-1);
          sleep(15000);
        
          // Lift stop
          Lift.setPower(0);
          sleep(50);
          
          encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);
          
            //              Next up is if its not visible, if it doesn't see, go down to this line... 1/13
          runtime.reset();
          
            while (opModeIsActive() && runtime.seconds() < 3.0 ) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int goldPosition = 0;
                        
                        for (Recognition recognition : updatedRecognitions) {
                            
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } 
                          else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } 
                        
                        } 
                        // || silverMineral2X != -1
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                          
                          // Lowest point
                          if (goldMineralX > silverMineral1X) {
                            goldPosition = 1;
                            telemetry.addData("Gold Mineral Position", "Right");
                            // Center
                          }
                          
                          // Highest point 
                          if (goldMineralX < silverMineral1X) {
                            goldPosition = 2;
                            telemetry.addData("Gold Mineral Position", "Center");
                            // Right
                          }
                          
                        }
                        
                        else {
                            goldPosition = 3;
                            telemetry.addData("Gold Mineral Position", "Left");
                            // Left
                          }
                        
                    // Wait for detection
                    // sleep(5000);
                    // runtime. Within 5 seconds or something then move onto this code
        
        if (goldPosition == 1) {
        telemetry.addData("Gold Mineral Position", "Right yay");
        
        // Forward +
        encoderDrive(DRIVE_SPEED, 1, 1, 1, 1, 1.0);
        
        // Turn left (45) +
        encoderDrive(DRIVE_SPEED, 3, 3, -3, -3, 2.0);
        
        // Forward +
        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 2.0);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 2.0);
        
//        Marker.setPosition(0.8);
//        sleep(2000);
//
//        Marker.setPosition(0.1);
//        sleep(1000);
        
        // Turn left
        encoderDrive(DRIVE_SPEED, 2, 2, -2, -2, 1.0);
        
        // Backward
        encoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 2.0);
        
        // Turn right
        encoderDrive(DRIVE_SPEED, -2, -2, 2, 2, 2.0);
        
        // Backward
        encoderDrive(DRIVE_SPEED, -6, -6, -6, -6, 2.0);

        // // Turn left
        // encoderDrive(DRIVE_SPEED, 11, 11, -11, -11, 2.0);
        
        // Strafe left
        encoderDrive(DRIVE_SPEED, -5, 5, 5, -5, 2.0);
        
        // Strafe right 
        encoderDrive(TURN_SPEED, 1, -1, -1, 1, 2.0);
        
        // Forward
        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2.0);
                    }

        else if (goldPosition == 2) {
          telemetry.addData("Gold Mineral Position", "Center yay");
        
        
        // CENTER /////////////////////////////////////////
        
        // Forward +
        encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 3.0);
                      
        // Turn right +
        encoderDrive(DRIVE_SPEED, 4, 4, -4, -4, 3.0);
        
//        Marker.setPosition(0.8);
//        sleep(2000);
//
//        Marker.setPosition(0.1);
//        sleep(1000);
        
        // Turn right +
        encoderDrive(DRIVE_SPEED, 10, 10, -10, -10, 3.0);
        
        // Forward
        encoderDrive(DRIVE_SPEED, 3.5, 3.5, 3.5, 3.5, 2.0);
        
        // Strafe right +
        encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 2.0);
        sleep(200);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -1, 1, 1, -1, 1.0);
        
        // Forward to park on crater +
        encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 3.0);
        
        // Strafe right +
        encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 1.0);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -1, 1, 1, -1, 1.0);
        
        // Forward to park on crater +
        encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                    }
                    
        else if (goldPosition == 3) {
          telemetry.addData("Gold Mineral Position", "Left yay");
          
        // Forward +
        encoderDrive(DRIVE_SPEED, 1, 1, 1, 1, 1.0);
        
        // Turn right (45) +
        encoderDrive(DRIVE_SPEED, -2.5, -2.5, 2.5, 2.5, 1.0);
        
        // Forward +
        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 2.0);
        
        // Turn left
        encoderDrive(DRIVE_SPEED, 5, 5, -5, -5, 2.0);
        
        // Forward 
        encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 2.0);
        
        // // Strafe right +
        // encoderDrive(TURN_SPEED, 3, -3, -3, 3, 1.0);
        
//        Marker.setPosition(0.8);
//        sleep(2000);
//
//        Marker.setPosition(0.1);
//        sleep(1000);
        
        
        // // Strafe left +
        // encoderDrive(DRIVE_SPEED, -3, 3, 3, -3, 2.0);
        
        // Backward
        encoderDrive(DRIVE_SPEED, -13, -13, -13, -13, 3.0);
        
        // // Turn right
        // encoderDrive(DRIVE_SPEED, -10, -10, 10, 10, 3.0);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -6, 6, 6, -6, 2.0);
        sleep(100);
        
        // Strafe right +
        encoderDrive(DRIVE_SPEED, 1, -1, -1, 1, 1.0);
        
        // Forward to park on crater +
        encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 3.0);
      }
                  
                    }
                      }
                      
                      telemetry.update();
                    }
            }
            // If nothing is visible, will run this code instead as a safety net
            
            telemetry.addData("Gold Mineral Position", "Center Nope");
            
            
            // CENTER /////////////////////////////////////////
        
        // Forward +
        encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 3.0);
                      
        // Turn right +
        encoderDrive(DRIVE_SPEED, 4, 4, -4, -4, 3.0);
        
//        Marker.setPosition(0.8);
//        sleep(2000);
//
//        Marker.setPosition(0.1);
//        sleep(1000);
        
        // Turn right +
        encoderDrive(DRIVE_SPEED, 10, 10, -10, -10, 3.0);
        
        // Forward
        encoderDrive(DRIVE_SPEED, 3.5, 3.5, 3.5, 3.5, 2.0);
        
        // Strafe right +
        encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 2.0);
        sleep(200);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -1, 1, 1, -1, 1.0);
        
        // Forward to park on crater +
        encoderDrive(DRIVE_SPEED, 11, 11, 11, 11, 3.0);
        
        // Strafe right +
        encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 1.0);
        
        // Strafe left +
        encoderDrive(DRIVE_SPEED, -1, 1, 1, -1, 1.0);
        
        // Forward to park on crater +
        encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
        
        // Bucket Down
        AngleL.setPosition(1.0);
        AngleR.setPosition(0.15);
        sleep(1000);
        
        sleep(10000);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    
    public void encoderDrive(double speed,
                             double leftFInches, double leftRInches, double rightFInches, double rightRInches,
                             double timeoutS) {
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
    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFTarget, newLeftRTarget, newRightFTarget, newRightRTarget);
    telemetry.addData("Path2",  "Running at %7d :%7d",
                                            lf.getCurrentPosition(),
                                            lr.getCurrentPosition(),
                                            rf.getCurrentPosition(),
                                            rr.getCurrentPosition());
                telemetry.update();
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
}
