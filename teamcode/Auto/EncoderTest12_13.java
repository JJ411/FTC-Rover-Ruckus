 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
 @Autonomous(name= "EncoderTest4Level", group = "Pushbot")
 public class EncoderTest12_13  extends LinearOpMode {

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

     static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
     static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
     static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                       (WHEEL_DIAMETER_INCHES * 3.1415);
     static final double     DRIVE_SPEED             = 0.5;
     static final double     TURN_SPEED              = 0.3;

     @Override
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

         rf.setDirection(DcMotor.Direction.REVERSE);
         rr.setDirection(DcMotor.Direction.REVERSE);

         // Send telemetry message to signify robot waiting;
         telemetry.addData("Status", "Resetting Encoders");
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

         waitForStart();

         //  **Check before testing/continuing**

         // No decimal returns interger. Decimal returns double //

          // 1) Left
          encoderDrive(TURN_SPEED, 5.0, 5.0, 5.0, 5.0, 5.0);
          // 2) Right
          encoderDrive(DRIVE_SPEED, -5, -5, 5, 5, 5.0);
          // 3) Forward
          encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 5.0);
          // 4) Backward
          encoderDrive(TURN_SPEED, -5, -5, -5, -5, 5.0);
          // 5) Strafe right, away from the hook
          encoderDrive(TURN_SPEED, 5, -5, -5, 5, 5.0);
          // 6) Strafe left, no wall touchy
          encoderDrive(TURN_SPEED, -5, 5, 5, -5, 5.0);


         telemetry.addData("Path", "Complete");
         telemetry.update();
     }

     /*
      *  Method to perfmorm a relative move, based on encoder counts.
      *  Encoders are not reset as the move is based on the current position.
      *  Move will stop if any of three conditions occur:
      *  1) Move gets to the desired position
      *  2) Move runs out of time
      *  3) Driver stops the opmode running.
      */
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
                                             // lf.getCurrentPosition(),
                                             // lr.getCurrentPosition(),
                                             rf.getCurrentPosition());

                                             // Test for right front level shifter,,, if it's working properly.
                                             // Light turns on WITHOUT the encoder wires,,, it turns off if the encoder wire is plugged in.

                                             // rr.getCurrentPosition());
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
