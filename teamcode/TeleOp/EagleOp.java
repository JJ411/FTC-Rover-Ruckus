package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="EagleOp✓", group="Linear Opmode")
@Disabled
public class EagleOp extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf = null;
    private DcMotor lr = null;
    private DcMotor rf = null;
    private DcMotor rr = null;

    private Servo inTake = null;

    private DcMotor Lift = null;
    private DcMotor Angle = null;
    private DcMotor Slide = null;

    // private    static final double CONTINUOUS_SERVO_FORWARD = 1.0;
    // private    static final double CONTINUOUS_SERVO_REVERSE = 0.0;
    // private    static final double CONTINUOUS_SERVO_STOP = 0.5;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        inTake = hardwareMap.get(Servo.class, "inTake");
        // inTake.setPower(CONTINUOUS_SERVO_STOP);

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Angle = hardwareMap.get(DcMotor.class, "Angle");
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Angle.setDirection(DcMotor.Direction.FORWARD);
        Slide.setDirection(DcMotor.Direction.REVERSE);

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();

    while (opModeIsActive()) {

      /*
        //Drive
        double driveLPower = gamepad1.left_stick_y;
        double driveRPower = -gamepad1.right_stick_y;
         *///player2
        double SlidePower = gamepad2.left_stick_y; // LJ
        double AnglePower = -gamepad2.right_stick_y;
        //Angle
        // double angleLPower = -gamepad1.right_stick_y;
        // double angleRPower = -gamepad1.right_stick_y;

        //Strafe
        //boolean leftBumper = gamepad1.left_bumper;
        //boolean rightBumper = gamepad1.right_bumper;

        //New Drive Program
        double LJ = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rightX = -gamepad1.right_stick_x;
        double angle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/4;
        final double vOne = LJ * Math.cos(angle)+rightX;
        final double vTwo = LJ * Math.sin(angle)-rightX;
        final double vThree = LJ * Math.sin(angle)+rightX;
        final double vFour = LJ * Math.cos(angle)-rightX;
        lf.setPower(vOne);
        rf.setPower(vTwo);
        lr.setPower(vThree);
        rr.setPower(vFour);

        telemetry.addData("Power to Front Left", vOne);
        telemetry.addData("Power to Front Right", vTwo);
        telemetry.addData("Power to Back Left", vThree);
        telemetry.addData("Power to Back Right", vFour);
        telemetry.update();

        // 4 December 2018
        // lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        /*OLD DRIVE PROGRAM
        if(Math.abs(gamepad1.left_stick_y)>.2||Math.abs(gamepad1.left_stick_x)>.2)
        {//Drive
            rf.setPower(gamepad1.left_stick_y+gamepad1.left_stick_x);
            lf.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);
            rr.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x);
            lr.setPower(gamepad1.left_stick_y+gamepad1.left_stick_x);
        }
        else
        {
            rf.setPower(0);
            lf.setPower(0);
            rr.setPower(0);
            lr.setPower(0);
        }
        if(Math.abs(gamepad1.right_stick_x)>.2)
        {
            rf.setPower(gamepad1.right_stick_x);
            lf.setPower(-gamepad1.right_stick_x);
            rr.setPower(gamepad1.right_stick_x);
            lr.setPower(-gamepad1.right_stick_x);
        }
        else
        {
            rf.setPower(0);
            lf.setPower(0);
            rr.setPower(0);
            lr.setPower(0);
        }
        OLD DRIVE PROGRAM*/
        //Player 2
        //double liftPower;
        double slidePower;
        double anglePower;
        boolean LBumper = gamepad2.left_bumper;
        boolean RBumper = gamepad2.right_bumper;
      // boolean BackB = gamepad2.back;
      // boolean StartB = gamepad2.start;
        // boolean LeftJoyB = gamepad2.left_stick_button;
        // boolean RightJoyB = gamepad2.right_stick_button;

        // liftPower = gamepad2.right_stick_y;
        // slidePower = gamepad2.left_stick_y;

        /*
        //Strafe left
        if(leftBumper) {
            lf.setPower(-1);
            lr.setPower(1);
            rf.setPower(-1);
            rr.setPower(1);
        }

        //Strafe right
        else if(rightBumper) {
            lf.setPower(1);
            lr.setPower(-1);
            rf.setPower(1);
            rr.setPower(-1);
        }

        else {
            //Drive
            lf.setPower(-driveLPower);
            lr.setPower(-driveLPower);
            rf.setPower(-driveRPower);
            rr.setPower(-driveRPower);

            //Angle
            // lf.setPower(angleLPower);
            // lr.setPower(-angleLPower);
            // rf.setPower(-angleRPower);
            // rr.setPower(angleRPower);
      }
      */
      //Bucket 1 is down
    //   if(gamepad2.a) {
    //       Bucket.setPosition(.94);
    //   }
    //   else if(gamepad2.x){
    //       Bucket.setPosition(.75);
    //   }
    //   else if(gamepad2.y) {
    //       Bucket.setPosition(0);
    //   }

      //Door
    //      if(gamepad2.dpad_up)
    //   {
    //         Door.setPosition(1);
    //   }
    //     else if(gamepad2.dpad_down)
    //     {
    //         Door.setPosition(0);
    //     }

        // inTake
        if(gamepad2.dpad_left){
            inTake.setPosition(1);
        }

        else if(gamepad2.dpad_right){
            inTake.setPosition(0);
        }

        else {
            inTake.setPosition(0.5);
        }

      /*
      //Angle Up
      if(gamepad2.y) {
          Angle.setPower(1);
      }
      else{
            Angle.setPower(0);
      }

      //Angle Down
      if(gamepad2.a) {
          Angle.setPower(-1);
      }
      else{
          Angle.setPower(0);
      }
      */
      Angle.setPower((AnglePower)/2);
      /*
      if(LBumper) {
          Slide.setPower(-1);
      }

      if(RBumper) {
          Slide.setPower(1);
      }

      if(gamepad2.dpad_left) {
          Slide.setPower(0);
      }
      */

      Slide.setPower(SlidePower);


      //Lift
      if(RBumper) {
          Lift.setPower(1);
      }
      else
      {
          Lift.setPower(0);
      }

      if(LBumper) {
          Lift.setPower(-1);
      }
      else
      {
          Lift.setPower(0);
      }




    }
  }
}

