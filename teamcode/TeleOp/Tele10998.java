package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tele10998", group="Linear Opmode")
@Disabled
public class Tele10998 extends LinearOpMode{

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
    private Servo Marker = null;
    private Servo AngleR = null;
    private Servo AngleL = null;


    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Flip = hardwareMap.get(DcMotor.class, "Flip");

        inTake = hardwareMap.get(Servo.class, "inTake");
        Marker = hardwareMap.get(Servo.class, "Marker");
        AngleR = hardwareMap.get(Servo.class, "AngleR");
        AngleL = hardwareMap.get(Servo.class, "AngleL");

        Lift.setDirection(DcMotor.Direction.FORWARD);
        Slide.setDirection(DcMotor.Direction.REVERSE);

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        Flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

    while (opModeIsActive()) {


        double SlidePower = gamepad2.right_stick_y;

        double slidePower;
        boolean LBumper = gamepad2.left_bumper;
        boolean RBumper = gamepad2.right_bumper;

        double LJ = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rightX = -gamepad1.right_stick_x;
        double angle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI/4;
        final double vOne = LJ * Math.cos(angle)+rightX;
        final double vTwo = LJ * Math.sin(angle)-rightX;
        final double vThree = LJ * Math.sin(angle)+rightX;
        final double vFour = LJ * Math.cos(angle)-rightX;


        boolean AngleUp = false;
        boolean AngleDown = false;
        boolean AngleMid = false;

        lf.setPower(vOne);
        rf.setPower(vTwo);
        lr.setPower(vThree);
        rr.setPower(vFour);

        telemetry.addData("Power to Front Left", vOne);
        telemetry.addData("Power to Front Right", vTwo);
        telemetry.addData("Power to Back Left", vThree);
        telemetry.addData("Power to Back Right", vFour);
        telemetry.update();

        // Lift
        if (RBumper) {
         Lift.setPower(1);
    }
        else {
         Lift.setPower(0);
    }

        if (LBumper) {
         Lift.setPower(-1);
    }
        else {
         Lift.setPower(0);
    }

        // inTake
        if (gamepad2.dpad_left){
         inTake.setPosition(0);
    }
        else if (gamepad2.dpad_right){
         inTake.setPosition(1);
    }
        else {
         inTake.setPosition(0.5);
    }


    // Angle
    if (gamepad2.dpad_down) {
        // Flip Down
        Flip.setTargetPosition(0);
        Flip.setPower(0.1);
        Flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Flip.setPower(0.1);
        // sleep(200);



        // sleep(100);

    }

    if (gamepad2.dpad_up) {
        // Flip Up
        Flip.setTargetPosition(300);
        Flip.setPower(-0.2);
        Flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // sleep(100);

    }

    // if (gamepad2.x) {
    //     AngleUp = true;
    //     telemetry.addData("AngleUp","true" );
    // }

    // if (gamepad2.x && runtime.seconds() < 5.0) {
    //     AngleUp = false;
    //     telemetry.addData("AngleUp", "false");
    // }

    // if (gamepad2.y) {
    //     AngleUp = true;
    //     AngleMid = true;
    // }

    // if (AngleUp == true && AngleMid == true) {
    //     AngleL.setPosition(0.3);
    //     AngleR.setPosition(0.7);
    //     sleep(2000);

    //     // Down
    //     AngleL.setPosition(1.0);
    //     AngleR.setPosition(0.15);
    // }

        // Bucket Up
      if (gamepad2.y) {
        AngleL.setPosition(0.15);
        AngleR.setPosition(1.0);
      }

      // Bucket Down
      if (gamepad2.a) {
        AngleL.setPosition(1.0);
        AngleR.setPosition(0.15);
      }

      // Bucket Half
      if (gamepad2.x) {
        AngleL.setPosition(0.3);
        AngleR.setPosition(0.7);
      }

    Slide.setPower(SlidePower);

        }
    }
}

