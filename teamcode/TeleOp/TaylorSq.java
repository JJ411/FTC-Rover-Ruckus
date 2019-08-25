package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="For Taylor^2", group="Taylors")

public class TaylorSq extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private Controller gamepad;

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

        inTake.setDirection(DcMotor.Direction.FORWARD);
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

            double SlidePower = -gamepad2.left_stick_y;

            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            // Drive reverse
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

            // Lift
            if (gamepad2.dpad_down || LBumper1) {
                Lift.setPower(-1); }
            else {
                Lift.setPower(0); }

            if (gamepad2.dpad_up || RBumper1) {
                Lift.setPower(1); }
            else {
                Lift.setPower(0); }

            // inTake
            if (gamepad2.a) {
                inTake.setPower(-1);
            }
            else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                inTake.setPower(1);
            }
            else if (gamepad2.left_trigger > 0.1) {
                inTake.setPower(0.5);
            }
            else if (gamepad2.right_trigger > 0.1) {
                inTake.setPower(-0.5);
            }
            else {
                inTake.setPower(0);
            }

            if (LBumper2) {
                // Flip Down
                Flip.setTargetPosition(0);
                Flip.setPower(0.1);
                Flip.setMode(DcMotor.RunMode.RUN_TO_POSITION); }

            if (RBumper2) {
                // Flip Up
                Flip.setTargetPosition(-223);
                Flip.setPower(-0.3);
                Flip.setMode(DcMotor.RunMode.RUN_TO_POSITION); }

            // Bucket Down
            if (gamepad2.x) {
                AngleL.setPosition(0.18);
                AngleR.setPosition(0.92); }

            // Bucket Up
            if (gamepad2.b) {
                AngleL.setPosition(1.0);
                AngleR.setPosition(0.15); }

            // Bucket Half
            if (gamepad2.y) {
                AngleL.setPosition(0.5);
                AngleR.setPosition(0.5); }

            Slide.setPower((-SlidePower)/4);
        }
    }
}

