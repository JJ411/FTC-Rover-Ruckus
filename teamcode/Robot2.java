package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Robot2 {

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


                      // Forward
                      encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 3.0);

                      // Backward
                      encoderDrive(DRIVE_SPEED, -8, -8, -8, -8, 3.0);

                      // Turn right
                      encoderDrive(TURN_SPEED, -15, -15, 15, 15, 3.0);

                      // Turn left
                      encoderDrive(TURN_SPEED, 15, 15, -15, -15, 3.0);

                      // Strafe left
                      encoderDrive(DRIVE_SPEED, -5, 5, 5, -5, 3.0);

                      // Strafe right
                      encoderDrive(DRIVE_SPEED, 5, -5, -5, 5, 3.0);
    }
}
