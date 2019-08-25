 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
 @Autonomous(name="Time", group="Linear Opmode")

 public class Timed extends LinearOpMode{

     private ElapsedTime runtime = new ElapsedTime();

     private DcMotor lf = null;
     private DcMotor lr = null;
     private DcMotor rf = null;
     private DcMotor rr = null;

     private Servo inTake = null;
     private Servo Marker = null;

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
         Marker = hardwareMap.get(Servo.class, "Marker");

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
         lf.setPower(0.5);
         lr.setPower(0.5);
         rf.setPower(0.5);
         rr.setPower(0.5);
         sleep(5000);


 }

 }
 }
