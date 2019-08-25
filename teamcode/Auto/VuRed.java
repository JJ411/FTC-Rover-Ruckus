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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="VuRedLeft", group ="Concept")
@Disabled
public class VuRed extends LinearOpMode {

    // public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    private ElapsedTime runtime = new ElapsedTime();
            //Motors
            private DcMotor leftMotor = null;
            private DcMotor rightMotor = null;
            private DcMotor liftMotor = null;
            private DcMotor liftMotor2 = null;
            private DcMotor MiddleMotor = null;
            private DcMotor relicMotor = null;

            //1st Slide Servo
            private Servo leftServo2 = null;
            private Servo rightServo2 = null;
            private Servo leftServo1 = null;
            private Servo rightServo1 = null;

            // 2nd Slide Servo
            private Servo LeftServo2 = null;
            private Servo RightServo2 = null;
            private Servo LeftServo1 = null;
            private Servo RightServo1 = null;

            private Servo jewelServo = null;
            private Servo jewelServo2 = null;
            private Servo wristServo1 = null;
            private Servo clawServo = null;
            private ColorSensor colorSensor = null;
            private ColorSensor colorSensor2 = null;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;
    //Andymark neverest motor 20 = 560 counts per motor rev
    //Andymark neverest motor 40 = 1120 counts per motor rev
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override public void runOpMode() {

            //Motor
            leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
            rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
            liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
            liftMotor2 = hardwareMap.get(DcMotor.class, "lift_motor2");
            relicMotor = hardwareMap.get(DcMotor.class, "relic_motor");
            MiddleMotor = hardwareMap.get(DcMotor.class, "MiddleMotor");
            //1st Slide Servo
            leftServo1 = hardwareMap.get(Servo.class, "left_servo1");
            rightServo1 = hardwareMap.get(Servo.class, "right_servo1");
            leftServo2 = hardwareMap.get(Servo.class, "left_servo2");
            rightServo2 = hardwareMap.get(Servo.class, "right_servo2");
            //2nd Slide Servo
            LeftServo1 = hardwareMap.get(Servo.class, "Left_Servo1");
            RightServo1 = hardwareMap.get(Servo.class, "Right_Servo1");
            LeftServo2 = hardwareMap.get(Servo.class, "Left_Servo2");
            RightServo2 = hardwareMap.get(Servo.class, "Right_Servo2");
            //Ect. Servo
            jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
            jewelServo2 = hardwareMap.get(Servo.class, "jewel_servo2");
            wristServo1 = hardwareMap.get(Servo.class, "wrist_servo1");
            clawServo = hardwareMap.get(Servo.class, "claw_servo");
            colorSensor = hardwareMap.colorSensor.get("color_sensor");
            colorSensor2 = hardwareMap.colorSensor.get("color_sensor2");



        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforihttp://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/EncoderRedRight.javaa license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Ad9g5RD/////AAAAGda4OsPbykPeuG0EhWVFG9oU3hgL1Ix8WQbwpCo1KoiYlt3IgiwonNpbu6NKgDVSu4zuqOcOEPAqcT+GI+O3N4O9Nd+XtdiBI6s98/1sRq8Ocz3lfmRqo9vGJPFkoxro0s/JpQt4lg7c2GBMVZX/Ojwy9Nw09Zy7h5pVce2WQTZ8UqEOfPNjRdNFx9d2Pv9mouraYTLQtXvY/72cQok0k7RKoMI/BquzM5RkEXL/FJs3ZXDwsDruE6Th5+U3UIAP1rrvHq11zT7ApdR2RD/uzjIsd/gAYUdGCaJOMX3NpLz0Dr2FFezPy/pUDY88YBMCqWQH4JNikBQVDmANQ+NxH57ZNCISYwVoPOTupyAO8P+k";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */

        jewelServo2.setPosition(0.45); //Down
        sleep(300);

            leftServo2.setPosition(0.45);
            rightServo2.setPosition(0.45);
            sleep(200);

            liftMotor.setPower(0.4);
            sleep(700);

            liftMotor.setPower(0);
            sleep(100);

        jewelServo.setPosition(0); //Middle position
        sleep(1500);

         if (colorSensor.red() > colorSensor.blue()) {
             //if it scans red on red side Needs to turn right

            jewelServo2.setPosition(0.7); //Hit with behind
            sleep(200);
            runtime.reset();

         }

         if (colorSensor.blue() > colorSensor.red()) {
             //if it scans blue on red side Needs to turn left

            jewelServo2.setPosition(0.1); //Hit with sensor
            sleep(200);
            runtime.reset();

         }

        sleep(500);
        jewelServo.setPosition(0.5); //Up
        sleep(1000);
        jewelServo2.setPosition(0.1); //Up with the sensor side
        sleep(200);

            // sleep(2000); //gives the bot time to scan. Ex. the code up there takes 3 seconds to complete.
                         //the pictograph doesn't need to be seen for maximum of 5 seconds
                         //if it scans before 5 seconds, it will continue to codes down there
                         //if it doesn't scan within 5 seconds, it will skip to the code where it doesn't see any of the pictograph

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */

                // telemetry.addData("VuMark", "%s visible", vuMark);
                    // if (vuMark != RelicRecoveryVuMark.RIGHT)
                    // if (vuMark != RelicRecoveryVuMark.LEFT){  //opposite, ex. if vuMark sees Right or Left, it doesn't show the sentence "IT WORKS, YOU DID IT". If it sees CENTER, it will show the sentence on the phone
                    //     telemetry.addData("IT WORKS, YOU DID IT!!", vuMark);

                    // }

                    //CENTER
                    if (vuMark != RelicRecoveryVuMark.RIGHT &&
                    vuMark != RelicRecoveryVuMark.LEFT) {
                    // if (vuMark != RelicRecoveryVuMark.LEFT)
                         telemetry.addData("VuMark", vuMark);
                         telemetry.update();

        encoderDrive(DRIVE_SPEED,  36, -36, 5.0);  //Forward

        encoderDrive(DRIVE_SPEED,  14, 14, 5.0);  //Right
        sleep(100);

            liftMotor.setPower(-0.3);
            sleep(400);
            runtime.reset();

            liftMotor.setPower(0);
            sleep(50);

            leftServo2.setPosition(0.67);
            rightServo2.setPosition(0.27);
            sleep(50);
            runtime.reset();
            sleep(100);

        //Makes sure the glyph is inside the cryptobox
        encoderDrive(0.7,  8, -8, 5.0); //Forward

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);
        sleep(50);
        runtime.reset();

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);
        sleep(50);
        runtime.reset();

                    //This is an attempt to score 2 additional glyphs in autonomous
                    //Not necessary, optional because of remaining of autonomous time (15s)

        encoderDrive(DRIVE_SPEED, -12, 12, 5.0); //Backward

            liftMotor2.setPower(0.4); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(800);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            LeftServo2.setPosition(0.63);
            RightServo2.setPosition(0.53); //2nd Top servo close

            LeftServo1.setPosition(0.45);
            RightServo1.setPosition(0.37); //Bottom servo close
            sleep(800);

        encoderDrive(DRIVE_SPEED, 12,  -12, 5.0); //Forward

            liftMotor2.setPower(0.5); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(600);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -27.1, -27.1, 5.0); //Turns back, faces the cryptobox

        encoderDrive(DRIVE_SPEED, -6.0, 6.0, 5.0);     //Backward
        // Red center
        encoderDrive(DRIVE_SPEED, 3.5, 3.5, 5.0);     //Angle the robot

        encoderDrive(DRIVE_SPEED, -18, 18, 5.0); //Backward into cryptobox

        encoderDrive(DRIVE_SPEED,  2, -2, 1.0); //Forward

        leftMotor.setPower(-0.6);
        rightMotor.setPower(-0.2);
        sleep(500);

                //Opens servo
                LeftServo2.setPosition(0.46);
                RightServo2.setPosition(0.66);

                LeftServo1.setPosition(0.01);
                RightServo1.setPosition(0.9);

        //Makes sures the glyph is inside the cryptobox
        encoderDrive(DRIVE_SPEED,  4, -4, 1.0); //Forward
        encoderDrive(DRIVE_SPEED,  -6, 6, 1.0); //Backward
        encoderDrive(DRIVE_SPEED,  5, -5, 1.0); //Forward


            liftMotor2.setPower(-0.3);
            sleep(800);
                                        //Dont have time to bring it down
            liftMotor2.setPower(0);
            sleep(0);

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);

        sleep(5000);    //Each cryptobox is 9 inches apart*****
                    }

                    //RIGHT
                    if (vuMark != RelicRecoveryVuMark.CENTER &&
                    vuMark != RelicRecoveryVuMark.LEFT) {
                    // if (vuMark != RelicRecoveryVuMark.LEFT){
                         telemetry.addData("VuMark", vuMark);
                         telemetry.update();

        encoderDrive(DRIVE_SPEED,  28, -28, 5.0);  //Forward

        encoderDrive(DRIVE_SPEED,  14, 14, 5.0);  //Right
        sleep(100);

            liftMotor.setPower(-0.3);
            sleep(400);
            runtime.reset();

            liftMotor.setPower(0);
            sleep(50);

            leftServo2.setPosition(0.67);
            rightServo2.setPosition(0.27);
            sleep(50);
            runtime.reset();
            sleep(100);

        //Makes sure the glyph is inside the cryptobox
        encoderDrive(0.7,  8, -8, 5.0); //Forward

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);
        sleep(50);
        runtime.reset();

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);
        sleep(50);
        runtime.reset();

                    //This is an attempt to score 2 additional glyphs in autonomous
                    //Not necessary, optional because of remaining of autonomous time (15s)

        encoderDrive(DRIVE_SPEED, -12, 12, 5.0); //Backward

            liftMotor2.setPower(0.4); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(800);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            LeftServo2.setPosition(0.63);
            RightServo2.setPosition(0.53); //2nd Top servo close

            LeftServo1.setPosition(0.45);
            RightServo1.setPosition(0.37); //Bottom servo close
            sleep(800);

        encoderDrive(DRIVE_SPEED, 12,  -12, 5.0); //Forward

            liftMotor2.setPower(0.5); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(600);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -27.1, -27.1, 5.0); //Turns back, faces the cryptobox

        encoderDrive(DRIVE_SPEED, -6.0, 6.0, 5.0);     //Backward
        // Red right
        encoderDrive(DRIVE_SPEED, -3.5, -3.5, 5.0);     //Angle the robot

        encoderDrive(DRIVE_SPEED, -18, 18, 5.0); //Backward into cryptobox

        encoderDrive(DRIVE_SPEED,  2, -2, 1.0); //Forward

        leftMotor.setPower(-0.6);
        rightMotor.setPower(-0.2);
        sleep(500);

                //Opens servo
                LeftServo2.setPosition(0.46);
                RightServo2.setPosition(0.66);

                LeftServo1.setPosition(0.01);
                RightServo1.setPosition(0.9);

        //Makes sures the glyph is inside the cryptobox
        encoderDrive(DRIVE_SPEED,  4, -4, 1.0); //Forward
        encoderDrive(DRIVE_SPEED,  -6, 6, 1.0); //Backward
        encoderDrive(DRIVE_SPEED,  5, -5, 1.0); //Forward


            liftMotor2.setPower(-0.3);
            sleep(800);
                                        //Dont have time to bring it down
            liftMotor2.setPower(0);
            sleep(0);

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);

        sleep(5000);
                    }
                    //LEFT
                    if (vuMark != RelicRecoveryVuMark.RIGHT &&
                    vuMark != RelicRecoveryVuMark.CENTER) {
                    // if (vuMark != RelicRecoveryVuMark.CENTER){
                        telemetry.addData("VuMark", vuMark);
                        telemetry.update();

        encoderDrive(DRIVE_SPEED,  44, -44, 5.0);  //Forward

        encoderDrive(DRIVE_SPEED,  14, 14, 5.0);  //Right
        sleep(100);

            liftMotor.setPower(-0.3);
            sleep(400);
            runtime.reset();

            liftMotor.setPower(0);
            sleep(50);

            leftServo2.setPosition(0.67);
            rightServo2.setPosition(0.27);
            sleep(50);
            runtime.reset();
            sleep(100);

        //Makes sure the glyph is inside the cryptobox
        encoderDrive(0.7,  8, -8, 5.0); //Forward

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);
        sleep(50);
        runtime.reset();

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);
        sleep(50);
        runtime.reset();

                    //This is an attempt to score 2 additional glyphs in autonomous
                    //Not necessary, optional because of remaining of autonomous time (15s)

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            liftMotor2.setPower(0.4); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(800);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            LeftServo2.setPosition(0.63);
            RightServo2.setPosition(0.53); //2nd Top servo close

            LeftServo1.setPosition(0.45);
            RightServo1.setPosition(0.37); //Bottom servo close
            sleep(800);

        encoderDrive(DRIVE_SPEED, 12, -12, 5.0); //Forward

            liftMotor2.setPower(0.5); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(600);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -27.1, -27.1, 5.0); //Turns back, faces the cryptobox

        encoderDrive(DRIVE_SPEED, -6.0, 6.0, 5.0);     //Backward
        //Red left
        encoderDrive(DRIVE_SPEED,  3.5, 3.5, 5.0);  //Angle the robot

        encoderDrive(DRIVE_SPEED, -18, 18, 5.0); //Backward into cryptobox

        encoderDrive(DRIVE_SPEED,  2, -2, 1.0); //Forward

        leftMotor.setPower(-0.2);
        rightMotor.setPower(-0.6);
        sleep(500);

                //Opens servo
                LeftServo2.setPosition(0.46);
                RightServo2.setPosition(0.66);

                LeftServo1.setPosition(0.01);
                RightServo1.setPosition(0.9);

        //Makes sures the glyph is inside the cryptobox
        encoderDrive(DRIVE_SPEED,  4, -4, 1.0); //Forward
        encoderDrive(DRIVE_SPEED,  -6, 6, 1.0); //Backward
        encoderDrive(DRIVE_SPEED,  5, -5, 1.0); //Forward


            liftMotor2.setPower(-0.3);
            sleep(800);
                                        //Dont have time to bring it down
            liftMotor2.setPower(0);
            sleep(0);

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);

        sleep(5000);
                    }

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                    if (vuMark != RelicRecoveryVuMark.RIGHT &&
                    vuMark != RelicRecoveryVuMark.CENTER &&
                    vuMark != RelicRecoveryVuMark.LEFT) {
                    // if (vuMark != RelicRecoveryVuMark.CENTER)
                    // if (vuMark != RelicRecoveryVuMark.LEFT)

                    telemetry.addData("VuMark", vuMark);
                    telemetry.update();

                    // sleep(6000);

        encoderDrive(DRIVE_SPEED,  36, -36, 5.0);  //Forward

        encoderDrive(DRIVE_SPEED,  14, 14, 5.0);  //Right
        sleep(100);

            liftMotor.setPower(-0.3);
            sleep(400);
            runtime.reset();

            liftMotor.setPower(0);
            sleep(50);

            leftServo2.setPosition(0.67);
            rightServo2.setPosition(0.27);
            sleep(50);
            runtime.reset();
            sleep(100);

        //Makes sure the glyph is inside the cryptobox
        encoderDrive(0.7,  8, -8, 5.0); //Forward

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);
        sleep(50);
        runtime.reset();

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);
        sleep(50);
        runtime.reset();

                    //This is an attempt to score 2 additional glyphs in autonomous
                    //Not necessary, optional because of remaining of autonomous time (15s)

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            liftMotor2.setPower(0.4); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(800);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -17, 17, 5.0); //Backward

            LeftServo2.setPosition(0.63);
            RightServo2.setPosition(0.53); //2nd Top servo close

            LeftServo1.setPosition(0.45);
            RightServo1.setPosition(0.37); //Bottom servo close
            sleep(800);

        encoderDrive(DRIVE_SPEED, 12,  -12, 5.0); //Forward

            liftMotor2.setPower(0.5); //Lifts liftMotor up leveled to 2nd level of glyph in glyph pit
            sleep(600);

            liftMotor2.setPower(0); //Stops liftMotor
            sleep(50);

        encoderDrive(DRIVE_SPEED, -27.1, -27.1, 5.0); //Turns back, faces the cryptobox

        encoderDrive(DRIVE_SPEED, -6.0, 6.0, 5.0);     //Backward
        //Red left
        encoderDrive(DRIVE_SPEED,  3.5, 3.5, 5.0);  //Angle the robot

        encoderDrive(DRIVE_SPEED, -18, 18, 5.0); //Backward into cryptobox

        encoderDrive(DRIVE_SPEED,  2, -2, 1.0); //Forward

        leftMotor.setPower(-0.2);
        rightMotor.setPower(-0.6);
        sleep(500);

                //Opens servo
                LeftServo2.setPosition(0.46);
                RightServo2.setPosition(0.66);

                LeftServo1.setPosition(0.01);
                RightServo1.setPosition(0.9);

        //Makes sures the glyph is inside the cryptobox
        encoderDrive(DRIVE_SPEED,  4, -4, 1.0); //Forward
        encoderDrive(DRIVE_SPEED,  -6, 6, 1.0); //Backward
        encoderDrive(DRIVE_SPEED,  5, -5, 1.0); //Forward


            liftMotor2.setPower(-0.3);
            sleep(800);
                                        //Dont have time to bring it down
            liftMotor2.setPower(0);
            sleep(0);

        LeftServo2.setPosition(0.15);
        RightServo2.setPosition(0.99);

        LeftServo1.setPosition(0.01);
        RightServo1.setPosition(0.9);

        sleep(5000);
                    }

            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                // telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                // telemetry.addData("Path2",  "Running at %7d :%7d",
                //         leftMotor.getCurrentPosition(),
                //         rightMotor.getCurrentPosition());
                // telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
