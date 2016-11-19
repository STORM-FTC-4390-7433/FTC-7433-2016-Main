package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;


import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.security.Timestamp;

/**
 * Created by grego on 10/18/2016.
 */
@Autonomous(name="visionBlue", group="Vision")
public class VuforiaOPBlue extends LinearOpMode {
    private DcMotor leftMotor = null, rightMotor = null;
    private boolean hardCode = true;
    private Servo beaconServo = null;
    private ColorSensor colorSensor;
    private DeviceInterfaceModule CDI;
    private float hsvValues[] = {0, 0, 0};
    private int distanceAdjust = -1000;
    private boolean adjust = false;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooterLeft = null;
    private DcMotor shooterRight = null;
    private DcMotor sweeper = null;
    private DcMotor conveyor = null;

    static final double     COUNTS_PER_MOTOR_REV    = 1368 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATL2vJ3/////AAAAGZZx51v2h0D2kh6vX9dkEVwwXavfMtPW74LnE7NWXWw2NChN8Td99tPKhECwV61l/fTsgxV43ktU6XBUlR9lZn1Z3BEd7nQPD+s4uscCWDSjTpXDdQZZWVD7Cfmp+ZK8ax49W55s1vC6mX3vED8miPeegc8DR1bT2BtjxLa0cD77nbeVN5ztUzZEGKPTZEhxGoxjqQsKOEUktyLo6NZIRTA5uEhOmVuwVWC1Iq49tfbjKnLe7t1qfzQlB6wri9DPUrtt3YeuyrNERLclghW7fz7GrfWooMfQIaNEbu/E7BhY95CDGy/Srl1ZpvingaBdcfpB7MAQ/+bFw93saY/lYwT7MXu9ctO9zv1rmuFEAJFp";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        beaconServo = hardwareMap.servo.get("beaconServo");
        colorSensor = hardwareMap.colorSensor.get("color");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        colorSensor.enableLed(false);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        beacons.activate();
        int noSee = 0;
        int see = 0;
        long lastTime = 0;
        boolean run = false;
        int state = 0;



        if(hardCode){
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 25, 25, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED, -30, 30, 1.5);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, 12, 12, 1.0);  // S3: Reverse 24 Inches with 4 Sec timeout

            hardCode = false;
        }

        while (opModeIsActive()) {
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            telemetry.addData("2 Clear", colorSensor.alpha());
            telemetry.addData("3 Red  ", colorSensor.red());
            telemetry.addData("4 Green", colorSensor.green());
            telemetry.addData("5 Blue ", colorSensor.blue());
            telemetry.addData("6 Hue  ", hsvValues[0]);



            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {

                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = (180 - Math.toDegrees(Math.atan2(translation.get(0), translation.get(2))));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);

                    telemetry.update();

                    if (state == -1) {
                        telemetry.addData("Status", "Resetting Encoders");    //
                        telemetry.update();
                        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        idle();
                        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        telemetry.addData("Path0", "Starting at %7d :%7d",
                                leftMotor.getCurrentPosition(),
                                rightMotor.getCurrentPosition());
                        telemetry.update();
                        encoderDrive(DRIVE_SPEED, 25, 25, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                        encoderDrive(TURN_SPEED, -14.8, 14.8, 1.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                        encoderDrive(DRIVE_SPEED, 12, 12, 1.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                        state = 0;
                    }

                    if (state == 0) {
                        if (degreesToTurn > 2 && degreesToTurn < 180) {

                            leftMotor.setPower(-.15);
                            rightMotor.setPower(.15);
                            adjust = false;


                        } else if (degreesToTurn < 358 && degreesToTurn > 180) {

                            leftMotor.setPower(.15);
                            rightMotor.setPower(-.15);
                            adjust = false;

                        } else if ((degreesToTurn >= 358 || degreesToTurn <= 2)) {

                            leftMotor.setPower(0);
                            rightMotor.setPower(0);
                            adjust = true;
                            state = 1;
                        }
                    }
                    if (state == 1) {

                        if (translation.get(2) < distanceAdjust  && translation.get(2) < -200) {
                            rightMotor.setPower(.75);
                            leftMotor.setPower(.75);
                        }
                        else if (translation.get(2) > -200 && adjust) {
                            state = 2;
                        }
                        else{
                            rightMotor.setPower(0);
                            leftMotor.setPower(0);
                            distanceAdjust /= 2;
                            state = 0;
                            adjust = false;
                        }
                    }

                    if (state == 2){
                        rightMotor.setPower(.5);
                        leftMotor.setPower(.5);
                        Thread.sleep(850);
                        rightMotor.setPower(0);
                        leftMotor.setPower(0);
                        state = 3;
                    }

                    if (state == 3) {
                        rightMotor.setPower(-.5);
                        leftMotor.setPower(-.5);
                        Thread.sleep(300);
                        rightMotor.setPower(0);
                        leftMotor.setPower(0);
                        if (colorSensor.red() > colorSensor.blue()){
                            beaconServo.setPosition(.75);
                        }
                        else if (colorSensor.blue() > colorSensor.red()) {
                            beaconServo.setPosition(.15);
                        }
                        state = 4;
                    }

                    if (state == 4) {
                        leftMotor.setPower(1);
                        rightMotor.setPower(1);
                        Thread.sleep(1200);
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = 5;
                    }

                    if (state == 5) {
                        Thread.sleep(1500);
                        state = 6;
                    }

                    if (state == 6){
                        leftMotor.setPower(-5);
                        rightMotor.setPower(-5);
                        Thread.sleep(500);
                        leftMotor.setPower(0);
                        rightMotor.setPower(0);
                        state = 7;
                    }

                    if (state == 7){
                        shooterLeft.setPower(1);
                        shooterRight.setPower(1);
                        Thread.sleep(2000);
                        conveyor.setPower(-1);
                        Thread.sleep(3000);
                        shooterLeft.setPower(0);
                        shooterRight.setPower(0);
                        conveyor.setPower(0);
                        state = 8;
                    }

                }
            }
            telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();


                // Allow time for other processes to run.
                idle();
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