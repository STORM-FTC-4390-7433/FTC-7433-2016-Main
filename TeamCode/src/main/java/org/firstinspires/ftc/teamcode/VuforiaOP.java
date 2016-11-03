package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import com.vuforia.ar.pl.SystemTools;

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
@Autonomous(name="vision", group="Vision")
public class VuforiaOP extends LinearOpMode {
    //OpenGLMatrix pose = new OpenGLMatrix();
    private DcMotor leftMotor = null, rightMotor = null;
    private Servo beaconServo = null;
    private ColorSensor colorSensor;
    private DeviceInterfaceModule CDI;
    private float hsvValues[] = {0, 0, 0};
    private int distanceAdjust = -1000;
    private boolean adjust = false;
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
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        beacons.activate();
        int noSee = 0;
        int see = 0;
        long lastTime = 0;
        boolean run = false;
        int state = 0;
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
                        rightMotor.setPower(.25);
                        leftMotor.setPower(.25);
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
                            beaconServo.setPosition(.6);
                            Thread.sleep(400);
                            beaconServo.setPosition(.5);
                        }
                        else if (colorSensor.blue() > colorSensor.red()) {
                            beaconServo.setPosition(.4);
                            Thread.sleep(400);
                            beaconServo.setPosition(.5);
                        }
                        state = 4;
                    }

                }
            }
            telemetry.update();
        }
    }
}