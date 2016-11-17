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
    private String color = "blue";
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATL2vJ3/////AAAAGZZx51v2h0D2kh6vX9dkEVwwXavfMtPW74LnE7NWXWw2NChN8Td99tPKhECwV61l/fTsgxV43ktU6XBUlR9lZn1Z3BEd7nQPD+s4uscCWDSjTpXDdQZZWVD7Cfmp+ZK8ax49W55s1vC6mX3vED8miPeegc8DR1bT2BtjxLa0cD77nbeVN5ztUzZEGKPTZEhxGoxjqQsKOEUktyLo6NZIRTA5uEhOmVuwVWC1Iq49tfbjKnLe7t1qfzQlB6wri9DPUrtt3YeuyrNERLclghW7fz7GrfWooMfQIaNEbu/E7BhY95CDGy/Srl1ZpvingaBdcfpB7MAQ/+bFw93saY/lYwT7MXu9ctO9zv1rmuFEAJFp";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;


        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        beaconServo = hardwareMap.servo.get("beaconServo");
        colorSensor = hardwareMap.colorSensor.get("color");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        colorSensor.enableLed(false);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();




        long lastTime = 0;
        boolean run = false;
        int state = 0;
        boolean[] beaconCapped = new boolean[4];
        boolean switchTarget = true;


        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.activate();
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");


        while (opModeIsActive()) {
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            telem();
            VuforiaTrackable target = null;


            for(int i = 0; i < beacons.size(); i++) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(i).getListener()).getPose();


                if (pose != null && switchTarget && !beaconCapped[i]) {
                    target = beacons.get(i);
                    switchTarget = false;
                }
            }

            OpenGLMatrix pose = null;
            if(target != null){
                pose = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
            }


            if (pose != null) {
                VectorF translation = pose.getTranslation();
                telem2();
                double degreesToTurn = (180 - Math.toDegrees(Math.atan2(translation.get(0), translation.get(2))));


                if (state == 0) {
                    if (degreesToTurn > 2 && degreesToTurn < 180) {
                        drive(-.15, .15, 0);
                        adjust = false;
                    } else if (degreesToTurn < 358 && degreesToTurn > 180) {
                        drive(.15, -.15, 0);
                        adjust = false;
                    } else if ((degreesToTurn >= 358 || degreesToTurn <= 2)) {
                        drive(0, 0, 0);
                        adjust = true;
                        state = 1;
                    }
                } else if (state == 1) {
                    if (translation.get(2) < distanceAdjust && translation.get(2) < -200) {
                        drive(.75, .75, 0);
                    } else if (translation.get(2) > -200 && adjust) {
                        state = 2;
                    } else {
                        drive(0, 0, 0);
                        distanceAdjust /= 2;
                        state = 0;
                        adjust = false;
                    }
                } else if (state == 2) {
                    drive(.25, .25, 850);
                    drive(0, 0, 0);
                    state = 3;
                } else if (state == 3) {
                    drive(-.5, -.5, 300);
                    drive(0, 0, 0);
                    if (colorSensor.red() > colorSensor.blue() && color == "blue") {
                        beaconServo.setPosition(.8);
                    } else if (colorSensor.blue() > colorSensor.red() && color == "blue") {
                        beaconServo.setPosition(.1);
                    }
                    state = 4;
                } else if (state == 4) {
                    drive(.75, .75, 800);
                    drive(0, 0, 0);
                    state = 5;
                } else if (state == 5) {
                    drive(-.75, -.75, 1600);
                    drive(0, 0, 0);
                    state = 1;
                    switchTarget = true;
                } else if(state == 6){
                    drive(-.75, .75, 500);
                    drive(0, 0, 0);

                }
            }
            telemetry.update();
        }
    }


    public void drive(double powerLeft, double powerRight, int duration){
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);
        try {
            Thread.sleep(duration);
        } catch (InterruptedException ex) {
        }
    }


    public void telem(){
        telemetry.addData("2 Clear", colorSensor.alpha());
        telemetry.addData("3 Red  ", colorSensor.red());
        telemetry.addData("4 Green", colorSensor.green());
        telemetry.addData("5 Blue ", colorSensor.blue());
        telemetry.addData("6 Hue  ", hsvValues[0]);
    }


    public void telem2(){
        //telemetry.addData(beac.getName() + "-Translation", translation);
        //telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
        telemetry.update();
    }


}


