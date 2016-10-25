package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        beacons.activate();
        int noSee = 0;
        int see = 0;
        long lastTime = 0;
        boolean run = false;
        int state = 0;
        while (opModeIsActive()) {
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {

                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = (180 - Math.toDegrees(Math.atan2(translation.get(0), translation.get(2))));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);

                    telemetry.update();

                    if (state == 0) {
                        if (degreesToTurn > 5 && degreesToTurn < 180) {

                            leftMotor.setPower(.3);
                            rightMotor.setPower(-.3);


                        } else if (degreesToTurn < 355 && degreesToTurn > 180) {

                            leftMotor.setPower(-.3);
                            rightMotor.setPower(.3);

                        } else if ((degreesToTurn >= 355 || degreesToTurn <= 5)) {

                            leftMotor.setPower(0);
                            rightMotor.setPower(0);

                            state = 1;

                        }
                    }

                    if (state == 1 &&  translation.get(2) < -175) {

                        leftMotor.setPower(.3);
                        rightMotor.setPower(.3);
                        Thread.sleep(400);
                        state = 0;
                    }


                   // double zTrans = pose.getTranslation().get(2);
                   // if (zTrans < -300) {
                  //      rightMotor.setPower(.75);
                   //     leftMotor.setPower(.75);
                   // }
                    //else{
                        /*rightMotor.setPower(0);
                        leftMotor.setPower(0);
                    break;*/
                    //}
                }
                /*else {

                    if (System.currentTimeMillis() - lastTime > 1000){
                        run = !run;
                        lastTime = System.currentTimeMillis();
                    }
                    if (run || System.currentTimeMillis() - lastTime > 500){
                        leftMotor.setPower(-.30);
                        rightMotor.setPower(.30);
                        lastTime = System.currentTimeMillis();
                        run = !run;
                    }
                    telemetry.addData("", "I dont see something" + noSee);
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                } */
            }
            telemetry.update();
            //}
        }

    /*public OpenGLMatrix getPose() {
        return pose;
    }
*/
    }
}