package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by grego on 10/18/2016.
 */

public class VuforiaOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATL2vJ3/////AAAAGZZx51v2h0D2kh6vX9dkEVwwXavfMtPW74LnE7NWXWw2NChN8Td99tPKhECwV61l/fTsgxV43ktU6XBUlR9lZn1Z3BEd7nQPD+s4uscCWDSjTpXDdQZZWVD7Cfmp+ZK8ax49W55s1vC6mX3vED8miPeegc8DR1bT2BtjxLa0cD77nbeVN5ztUzZEGKPTZEhxGoxjqQsKOEUktyLo6NZIRTA5uEhOmVuwVWC1Iq49tfbjKnLe7t1qfzQlB6wri9DPUrtt3YeuyrNERLclghW7fz7GrfWooMfQIaNEbu/E7BhY95CDGy/Srl1ZpvingaBdcfpB7MAQ/+bFw93saY/lYwT7MXu9ctO9zv1rmuFEAJFp";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2017-17");

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()){
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "Degrees", degreesToTurn);

                }
            }
            telemetry.update();
        }
    }

}
