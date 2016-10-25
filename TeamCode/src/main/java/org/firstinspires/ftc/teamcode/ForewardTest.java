package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by grego on 10/22/2016.
 */
/*
@Autonomous(name="ForewardTest", group="Vision")
public class ForewardTest extends OpMode {

    private DcMotor leftMotor = null, rightMotor = null;
    VuforiaOP vuforia = new VuforiaOP();
    double zTrans = vuforia.getPose().getTranslation().get(2);

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        zTrans = vuforia.getPose().getTranslation().get(2);
        if (zTrans < -100) {

            rightMotor.setPower(.75);
            leftMotor.setPower(.75);


        }
        else{
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }


    }
}

*/