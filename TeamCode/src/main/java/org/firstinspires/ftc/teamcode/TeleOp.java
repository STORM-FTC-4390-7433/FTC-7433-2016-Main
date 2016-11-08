package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by grego on 11/5/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Movement", group="TeleOp")
public class TeleOp extends OpMode {
    private DcMotor shooterLeft = null;
    private DcMotor shooterRight = null;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor sweeper = null;
    private DcMotor conveyor = null;
    private Servo beaconServo = null;
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        beaconServo = hardwareMap.servo.get("beaconServo");
        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void loop() {
        float throttleLeft = -gamepad1.left_stick_y;
        float throttleRight = -gamepad1.right_stick_y;
        float shooterPower = -gamepad2.left_stick_y;
        //float sweepconveyPower = gamepad2.right_stick_y;
        float sweepconveyPower;

//        if(gamepad2.x)
//            shooterPower = 1;
//        else if(gamepad2.b)
//            shooterPower = -1;
//        else
//            shooterPower = 0;

        if(gamepad2.y)
            sweepconveyPower = 1;
        else if(gamepad2.a)
            sweepconveyPower = -1;
        else
            sweepconveyPower = 0;

        shooterPower = Range.clip(shooterPower, -1, 1);
        //sweepconveyPower = Range.clip(sweepconveyPower, -1, 1);
        throttleRight = Range.clip(throttleRight, -1, 1);
        throttleLeft = Range.clip(throttleLeft, -1, 1);

        throttleRight = (float)scaleInput(throttleRight);
        throttleLeft =  (float)scaleInput(throttleLeft);
        shooterPower = (float)scaleInput(shooterPower);
        sweepconveyPower = (float) Math.ceil(sweepconveyPower);

        leftMotor.setPower(throttleLeft);
        rightMotor.setPower(throttleRight);
        shooterLeft.setPower(shooterPower);
        shooterRight.setPower(shooterPower);
        sweeper.setPower(sweepconveyPower);
        conveyor.setPower(sweepconveyPower);

    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
