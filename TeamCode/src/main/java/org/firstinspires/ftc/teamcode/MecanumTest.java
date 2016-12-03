package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by owner on 10/13/2016.
 */
@TeleOp(name = "TEST", group = "TeleOp")
public class MecanumTest extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    @Override
    public void init(){

        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft = hardwareMap.dcMotor.get("BACK_LEFT");

    }

    @Override
    public void loop(){


    drive();

}
    double constrain(double x){

        double speed;
        speed = x;

        speed = Range.clip(speed, -1, 1);

        return speed;

    }

    public void drive(){

        backLeft.setPower(constrain(gamepad1.right_stick_x - gamepad1.left_stick_x  + gamepad1.left_stick_y));
        frontRight.setPower(constrain(gamepad1.right_stick_x + gamepad1.left_stick_x - gamepad1.left_stick_y));
        frontLeft.setPower(constrain(gamepad1.right_stick_x + gamepad1.left_stick_x + gamepad1.left_stick_y));
        backRight.setPower(constrain(gamepad1.right_stick_x - gamepad1.left_stick_x - gamepad1.left_stick_y));

    }


}
