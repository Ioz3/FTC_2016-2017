package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
/**
 * Created by owner on 10/14/2016.
 */
@TeleOp(name = "DEADZONE", group = "TeleOp")
public class DeadZoneTest extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;


    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;

    @Override
    public void init(){

        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft = hardwareMap.dcMotor.get("BACK_LEFT");

    }

    @Override
    public void loop(){

        deadZone();

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

    public void deadZone(){

        double x = Math.abs(gamepad1.left_stick_x);
        double y = Math.abs(gamepad1.left_stick_y);

        if (Math.sqrt((x*x)+(y*y)) < 0.3){
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
        }
        else{
            drive();
        }
    }
}
