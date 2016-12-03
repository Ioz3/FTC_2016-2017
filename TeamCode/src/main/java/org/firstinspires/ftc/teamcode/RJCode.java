package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * Created by owner on 10/20/2016.
 */
@TeleOp(name = "RJCODE", group = "TeleOp")
public class RJCode extends OpMode{

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;


    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;
    double expoCurve = 1.0;

    double deadZoneArea = 0.2;

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

    public void drive(double x, double y, double z){

        backLeft.setPower(expo(constrain(x - y  - z),expoCurve));
        frontRight.setPower(expo(constrain(x + y + z),expoCurve));
        frontLeft.setPower(expo(constrain(x + y - z),expoCurve));
        backRight.setPower(expo(constrain(x - y + z),expoCurve));

    }

    public void rotate(){

        backLeft.setPower(expo(constrain(gamepad1.right_stick_x),expoCurve));
        frontRight.setPower(expo(constrain(gamepad1.right_stick_x),expoCurve));
        frontLeft.setPower(expo(constrain(gamepad1.right_stick_x),expoCurve));
        backRight.setPower(expo(constrain(gamepad1.right_stick_x),expoCurve));

    }

    public void deadZone(){

        double x = Math.abs(gamepad1.left_stick_x);
        double y = Math.abs(gamepad1.left_stick_y);

        if (Math.sqrt((x*x)+(y*y)) > deadZoneArea){
            drive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);
        }
        else if(gamepad1.right_stick_x > deadZoneArea || gamepad1.right_stick_x < -deadZoneArea){
            rotate();
        }
        else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
        }
    }

    double expo(double x, double a){

        double y = x;
        y = a * Math.pow(y, 3) + (1-a)*y;
        return y;

    }

    private void debug() {

        telemetry.addData("MOTOR_STUFF_FR", frontRight.getPower());
        telemetry.addData("MOTOR_STUFF_FL", frontLeft.getPower());
        telemetry.addData("MOTOR_STUFF_BR", backRight.getPower());
        telemetry.addData("MOTOR_STUFF_BL", backLeft.getPower());

    }
}
