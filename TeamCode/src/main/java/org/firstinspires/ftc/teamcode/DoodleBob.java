package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by TigeRobots on 11/18/2016.
 * This is a basic teleOp code to be used for
 * testing and debugging purposes
 */

public class DoodleBob extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    double turn    = gamepad1.right_stick_x;
    double forward = gamepad1.left_stick_x;
    double strafe  = gamepad1.left_stick_y;


    public void runOpMode(){

        roboInit();
        drive(turn, forward, strafe);
        rotate();

    }

    public void roboInit(){

        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");

    }

    public void drive(double turn, double forward, double strafe){

            backLeft.setPower(constrain(turn - forward - strafe));
            frontRight.setPower(constrain(turn + forward + strafe));
            frontLeft.setPower(constrain(turn + forward - strafe));
            backRight.setPower(constrain(turn - forward + strafe));

        }

    public void rotate(){

        backLeft.setPower(constrain(turn));
        frontRight.setPower(constrain(turn));
        frontLeft.setPower(constrain(turn));
        backRight.setPower(constrain(turn));


    }

    double constrain(double x){

        double speed;
        speed = x;

        speed = Range.clip(speed, -1, 1);

        return speed;

    }


}
