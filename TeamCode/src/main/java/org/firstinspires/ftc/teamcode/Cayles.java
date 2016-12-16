package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by TigeRobots on 11/21/2016.
 */
@TeleOp(name ="Cayles", group = "TeleOp")
@Disabled
public class Cayles extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    double expoCurve    = 1.0;

    double turn         = gamepad1.right_stick_x;
    double strafe       = gamepad1.left_stick_x;
    double forward      = gamepad1.left_stick_y;

    @Override
    public void runOpMode(){

        //roboInit();
        drive();
        //rotate();

    }

    public void roboInit(){

        frontRight   = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft    = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight    = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft     = hardwareMap.dcMotor.get("BACK_LEFT");

    }

    public void drive(){

        frontRight.setPower(expo(constrain(turn + strafe - forward), expoCurve));
        frontLeft.setPower(expo(constrain(turn + strafe + forward), expoCurve));
        backRight.setPower(expo(constrain(turn - strafe - forward), expoCurve));
        backLeft.setPower(expo(constrain(turn - strafe + forward), expoCurve));

    }

    public void rotate(){

        frontRight.setPower(expo(constrain(turn), expoCurve));
        frontLeft.setPower(expo(constrain(turn), expoCurve));
        backRight.setPower(expo(constrain(turn), expoCurve));
        backLeft.setPower(expo(constrain(turn), expoCurve));

    }

    double constrain(double x){

        double speed;
        speed = x;

        speed = Range.clip(speed, -1, 1);

        return speed;

    }

    double expo(double x, double a){

        double y = x;
        y = a * Math.pow(y, 3) + (1-a)*y;
        return y;

    }
}
    //public void runOpMode(){}
