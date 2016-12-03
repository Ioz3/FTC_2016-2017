package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by TigeRobots on 11/21/2016.
 */
@TeleOp (name = "NewTeleOp", group = "TeleOp")
public class NewTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

   /* DcMotor inTake;
    DcMotor upTake;
    DcMotor flicker;
    DcMotor lift;

    Servo pressRight;
    Servo pressLeft;
    Servo stopOne;
    Servo stopTwo;
    Servo stopThree;
    Servo ballRelease;
    Servo forksRelease;
    Servo forksStage;*/

    double deadZoneArea = 0.2;
    double expoCurve    = 1.0;
    double turn;
    double strafe;
    double forward;

    final double MIN_SPEED = -1;
    final double MAX_SPEED = 1;

    public void runOpMode(){

        roboInit();

        runtime.reset();

        waitForStart();

        while(opModeIsActive()){

            telemetry.addData("status", "run time :" + runtime.toString());

            drive();
            //deadZone();

            idle();

        }

    }
    private void roboInit(){

        frontRight   = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft    = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight    = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft     = hardwareMap.dcMotor.get("BACK_LEFT");

       /* inTake       = hardwareMap.dcMotor.get("IN_TAKE");
        upTake       = hardwareMap.dcMotor.get("UP_TAKE");
        flicker      = hardwareMap.dcMotor.get("FLICKER");
        lift         = hardwareMap.dcMotor.get("LIFT");

        pressRight   = hardwareMap.servo.get("PRESS_RIGHT");
        pressLeft    = hardwareMap.servo.get("PRESS_LEFT");
        stopOne      = hardwareMap.servo.get("STOP_ONE");
        stopTwo      = hardwareMap.servo.get("STOP_TWO");
        stopThree    = hardwareMap.servo.get("STOP_THREE");
        ballRelease  = hardwareMap.servo.get("BALL_RELEASE");
        forksRelease = hardwareMap.servo.get("FORKS_RELEASE");
        forksStage   = hardwareMap.servo.get("FORKS_STAGE");*/

    }

    public void drive(){

        turn         = gamepad1.right_stick_x;
        strafe       = gamepad1.left_stick_x;
        forward      = gamepad1.left_stick_y;

        frontRight.setPower(expo(constrain(turn + strafe + forward), expoCurve));
        frontLeft.setPower(expo(constrain(turn + strafe - forward), expoCurve));
        backRight.setPower(expo(constrain(turn - strafe + forward), expoCurve));
        backLeft.setPower(expo(constrain(turn - strafe - forward), expoCurve));

    }

    public void rotate(){

        turn         = gamepad1.right_stick_x;

        frontRight.setPower(expo(constrain(turn), expoCurve));
        frontLeft.setPower(expo(constrain(turn), expoCurve));
        backRight.setPower(expo(constrain(turn), expoCurve));
        backLeft.setPower(expo(constrain(turn), expoCurve));

    }

    double constrain(double x){

        double speed;
        speed = x;

        speed = Range.clip(speed, MIN_SPEED, MAX_SPEED);

        return speed;

    }

    public void deadZone(){

        double x = Math.abs(strafe);
        double y = Math.abs(forward);

        if (Math.sqrt((x*x)+(y*y)) > deadZoneArea){
            drive();
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
}