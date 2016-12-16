package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by TigeRobots on 11/18/2016.
 *
 * Final Autonomous code for Ftc 2016-2017
 */
@Autonomous(name = "FinalAuto", group = "Auto")
@Disabled
public class FinalAuto extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor inTake;
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
    Servo forksStage;

    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;
    double distanceTraveled;
    double currentTime;

    int    value = 1;
    int    encoderValue = 0;

    public void runOpMode(){

        roboInit();
        drive(0, 0, 0, 0, 0);

    }

    public void roboInit(){

        frontRight   = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft    = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight    = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft     = hardwareMap.dcMotor.get("BACK_LEFT");
        inTake       = hardwareMap.dcMotor.get("IN_TAKE");
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
        forksStage   = hardwareMap.servo.get("FORKS_STAGE");

    }

    public void drive(double motor1, double motor2, double motor3, double motor4, int distance){

        motorOne           = motor1;
        motorTwo           = motor2;
        motorThree         = motor3;
        motorFour          = motor4;
        distanceTraveled   = distance;

        if(Math.abs(frontRight.getCurrentPosition() - encoderValue) <= distance) {
            frontRight.setPower(motorOne);
            frontLeft.setPower(motorTwo);
            backRight.setPower(motorThree);
            backLeft.setPower(motorFour);
        }
        else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
            encoderValue = frontRight.getCurrentPosition();
            value++;
        }

    }
}
