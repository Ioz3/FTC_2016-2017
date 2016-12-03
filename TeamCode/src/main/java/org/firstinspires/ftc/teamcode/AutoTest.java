package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by owner on 10/21/2016.
 * PLEASE READ
 * **********CAYLEE, RUBY, ALYSSA***********
 * check to see what is being called in your loop method
 * do some tests to see how to rotate draw a diagram to see what each wheel should do
 * set a goal and work it out one step at a time
 * you already have everything you need to start a simple auto remember if things aren't working right
 * check things with your debug method ex.(check speed, distance, or what case you are currently on)
 * look things up if you cant figure it out
 * one thing to think about is the way you are handling distance does the encoder reset or does it leave
 * off at the last known position if you go straight for 300 rotations
 * and then you want to go left for 500 rotations is that 300 + 500
 * do some testing to find out try to figure out a way to handle it in a simple way
 * remember you have variables that can contain any current or previous number you want to save
 */

@Autonomous(name = "AutoTest",group = "Auto")
public class AutoTest extends OpMode{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;
    double distanceTraveled;
    double currentTime;

    int    value = 1;
    int    encoderValue = 0;

    @Override
    public void init(){

        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");

    }
    @Override
    public void loop(){

        autonomous();
        debug();

    }
    public void autonomous(){

        switchCase();
    }
    public void switchCase(){

        switch(value){

            case 1: drive(-0.5, 0.5, -0.5, 0.5, 2650 );
                break;
            case 2: delay(0.5);value++;encoderValue = frontRight.getCurrentPosition();
                break;
            case 3: drive(-0.5, -0.5 , 0.5, 0.5, 3000);
                break;
            case 4: delay(0.5);value++;encoderValue = frontRight.getCurrentPosition();
                break;
            case 5:drive(-0.5, 0.5, -0.5, 0.5, 5250);
                break;
            case 6:delay(0.5);value++;encoderValue = frontRight.getCurrentPosition();
                break;
            case 7:drive(0.5, 0.5, -0.5, -0.5, 6500);
                break;
            case 8:delay(0.5);value++;encoderValue = frontRight.getCurrentPosition();
                break;
            case 9:drive(0.5, -0.5, 0.5, -0.5, 8000);
                break;
            default:frontLeft.setPower(0.0);frontRight.setPower(0.0);backLeft.setPower(0.0);backRight.setPower(0.0);
                break;

        }

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

    public void delay(double time){

        currentTime = getRuntime();

        while(getRuntime() - currentTime < time){

            frontLeft.setPower(0.0);frontRight.setPower(0.0);backLeft.setPower(0.0);backRight.setPower(0.0);

        }

    }

    public void debug(){

        telemetry.addData("FRONT_RIGHT", frontRight.getCurrentPosition());
        telemetry.addData("DISTANCE", distanceTraveled);
        telemetry.addData("WHAT_IS_COMPARED", Math.abs(frontRight.getCurrentPosition()-encoderValue));
        telemetry.addData("CASE", value);

    }
}
