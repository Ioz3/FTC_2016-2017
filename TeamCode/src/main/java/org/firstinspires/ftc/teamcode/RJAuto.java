package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by cicada01 on 12/9/16.
 */
@Autonomous(name = "RJAUTO", group = "Auto")
public class RJAuto extends LinearOpMode {

    //motor
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    private DcMotor uptake;
    private DcMotor intake;
    private DcMotor shooter;

    //servo
    private Servo   rightButtonServo;
    private Servo   leftButtonServo;
    private Servo   loadFront;

    //shooter
    private double  shooterSpeed;
    private int     shooterPosition;
    private int     currentShootPosition;
    private int     previousShootPosition;

    //beacon
    private double  beaconPositionIn;
    private double  beaconPositionOut;

    //uptake and intake
    private double  inUpGo;
    private double  inUpStop;
    private double  setInUp;
    private boolean isRunning;

    //reload
    private double  loadFrontPosUp;
    private double  loadFrontPosDown;
    private double  loadFrontTime;
    private boolean loadIsReady;


    double distanceTraveled;
    double currentTime;

    int    value = 1;
    int    encoderValue = 0;

    public void runOpMode()  {

        roboInit();

        waitForStart();

        //our main teleop loop
        while(opModeIsActive()) {

            debug();

            idle();

        }

    }

    public void roboInit(){

        //MOTOR INIT
        frontRight  = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft   = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight   = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft    = hardwareMap.dcMotor.get("BACK_LEFT");
        intake      = hardwareMap.dcMotor.get("INTAKE");
        uptake      = hardwareMap.dcMotor.get("UPTAKE");
        shooter     = hardwareMap.dcMotor.get("SHOOTER");

        //SERVO INIT
        rightButtonServo    = hardwareMap.servo.get("RIGHT_BUTTON");
        leftButtonServo     = hardwareMap.servo.get("LEFT_BUTTON");
        loadFront           = hardwareMap.servo.get("LOAD_FRONT");

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

    public void drive(double motorOne, double motorTwo, double motorThree, double motorFour, int distance){

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



    }
}
