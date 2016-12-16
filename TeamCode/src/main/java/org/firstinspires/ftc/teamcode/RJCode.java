package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * Created by owner on 10/20/2016.
 */
@TeleOp(name = "RJCODE", group = "TeleOp")
public class RJCode extends LinearOpMode {

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

    //drive train
    private double  driveY;
    private double  driveX;
    private double  driveRotate;

    //shooter
    private boolean shooterButton;
    private boolean previousShooterButton;
    private double  shooterSpeed;
    private int     shooterPosition;
    private int     currentShootPosition;
    private int     previousShootPosition;

    //beacon
    private double  rBeaconPositionIn;
    private double  lBeaconPositionIn;
    private double  rBeaconPositionOut;
    private double  lBeaconPositionOut;
    private double  rightButtonPosition;
    private double  leftButtonPosition;
    private boolean leftBeaconButton;
    private boolean rightBeaconButton;
    private boolean leftBeaconCurrent;
    private boolean leftBeaconPrevious;
    private boolean rightBeaconCurrent;
    private boolean rightBeaconPrevious;

    //uptake and intake
    private double  inUpGo;
    private double  inUpStop;
    private double  setInUp;
    private boolean inUpTakeButton;
    private boolean inUpReverseButton;
    private boolean inUpTakeCurrent;
    private boolean inUpTakePrevious;
    private boolean isRunning;

    //reload
    private double  loadFrontPosUp;
    private double  loadFrontPosDown;
    private double  loadFrontTime;
    private boolean loadIsReady;
    private boolean loadButton;
    private boolean loadCurrentPress;
    private boolean loadPreviousPress;


    //various tools
    private double currentTime;

    private double  expoCurve      = 1.0;
    private double  deadZoneArea   = 0.2;

    private boolean current;
    private boolean previous;

    public void roboInit(){

        //MOTOR INIT
        frontRight  = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft   = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight   = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft    = hardwareMap.dcMotor.get("BACK_LEFT");
        intake      = hardwareMap.dcMotor.get("INTAKE");
        uptake      = hardwareMap.dcMotor.get("UPTAKE");
        shooter     = hardwareMap.dcMotor.get("SHOOTER");

        //MOTOR REVERSE
        shooter.setDirection(DcMotor.Direction.REVERSE);

        //ENCODER TURN OFF
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SERVO INIT
        rightButtonServo    = hardwareMap.servo.get("RIGHT_BUTTON");
        leftButtonServo     = hardwareMap.servo.get("LEFT_BUTTON");
        loadFront           = hardwareMap.servo.get("LOAD_FRONT");

        //MECANUM DRIVE
        driveX       = gamepad1.left_stick_x;
        driveY       = gamepad1.left_stick_y;
        driveRotate  = gamepad1.right_stick_x;

        //SHOOTER VARIABLES
        shooterButton           = false;
        previousShooterButton   = false;
        shooterSpeed            = 1.0;
        shooterPosition         = 3400; //TODO find the position for shooter
        currentShootPosition    = shooter.getCurrentPosition();
        previousShootPosition   = shooter.getCurrentPosition();

        //BEACON BUTTON VARIABLES
        lBeaconPositionIn    = 0.3;
        lBeaconPositionOut   = 0.75;
        rBeaconPositionIn    = 0.75;
        rBeaconPositionOut   = 0.3; //TODO find the position of the beacon button servos
        rightButtonPosition = rBeaconPositionIn;
        leftButtonPosition  = lBeaconPositionIn;
        leftBeaconButton    = false;
        rightBeaconButton   = false;
        leftBeaconCurrent   = false;
        rightBeaconCurrent  = false;

        //INTAKE AND UPTAKE VARIABLES
        inUpGo              = 1.0;
        inUpStop            = 0.0;
        setInUp             = inUpStop;
        inUpTakeButton      = false;
        inUpReverseButton   = false;
        isRunning           = false;

        //RELOAD VARIABLES
        loadFrontPosUp      = 0.0; //TODO find the time and position for reloading
        loadFrontPosDown    = 1.0;
        loadFrontTime       = 0.3;
        loadIsReady         = false;
        loadButton          = false;
        loadCurrentPress    = false;
        loadPreviousPress   = false;

        //VARIABLES FOR OTHER METHODS
        current     = false;
        previous    = false;

    }

    public void runOpMode(){

        roboInit();

        waitForStart();

        //our main teleop loop
        while(opModeIsActive()) {

            driveUpDate();
            drive(driveRotate, driveY, driveX);
            shooter();
            reload();
            beacon();
            intakeAndUptake();

            debug();

            idle();

        }

    }

    private void driveUpDate(){

        driveX       = gamepad1.left_stick_x;
        driveY       = gamepad1.left_stick_y;
        driveRotate  = gamepad1.right_stick_x;

    }

    public void drive(double x, double y, double z){

        backRight.setPower(expo(constrain(x + y - z),expoCurve));
        backLeft.setPower(expo(constrain(x - y  - z),expoCurve));
        frontRight.setPower(expo(constrain(x + y + z),expoCurve));
        frontLeft.setPower(expo(constrain(x - y + z),expoCurve));

    }

    private void shooter(){

        double speed;
        shooterButton = gamepad1.right_bumper;
        currentShootPosition = shooter.getCurrentPosition();

        if(currentShootPosition - previousShootPosition <= shooterPosition){

            speed = shooterSpeed;

        }
        //this is where you press the button and it shoots
        else if(currentShootPosition - previousShootPosition >= shooterPosition && shooterButton && !previousShooterButton){

            speed                   = shooterSpeed;
            previousShootPosition   = currentShootPosition;

        }
        else{speed = 0.0;}

        previousShooterButton = shooterButton;

        shooter.setPower(speed);

    }

    private void reload(){

        loadButton       = gamepad1.left_bumper;
        loadCurrentPress = loadButton;

        if(loadCurrentPress && !loadPreviousPress){

            loadIsReady = true;
            currentTime = getRuntime();

        }

        if(loadIsReady){

            delayServoPosition(loadFrontTime, loadFront, loadFrontPosUp, loadFrontPosDown, loadIsReady);

        }

        else if(!loadIsReady){loadFront.setPosition(loadFrontPosDown);}

        loadPreviousPress = loadCurrentPress;

    }

    private void intakeAndUptake(){

        inUpTakeButton      = gamepad1.a;
        inUpReverseButton   = gamepad1.y;
        setInUp             = inUpStop;
        inUpTakeCurrent     = inUpTakeButton;

        //this is where you press the button to activate the uptake and intake
        if(inUpTakeCurrent && !inUpTakePrevious && !isRunning && !gamepad1.y && !gamepad1.start){

            isRunning   = true;

        }

        else if(inUpTakeCurrent && !inUpTakePrevious && isRunning && !gamepad1.y){

            isRunning   = false;

        }

        inUpTakePrevious = inUpTakeCurrent;

        if(isRunning && !inUpReverseButton){

            intake.setPower(inUpGo);
            uptake.setPower(inUpGo);

        }

        else if(!isRunning && !inUpReverseButton){

            intake.setPower(inUpStop);
            uptake.setPower(inUpStop);

        }

        if (inUpReverseButton){

            intake.setPower(-inUpGo);
            uptake.setPower(-inUpGo);

        }

    }

    private void beacon(){ //todo buttons are not working

        leftBeaconButton    = gamepad1.x;
        rightBeaconButton   = gamepad1.b;

        leftBeaconCurrent   = leftBeaconButton;
        rightBeaconCurrent  = rightBeaconButton;

        if (leftBeaconCurrent && !leftBeaconPrevious && leftButtonServo.getPosition() == lBeaconPositionIn){

            leftButtonPosition = lBeaconPositionOut;

        }
        else if (leftBeaconCurrent && !leftBeaconPrevious && leftButtonServo.getPosition() == lBeaconPositionOut){

            leftButtonPosition = lBeaconPositionIn;

        }
        if (rightBeaconCurrent && !rightBeaconPrevious && rightButtonServo.getPosition() == rBeaconPositionIn){

            rightButtonPosition = rBeaconPositionOut;

        }
        else if (rightBeaconCurrent && !rightBeaconPrevious && rightButtonServo.getPosition() == rBeaconPositionOut){

            rightButtonPosition = rBeaconPositionIn;

        }

        leftBeaconPrevious   = leftBeaconCurrent;
        rightBeaconPrevious  = rightBeaconCurrent;

        rightButtonServo.setPosition(rightButtonPosition);
        leftButtonServo.setPosition(leftButtonPosition);

    }

    //TOOLS AND MATHS
    private void deadZone(){

        double x = Math.abs(gamepad1.left_stick_x);
        double y = Math.abs(gamepad1.left_stick_y);

        if (Math.sqrt((x*x)+(y*y)) > deadZoneArea){
            drive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);
        }
        else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
        }
    }

    private double expo(double x, double a){

        double y = x;
        y = a * Math.pow(y, 3) + (1-a)*y;
        return y;

    }

    double constrain(double x){

        double speed;
        speed = x;

        speed = Range.clip(speed, -1, 1);

        return speed;

    }

    double constrain(double x, double y, double z){

        double speed;
        double low;
        double high;
        speed   = x;
        low     = y;
        high    = z;

        speed = Range.clip(speed, low, high);

        return speed;

    }

    private void delayMotorSpeed(double time, DcMotor motor1, double speed){


        while(getRuntime() - currentTime < time){

            motor1.setPower(speed);

        }


    }

    private boolean delayServoPosition(double time, Servo servo1, double position1, double position2, boolean isReady){

        if(getRuntime() - currentTime < time){servo1.setPosition(position1);}
        else{servo1.setPosition(position2);isReady = false;}

        return isReady;
    }

    private void debug() {

        telemetry.addData("MOTOR_STUFF_FR", frontRight.getPower());
        telemetry.addData("MOTOR_STUFF_FL", frontLeft.getPower());
        telemetry.addData("MOTOR_STUFF_BR", backRight.getPower());
        telemetry.addData("MOTOR_STUFF_BL", backLeft.getPower());
        telemetry.addData("FRONT_LEFT", loadFront.getPosition());
        telemetry.addData("IS_READY", loadIsReady);
        telemetry.addData("CURRENT_TIME",(getRuntime()-currentTime));
        telemetry.update();

    }
}
