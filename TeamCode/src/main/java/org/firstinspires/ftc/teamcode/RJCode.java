package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * Created by owner on 10/20/2016.
 */
@TeleOp(name = "RJCODE", group = "TeleOp")
public class RJCode extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor uptake;
    DcMotor intake;
    DcMotor shooter;

    Servo rightButtonServo;
    Servo leftButtonServo;
    Servo scissorLiftServo;

    double driveY;
    double driveX;
    double driveRotate;

    boolean shooterButton;
    double shooterSpeed;
    int shooterPosition;
    int currentShootPosition;
    int previousShootPosition;

    double beaconPositionIn;
    double beaconPositionOut;
    boolean leftBeaconButton;
    boolean rightBeaconButton;
    boolean leftBeaconCurrent;
    boolean leftBeaconPrevious;
    boolean rightBeaconCurrent;
    boolean rightBeaconPrevious;

    double expoCurve    = 1.0;
    double deadZoneArea = 0.2;

    boolean current;
    boolean previous;

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
        scissorLiftServo    = hardwareMap.servo.get("SCISSOR_SERVO");

        //MECANUM DRIVE
        driveX       = gamepad1.left_stick_x;
        driveY       = gamepad1.left_stick_y;
        driveRotate  = gamepad1.right_stick_x;

        //SHOOTER VARIABLES
        shooterButton           = gamepad1.right_bumper;
        shooterSpeed            = 1.0;
        shooterPosition         = 1000; //TODO find the position for shooter
        currentShootPosition    = shooter.getCurrentPosition();
        previousShootPosition   = shooter.getCurrentPosition();

        //BEACON BUTTON VARIABLES
        beaconPositionIn    = 0.0;
        beaconPositionOut   = 1.0; //TODO find the position of the beacon button servos
        leftBeaconButton    = gamepad1.x;
        rightBeaconButton   = gamepad1.b;
        leftBeaconCurrent   = false;
        rightBeaconCurrent  = false;

        //VARIABLES FOR OTHER METHODS
        current     = false;
        previous    = false;

    }

    public void runOpMode(){

        roboInit();

        waitForStart();

        while(opModeIsActive()) {

            drive(driveRotate, driveY, driveX);
            shooter();
            beacon();

            debug();

            idle();

        }

    }

    public void drive(double x, double y, double z){

        backLeft.setPower(expo(constrain(x - y  - z),expoCurve));
        frontRight.setPower(expo(constrain(x + y + z),expoCurve));
        frontLeft.setPower(expo(constrain(x + y - z),expoCurve));
        backRight.setPower(expo(constrain(x - y + z),expoCurve));

    }

    public void shooter(){

        double speed;
        currentShootPosition = shooter.getCurrentPosition();

        if(currentShootPosition - previousShootPosition <= shooterPosition){

            speed = shooterSpeed;

        }
        else if(currentShootPosition - previousShootPosition >= shooterPosition && shooterButton){

            speed                   = shooterSpeed;
            previousShootPosition   = currentShootPosition;

        }
        else{speed = 0.0;}

        shooter.setPower(speed);

    }

    public void intakeAndUptake(){

        //TODO code the intake and uptake

    }

    public void beacon(){

        leftBeaconCurrent   = leftBeaconButton;
        rightBeaconCurrent  = rightBeaconButton;

        if (leftBeaconCurrent && !leftBeaconPrevious && leftButtonServo.getPosition() == beaconPositionIn){

            leftButtonServo.setPosition(beaconPositionOut);

        }
        else if (leftBeaconCurrent && !leftBeaconPrevious && leftButtonServo.getPosition() == beaconPositionOut){

            leftButtonServo.setPosition(beaconPositionIn);

        }
        if (rightBeaconCurrent && !rightBeaconPrevious && rightButtonServo.getPosition() == beaconPositionIn){

            rightButtonServo.setPosition(beaconPositionOut);

        }
        else if (rightBeaconCurrent && !rightBeaconPrevious && rightButtonServo.getPosition() == beaconPositionOut){

            rightButtonServo.setPosition(beaconPositionIn);

        }

        leftBeaconPrevious   = leftBeaconCurrent;
        rightBeaconPrevious  = rightBeaconCurrent;

    }

    public void deadZone(){

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

    double expo(double x, double a){

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


    private void debug() {

        telemetry.addData("MOTOR_STUFF_FR", frontRight.getPower());
        telemetry.addData("MOTOR_STUFF_FL", frontLeft.getPower());
        telemetry.addData("MOTOR_STUFF_BR", backRight.getPower());
        telemetry.addData("MOTOR_STUFF_BL", backLeft.getPower());

    }
}
