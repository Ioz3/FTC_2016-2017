package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by TigeRobots on 11/12/2016.
 */
@Autonomous(name = "Compass&&Gyro", group = "Sensor")
@Disabled
public class CompassGyroTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    /*ElapsedTime timer = new ElapsedTime();*/

    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cCompassSensor compass;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    int value = 1;
    int encoderValue = 0;
    int xVal, yVal, zVal = 0;
    int heading = 0;
    int angleZ = 0;

    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;
    double distanceTraveled;
    double currentTime;
    double accelMagnitude= 0;

    boolean lastResetState = false;
    boolean curResetState  = false;

    public void runOpMode(){

        roboInit();

        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        runtime.reset();

        waitForStart();

        if (!isStopRequested() && !compass.isCalibrating()) {

            telemetry.log().clear();
            telemetry.log().add("Calibration started");
            telemetry.log().add("Slowly rotate compass 360deg");
            telemetry.log().add("Press 'B' when complete");
            compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
            runtime.reset();

            while (opModeIsActive()) {

                doTelemetry();
                idle();

            }

        }
        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            switchCase();
            gyrostuff();
            debug();

        }

    }

    public void roboInit(){

        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");
        gyro       = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GYRO");

    }

    public void doTelemetry(){

        if (compass.isCalibrating()) {

            telemetry.addData("compass", "calibrating %s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");

        }

        else {

            telemetry.addData("heading", "%.1f", compass.getDirection());

            Acceleration accel = compass.getAcceleration();
            telemetry.addData("accel", accel);
            telemetry.addData("accel magnitude", "%.3f", accelMagnitude);

            telemetry.addData("mag flux", compass.getMagneticFlux());

        }

        telemetry.addData("command", "%s", compass.readCommand());

        telemetry.update();
    }

    public void gyrostuff(){

        curResetState = (gamepad1.a && gamepad1.b);
        if(curResetState && !lastResetState)  {
            gyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;

        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        heading = gyro.getHeading();
        angleZ  = gyro.getIntegratedZValue();

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);

        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);
        telemetry.update();

    }

    public void switchCase() {

        switch (value) {

            case 1:
                drive(-0.5, 0.5, -0.5, 0.5, 2650);
                break;
            case 2:
                delay(0.5);
                value++;
                accelMagnitude = frontRight.getCurrentPosition();
                break;
            case 3:
                drive(0.5, 0.5, -0.5, -0.5, 3000);
                break;
            case 4:
                delay(0.5);
                value++;
                accelMagnitude = frontRight.getCurrentPosition();
                break;
            case 5:
                drive(-0.5, 0.5, -0.5, 0.5, 5250);
                break;
            case 6:
                delay(0.5);
                value++;
                accelMagnitude = frontRight.getCurrentPosition();
                break;
            case 7:
                drive(-0.5, -0.5, 0.5, 0.5, 6500);
                break;
            case 8:
                delay(0.5);
                value++;
                accelMagnitude = frontRight.getCurrentPosition();
                break;
            case 9:
                drive(0.5, -0.5, 0.5, -0.5, 8000);
                break;
            default:
                frontLeft.setPower(0.0);
                frontRight.setPower(0.0);
                backLeft.setPower(0.0);
                backRight.setPower(0.0);
                break;
        }
    }
    public void drive(double motor1, double motor2, double motor3, double motor4, int distance){

        double gyroOffset = (gyro.getIntegratedZValue()*0.03)-0.00;

        motorOne = motor1;
        motorTwo = motor2;
        motorThree = motor3;
        motorFour = motor4;
        distanceTraveled = distance;

        if (Math.abs(frontRight.getCurrentPosition() - accelMagnitude ) <= distance) {
            frontRight.setPower(motorOne + gyroOffset);
            frontLeft.setPower(motorTwo + gyroOffset);
            backRight.setPower(motorThree + gyroOffset);
            backLeft.setPower(motorFour + gyroOffset);
        } else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
            accelMagnitude = frontRight.getCurrentPosition();
            value++;
        }
    }

    public void delay(double time) {

        currentTime = getRuntime();

        while (getRuntime() - currentTime < time) {

            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);

        }
    }

    public void debug(){

        telemetry.addData("FRONT_RIGHT", frontRight.getCurrentPosition());
        telemetry.addData("DISTANCE", distanceTraveled);
        telemetry.addData("WHAT_IS_COMPARED", Math.abs(frontRight.getCurrentPosition()-accelMagnitude));
        telemetry.addData("CASE", value);

    }
}

