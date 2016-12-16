package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by TigeRobots on 11/11/2016.
 */
@Autonomous(name = "GyroAutoTest", group = "Sensor")
@Disabled
public class GyroAutoTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    int value = 1;
    int encoderValue = 0;
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;

    ModernRoboticsI2cGyro gyro;

    double motorOne;
    double motorTwo;
    double motorThree;
    double motorFour;
    double distanceTraveled;
    double currentTime;

    boolean lastResetState = false;
    boolean curResetState  = false;

    public void runOpMode() {

        roboInit();

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

        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            switchCase();
            gyrostuff();
        }
    }

    public void switchCase() {

        switch (value) {

            case 1:
                drive(-0.5, 0.5, -0.5, 0.5, 2650);
                break;
            case 2:
                delay(0.5);
                value++;
                encoderValue = frontRight.getCurrentPosition();
                break;
            case 3:
                drive(0.5, 0.5, -0.5, -0.5, 3000);
                break;
            case 4:
                delay(0.5);
                value++;
                encoderValue = frontRight.getCurrentPosition();
                break;
            case 5:
                drive(-0.5, 0.5, -0.5, 0.5, 5250);
                break;
            case 6:
                delay(0.5);
                value++;
                encoderValue = frontRight.getCurrentPosition();
                break;
            case 7:
                drive(-0.5, -0.5, 0.5, 0.5, 6500);
                break;
            case 8:
                delay(0.5);
                value++;
                encoderValue = frontRight.getCurrentPosition();
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

    public void drive(double motor1, double motor2, double motor3, double motor4, int distance) {

        double gyroOffset = (gyro.getIntegratedZValue()*0.03)-0.00;

        motorOne = motor1;
        motorTwo = motor2;
        motorThree = motor3;
        motorFour = motor4;
        distanceTraveled = distance;

        if (Math.abs(frontRight.getCurrentPosition() - encoderValue) <= distance) {
            frontRight.setPower(motorOne + gyroOffset);
            frontLeft.setPower(motorTwo + gyroOffset);
            backRight.setPower(motorThree + gyroOffset);
            backLeft.setPower(motorFour + gyroOffset);
        } else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
            encoderValue = frontRight.getCurrentPosition();
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
        public void gyrostuff(){

            // if the A and B buttons are pressed just now, reset Z heading.
            curResetState = (gamepad1.a && gamepad1.b);
            if(curResetState && !lastResetState)  {
                gyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            // get the x, y, and z values (rate of change of angle).
            xVal = gyro.rawX();
            yVal = gyro.rawY();
            zVal = gyro.rawZ();

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            heading = gyro.getHeading();
            angleZ  = gyro.getIntegratedZValue();

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);

            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);
            telemetry.update();

        }


    public void roboInit() {

        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
        gyro       = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GYRO");

    }
}