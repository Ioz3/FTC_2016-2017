package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //gyro
    ModernRoboticsI2cGyro gyro;

    //color sensor
    ColorSensor lineColor;
    ColorSensor beaconColor;

    //encoders
    static final double     COUNTS_PER_MOTOR_REV    = 420;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.94;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime                     = new ElapsedTime();

    //shooter
    private double  shooterSpeed;
    private int     shooterPosition;
    private int     currentShootPosition;
    private int     previousShootPosition;

    //beacon
    private double  beaconPositionIn;
    private double  beaconPositionOut;
    private int     beaconLineColor;
    private boolean beaconLine;

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

    //gyro rotate
    private boolean curResetState;
    private boolean lastResetState;
    int xVal, yVal, zVal;     // Gyro rate Values
    int heading;              // Gyro integrated heading
    int angleZ;

    double distanceTraveled;
    double currentTime;

    int    value = 1;
    int    encoderValue = 0;

    public void runOpMode()  {

        roboInit();

        gyroInit();

        waitForStart();

        //our main teleop loop
        while(opModeIsActive()) {

            beaconSwitchCase();
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

        //GYRO INIT
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GYRO");

        //COLOR SENSOR
        lineColor   = hardwareMap.colorSensor.get("LINE_COLOR");
        beaconColor = hardwareMap.colorSensor.get("BEACON_COLOR");

        //BEACON
        beaconPositionIn    = 0.0; //TODO set inital positions
        beaconPositionOut   = 0.0;
        beaconLine          = false;
        beaconLineColor     = 0; //find the color of the line

        //GYRO ROTATE
        curResetState   = false;
        lastResetState  = false;
        xVal            = 0;
        yVal            = 0;
        zVal            = 0;     // Gyro rate Values
        heading = 0;              // Gyro integrated heading
        angleZ = 0;

    }

    public void gyroInit(){

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

    }

    public void beaconSwitchCase(){

        switch(value){

            case 1: encoderDrive(DRIVE_SPEED,  48,  48, 5.0);value++;
                break;
            case 2: moveToBeacon(-0.5, 0.5, -0.5, 0.5);
                break;
            case 3: delay(0.5);gyroRotate(0.5, 0.5, 0.5, 0.5, 90);
                break;
            default:frontLeft.setPower(0.0);frontRight.setPower(0.0);backLeft.setPower(0.0);backRight.setPower(0.0);
                break;

        }

    }

    public void moveToBeacon(double motorOne, double motorTwo, double motorThree, double motorFour){

        //TODO get the values for the lines on the floor

        lineColor.alpha();

        if(!beaconLine) {
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

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroRotate(double motorOne, double motorTwo, double motorThree, double motorFour, int heading) {

        double gyroOffset = (gyro.getIntegratedZValue()*0.03)-heading;

        if (heading == gyro.getHeading()) {
            frontRight.setPower(motorOne + gyroOffset);
            frontLeft.setPower(motorTwo + gyroOffset);
            backRight.setPower(motorThree + gyroOffset);
            backLeft.setPower(motorFour + gyroOffset);
        } else {
            frontRight.setPower(0.0);
            frontLeft.setPower(0.0);
            backRight.setPower(0.0);
            backLeft.setPower(0.0);
            value++;
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


    }

    public void debug(){


        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);

        telemetry.addData("CHECK_LINE_COLOR", lineColor.alpha());
        telemetry.addData("CHECK_RED_COLOR", beaconColor.red());
        telemetry.addData("CHECK_BLUE_COLOR", beaconColor.blue());

        telemetry.update();

    }
}
