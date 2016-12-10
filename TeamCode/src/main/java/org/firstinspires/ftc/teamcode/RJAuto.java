package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

            encoderDrive();
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

    public void encoderDrive(){

        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);

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

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void debug(){



    }
}
