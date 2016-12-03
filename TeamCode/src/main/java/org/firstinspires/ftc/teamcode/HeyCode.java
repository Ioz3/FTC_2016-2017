package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;
/**
 * Created by TigeRobots on 11/10/2016.
 */
@TeleOp(name = "HeyCode" , group = "TeleOp")
@Disabled
public class HeyCode extends LinearOpMode {
     DcMotor frontRightmotor;
     DcMotor frontLeftmotor;
     DcMotor backRightmotor;
     DcMotor backLeftmotor;

    public void roboinit () {
        frontRightmotor =hardwareMap.dcMotor.get("FRONT_RIGHT_MOTOR");
        frontLeftmotor  =hardwareMap.dcMotor.get("FRONT_LEFT_MOTOR");
        backRightmotor  =hardwareMap.dcMotor.get("BACK_RIGHT_MOTOR");
        backLeftmotor   =hardwareMap.dcMotor.get("BACK_LEFT_MOTOR");
    }


    @Override
   public void runOpMode (){

        ModernRoboticsI2cGyro gyro;
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState = false;

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating()) {

            sleep(50);
            idle();

        }

        while (opModeIsActive()) {

            drive(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x);

            curResetState = (gamepad1.a && gamepad1.b);
            if (curResetState && !lastResetState) {
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
           telemetry.addData("1", "Int. Ang. %03d", angleZ);
           telemetry.addData("2", "X av. %03d", xVal);
           telemetry.addData("3", "Y av. %03d", zVal);
           telemetry.addData("4", "Z av. %03d", yVal);
            telemetry.update();
        }



        telemetry.addData(">", "Gyro Calibrated. Press Start.");
        telemetry.update();

        waitForStart();
   }
    double constrain(double x) {

        double speed;
        speed = x;

        speed = Range.clip(speed, -1, 1);

        return speed;
    }

    public void drive(double x, double y, double z) {

        frontRightmotor.setPower(constrain(x + y + z));
        frontLeftmotor.setPower(constrain(x + y - z));
        backRightmotor.setPower(constrain(x - y + z));
        backLeftmotor.setPower(constrain(x - y - z));
    }
}

