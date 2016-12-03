package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by TigeRobots on 11/11/2016.
 */
@Autonomous(name = "Compass", group = "Sensor")
public class CompassTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cCompassSensor compass;

    ElapsedTime timer = new ElapsedTime();

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;


    public void runOpMode(){

        roboInit();

        while (opModeIsActive()) {

            while (opModeIsActive()) {

                    if (gamepad1.a && !compass.isCalibrating()) {

                        telemetry.log().clear();
                        telemetry.log().add("Calibration started");
                        telemetry.log().add("Slowly rotate compass 360deg");
                        telemetry.log().add("Press 'B' when complete");
                        compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
                        timer.reset();

                        while (gamepad1.a && opModeIsActive()) {
                            doTelemetry();
                            idle();
                        }
                    }

                if (gamepad1.b && compass.isCalibrating()) {

                    telemetry.log().clear();
                    telemetry.log().add("Calibration complete");
                    compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

                    if (compass.calibrationFailed()) {
                        telemetry.log().add("Calibration failed");
                        compass.writeCommand(ModernRoboticsI2cCompassSensor.Command.NORMAL);
                    }

                    while (gamepad1.a && opModeIsActive()) {
                        doTelemetry();
                        idle();
                    }
                }


            }
    }
    }
    protected void doTelemetry() {

        if (compass.isCalibrating()) {

            telemetry.addData("compass", "calibrating %s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");

        } else {

            telemetry.addData("heading", "%.1f", compass.getDirection());

            Acceleration accel = compass.getAcceleration();
            double accelMagnitude = Math.sqrt(accel.xAccel*accel.xAccel + accel.yAccel*accel.yAccel + accel.zAccel*accel.zAccel);
            telemetry.addData("accel", accel);
            telemetry.addData("accel magnitude", "%.3f", accelMagnitude);

            telemetry.addData("mag flux", compass.getMagneticFlux());
        }

        telemetry.addData("command", "%s", compass.readCommand());

        telemetry.update();
    }

    public void roboInit(){

        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
        compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

    }

}
