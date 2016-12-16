package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SamTest extends LinearOpMode {

    DcMotor motor;
    TouchSensor touchSensor;

    public void roboInit () {

        motor       = hardwareMap.dcMotor.get("MOTOR");
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");

    }
    public void runOpMode () {

        roboInit();

        waitForStart();


        while(opModeIsActive()){

            motor.setPower(1);

            debug();

            idle();

        }

    }

    public void debug(){

        touchSensor.isPressed();

    }

}
